#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <thread>
#include <chrono>
#include <random>
#include <memory>

namespace gazebo
{
  class ElevatorController : public ModelPlugin
  {
    // Elevator states
    enum ElevatorState
    {
      IDLE,
      CLOSING_DOORS,
      MOVING_UP,
      MOVING_DOWN,
      OPENING_DOORS
    };

    public:
      ElevatorController() : ModelPlugin()
      {
        // Initialize random number generator
        std::random_device rd;
        generator.seed(rd());
        stop_duration_dist = std::uniform_real_distribution<double>(10.0, 15.0);
      }

      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
      {
        // Store the pointer to the model
        this->model = _parent;
        this->world = _parent->GetWorld();

        // Get elevator ID from SDF
        if (_sdf->HasElement("elevator_id"))
        {
          this->elevator_id = _sdf->Get<int>("elevator_id");
        }
        else
        {
          this->elevator_id = 1;
        }

        // Initialize ROS2
        if (!rclcpp::ok())
        {
          rclcpp::init(0, nullptr);
        }

        // Create ROS2 node
        std::string node_name = "elevator_controller_" + std::to_string(elevator_id);
        this->ros_node = rclcpp::Node::make_shared(node_name);

        // Create ROS2 publishers
        std::string ns = "elevator_" + std::to_string(elevator_id);
        this->current_floor_pub = this->ros_node->create_publisher<std_msgs::msg::Int32>(
            ns + "/current_floor", 10);
        this->door_state_pub = this->ros_node->create_publisher<std_msgs::msg::Bool>(
            ns + "/door_state", 10);
        this->car_position_pub = this->ros_node->create_publisher<geometry_msgs::msg::PoseStamped>(
            ns + "/car_position", 10);

        // Initialize elevator parameters
        this->floor_heights = {0.0, 2.5, 5.0, 7.5}; // Ground, Floor 1, Floor 2, Floor 3
        this->current_floor = 0;
        this->target_floor = 0;
        this->current_state = IDLE;
        this->doors_open = true;
        this->movement_speed = 1.0; // m/s
        this->door_operation_time = 2.0; // seconds
        
        // Get initial position
        this->initial_pose = this->model->WorldPose();
        
        // Initialize timing
        this->last_update_time = this->world->SimTime();
        this->state_start_time = this->world->SimTime();
        this->stop_duration = stop_duration_dist(generator);

        // Get joint pointers
        this->left_door_joint = this->model->GetJoint("left_door_joint");
        this->right_door_joint = this->model->GetJoint("right_door_joint");

        if (!this->left_door_joint || !this->right_door_joint)
        {
          RCLCPP_ERROR(this->ros_node->get_logger(), 
                      "ElevatorController: Could not find door joints!");
          return;
        }

        // Create executor thread for ROS2
        this->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        this->executor->add_node(this->ros_node);
        this->executor_thread = std::thread([this]() {
          this->executor->spin();
        });

        // Listen to the update event
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ElevatorController::OnUpdate, this));

        // Schedule first movement
        this->ScheduleNextMovement();

        RCLCPP_INFO(this->ros_node->get_logger(), 
                   "ElevatorController: Loaded for elevator %d", elevator_id);
      }

      ~ElevatorController()
      {
        // Cleanup ROS2
        if (this->executor_thread.joinable())
        {
          this->executor->cancel();
          this->executor_thread.join();
        }
      }

    private:
      void OnUpdate()
      {
        common::Time current_time = this->world->SimTime();
        double dt = (current_time - this->last_update_time).Double();
        this->last_update_time = current_time;

        // State machine
        switch (this->current_state)
        {
          case IDLE:
            HandleIdleState(current_time);
            break;
          case CLOSING_DOORS:
            HandleClosingDoorsState(current_time);
            break;
          case MOVING_UP:
            HandleMovingUpState(current_time, dt);
            break;
          case MOVING_DOWN:
            HandleMovingDownState(current_time, dt);
            break;
          case OPENING_DOORS:
            HandleOpeningDoorsState(current_time);
            break;
        }

        // Publish current status
        PublishStatus();
      }

      void HandleIdleState(common::Time current_time)
      {
        // Check if it's time to move
        double elapsed = (current_time - this->state_start_time).Double();
        if (elapsed >= this->stop_duration)
        {
          // Start closing doors
          this->current_state = CLOSING_DOORS;
          this->state_start_time = current_time;
          RCLCPP_INFO(this->ros_node->get_logger(), 
                     "Elevator %d: Starting to close doors", elevator_id);
        }
      }

      void HandleClosingDoorsState(common::Time current_time)
      {
        double elapsed = (current_time - this->state_start_time).Double();
        double progress = elapsed / this->door_operation_time;
        
        if (progress >= 1.0)
        {
          // Doors fully closed
          this->doors_open = false;
          this->SetDoorPosition(0.0);
          
          // Start moving
          if (this->target_floor > this->current_floor)
          {
            this->current_state = MOVING_UP;
            RCLCPP_INFO(this->ros_node->get_logger(), 
                       "Elevator %d: Moving up to floor %d", elevator_id, target_floor);
          }
          else if (this->target_floor < this->current_floor)
          {
            this->current_state = MOVING_DOWN;
            RCLCPP_INFO(this->ros_node->get_logger(), 
                       "Elevator %d: Moving down to floor %d", elevator_id, target_floor);
          }
          else
          {
            // Same floor, go back to idle
            this->current_state = OPENING_DOORS;
          }
          this->state_start_time = current_time;
        }
        else
        {
          // Animate door closing
          double door_pos = (1.0 - progress) * 0.6; // 0.6m is full open position
          this->SetDoorPosition(door_pos);
        }
      }

      void HandleMovingUpState(common::Time current_time, double dt)
      {
        double target_height = this->floor_heights[this->target_floor];
        double current_height = this->model->WorldPose().Pos().Z();
        
        if (current_height >= target_height - 0.01) // Close enough
        {
          // Arrived at target floor
          this->current_floor = this->target_floor;
          this->SetElevatorHeight(target_height);
          this->current_state = OPENING_DOORS;
          this->state_start_time = current_time;
          RCLCPP_INFO(this->ros_node->get_logger(), 
                     "Elevator %d: Arrived at floor %d", elevator_id, current_floor);
        }
        else
        {
          // Continue moving up
          double new_height = current_height + this->movement_speed * dt;
          if (new_height > target_height)
            new_height = target_height;
          this->SetElevatorHeight(new_height);
        }
      }

      void HandleMovingDownState(common::Time current_time, double dt)
      {
        double target_height = this->floor_heights[this->target_floor];
        double current_height = this->model->WorldPose().Pos().Z();
        
        if (current_height <= target_height + 0.01) // Close enough
        {
          // Arrived at target floor
          this->current_floor = this->target_floor;
          this->SetElevatorHeight(target_height);
          this->current_state = OPENING_DOORS;
          this->state_start_time = current_time;
          RCLCPP_INFO(this->ros_node->get_logger(), 
                     "Elevator %d: Arrived at floor %d", elevator_id, current_floor);
        }
        else
        {
          // Continue moving down
          double new_height = current_height - this->movement_speed * dt;
          if (new_height < target_height)
            new_height = target_height;
          this->SetElevatorHeight(new_height);
        }
      }

      void HandleOpeningDoorsState(common::Time current_time)
      {
        double elapsed = (current_time - this->state_start_time).Double();
        double progress = elapsed / this->door_operation_time;
        
        if (progress >= 1.0)
        {
          // Doors fully open
          this->doors_open = true;
          this->SetDoorPosition(0.6);
          this->current_state = IDLE;
          this->state_start_time = current_time;
          this->stop_duration = stop_duration_dist(generator);
          this->ScheduleNextMovement();
          RCLCPP_INFO(this->ros_node->get_logger(), 
                     "Elevator %d: Doors opened at floor %d", elevator_id, current_floor);
        }
        else
        {
          // Animate door opening
          double door_pos = progress * 0.6; // 0.6m is full open position
          this->SetDoorPosition(door_pos);
        }
      }

      void SetDoorPosition(double position)
      {
        if (this->left_door_joint && this->right_door_joint)
        {
          this->left_door_joint->SetPosition(0, -position);
          this->right_door_joint->SetPosition(0, position);
        }
      }

      void SetElevatorHeight(double height)
      {
        ignition::math::Pose3d current_pose = this->model->WorldPose();
        ignition::math::Pose3d new_pose = current_pose;
        new_pose.Pos().Z() = height;
        this->model->SetWorldPose(new_pose);
      }

      void ScheduleNextMovement()
      {
        // Simple random floor selection
        std::uniform_int_distribution<int> floor_dist(0, 3);
        do {
          this->target_floor = floor_dist(generator);
        } while (this->target_floor == this->current_floor);
        
        RCLCPP_INFO(this->ros_node->get_logger(), 
                   "Elevator %d: Next target floor: %d", elevator_id, target_floor);
      }

      void PublishStatus()
      {
        // Publish current floor
        auto floor_msg = std_msgs::msg::Int32();
        floor_msg.data = this->current_floor;
        this->current_floor_pub->publish(floor_msg);

        // Publish door state
        auto door_msg = std_msgs::msg::Bool();
        door_msg.data = this->doors_open;
        this->door_state_pub->publish(door_msg);

        // Publish car position
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = this->ros_node->now();
        pose_msg.header.frame_id = "world";
        
        ignition::math::Pose3d current_pose = this->model->WorldPose();
        pose_msg.pose.position.x = current_pose.Pos().X();
        pose_msg.pose.position.y = current_pose.Pos().Y();
        pose_msg.pose.position.z = current_pose.Pos().Z();
        pose_msg.pose.orientation.x = current_pose.Rot().X();
        pose_msg.pose.orientation.y = current_pose.Rot().Y();
        pose_msg.pose.orientation.z = current_pose.Rot().Z();
        pose_msg.pose.orientation.w = current_pose.Rot().W();
        
        this->car_position_pub->publish(pose_msg);
      }

      // Private member variables
      physics::ModelPtr model;
      physics::WorldPtr world;
      physics::JointPtr left_door_joint;
      physics::JointPtr right_door_joint;
      
      std::shared_ptr<rclcpp::Node> ros_node;
      rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_floor_pub;
      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr door_state_pub;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr car_position_pub;
      
      std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
      std::thread executor_thread;
      
      event::ConnectionPtr updateConnection;
      
      int elevator_id;
      std::vector<double> floor_heights;
      int current_floor;
      int target_floor;
      ElevatorState current_state;
      bool doors_open;
      double movement_speed;
      double door_operation_time;
      double stop_duration;
      
      ignition::math::Pose3d initial_pose;
      common::Time last_update_time;
      common::Time state_start_time;
      
      std::mt19937 generator;
      std::uniform_real_distribution<double> stop_duration_dist;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ElevatorController)
}
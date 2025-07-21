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

      void LoadConfiguration()
      {
        // Declare parameters with default values
        this->ros_node->declare_parameter("floor_heights", std::vector<double>{0.0, 2.5, 5.0, 7.5});
        this->ros_node->declare_parameter("movement_speed", 1.0);
        this->ros_node->declare_parameter("door_operation_time", 2.0);
        this->ros_node->declare_parameter("stop_duration_min", 10.0);
        this->ros_node->declare_parameter("stop_duration_max", 15.0);
        this->ros_node->declare_parameter("stop_duration_multiplier", 1.0);
        this->ros_node->declare_parameter("initial_floor", 0);
        this->ros_node->declare_parameter("initial_doors_open", true);
        this->ros_node->declare_parameter("log_level", "INFO");

        // Load parameters
        this->floor_heights = this->ros_node->get_parameter("floor_heights").as_double_array();
        this->movement_speed = this->ros_node->get_parameter("movement_speed").as_double();
        this->door_operation_time = this->ros_node->get_parameter("door_operation_time").as_double();
        
        double stop_min = this->ros_node->get_parameter("stop_duration_min").as_double();
        double stop_max = this->ros_node->get_parameter("stop_duration_max").as_double();
        double multiplier = this->ros_node->get_parameter("stop_duration_multiplier").as_double();
        
        this->config_initial_floor = this->ros_node->get_parameter("initial_floor").as_int();
        this->config_initial_doors_open = this->ros_node->get_parameter("initial_doors_open").as_bool();
        
        // Apply multiplier to stop duration range
        this->stop_duration_min = stop_min * multiplier;
        this->stop_duration_max = stop_max * multiplier;
        
        // Update random distribution
        this->stop_duration_dist = std::uniform_real_distribution<double>(
            this->stop_duration_min, this->stop_duration_max);

        RCLCPP_INFO(this->ros_node->get_logger(), 
                   "Elevator %d configuration loaded - Speed: %.1f m/s, Stop duration: %.1f-%.1f s", 
                   elevator_id, movement_speed, stop_duration_min, stop_duration_max);
      }

      double GetRandomStopDuration()
      {
        return this->stop_duration_dist(generator);
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

        // Load configuration parameters
        this->LoadConfiguration();

        // Create ROS2 publishers
        std::string ns = "elevator_" + std::to_string(elevator_id);
        this->current_floor_pub = this->ros_node->create_publisher<std_msgs::msg::Int32>(
            ns + "/current_floor", 10);
        this->door_state_pub = this->ros_node->create_publisher<std_msgs::msg::Bool>(
            ns + "/door_state", 10);
        this->car_position_pub = this->ros_node->create_publisher<geometry_msgs::msg::PoseStamped>(
            ns + "/car_position", 10);

        // Initialize elevator parameters from configuration
        this->current_floor = this->config_initial_floor;
        this->target_floor = this->config_initial_floor;
        this->current_state = IDLE;
        this->doors_open = this->config_initial_doors_open;
        
        // Get initial position
        this->initial_pose = this->model->WorldPose();
        
        // Set initial height based on configuration
        double initial_height = this->floor_heights[this->config_initial_floor];
        this->SetElevatorHeight(initial_height);
        
        // Initialize timing
        this->last_update_time = this->world->SimTime();
        this->state_start_time = this->world->SimTime();
        this->stop_duration = this->GetRandomStopDuration();

        this->door_joint = this->model->GetJoint("door_joint");

        if (!this->door_joint)
        {
            RCLCPP_ERROR(this->ros_node->get_logger(), 
                        "ElevatorController: Could not find door joint!");
            return;
        }

        if (this->doors_open)
        {
          if (this->door_joint)
          {
              this->SetDoorPosition(-1.2); // Start open
          }
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
            // Start closing doors (not moving directly)
            this->current_state = CLOSING_DOORS;
            this->state_start_time = current_time;
            RCLCPP_INFO(this->ros_node->get_logger(), 
                      "Elevator %d: Starting to close door", elevator_id);
        }
      }

      void HandleClosingDoorsState(common::Time current_time)
      {
        double elapsed = (current_time - this->state_start_time).Double();
        double progress = elapsed / this->door_operation_time;
        
        if (progress >= 1.0)
        {
            // Door fully closed
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
            this->state_start_time = current_time;
        }
        else
        {
            // Animate door closing (0 to -1.2)
            double door_pos = -1.2 + (progress * 1.2);
            this->SetDoorPosition(door_pos);
        }
      }
      
      void HandleMovingUpState(common::Time current_time, double dt)
      {
          double target_height = this->floor_heights[this->target_floor];
          double current_height = this->model->WorldPose().Pos().Z();
          
          if (current_height >= target_height - 0.05)
          {
              // Arrived - start opening doors
              this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
              this->current_floor = this->target_floor;
              this->current_state = OPENING_DOORS;  // Go to door opening, not IDLE
              this->state_start_time = current_time;
              RCLCPP_INFO(this->ros_node->get_logger(), 
                        "Elevator %d: Arrived at floor %d, opening door", elevator_id, current_floor);
          }
          else
          {
              // Keep moving up
              this->model->SetLinearVel(ignition::math::Vector3d(0, 0, this->movement_speed));
          }
      }

      void HandleMovingDownState(common::Time current_time, double dt)
      {
          double target_height = this->floor_heights[this->target_floor];
          double current_height = this->model->WorldPose().Pos().Z();
          
          if (current_height <= target_height + 0.05)
          {
              // Arrived - start opening doors
              this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
              this->current_floor = this->target_floor;
              this->current_state = OPENING_DOORS;  // Go to door opening, not IDLE
              this->state_start_time = current_time;
              RCLCPP_INFO(this->ros_node->get_logger(), 
                        "Elevator %d: Arrived at floor %d, opening door", elevator_id, current_floor);
          }
          else
          {
              // Keep moving down
              this->model->SetLinearVel(ignition::math::Vector3d(0, 0, -this->movement_speed));
          }
      }

      void HandleOpeningDoorsState(common::Time current_time)
      {
        double elapsed = (current_time - this->state_start_time).Double();
        double progress = elapsed / this->door_operation_time;
        
        if (progress >= 1.0)
        {
            // Door fully open
            this->doors_open = true;
            this->SetDoorPosition(-1.2);
            this->current_state = IDLE;
            this->state_start_time = current_time;
            this->stop_duration = this->GetRandomStopDuration();
            this->ScheduleNextMovement();
            RCLCPP_INFO(this->ros_node->get_logger(), 
                      "Elevator %d: Door opened at floor %d", elevator_id, current_floor);
        }
        else
        {
            // Animate door opening (-1.2 to 0)
            double door_pos = progress * -1.2;
            this->SetDoorPosition(door_pos);
        }
      }

      void SetDoorPosition(double position)
      {
        if (this->door_joint)
        {
            // position: 0 = closed, -1.2 = fully open
            this->door_joint->SetPosition(0, position);
        }
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

      void SetElevatorHeight(double target_height)
      {
          ignition::math::Pose3d current_pose = this->model->WorldPose();
          ignition::math::Pose3d new_pose = current_pose;
          new_pose.Pos().Z() = target_height;
          new_pose.Rot() = ignition::math::Quaterniond::Identity; // Keep upright
          this->model->SetWorldPose(new_pose);
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
      physics::JointPtr door_joint;
      
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
      double stop_duration_min;
      double stop_duration_max;
      
      // Configuration parameters
      int config_initial_floor;
      bool config_initial_doors_open;
      
      ignition::math::Pose3d initial_pose;
      common::Time last_update_time;
      common::Time state_start_time;
      
      std::mt19937 generator;
      std::uniform_real_distribution<double> stop_duration_dist;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ElevatorController)
}
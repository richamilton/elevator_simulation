#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

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

      void handle_goto_request(
          const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
      {
          int target_floor = request->a;
          
          if (target_floor >= 0 && target_floor < static_cast<int>(this->floor_heights.size()))
          {
              this->requested_target_floor = target_floor;
              this->has_external_request = true;
              this->autonomous_mode = false;
              response->sum = 1; // Success
              
              RCLCPP_INFO(this->ros_node->get_logger(), 
                         "Elevator %d: Goto request for floor %d", 
                         elevator_id, target_floor);
          }
          else
          {
              response->sum = 0; // Failed
              RCLCPP_WARN(this->ros_node->get_logger(), 
                         "Elevator %d: Invalid floor request: %d", 
                         elevator_id, target_floor);
          }
      }
      
      void handle_hold_request(
          const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
      {
          int target_floor = request->a;
          
          if (target_floor >= 0 && target_floor < static_cast<int>(this->floor_heights.size()))
          {
              this->requested_target_floor = target_floor;
              this->has_external_request = true;
              this->is_held = true;
              this->autonomous_mode = false;
              response->sum = 1; // Success
              
              RCLCPP_INFO(this->ros_node->get_logger(), 
                         "Elevator %d: Hold request for floor %d", 
                         elevator_id, target_floor);
          }
          else
          {
              response->sum = 0; // Failed
              RCLCPP_WARN(this->ros_node->get_logger(), 
                         "Elevator %d: Invalid hold floor request: %d", 
                         elevator_id, target_floor);
          }
      }
      
      void handle_release_request(
          const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
      {
          (void)request; // Unused parameter
          
          this->is_held = false;
          this->autonomous_mode = true;
          response->sum = 1; // Success
          
          RCLCPP_INFO(this->ros_node->get_logger(), 
                     "Elevator %d: Released from hold, returning to autonomous mode", 
                     elevator_id);
      }

      void LoadConfiguration()
      {
        // Declare parameters with default values      
        //  if (this->sdf->HasElement("floor_heights")) {
        //       // SDF stores as string, so parse it into a vector<double>
        //       std::string heights_str = this->sdf->Get<std::string>("floor_heights");
        //       std::stringstream ss(heights_str);
        //       double val;
        //       this->floor_heights.clear();
        //       while (ss >> val) {
        //         this->floor_heights.push_back(val);
        //         // skip commas or spaces
        //         if (ss.peek() == ',' || ss.peek() == ' ')
        //           ss.ignore();
        //       }
        // } else {
        //     this->ros_node->declare_parameter("floor_heights", std::vector<double>{0.0, 2.5, 5.0, 7.5});
        //     this->floor_heights = this->ros_node->get_parameter("floor_heights").as_double_array();
        // }
        this->ros_node->declare_parameter("floor_heights", std::vector<double>{0.0, 2.5, 5.0, 7.5});
        this->floor_heights = this->ros_node->get_parameter("floor_heights").as_double_array();

        if (this->sdf->HasElement("movement_speed")) {
            this->movement_speed = this->sdf->Get<double>("movement_speed");
        } else {
            this->ros_node->declare_parameter("movement_speed", 1.0);
            this->movement_speed = this->ros_node->get_parameter("movement_speed").as_double();
        }

        if (this->sdf->HasElement("door_operation_time")) {
            this->door_operation_time = this->sdf->Get<double>("door_operation_time");
        } else {
            this->ros_node->declare_parameter("door_operation_time", 2.0);
            this->door_operation_time = this->ros_node->get_parameter("door_operation_time").as_double();
        }
        
        if (this->sdf->HasElement("stop_duration_min")) {
            double stop_min = this->sdf->Get<double>("stop_duration_min");
            double stop_max = this->sdf->Get<double>("stop_duration_max");
            double multiplier = this->sdf->Get<double>("stop_duration_multiplier");
            this->stop_duration_min = stop_min * multiplier;
            this->stop_duration_max = stop_max * multiplier;
        } else {
            this->ros_node->declare_parameter("stop_duration_min", 10.0);
            this->ros_node->declare_parameter("stop_duration_max", 15.0);
            this->ros_node->declare_parameter("stop_duration_multiplier", 1.0);
            double stop_min = this->ros_node->get_parameter("stop_duration_min").as_double();
            double stop_max = this->ros_node->get_parameter("stop_duration_max").as_double();
            double multiplier = this->ros_node->get_parameter("stop_duration_multiplier").as_double();
            this->stop_duration_min = stop_min * multiplier;
            this->stop_duration_max = stop_max * multiplier;
        }

        if (this->sdf->HasElement("door_open_width")) {
            this->door_open_width = this->sdf->Get<double>("door_open_width");
        } else {
            this->ros_node->declare_parameter("door_open_width", 1.7);
            this->door_open_width = this->ros_node->get_parameter("door_open_width").as_double();
            this->door_open_position = -1 * this->door_open_width; // Open position is negative
        }

        if (this->sdf->HasElement("door_open_direction")) {
            this->door_open_direction = this->sdf->Get<int>("door_open_direction");
            this->door_open_position = this->door_open_direction * this->door_open_width;
        } else {
            this->ros_node->declare_parameter("door_open_direction", -1);
            this->door_open_direction = this->ros_node->get_parameter("door_open_direction").as_int();
            this->door_open_position = this->door_open_direction * this->door_open_width;
        }

        if (this->sdf->HasElement("initial_floor")) {
            this->config_initial_floor = this->sdf->Get<int>("initial_floor");
        } else {
            this->ros_node->declare_parameter("initial_floor", 0);
            this->config_initial_floor = this->ros_node->get_parameter("initial_floor").as_int();
        }
        
        if (this->sdf->HasElement("initial_doors_open")) {
            this->config_initial_doors_open = this->sdf->Get<bool>("initial_doors_open");
        } else {
            this->ros_node->declare_parameter("initial_doors_open", true);
            this->config_initial_doors_open = this->ros_node->get_parameter("initial_doors_open").as_bool();
        }

        this->ros_node->declare_parameter("log_level", "INFO");
        
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
        // Store SDF pointer for configuration reading
        this->sdf = _sdf;

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

        // Create ROS2 service servers
        this->goto_service = this->ros_node->create_service<example_interfaces::srv::AddTwoInts>(
            ns + "/goto", 
            std::bind(&ElevatorController::handle_goto_request, this, 
                     std::placeholders::_1, std::placeholders::_2));
        this->hold_service = this->ros_node->create_service<example_interfaces::srv::AddTwoInts>(
            ns + "/hold", 
            std::bind(&ElevatorController::handle_hold_request, this, 
                     std::placeholders::_1, std::placeholders::_2));
        this->release_service = this->ros_node->create_service<example_interfaces::srv::AddTwoInts>(
            ns + "/release", 
            std::bind(&ElevatorController::handle_release_request, this, 
                     std::placeholders::_1, std::placeholders::_2));

        // Initialize elevator parameters from configuration
        this->current_floor = this->config_initial_floor;
        this->target_floor = this->config_initial_floor;
        this->current_state = IDLE;
        this->doors_open = this->config_initial_doors_open;
        
        // Initialize external request handling
        this->has_external_request = false;
        this->requested_target_floor = 0;
        this->is_held = false;
        this->autonomous_mode = true;
        
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
              this->SetDoorPosition(this->door_open_direction * this->door_open_width); // Start open
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
        // If held, don't move
        if (this->is_held)
        {
            if (this->target_floor != this->current_floor)
            {
                this->current_state = CLOSING_DOORS;
                this->state_start_time = current_time;
                RCLCPP_INFO(this->ros_node->get_logger(), 
                          "Elevator %d: External request - starting to close door", elevator_id);
            }
            return;
        }
        
        // Check for external requests
        if (this->has_external_request)
        {
            this->target_floor = this->requested_target_floor;
            this->has_external_request = false;
            this->autonomous_mode = true;
            if (this->target_floor != this->current_floor)
            {
                this->current_state = CLOSING_DOORS;
                this->state_start_time = current_time;
                RCLCPP_INFO(this->ros_node->get_logger(), 
                          "Elevator %d: External request - starting to close door", elevator_id);
            }
            return;
        }
        
        // Check if it's time to move (autonomous mode only)
        if (this->autonomous_mode)
        {
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
            // Animate door closing (door_open_position to 0)
            double door_pos = this->door_open_position + (-1 * this->door_open_direction * progress * this->door_open_width);
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
            this->SetDoorPosition(this->door_open_position);
            // If held, don't move
            if (this->is_held && !this->has_external_request)
            {
                return;
            }
            this->current_state = IDLE;
            this->state_start_time = current_time;
            this->stop_duration = this->GetRandomStopDuration();
            this->ScheduleNextMovement();
            RCLCPP_INFO(this->ros_node->get_logger(), 
                      "Elevator %d: Door opened at floor %d", elevator_id, current_floor);
        }
        else
        {
            // Animate door opening (0 to door_open_position)
            double door_pos = progress * this->door_open_position;
            this->SetDoorPosition(door_pos);
        }
      }

      void SetDoorPosition(double position)
      {
        if (this->door_joint)
        {
            // position: 0 = closed, door_open_position = fully open
            this->door_joint->SetPosition(0, position);
        }
      }

      void ScheduleNextMovement()
      {
          // Check for external requests first
          if (this->has_external_request)
          {
              this->target_floor = this->requested_target_floor;
              this->has_external_request = false;
              this->autonomous_mode = true;
              RCLCPP_INFO(this->ros_node->get_logger(), 
                        "Elevator %d: External request - target floor: %d", elevator_id, target_floor);
              return;
          }
          
          // Only do autonomous movement if not held and in autonomous mode
          if (!this->is_held && this->autonomous_mode)
          {
              // Simple random floor selection
              std::uniform_int_distribution<int> floor_dist(0, 3);
              do {
                  this->target_floor = floor_dist(generator);
              } while (this->target_floor == this->current_floor);
              
              RCLCPP_INFO(this->ros_node->get_logger(), 
                        "Elevator %d: Autonomous - next target floor: %d", elevator_id, target_floor);
          }
      }

      void SetElevatorHeight(double target_height)
      {
          ignition::math::Pose3d current_pose = this->model->WorldPose();
          ignition::math::Pose3d new_pose = current_pose;
          new_pose.Pos().Z() = target_height;
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
      // SDF pointer for configuration reading
      sdf::ElementPtr sdf;

      // Private member variables
      physics::ModelPtr model;
      physics::WorldPtr world;
      physics::JointPtr door_joint;
      
      std::shared_ptr<rclcpp::Node> ros_node;
      rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_floor_pub;
      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr door_state_pub;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr car_position_pub;
      
      rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr goto_service;
      rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr hold_service;
      rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr release_service;
      
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
      double door_open_width;
      double door_open_position;
      int door_open_direction;

      // Configuration parameters
      int config_initial_floor;
      bool config_initial_doors_open;
      
      // External request handling
      bool has_external_request;
      int requested_target_floor;
      bool is_held;
      bool autonomous_mode;
      
      ignition::math::Pose3d initial_pose;
      common::Time last_update_time;
      common::Time state_start_time;
      
      std::mt19937 generator;
      std::uniform_real_distribution<double> stop_duration_dist;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ElevatorController)
}
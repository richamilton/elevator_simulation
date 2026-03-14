#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>

#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/components/Component.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/sdf.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

#include <thread>
#include <chrono>
#include <random>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace avdr_gz_worlds
{

class ElevatorController
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{
    enum ElevatorState { IDLE, CLOSING_DOORS, MOVING_UP, MOVING_DOWN, OPENING_DOORS };
    enum OperatingMode { AUTONOMOUS, REQUEST_GOTO, REQUEST_HOLD };

public:
    ElevatorController()
    {
        std::random_device rd;
        generator.seed(rd());
    }

    void Configure(
        const ignition::gazebo::Entity & _entity,
        const std::shared_ptr<const sdf::Element> & _sdf,
        ignition::gazebo::EntityComponentManager & _ecm,
        ignition::gazebo::EventManager & /*_eventMgr*/) override
    {
        this->model = ignition::gazebo::Model(_entity);

        if (!this->model.Valid(_ecm))
        {
            ignerr << "ElevatorController: Invalid model entity\n";
            return;
        }

        if (_sdf->HasElement("elevator_id"))
            this->elevator_id = _sdf->Get<int>("elevator_id");

        if (_sdf->HasElement("movement_speed"))
            this->movement_speed = _sdf->Get<double>("movement_speed");

        if (_sdf->HasElement("movement_accel"))
            this->movement_accel = _sdf->Get<double>("movement_accel");

        if (_sdf->HasElement("door_operation_time"))
            this->door_operation_time = _sdf->Get<double>("door_operation_time");

        if (_sdf->HasElement("stop_duration_min") && _sdf->HasElement("stop_duration_max"))
        {
            double multiplier = _sdf->HasElement("stop_duration_multiplier")
                ? _sdf->Get<double>("stop_duration_multiplier") : 1.0;
            this->stop_duration_min = _sdf->Get<double>("stop_duration_min") * multiplier;
            this->stop_duration_max = _sdf->Get<double>("stop_duration_max") * multiplier;
        }

        if (_sdf->HasElement("door_open_width"))
            this->door_open_width = _sdf->Get<double>("door_open_width");

        if (_sdf->HasElement("door_open_direction"))
            this->door_open_direction = _sdf->Get<int>("door_open_direction");

        this->door_open_position = this->door_open_direction * this->door_open_width;

        if (_sdf->HasElement("initial_floor"))
            this->current_floor = _sdf->Get<int>("initial_floor");

        if (_sdf->HasElement("initial_doors_open"))
            this->doors_open = _sdf->Get<bool>("initial_doors_open");

        if (_sdf->HasElement("floor_heights"))
        {
            std::string s = _sdf->Get<std::string>("floor_heights");
            s.erase(std::remove(s.begin(), s.end(), '['), s.end());
            s.erase(std::remove(s.begin(), s.end(), ']'), s.end());
            std::stringstream ss(s);
            std::string token;
            this->floor_heights.clear();
            while (std::getline(ss, token, ','))
            {
                try { this->floor_heights.push_back(std::stod(token)); } catch (...) {}
            }
        }

        if (this->floor_heights.empty())
            this->floor_heights = {-0.1, 2.5, 5.0, 7.5};

        this->target_floor = this->current_floor;
        this->current_z = this->floor_heights[this->current_floor];
        this->move_origin_z = this->current_z;
        this->stop_duration_dist = std::uniform_real_distribution<double>(
            this->stop_duration_min, this->stop_duration_max);
        this->stop_duration = GetRandomStopDuration();

        // Find door joint and platform link
        this->elevator_joint = this->model.JointByName(_ecm, "elevator_joint");
        if (this->elevator_joint == ignition::gazebo::kNullEntity)
        {
            ignerr << "ElevatorController " << elevator_id << ": Could not find elevator_joint\n";
            return;
        }

        this->door_joint = this->model.JointByName(_ecm, "door_joint");
        if (this->door_joint == ignition::gazebo::kNullEntity)
        {
            ignerr << "ElevatorController " << elevator_id << ": Could not find door_joint\n";
            return;
        }

        this->platform_link = this->model.LinkByName(_ecm, "platform");
        if (this->platform_link == ignition::gazebo::kNullEntity)
        {
            ignerr << "ElevatorController " << elevator_id << ": Could not find platform link\n";
            return;
        }

        // Enable required ECM components
        // JointPositionReset is created on first use in SetDoorPosition

        if (!_ecm.Component<ignition::gazebo::components::WorldPose>(this->model.Entity()))
            _ecm.CreateComponent(this->model.Entity(), ignition::gazebo::components::WorldPose());

        if (!_ecm.Component<ignition::gazebo::components::Pose>(this->model.Entity()))
            _ecm.CreateComponent(this->model.Entity(), ignition::gazebo::components::Pose());
        if (!_ecm.Component<ignition::gazebo::components::JointPosition>(this->elevator_joint))
           _ecm.CreateComponent(this->elevator_joint, ignition::gazebo::components::JointPosition());
        // initial_pose is captured on the first PreUpdate tick once the ECM
        // has been fully populated by the physics system.
        this->pose_initialized = false;

        // Initialize ROS2
        if (!rclcpp::ok())
            rclcpp::init(0, nullptr);


        std::string node_name = "elevator_controller_" + std::to_string(elevator_id);
        this->ros_node = rclcpp::Node::make_shared(node_name);

        std::string ns = "elevator_" + std::to_string(elevator_id);

        // Set initial state
        // Note: WorldPose is not available in Configure - it will be initialized in PreUpdate
        SetDoorPosition(_ecm, this->doors_open ? this->door_open_position : 0.0);

        this->current_floor_pub = this->ros_node->create_publisher<std_msgs::msg::Int32>(
            ns + "/current_floor", 10);
        this->door_state_pub = this->ros_node->create_publisher<std_msgs::msg::Bool>(
            ns + "/door_state", 10);
        this->car_position_pub = this->ros_node->create_publisher<geometry_msgs::msg::PoseStamped>(
            ns + "/car_position", 10);

        this->goto_service = this->ros_node->create_service<example_interfaces::srv::AddTwoInts>(
            ns + "/goto",
            std::bind(&ElevatorController::HandleGotoRequest, this,
                std::placeholders::_1, std::placeholders::_2));
        this->hold_service = this->ros_node->create_service<example_interfaces::srv::AddTwoInts>(
            ns + "/hold",
            std::bind(&ElevatorController::HandleHoldRequest, this,
                std::placeholders::_1, std::placeholders::_2));
        this->release_service = this->ros_node->create_service<example_interfaces::srv::AddTwoInts>(
            ns + "/release",
            std::bind(&ElevatorController::HandleReleaseRequest, this,
                std::placeholders::_1, std::placeholders::_2));

        this->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        this->executor->add_node(this->ros_node);
        this->executor_thread = std::thread([this]() { this->executor->spin(); });

        ScheduleNextMovement();

        RCLCPP_INFO(this->ros_node->get_logger(),
            "ElevatorController loaded for elevator %d (Ignition Gazebo)", elevator_id);
    }

    void PreUpdate(
        const ignition::gazebo::UpdateInfo & _info,
        ignition::gazebo::EntityComponentManager & _ecm) override
    {
        if (_info.paused)
            return;

        // Capture spawn X/Y on first tick that the ECM has a non-zero pose.
        // Retry each tick until valid — the physics system may need a few ticks
        // to populate the Pose component from the SDF world pose.
        if (!this->pose_initialized)
        {
            // Read Pose component (contains SDF <pose> data)
            auto * pose_c = _ecm.Component<ignition::gazebo::components::Pose>(this->model.Entity());
            if (pose_c)
            {
                const auto & p = pose_c->Data();

                // SDF pose is in world coordinates for top-level models
                // Store model's pose, accounting for platform link offset (0.05m)
                // floor_heights represent platform surface height, so subtract offset
                this->initial_pose = ignition::math::Pose3d(
                    p.Pos().X(),
                    p.Pos().Y(),
                    - 0.15,
                    p.Rot().Roll(),
                    p.Rot().Pitch(),
                    p.Rot().Yaw()
                );
                this->pose_initialized = true;
                this->SetElevatorPosition(_ecm, this->initial_pose.Pos().Z());
                // this->SetElevatorHeight(_ecm, this->initial_pose.Pos().Z());
                
                RCLCPP_WARN(this->ros_node->get_logger(),
                    "Elevator %d: pose locked X=%f Y=%f Z=%f",
                    elevator_id, p.Pos().X(), p.Pos().Y(), this->initial_pose.Pos().Z());
                
                
            }
            // Don't run state machine or apply commands until pose is known
            return;
        }

        double sim_time = std::chrono::duration<double>(_info.simTime).count();

        switch (this->current_state)
        {
            case IDLE:          HandleIdleState(_ecm, sim_time);           break;
            case CLOSING_DOORS: HandleClosingDoorsState(_ecm, sim_time);   break;
            case MOVING_UP:     HandleMovingUpState(_ecm, sim_time);   break;
            case MOVING_DOWN:   HandleMovingDownState(_ecm, sim_time); break;
            case OPENING_DOORS: HandleOpeningDoorsState(_ecm, sim_time);   break;
        }

        // Always re-apply elevator pose every tick to prevent the model drifting
        // when WorldPoseCmd is not actively issued (Ignition drops the constraint).
        // Door position is only re-applied when animating to avoid joint flicker.
        // SetElevatorHeight(_ecm, this->current_z);

        bool doors_animating = (this->current_state == OPENING_DOORS || this->current_state == CLOSING_DOORS);
        if (doors_animating)
            SetDoorPosition(_ecm, this->current_door_position);

        // Publish ROS status at ~5 Hz
        if (++this->tick_counter % 200 == 0)
            PublishStatus(_ecm);
    }

    ~ElevatorController()
    {
        if (this->executor_thread.joinable())
        {
            this->executor->cancel();
            this->executor_thread.join();
        }
    }

private:
    // ── ROS service handlers ───────────────────────────────────────────────

    void HandleGotoRequest(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        int floor = request->a;
        if (floor >= 0 && floor < static_cast<int>(floor_heights.size()))
        {
            this->target_floor = floor;
            this->operating_mode = REQUEST_GOTO;
            response->sum = 1;
            RCLCPP_INFO(ros_node->get_logger(), "Elevator %d: goto floor %d", elevator_id, floor);
        }
        else
        {
            response->sum = 0;
            RCLCPP_WARN(ros_node->get_logger(), "Elevator %d: invalid goto floor %d", elevator_id, floor);
        }
    }

    void HandleHoldRequest(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        int floor = request->a;
        if (floor >= 0 && floor < static_cast<int>(floor_heights.size()))
        {
            this->target_floor = floor;
            this->operating_mode = REQUEST_HOLD;
            response->sum = 1;
            RCLCPP_INFO(ros_node->get_logger(), "Elevator %d: hold at floor %d", elevator_id, floor);
        }
        else
        {
            response->sum = 0;
            RCLCPP_WARN(ros_node->get_logger(), "Elevator %d: invalid hold floor %d", elevator_id, floor);
        }
    }

    void HandleReleaseRequest(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> /*request*/,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        this->operating_mode = AUTONOMOUS;
        response->sum = 1;
        RCLCPP_INFO(ros_node->get_logger(), "Elevator %d: released to autonomous", elevator_id);
    }

    // ── State machine ──────────────────────────────────────────────────────

    void HandleIdleState(ignition::gazebo::EntityComponentManager & /*_ecm*/, double sim_time)
    {
        if (this->current_floor != this->target_floor)
        {
            this->current_state = CLOSING_DOORS;
            this->state_start_time = sim_time;
            RCLCPP_INFO(ros_node->get_logger(), "Elevator %d: closing doors", elevator_id);
            return;
        }

        if (this->operating_mode == REQUEST_HOLD)
            return;

        if ((sim_time - this->state_start_time) >= this->stop_duration)
        {
            this->current_state = CLOSING_DOORS;
            this->state_start_time = sim_time;
            RCLCPP_INFO(ros_node->get_logger(), "Elevator %d: closing doors (autonomous)", elevator_id);
        }
    }

    void HandleClosingDoorsState(ignition::gazebo::EntityComponentManager & _ecm, double sim_time)
    {
        double progress = (sim_time - this->state_start_time) / this->door_operation_time;

        if (progress >= 1.0)
        {
            this->doors_open = false;
            SetDoorPosition(_ecm, 0.0);

            if (this->operating_mode == AUTONOMOUS)
                ScheduleNextMovement();

            if (this->target_floor > this->current_floor)
            {
                this->move_origin_z = this->current_z;
                this->current_state = MOVING_UP;
                RCLCPP_INFO(ros_node->get_logger(),
                    "Elevator %d: moving up to floor %d", elevator_id, target_floor);
            }
            else if (this->target_floor < this->current_floor)
            {
                this->move_origin_z = this->current_z;
                this->current_state = MOVING_DOWN;
                RCLCPP_INFO(ros_node->get_logger(),
                    "Elevator %d: moving down to floor %d", elevator_id, target_floor);
            }
            else
            {
                this->current_state = OPENING_DOORS;
            }
            this->state_start_time = sim_time;
        }
        else
        {
            double door_pos = this->door_open_position
                + (-1.0 * this->door_open_direction * progress * this->door_open_width);
            SetDoorPosition(_ecm, door_pos);
        }
    }

    // Trapezoidal velocity profile: ramp up → cruise → ramp down.
    // Returns the velocity to use given distance travelled and distance remaining.
    double TrapezoidalVelocity(double dist_travelled, double dist_remaining)
    {
        double ramp_dist = (this->movement_speed * this->movement_speed) / (2.0 * this->movement_accel);
        double total = dist_travelled + dist_remaining;

        double v;
        if (total < 2.0 * ramp_dist)
        {
            // Short travel: triangle profile — peak at midpoint
            double half = total / 2.0;
            if (dist_travelled < half)
                v = std::sqrt(2.0 * this->movement_accel * dist_travelled);
            else
                v = std::sqrt(2.0 * this->movement_accel * dist_remaining);
        }
        else if (dist_travelled < ramp_dist)
            v = std::sqrt(2.0 * this->movement_accel * dist_travelled);  // ramp up
        else if (dist_remaining < ramp_dist)
            v = std::sqrt(2.0 * this->movement_accel * dist_remaining);  // ramp down
        else
            v = this->movement_speed;  // cruise

        return std::max(v, this->movement_speed * 0.05);  // minimum 5% speed to always make progress
    }

    void HandleMovingUpState(
        ignition::gazebo::EntityComponentManager & _ecm, double sim_time)
    {
        double target_z = this->floor_heights[this->target_floor];
        this->current_z = _ecm.Component<ignition::gazebo::components::JointPosition>(this->elevator_joint)->Data()[0];
        // RCLCPP_INFO(ros_node->get_logger(),
        //         "Elevator %d: current z %f", elevator_id, this->current_z);
        if (this->current_z >= target_z)
        {
            // // Hard-snap current_z to eliminate any accumulated floating point drift
            // this->SetElevatorHeight(_ecm, target_z);
            // Bring elevator to a stop
            this->SetElevatorVelocity(_ecm, 0.0);

            // this->current_z = target_z;
            this->current_floor = this->target_floor;
            this->current_state = OPENING_DOORS;
            this->state_start_time = sim_time;
            // RCLCPP_INFO(ros_node->get_logger(),
            //     "Elevator %d: arrived at floor %d at %f", elevator_id, current_floor, this->current_z);
        }
        else {
            // Determine velocity based on trapezoidal profile
            double dist_remaining = target_z - this->current_z;
            double dist_travelled = this->current_z - this->move_origin_z;
            double v = TrapezoidalVelocity(std::max(dist_travelled, 0.0), dist_remaining);
            // Apply velocity command to move the elevator up
            this->SetElevatorVelocity(_ecm, v);

            // TODO: Remove ONLY for debugging
            // auto vel_cmd = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(this->elevator_joint);
            // double current_vel_cmd = vel_cmd ? vel_cmd->Data()[0] : 0.0;
            // RCLCPP_INFO(ros_node->get_logger(),
            //     "Elevator %d: moving up v_cmd=%.3f actual_v_cmd=%.3f current_z=%.3f target_z=%.3f",
            //     elevator_id, v, current_vel_cmd, this->current_z, target_z);
        }
    }

    void HandleMovingDownState(
        ignition::gazebo::EntityComponentManager & _ecm, double sim_time)
    {

        double target_z = this->floor_heights[this->target_floor];
        this->current_z = _ecm.Component<ignition::gazebo::components::JointPosition>(this->elevator_joint)->Data()[0];
        // RCLCPP_INFO(ros_node->get_logger(),
        //         "Elevator %d: current z %f", elevator_id, this->current_z);

        // this->current_z = std::round(this->current_z * 1000.0) / 1000.0;
        if (this->current_z <= target_z)
        {
            // // Hard-snap current_z to eliminate any accumulated floating point drift
            // this->SetElevatorHeight(_ecm, target_z);
            // Bring elevator to a stop
            this->SetElevatorVelocity(_ecm, 0.0);

            // this->current_z = target_z;
            this->current_floor = this->target_floor;
            this->current_state = OPENING_DOORS;
            this->state_start_time = sim_time;
            // RCLCPP_INFO(ros_node->get_logger(),
            //     "Elevator %d: arrived at floor %d at %f", elevator_id, current_floor, this->current_z);
        }
        else {
            // Determine velocity based on trapezoidal profile
            double dist_remaining = this->current_z - target_z;
            double dist_travelled = this->move_origin_z - this->current_z;
            double v = TrapezoidalVelocity(std::max(dist_travelled, 0.0), dist_remaining);
            // Apply velocity command to move the elevator down
            this->SetElevatorVelocity(_ecm, -v);

            // TODO: Remove ONLY for debugging
            // auto vel_cmd = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(this->elevator_joint);
            // double current_vel_cmd = vel_cmd ? vel_cmd->Data()[0] : 0.0;
            // RCLCPP_INFO(ros_node->get_logger(),
            //     "Elevator %d: moving down v_cmd=%.3f actual_v_cmd=%.3f current_z=%.3f target_z=%.3f",
            //     elevator_id, -v, current_vel_cmd, this->current_z, target_z);
        }
    }

    void HandleOpeningDoorsState(ignition::gazebo::EntityComponentManager & _ecm, double sim_time)
    {
        double progress = (sim_time - this->state_start_time) / this->door_operation_time;

        if (progress >= 1.0)
        {
            this->doors_open = true;
            SetDoorPosition(_ecm, this->door_open_position);
            // Hard-snap current_z to eliminate any accumulated floating point drift
            // this->current_z = this->floor_heights[this->current_floor];
            this->current_state = IDLE;
            this->state_start_time = sim_time;
            this->stop_duration = GetRandomStopDuration();
            RCLCPP_INFO(ros_node->get_logger(),
                "Elevator %d: door open at floor %d", elevator_id, current_floor);
    }
    else
        {
            SetDoorPosition(_ecm, progress * this->door_open_position);
        }
    }

    // ── ECM helpers ────────────────────────────────────────────────────────

    void SetDoorPosition(ignition::gazebo::EntityComponentManager & _ecm, double position)
    {
        this->current_door_position = position;
        auto * c = _ecm.Component<ignition::gazebo::components::JointPositionReset>(this->door_joint);
        if (c)
            c->Data() = {position};
        else
            _ecm.CreateComponent(this->door_joint,
                ignition::gazebo::components::JointPositionReset({position}));
    }

    void SetElevatorHeight(ignition::gazebo::EntityComponentManager & _ecm, double z)
    {
        auto *c = _ecm.Component<ignition::gazebo::components::JointPositionReset>(this->elevator_joint);
        if (c)
            c->Data() = {z};
        else
            _ecm.CreateComponent(this->elevator_joint,
                ignition::gazebo::components::JointPositionReset({z}));
    }

    void SetElevatorVelocity(ignition::gazebo::EntityComponentManager & _ecm, double v)
    {
        auto *cmd = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(this->elevator_joint);

        if (cmd)
        {
            cmd->Data() = {v};
        }
        else
        {
            _ecm.CreateComponent(
                this->elevator_joint,
                ignition::gazebo::components::JointVelocityCmd({v}));
        }
    }

    void SetElevatorPosition(ignition::gazebo::EntityComponentManager & _ecm, double z)
    {
        // Always use the locked initial XY + yaw so physics/collisions cannot
        // drift the elevator horizontally. Only Z is allowed to change.
        // z parameter is the desired platform surface height
        // initial_pose stores model origin, which is 0.05m below platform
        ignition::math::Pose3d cmd_pose = this->initial_pose;
        cmd_pose.Pos().Z() = z;  // Model origin is 0.05m below platform

        // Directly set the WorldPoseCmd component to command the model position
        auto * pose_c = _ecm.Component<ignition::gazebo::components::WorldPoseCmd>(this->model.Entity());
        if (pose_c)
        {
            *pose_c = ignition::gazebo::components::WorldPoseCmd(cmd_pose);
        }
        else
        {
            _ecm.CreateComponent(this->model.Entity(), ignition::gazebo::components::WorldPoseCmd(cmd_pose));
        }
    }

    void PublishStatus(ignition::gazebo::EntityComponentManager & _ecm)
    {
        std_msgs::msg::Int32 floor_msg;
        floor_msg.data = this->current_floor;
        this->current_floor_pub->publish(floor_msg);

        std_msgs::msg::Bool door_msg;
        door_msg.data = this->doors_open;
        this->door_state_pub->publish(door_msg);

        auto * pose_comp = _ecm.Component<ignition::gazebo::components::Pose>(this->model.Entity());
        if (pose_comp)
        {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = this->ros_node->now();
            pose_msg.header.frame_id = "world";
            const auto & pos = pose_comp->Data().Pos();
            const auto & rot = pose_comp->Data().Rot();
            pose_msg.pose.position.x = pos.X();
            pose_msg.pose.position.y = pos.Y();
            pose_msg.pose.position.z = pos.Z();
            pose_msg.pose.orientation.x = rot.X();
            pose_msg.pose.orientation.y = rot.Y();
            pose_msg.pose.orientation.z = rot.Z();
            pose_msg.pose.orientation.w = rot.W();
            this->car_position_pub->publish(pose_msg);
        }
    }

    void ScheduleNextMovement()
    {
        std::uniform_int_distribution<int> floor_dist(
            0, static_cast<int>(floor_heights.size()) - 1);
        do {
            this->target_floor = floor_dist(generator);
        } while (this->target_floor == this->current_floor);

        RCLCPP_INFO(ros_node->get_logger(),
            "Elevator %d: autonomous next target floor: %d", elevator_id, target_floor);
    }

    double GetRandomStopDuration()
    {
        return this->stop_duration_dist(generator);
    }

    // ── Members ────────────────────────────────────────────────────────────

    ignition::gazebo::Model model;
    ignition::gazebo::Entity elevator_joint{ignition::gazebo::kNullEntity};
    ignition::gazebo::Entity door_joint{ignition::gazebo::kNullEntity};
    ignition::gazebo::Entity platform_link{ignition::gazebo::kNullEntity};

    std::shared_ptr<rclcpp::Node> ros_node;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_floor_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr door_state_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr car_position_pub;
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr goto_service;
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr hold_service;
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr release_service;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    std::thread executor_thread;

    int elevator_id{1};
    std::vector<double> floor_heights{-0.1, 2.5, 5.0, 7.5};
    int current_floor{0};
    int target_floor{0};
    bool doors_open{true};

    double movement_speed{3.0};
    double movement_accel{1.5};  // m/s² — ramp up/down acceleration
    double door_operation_time{2.0};
    double stop_duration_min{10.0};
    double stop_duration_max{15.0};
    double stop_duration{10.0};
    double door_open_width{1.4};
    int door_open_direction{-1};
    double door_open_position{-1.4};

    ElevatorState current_state{IDLE};
    OperatingMode operating_mode{AUTONOMOUS};
    double state_start_time{0.0};
    double current_z{0.0};
    double move_origin_z{0.0};
    double current_door_position{0.0};
    uint64_t tick_counter{0};
    bool pose_initialized{false};
    ignition::math::Pose3d initial_pose;

    std::mt19937 generator;
    std::uniform_real_distribution<double> stop_duration_dist{10.0, 15.0};
};

}  // namespace avdr_gz_worlds

IGNITION_ADD_PLUGIN(
    avdr_gz_worlds::ElevatorController,
    ignition::gazebo::System,
    avdr_gz_worlds::ElevatorController::ISystemConfigure,
    avdr_gz_worlds::ElevatorController::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(avdr_gz_worlds::ElevatorController, "avdr_gz_worlds::ElevatorController")
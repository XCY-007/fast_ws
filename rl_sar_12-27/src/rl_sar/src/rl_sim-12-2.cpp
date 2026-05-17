/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sim.hpp"
#include <fstream>
#include <iomanip>

RL_Sim::RL_Sim()
#if defined(USE_ROS2)
    : rclcpp::Node("rl_sim_node")
#endif
{
#if defined(USE_ROS1)
    this->ang_vel_type = "ang_vel_world";
    ros::NodeHandle nh;
    nh.param<std::string>("ros_namespace", this->ros_namespace, "");
    nh.param<std::string>("robot_name", this->robot_name, "");
#elif defined(USE_ROS2)
    this->ang_vel_type = "ang_vel_body";
    this->ros_namespace = this->get_namespace();
    // get params from param_node
    param_client = this->create_client<rcl_interfaces::srv::GetParameters>("/param_node/get_parameters");
    while (!param_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()) {
            std::cout << LOGGER::ERROR << "Interrupted while waiting for param_node service. Exiting." << std::endl;
            return;
        }
        std::cout << LOGGER::WARNING << "Waiting for param_node service to be available..." << std::endl;
    }
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"robot_name", "gazebo_model_name"};
    // Use a timeout for the future
    auto future = param_client->async_send_request(request);
    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(5));
    if (status == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = future.get();
        if (result->values.size() < 2)
        {
            std::cout << LOGGER::ERROR << "Failed to get all parameters from param_node" << std::endl;
        }
        else
        {
            this->robot_name = result->values[0].string_value;
            this->gazebo_model_name = result->values[1].string_value;
            std::cout << LOGGER::INFO << "Get param robot_name: " << this->robot_name << std::endl;
            std::cout << LOGGER::INFO << "Get param gazebo_model_name: " << this->gazebo_model_name << std::endl;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "Failed to call param_node service" << std::endl;
    }
#endif

    // read params from yaml
    this->ReadYamlBase(this->robot_name);

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "No FSM registered for robot: " << this->robot_name << std::endl;
    }

    for (auto& fz : foot_force_z_) {
        fz.store(0.0);
    }

    // init torch
    torch::autograd::GradMode::set_enabled(false);
    torch::set_num_threads(4);

    // init robot
#if defined(USE_ROS1)
    this->joint_publishers_commands.resize(this->params.num_of_dofs);
#elif defined(USE_ROS2)
    this->robot_command_publisher_msg.motor_command.resize(this->params.num_of_dofs);
    this->robot_state_subscriber_msg.motor_state.resize(this->params.num_of_dofs);
#endif
    this->InitOutputs();
    this->InitControl();

#if defined(USE_ROS1)
    this->StartJointController(this->ros_namespace, this->params.joint_controller_names);
    // publisher
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        const std::string &joint_controller_name = this->params.joint_controller_names[i];
        const std::string topic_name = this->ros_namespace + joint_controller_name + "/command";
        this->joint_publishers[joint_controller_name] =
            nh.advertise<robot_msgs::MotorCommand>(topic_name, 10);
    }

    // subscriber
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Sim::CmdvelCallback, this);
    this->joy_subscriber = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &RL_Sim::JoyCallback, this);
    this->model_state_subscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, &RL_Sim::ModelStatesCallback, this);
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        const std::string &joint_controller_name = this->params.joint_controller_names[i];
        const std::string topic_name = this->ros_namespace + joint_controller_name + "/state";
        this->joint_subscribers[joint_controller_name] =
            nh.subscribe<robot_msgs::MotorState>(topic_name, 10,
                [this, joint_controller_name](const robot_msgs::MotorState::ConstPtr &msg)
                {
                    this->JointStatesCallback(msg, joint_controller_name);
                }
            );
        this->joint_positions[joint_controller_name] = 0.0;
        this->joint_velocities[joint_controller_name] = 0.0;
        this->joint_efforts[joint_controller_name] = 0.0;
    }

    // service
    nh.param<std::string>("gazebo_model_name", this->gazebo_model_name, "");
    this->gazebo_pause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    this->gazebo_unpause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    this->gazebo_reset_world_client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
#elif defined(USE_ROS2)
    this->StartJointController(this->ros_namespace, this->params.joint_names);
    // publisher
    this->robot_command_publisher = this->create_publisher<robot_msgs::msg::RobotCommand>(
        this->ros_namespace + "robot_joint_controller/command", rclcpp::SystemDefaultsQoS());

    // subscriber 订阅器全在这
    this->cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this] (const geometry_msgs::msg::Twist::SharedPtr msg) {this->CmdvelCallback(msg);}
    );
    this->joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", rclcpp::SystemDefaultsQoS(),
        [this] (const sensor_msgs::msg::Joy::SharedPtr msg) {this->JoyCallback(msg);}
    );
    this->gazebo_imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SystemDefaultsQoS(), [this] (const sensor_msgs::msg::Imu::SharedPtr msg) {this->GazeboImuCallback(msg);}
    );
    this->robot_state_subscriber = this->create_subscription<robot_msgs::msg::RobotState>(
        this->ros_namespace + "robot_joint_controller/state", rclcpp::SystemDefaultsQoS(),
        [this] (const robot_msgs::msg::RobotState::SharedPtr msg) {this->RobotStateCallback(msg);}
    );
    this->FLfoot_subscriber = this->create_subscription<gazebo_msgs::msg::ContactsState>(
        "/FL_foot_contact", rclcpp::SystemDefaultsQoS(), [this] (const gazebo_msgs::msg::ContactsState::SharedPtr msg) {this->GazeboFootCallback(msg, 0);}
    );
    this->FRfoot_subscriber = this->create_subscription<gazebo_msgs::msg::ContactsState>(
        "/FR_foot_contact", rclcpp::SystemDefaultsQoS(), [this] (const gazebo_msgs::msg::ContactsState::SharedPtr msg) {this->GazeboFootCallback(msg, 1);}
    );
    this->RLfoot_subscriber = this->create_subscription<gazebo_msgs::msg::ContactsState>(
        "/RL_foot_contact", rclcpp::SystemDefaultsQoS(), [this] (const gazebo_msgs::msg::ContactsState::SharedPtr msg) {this->GazeboFootCallback(msg, 2);}
    );
    this->RRfoot_subscriber = this->create_subscription<gazebo_msgs::msg::ContactsState>(
        "/RR_foot_contact", rclcpp::SystemDefaultsQoS(), [this] (const gazebo_msgs::msg::ContactsState::SharedPtr msg) {this->GazeboFootCallback(msg, 3);}
    );
    // 订阅深度图话题
    this->depth_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth/image_raw", rclcpp::SystemDefaultsQoS(), [this] (const sensor_msgs::msg::Image::SharedPtr msg) {this->DepthCallback(msg);}
    );

    // service
    this->gazebo_pause_physics_client = this->create_client<std_srvs::srv::Empty>("/pause_physics");
    this->gazebo_unpause_physics_client = this->create_client<std_srvs::srv::Empty>("/unpause_physics");
    this->gazebo_reset_world_client = this->create_client<std_srvs::srv::Empty>("/reset_world");

    auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result = this->gazebo_reset_world_client->async_send_request(empty_request);
#endif

    // loop
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.dt , std::bind(&RL_Sim::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.dt * this->params.decimation, std::bind(&RL_Sim::RunModel, this));
    this->loop_control->start();
    this->loop_rl->start();

    // keyboard
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Sim::KeyboardInterface, this));
    this->loop_keyboard->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.num_of_dofs);
    this->plot_target_joint_pos.resize(this->params.num_of_dofs);
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<double>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<double>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.001, std::bind(&RL_Sim::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

    std::cout << LOGGER::INFO << "RL_Sim start" << std::endl;
}

RL_Sim::~RL_Sim()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Sim exit" << std::endl;
}

void RL_Sim::StartJointController(const std::string& ros_namespace, const std::vector<std::string>& names)
{
#if defined(USE_ROS1)
    pid_t pid0 = fork();
    if (pid0 == 0)
    {
        std::string cmd = "rosrun controller_manager spawner joint_state_controller ";
        for (const auto& name : names)
        {
            cmd += name + " ";
        }
        cmd += "__ns:=" + ros_namespace;
        // cmd += " > /dev/null 2>&1";  // Comment this line to see the output
        execlp("sh", "sh", "-c", cmd.c_str(), nullptr);
        exit(1);
    }
#elif defined(USE_ROS2)
    const char* ros_distro = std::getenv("ROS_DISTRO");
    std::string spawner = (ros_distro && std::string(ros_distro) == "foxy") ? "spawner.py" : "spawner";

    std::filesystem::path tmp_path = std::filesystem::temp_directory_path() / "robot_joint_controller_params.yaml";
    {
        std::ofstream tmp_file(tmp_path);
        if (!tmp_file)
        {
            throw std::runtime_error("Failed to create temporary parameter file");
        }

        tmp_file << "/robot_joint_controller:\n";
        tmp_file << "    ros__parameters:\n";
        tmp_file << "        joints:\n";
        for (const auto& name : names)
        {
            tmp_file << "            - " << name << "\n";
        }
    }

    pid_t pid = fork();
    if (pid == 0)
    {
        std::string cmd = "ros2 run controller_manager " + spawner + " robot_joint_controller ";
        cmd += "-p " + tmp_path.string() + " ";
        // cmd += " > /dev/null 2>&1";  // Comment this line to see the output
        execlp("sh", "sh", "-c", cmd.c_str(), nullptr);
        exit(1);
    }
    else if (pid > 0)
    {
        int status;
        waitpid(pid, &status, 0);

        if (WIFEXITED(status) && WEXITSTATUS(status) != 0)
        {
            throw std::runtime_error("Failed to start joint controller");
        }

        std::filesystem::remove(tmp_path);
    }
    else
    {
        throw std::runtime_error("fork() failed");
    }
#endif
}

void RL_Sim::GetState(RobotState<double> *state)
{
#if defined(USE_ROS1)
    const auto &orientation = this->pose.orientation;
    const auto &angular_velocity = this->vel.angular;
#elif defined(USE_ROS2)
    const auto &orientation = this->gazebo_imu.orientation;
    const auto &angular_velocity = this->gazebo_imu.angular_velocity;
#endif

    state->imu.quaternion[0] = orientation.w;
    state->imu.quaternion[1] = orientation.x;
    state->imu.quaternion[2] = orientation.y;
    state->imu.quaternion[3] = orientation.z;

    state->imu.gyroscope[0] = angular_velocity.x;
    state->imu.gyroscope[1] = angular_velocity.y;
    state->imu.gyroscope[2] = angular_velocity.z;

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
#if defined(USE_ROS1)
        state->motor_state.q[i] = this->joint_positions[this->params.joint_controller_names[this->params.joint_mapping[i]]];
        state->motor_state.dq[i] = this->joint_velocities[this->params.joint_controller_names[this->params.joint_mapping[i]]];
        state->motor_state.tau_est[i] = this->joint_efforts[this->params.joint_controller_names[this->params.joint_mapping[i]]];
#elif defined(USE_ROS2)
        state->motor_state.q[i] = this->robot_state_subscriber_msg.motor_state[this->params.joint_mapping[i]].q;
        state->motor_state.dq[i] = this->robot_state_subscriber_msg.motor_state[this->params.joint_mapping[i]].dq;
        state->motor_state.tau_est[i] = this->robot_state_subscriber_msg.motor_state[this->params.joint_mapping[i]].tau_est;
#endif
    }
}

void RL_Sim::SetCommand(const RobotCommand<double> *command)
{
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
#if defined(USE_ROS1)
        this->joint_publishers_commands[this->params.joint_mapping[i]].q = command->motor_command.q[i];
        this->joint_publishers_commands[this->params.joint_mapping[i]].dq = command->motor_command.dq[i];
        this->joint_publishers_commands[this->params.joint_mapping[i]].kp = command->motor_command.kp[i];
        this->joint_publishers_commands[this->params.joint_mapping[i]].kd = command->motor_command.kd[i];
        this->joint_publishers_commands[this->params.joint_mapping[i]].tau = command->motor_command.tau[i];
#elif defined(USE_ROS2)
        this->robot_command_publisher_msg.motor_command[this->params.joint_mapping[i]].q = command->motor_command.q[i];
        this->robot_command_publisher_msg.motor_command[this->params.joint_mapping[i]].dq = command->motor_command.dq[i];
        this->robot_command_publisher_msg.motor_command[this->params.joint_mapping[i]].kp = command->motor_command.kp[i];
        this->robot_command_publisher_msg.motor_command[this->params.joint_mapping[i]].kd = command->motor_command.kd[i];
        this->robot_command_publisher_msg.motor_command[this->params.joint_mapping[i]].tau = command->motor_command.tau[i];
#endif
    }

#if defined(USE_ROS1)
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->joint_publishers[this->params.joint_controller_names[i]].publish(this->joint_publishers_commands[i]);
    }
#elif defined(USE_ROS2)
    this->robot_command_publisher->publish(this->robot_command_publisher_msg);
#endif
}

void RL_Sim::RobotControl()
{
    if (this->control.current_keyboard == Input::Keyboard::R || this->control.current_gamepad == Input::Gamepad::RB_Y)
    {
#if defined(USE_ROS1)
        std_srvs::Empty empty;
        this->gazebo_reset_world_client.call(empty);
#elif defined(USE_ROS2)
        auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = this->gazebo_reset_world_client->async_send_request(empty_request);
#endif
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Enter || this->control.current_gamepad == Input::Gamepad::RB_X)
    {
        if (simulation_running)
        {
#if defined(USE_ROS1)
            std_srvs::Empty empty;
            this->gazebo_pause_physics_client.call(empty);
#elif defined(USE_ROS2)
            auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = this->gazebo_pause_physics_client->async_send_request(empty_request);
#endif
            std::cout << std::endl << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
#if defined(USE_ROS1)
            std_srvs::Empty empty;
            this->gazebo_unpause_physics_client.call(empty);
#elif defined(USE_ROS2)
            auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = this->gazebo_unpause_physics_client->async_send_request(empty_request);
#endif
            std::cout << std::endl << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
        this->control.current_keyboard = this->control.last_keyboard;
    }

    if (simulation_running)
    {
        this->motiontime++;

        if (this->control.current_keyboard == Input::Keyboard::W)
        {
            this->control.x += 0.1;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::S)
        {
            this->control.x -= 0.1;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::A)
        {
            this->control.y += 0.1;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::D)
        {
            this->control.y -= 0.1;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::Q)
        {
            this->control.yaw += 0.1;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::E)
        {
            this->control.yaw -= 0.1;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::Space)
        {
            this->control.x = 0;
            this->control.y = 0;
            this->control.yaw = 0;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::N || this->control.current_gamepad == Input::Gamepad::X)
        {
            this->control.navigation_mode = !this->control.navigation_mode;
            std::cout << std::endl << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
            this->control.current_keyboard = this->control.last_keyboard;
            this->control.current_gamepad = this->control.last_gamepad;
        }

        this->GetState(&this->robot_state);
        this->StateController(&this->robot_state, &this->robot_command);
        this->SetCommand(&this->robot_command);
    }
}

#if defined(USE_ROS1)
void RL_Sim::ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    this->vel = msg->twist[2];
    this->pose = msg->pose[2];
}
#elif defined(USE_ROS2)
void RL_Sim::GazeboImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    this->gazebo_imu = *msg;
}
#endif

// void RL_Sim::GazeboFootCallback(const gazebo_msgs::msg::ContactsState::SharedPtr msg, int foot_number)
// {
//     switch (foot_number)
//     {
//         case 0:
//             this->FLfoot_force = *msg;
//             break;
//         case 1:
//             this->FRfoot_force = *msg;
//             break;
//         case 2:
//             this->RLfoot_force = *msg;
//             break;
//         case 3:
//             this->RRfoot_force = *msg;
//             break;
//         default:
//             break;
//     }
// }

void RL_Sim::GazeboFootCallback(const gazebo_msgs::msg::ContactsState::SharedPtr msg, int foot_number)
{
    double fz = 0.0;

    if (msg && !msg->states.empty()) {
        // 取第一个接触点的垂直力
        fz = msg->states[0].total_wrench.force.z;
    }

    // 存入原子数组
    foot_force_z_[foot_number].store(fz, std::memory_order_relaxed);
}

void RL_Sim::DepthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 假设 msg->encoding 是 "32FC1" 或 "16UC1"
    // 如果是 16UC1，先转 float
    int H = msg->height;
    int W = msg->width;

    torch::Tensor depth_tensor;

    if (msg->encoding == "16UC1") {
        auto data_ptr = reinterpret_cast<const uint16_t*>(msg->data.data());
        depth_tensor = torch::from_blob((void*)data_ptr, {H, W}, torch::kInt16).to(torch::kFloat32);
        depth_tensor = depth_tensor / 1000.0f; // mm -> m
    } else if (msg->encoding == "32FC1") {
        auto data_ptr = reinterpret_cast<const float*>(msg->data.data());
        depth_tensor = torch::from_blob((void*)data_ptr, {H, W}, torch::kFloat32).clone();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported depth encoding: %s", msg->encoding.c_str());
        return;
    }

    // 1. Crop 下2行，左右各4列
    depth_tensor = depth_tensor.index({
        torch::indexing::Slice(0, H - 2),
        torch::indexing::Slice(4, W - 4)
    });

    // 2. Resize 到 (58,87) using interpolate
    depth_tensor = depth_tensor.unsqueeze(0).unsqueeze(0); // [1,1,H,W]
    depth_tensor = torch::nn::functional::interpolate(
        depth_tensor,
        torch::nn::functional::InterpolateFuncOptions()
            .size(std::vector<int64_t>({58, 87}))
            .mode(torch::kBicubic)
            .align_corners(false)
    );
    depth_tensor = depth_tensor.squeeze(); // [58,87]
    //std::cout << depth_tensor << std::endl;

    // 3. Normalize [-0.5,0.5]
    depth_tensor = depth_tensor / 2.0f - 0.5f;

    // 4. 保存到类成员
    this->depth = depth_tensor;

    //RCLCPP_INFO(this->get_logger(), "Processed depth tensor size: [%ld, %ld]",this->depth.size(0), this->depth.size(1));
    //std::cout << this->depth << std::endl;
}



void RL_Sim::CmdvelCallback(
#if defined(USE_ROS1)
    const geometry_msgs::Twist::ConstPtr &msg
#elif defined(USE_ROS2)
    const geometry_msgs::msg::Twist::SharedPtr msg
#endif
)
{
    this->cmd_vel = *msg;
}

void RL_Sim::JoyCallback(
#if defined(USE_ROS1)
    const sensor_msgs::Joy::ConstPtr &msg
#elif defined(USE_ROS2)
    const sensor_msgs::msg::Joy::SharedPtr msg
#endif
)
{
    this->joy_msg = *msg;

    // joystick control
    // Description of buttons and axes(F710):
    // |__ buttons[]: A=0, B=1, X=2, Y=3, LB=4, RB=5, back=6, start=7, power=8, stickL=9, stickR=10
    // |__ axes[]: Lx=0, Ly=1, Rx=3, Ry=4, LT=2, RT=5, DPadX=6, DPadY=7

    if (this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::A);
    if (this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::B);
    if (this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::X);
    if (this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->joy_msg.buttons[4]) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->joy_msg.buttons[5]) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->joy_msg.buttons[9]) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->joy_msg.buttons[10]) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->joy_msg.axes[7] > 0) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->joy_msg.axes[7] < 0) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->joy_msg.axes[6] < 0) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->joy_msg.axes[6] > 0) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[9]) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[10]) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[7] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[7] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[6] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[6] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[0]) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[1]) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[2]) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[3]) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[9]) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[10]) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[7] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[7] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[6] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[6] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[5]) this->control.SetGamepad(Input::Gamepad::LB_RB);

    this->control.x = this->joy_msg.axes[1] * 1.5; // LY
    this->control.y = this->joy_msg.axes[0] * 1.5; // LX
    this->control.yaw = this->joy_msg.axes[3] * 1.5; // RX
}

#if defined(USE_ROS1)
void RL_Sim::JointStatesCallback(const robot_msgs::MotorState::ConstPtr &msg, const std::string &joint_controller_name)
{
    this->joint_positions[joint_controller_name] = msg->q;
    this->joint_velocities[joint_controller_name] = msg->dq;
    this->joint_efforts[joint_controller_name] = msg->tau_est;
}
#elif defined(USE_ROS2)
void RL_Sim::RobotStateCallback(const robot_msgs::msg::RobotState::SharedPtr msg)
{
    this->robot_state_subscriber_msg = *msg;
}
#endif

void RL_Sim::RunModel() // 在这里获取OBS原始数据
{
    if (this->rl_init_done && simulation_running)
    {
        this->episode_length_buf += 1;
        // this->obs.lin_vel = torch::tensor({{this->vel.linear.x, this->vel.linear.y, this->vel.linear.z}});
        this->obs.ang_vel = torch::tensor(this->robot_state.imu.gyroscope).unsqueeze(0);
        if (this->control.navigation_mode)
        {
            this->obs.commands = torch::tensor({{this->cmd_vel.linear.x, this->cmd_vel.linear.y, this->cmd_vel.angular.z}});
        }
        else
        {
            this->obs.commands = torch::tensor({{this->control.x, this->control.y, this->control.yaw}});
        }
        this->obs.base_quat = torch::tensor(this->robot_state.imu.quaternion).unsqueeze(0);
        //std::cout << "base_quat: " << this->obs.base_quat << std::endl;
        this->obs.dof_pos = torch::tensor(this->robot_state.motor_state.q).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);
        this->obs.dof_vel = torch::tensor(this->robot_state.motor_state.dq).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);
        
        // 四个脚依次检测
        // std::vector<gazebo_msgs::msg::ContactsState> foot_forces = {
        //     this->FLfoot_force, 
        //     this->FRfoot_force,
        //     this->RLfoot_force, 
        //     this->RRfoot_force
        // };
        // for (int i = 0; i < 4; i++)
        // {
        //     if (!foot_forces[i].states.empty())  // 有接触数据
        //     {
        //         double fz = foot_forces[i].states[0].total_wrench.force.z;
        //         if (fz > 2.0)
        //         {
        //             this->obs.foot_contact[0][i] = 1.0;
        //         }
        //     }
        //     else
        //     {
        //         this->obs.foot_contact[0][i] = 0.0;
        //     }
        // }
        for (int i = 0; i < 4; i++)
        {
            double fz = foot_force_z_[i].load(std::memory_order_relaxed);

            if (fz > 2.0)
                this->obs.foot_contact[0][i] = 1.0;
            else
                this->obs.foot_contact[0][i] = 0.0;
        }
        this->obs.foot_contact = this->obs.foot_contact - 0.5;
        //std::cout << "foot_contact: " << this->obs.foot_contact << std::endl;
        // 接触获取完毕


        this->obs.actions = this->Forward(); // 在这里组装OBS并前向推理
        //std::cout << "actions: " << this->obs.actions << std::endl;
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (this->output_dof_pos.defined() && this->output_dof_pos.numel() > 0)
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (this->output_dof_vel.defined() && this->output_dof_vel.numel() > 0)
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (this->output_dof_tau.defined() && this->output_dof_tau.numel() > 0)
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        // this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
        torch::Tensor tau_est = torch::zeros({1, this->params.num_of_dofs});
        for (int i = 0; i < this->params.num_of_dofs; ++i)
        {
            tau_est[0][i] = this->joint_efforts[this->params.joint_controller_names[i]];
        }
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

// torch::Tensor RL_Sim::Forward()
// {
//     torch::autograd::GradMode::set_enabled(false);

//     torch::Tensor clamped_obs = this->ComputeObservation();

//     torch::Tensor actions;
//     if (this->params.observations_history.size() != 0)
//     {
//         this->history_obs_buf.insert(clamped_obs);
//         this->history_obs = this->history_obs_buf.get_obs_vec(this->params.observations_history);
//         actions = this->model.forward({this->history_obs}).toTensor();
//     }
//     else
//     {
//         actions = this->model.forward({clamped_obs}).toTensor();
//     }

//     if (this->params.clip_actions_upper.numel() != 0 && this->params.clip_actions_lower.numel() != 0)
//     {
//         return torch::clamp(actions, this->params.clip_actions_lower, this->params.clip_actions_upper);
//     }
//     else
//     {
//         return actions;
//     }
// }
torch::Tensor RL_Sim::Forward()
{
    torch::autograd::GradMode::set_enabled(false);

    // ===== 1. 获取当前观测 =====
    torch::Tensor clamped_obs = this->ComputeObservation();

    torch::Tensor actions;

    if (this->params.observations_history.size() != 0)
    {
        // ===== 2. 插入到历史缓存 =====
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.observations_history);
        // history_obs: (batch, 252) = 42 * 6

        int obs_dim = 45 + 42;

        // ===== 3. 拆分当前 obs 和历史 obs =====
        torch::Tensor current_obs = this->history_obs.index({
            torch::indexing::Slice(),
            torch::indexing::Slice(-obs_dim, torch::indexing::None)
        });
        //saveObsToCSV(current_obs, "obs_log.csv");
        torch::Tensor history_obs = this->history_obs.index({
            torch::indexing::Slice(),
            torch::indexing::Slice(0, -obs_dim)
        });

        // obs_p: [B, 48]
        torch::Tensor obs_p = current_obs.slice(1, 0, 45);
        //std::cout << "obs_p: " << obs_p << std::endl;

        // obs_r: [B, 42]
        torch::Tensor obs_r = current_obs.slice(1, 45, 87);

        int n_history = 870 / 87;

        // 变形 [B, T, 87]
        torch::Tensor history_obs_flatten = history_obs.view({-1, n_history, 87});

        // [B, 48*T]
        torch::Tensor obs_p_history = history_obs_flatten.slice(2, 0, 45)
                                                .reshape({history_obs.size(0), -1});

        // [B, 42*5]
        torch::Tensor obs_r_history = history_obs_flatten.slice(1, n_history - 5, n_history)  // 取最后5帧
                                                .slice(2, 45, 87)                   // 后42维
                                                .reshape({history_obs.size(0), -1});
        
        obs_p.index({torch::indexing::Slice(), 6}) = this->obs.commands.index({torch::indexing::Slice(), 0});
        obs_p.index({torch::indexing::Slice(), 7}) = this->obs.commands.index({torch::indexing::Slice(), 1});
        obs_p.index({torch::indexing::Slice(), 8}) = this->obs.commands.index({torch::indexing::Slice(), 2});

        // 拼接
        torch::Tensor obs_r_all = torch::cat({obs_r, obs_r_history}, /*dim=*/-1); 

        // 模式切换
        torch::Tensor x = obs_r.index({torch::indexing::Slice(), 5});
        torch::Tensor parkour_val = obs_p.index({torch::indexing::Slice(), 6});

        // 条件切换 1 recovery ；0 parkour
        bool switch_to_recovery = (this->mode == 0 && torch::any(x > 0.5).item<bool>()) ||
                                torch::any(parkour_val < 0.05).item<bool>();

        bool switch_to_parkour = (this->mode == 1 && torch::any(x < -0.7).item<bool>()) &&
                                torch::any(parkour_val > 0.05).item<bool>();

        // 更新 mode
        if (switch_to_recovery) {
            this->mode = 1;
        } else if (switch_to_parkour) {
            this->mode = 0;
        }
        //std::cout << "Current mode: " << (this->mode == 1 ? "recovery" : "parkour") << std::endl;

        // mode recovery
        // ===== 调用 run_vae 得到 latent =====
        if (this->mode == 1) {
            auto vae_output_ivalue = this->extra_model.run_method("run_vae", obs_r_history);
            auto vae_output = vae_output_ivalue.toGenericDict();

            torch::Tensor m_hat = vae_output.at("m_hat").toTensor();
            torch::Tensor c_hat = vae_output.at("c_hat").toTensor();
            torch::Tensor z     = vae_output.at("z").toTensor();

            torch::Tensor mc_latent = torch::cat({m_hat, c_hat, z}, -1);

            // ===== 拼接 latent + 当前 obs =====
            torch::Tensor obs_input;
            obs_input = torch::cat({mc_latent, obs_r}, -1);
            
            // ===== 输入策略网络 =====
            actions = this->extra_model.forward({obs_input}).toTensor();
        }
        else if (this->mode == 0)
        {   
            this->t ++;
            // mode parkour
            // ---------------相机
            // 将单帧 this->depth ([58,87]) 转为 depth_camera [1,1,58,87]
            torch::Tensor depth_camera = this->depth.unsqueeze(0).unsqueeze(0);

            // 初始化双帧缓存 camera_input: [1,2,58,87]
            if (!this->camera_input.defined() || this->camera_input.numel() == 0)
            {
                // depth_camera: [1,1,58,87], 复制成两通道 [1,2,58,87]
                this->camera_input = torch::cat({depth_camera, depth_camera}, /*dim=*/1);
            }

            std::vector<torch::jit::IValue> inputs;
            
            torch::Tensor depth_latent_and_yaw;
            if(this->t == 5)
            {
                torch::Tensor newest = depth_camera.index({torch::indexing::Slice(), 0}).clone(); // [1,58,87]
                // expand newest to [1,1,58,87]
                newest = newest.unsqueeze(1);
                torch::Tensor second = this->camera_input.index({torch::indexing::Slice(), 1}).clone(); // [1,58,87]
                second = second.unsqueeze(1);
                this->camera_input = torch::cat({second, newest}, /*dim=*/1); // [1,2,58,87]  // 老的前新的后，是push进去的

                // 传入模型的 camera_input 为类成员
                torch::Tensor camera_input = this->camera_input;

                {
                    std::vector<torch::jit::IValue> inputs = {obs_p_history, camera_input};
                    depth_latent_and_yaw = this->camera_model.forward(inputs).toTensor();
                }
                this->last_depth_latent_and_yaw = depth_latent_and_yaw;
                this->t = 0;
                inputs.clear();
                
            }
            else
            {
                depth_latent_and_yaw = this->last_depth_latent_and_yaw;
            }

            torch::Tensor depth_latent = depth_latent_and_yaw;

            torch::Tensor obs_parkour = obs_p;

            {
                std::vector<torch::jit::IValue> inputs = {depth_latent, obs_parkour};
                actions = this->model.forward(inputs).toTensor();
            }
            //std::cout << "actions: " << actions << std::endl;
            inputs.clear();
        }
    }
    else
    {
        // 没有历史，直接用当前 obs
        actions = this->model.forward({clamped_obs}).toTensor();
    }

    // ===== 7. 动作裁剪 =====
    if (this->params.clip_actions_upper.numel() != 0 && this->params.clip_actions_lower.numel() != 0)
    {
        //std::cout << "Clamping actions" << std::endl;
        //TORCH_CHECK(actions.defined(), "actions is undefined before clamp");
        return torch::clamp(actions, this->params.clip_actions_lower, this->params.clip_actions_upper);
    }
    else
    {
        return actions;
    }
}



void RL_Sim::saveObsToCSV(const torch::Tensor& obs, const std::string& filename) {
    // detach + to CPU + to float32
    auto obs_cpu = obs.detach().to(torch::kCPU).to(torch::kF32);

    // open file in append mode
    std::ofstream file(filename, std::ios::app);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open " + filename);
    }

    auto accessor = obs_cpu.accessor<float, 2>();  // (num_envs, obs_dim)

    for (int i = 0; i < obs_cpu.size(0); i++) {
        for (int j = 0; j < obs_cpu.size(1); j++) {
            file << std::setprecision(6) << accessor[i][j];
            if (j < obs_cpu.size(1) - 1)
                file << ",";
        }
        file << "\n";
    }
    file.close();
}

void RL_Sim::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
#if defined(USE_ROS1)
        this->plot_real_joint_pos[i].push_back(this->joint_positions[this->params.joint_controller_names[i]]);
        this->plot_target_joint_pos[i].push_back(this->joint_publishers_commands[i].q);
#elif defined(USE_ROS2)
        this->plot_real_joint_pos[i].push_back(this->robot_state_subscriber_msg.motor_state[i].q);
        this->plot_target_joint_pos[i].push_back(this->robot_command_publisher_msg.motor_command[i].q);
#endif
        plt::subplot(this->params.num_of_dofs, 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.01);
}

#if defined(USE_ROS1)
void signalHandler(int signum)
{
    ros::shutdown();
    exit(0);
}
#endif

int main(int argc, char **argv)
{
#if defined(USE_ROS1)
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_sar");
    RL_Sim rl_sar;
    ros::spin();
#elif defined(USE_ROS2)
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RL_Sim>());
    rclcpp::shutdown();
#endif
    return 0;
}


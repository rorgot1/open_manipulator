// Copyright 2024 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Hye-jong KIM, Sungho Woo

#include <algorithm>
#include <memory>
#include <cmath>

#include "open_manipulator_x_teleop/open_manipulator_x_teleop.hpp"

// KeyboardReader
KeyboardReader::KeyboardReader()
: kfd(0)
{
  tcgetattr(kfd, &cooked);
  struct termios raw;
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
}

void KeyboardReader::readOne(char * c)
{
  int rc = read(kfd, c, 1);
  if (rc < 0) {
    throw std::runtime_error("read failed");
  }
}

void KeyboardReader::shutdown()
{
  tcsetattr(kfd, TCSANOW, &cooked);
}

// KeyboardServo
KeyboardServo::KeyboardServo()
: publish_joint_(false), publish_gripper_(false), object_attached_(false)
{
  nh_ = rclcpp::Node::make_shared("servo_keyboard_input");

  servo_start_client_ = nh_->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
  servo_stop_client_ = nh_->create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");

  joint_pub_ = nh_->create_publisher<control_msgs::msg::JointJog>(ARM_JOINT_TOPIC, ROS_QUEUE_SIZE);
  client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(nh_, "gripper_controller/gripper_cmd");
  attach_client_ = nh_->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
  detach_client_ = nh_->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");

  executor_.add_node(nh_); // เพิ่ม node เข้า executor ครั้งเดียว
}

KeyboardServo::~KeyboardServo()
{
  stop_moveit_servo();
}

int KeyboardServo::keyLoop()
{
  char c;

  connect_moveit_servo();
  start_moveit_servo();

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Joint Control Keys:");
  puts("  1/q: Joint1 +/-");
  puts("  2/w: Joint2 +/-");
  puts("  3/e: Joint3 +/-");
  puts("  4/r: Joint4 +/-");
  puts("Use o|p to gradually open/close the gripper step-by-step, '9' to grasp 'my_box', '0' to release.");
  puts("'ESC' to quit.");

  std::thread{std::bind(&KeyboardServo::pub, this)}.detach();

  bool servoing = true;
  float current_gripper_pos = -0.01;
  const float GRIPPER_STEP = 0.0029;

  while (servoing && rclcpp::ok()) {
    try {
      input.readOne(&c);
    } catch (const std::runtime_error &) {
      perror("read():");
      return -1;
     }

    RCLCPP_INFO(nh_->get_logger(), "value: 0x%02X", c);

    joint_msg_.joint_names.clear();
    joint_msg_.velocities.clear();

    switch (c) {
      case KEYCODE_1:
        joint_msg_.joint_names.push_back("joint1");
        joint_msg_.velocities.push_back(ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint1 +");
        break;
      case KEYCODE_2:
        joint_msg_.joint_names.push_back("joint2");
        joint_msg_.velocities.push_back(ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint2 +");
        break;
      case KEYCODE_3:
        joint_msg_.joint_names.push_back("joint3");
        joint_msg_.velocities.push_back(ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint3 +");
        break;
      case KEYCODE_4:
        joint_msg_.joint_names.push_back("joint4");
        joint_msg_.velocities.push_back(ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint4 +");
        break;
      case KEYCODE_Q:
        joint_msg_.joint_names.push_back("joint1");
        joint_msg_.velocities.push_back(-ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint1 -");
        break;
      case KEYCODE_W:
        joint_msg_.joint_names.push_back("joint2");
        joint_msg_.velocities.push_back(-ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint2 -");
        break;
      case KEYCODE_E:
        joint_msg_.joint_names.push_back("joint3");
        joint_msg_.velocities.push_back(-ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint3 -");
        break;
      case KEYCODE_R:
        joint_msg_.joint_names.push_back("joint4");
        joint_msg_.velocities.push_back(-ARM_JOINT_VEL);
        publish_joint_ = true;
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint4 -");
        break;
      case KEYCODE_O:
        if (current_gripper_pos < 0.019) {
          current_gripper_pos += GRIPPER_STEP;
          if (current_gripper_pos > 0.019) current_gripper_pos = 0.019;
          send_gradual_goal(current_gripper_pos, current_gripper_pos, 1, 0);
          RCLCPP_INFO_STREAM(nh_->get_logger(), "Gripper Opening Step: " << current_gripper_pos);
        } else {
          RCLCPP_INFO_STREAM(nh_->get_logger(), "Gripper Fully Open");
        }
        break;
      case KEYCODE_P:
        if (current_gripper_pos > -0.01) {
          current_gripper_pos -= GRIPPER_STEP;
          if (current_gripper_pos < -0.01) current_gripper_pos = -0.01;
          send_gradual_goal(current_gripper_pos, current_gripper_pos, 1, 0);
          RCLCPP_INFO_STREAM(nh_->get_logger(), "Gripper Closing Step: " << current_gripper_pos);
        } else {
          RCLCPP_INFO_STREAM(nh_->get_logger(), "Gripper Fully Closed");
        }
        break;
      case KEYCODE_9:
        {
          auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
          request->model1_name = "open_manipulator_x_system";
          request->link1_name = "gripper_right_link";
          request->model2_name = "my_box";
          request->link2_name = "box_link";

          if (attach_client_->wait_for_service(std::chrono::seconds(1))) {
            auto future = attach_client_->async_send_request(request);
            executor_.spin_until_future_complete(future); // ใช้ executor แทน
            if (future.get()->success) { // ตรวจสอบผลลัพธ์จาก service
              RCLCPP_INFO(nh_->get_logger(), "Grasped my_box successfully");
              object_attached_ = true;
            } else {
              RCLCPP_ERROR(nh_->get_logger(), "Failed to grasp my_box: %s", future.get()->message.c_str());
            }
          } else {
            RCLCPP_ERROR(nh_->get_logger(), "Attach service not available");
          }
        }
        break;
      case KEYCODE_0:
        if (object_attached_) {
          auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
          request->model1_name = "open_manipulator_x_system";
          request->link1_name = "gripper_right_link";
          request->model2_name = "my_box";
          request->link2_name = "box_link";

          if (detach_client_->wait_for_service(std::chrono::seconds(1))) {
            auto future = detach_client_->async_send_request(request);
            executor_.spin_until_future_complete(future); // ใช้ executor แทน
            if (future.get()->success) {
              RCLCPP_INFO(nh_->get_logger(), "Released my_box successfully");
              object_attached_ = false;
            } else {
              RCLCPP_ERROR(nh_->get_logger(), "Failed to release my_box: %s", future.get()->message.c_str());
            }
          } else {
            RCLCPP_ERROR(nh_->get_logger(), "Detach service not available");
          }
        } else {
          RCLCPP_WARN(nh_->get_logger(), "No object attached to release");
        }
        break;
      case KEYCODE_ESC:
        RCLCPP_INFO_STREAM(nh_->get_logger(), "quit");
        servoing = false;
        break;
      default:
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Unassigned input : " << c);
        break;
    }

    executor_.spin_some(); // spin เพื่อประมวลผล callback อื่นๆ
  }

  return 0;
}

void KeyboardServo::send_goal(float position)
{
  auto goal_msg = control_msgs::action::GripperCommand::Goal();
  goal_msg.command.position = position;
  goal_msg.command.max_effort = 100.0;

  auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&KeyboardServo::goal_result_callback, this, std::placeholders::_1);

  RCLCPP_INFO(nh_->get_logger(), "Sending goal");
  client_->async_send_goal(goal_msg, send_goal_options);
}

void KeyboardServo::send_gradual_goal(float start_pos, float end_pos, int steps, int delay_ms)
{
  for (int i = 0; i <= steps; i++) {
    float position = start_pos + (end_pos - start_pos) * i / steps;
    auto goal_msg = control_msgs::action::GripperCommand::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = 100.0;

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&KeyboardServo::goal_result_callback, this, std::placeholders::_1);

    RCLCPP_INFO(nh_->get_logger(), "Sending gradual goal: position = %f", position);
    client_->async_send_goal(goal_msg, send_goal_options);

    rclcpp::sleep_for(std::chrono::milliseconds(delay_ms));
  }
  publish_gripper_ = true;
}

void KeyboardServo::connect_moveit_servo()
{
  for (int i = 0; i < 10; i++) {
    if (servo_start_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO_STREAM(nh_->get_logger(), "SUCCESS TO CONNECT SERVO START SERVER");
      break;
    }
    RCLCPP_WARN_STREAM(nh_->get_logger(), "WAIT TO CONNECT SERVO START SERVER");
    if (i == 9) {
      RCLCPP_ERROR_STREAM(
        nh_->get_logger(),
        "fail to connect moveit_servo." <<
          "please launch 'servo.launch' at 'open_manipulator_x_moveit_configs' pkg.");
    }
  }
  for (int i = 0; i < 10; i++) {
    if (servo_stop_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO_STREAM(nh_->get_logger(), "SUCCESS TO CONNECT SERVO STOP SERVER");
      break;
    }
    RCLCPP_WARN_STREAM(nh_->get_logger(), "WAIT TO CONNECT SERVO STOP SERVER");
    if (i == 9) {
      RCLCPP_ERROR_STREAM(
        nh_->get_logger(),
        "fail to connect moveit_servo." <<
          "please launch 'servo.launch' at 'open_manipulator_x_moveit_configs' pkg.");
    }
  }
}

void KeyboardServo::start_moveit_servo()
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "call 'moveit_servo' start srv.");
  auto future = servo_start_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  executor_.spin_until_future_complete(future);
  if (future.get()->success) {
    RCLCPP_INFO_STREAM(nh_->get_logger(), "SUCCESS to start 'moveit_servo'");
  } else {
    RCLCPP_ERROR_STREAM(nh_->get_logger(), "FAIL to start 'moveit_servo', execute without 'moveit_servo'");
  }
}

void KeyboardServo::stop_moveit_servo()
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "call 'moveit_servo' END srv.");
  auto future = servo_stop_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  executor_.spin_until_future_complete(future);
  if (future.get()->success) {
    RCLCPP_INFO_STREAM(nh_->get_logger(), "SUCCESS to stop 'moveit_servo'");
  }
}

void KeyboardServo::pub()
{
  while (rclcpp::ok()) {
    if (publish_joint_) {
      joint_msg_.header.stamp = nh_->now();
      joint_msg_.header.frame_id = BASE_FRAME_ID;
      joint_pub_->publish(joint_msg_);
      publish_joint_ = false;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "Joint PUB");
    }
    if (publish_gripper_) {
      publish_gripper_ = false;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
}

void KeyboardServo::goal_result_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(nh_->get_logger(), "Gripper action succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(nh_->get_logger(), "Gripper action aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(nh_->get_logger(), "Gripper action canceled");
      break;
    default:
      RCLCPP_ERROR(nh_->get_logger(), "Unknown result code");
      break;
  }
}
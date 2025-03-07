#ifndef OPEN_MANIPULATOR_X_TELEOP__OPEN_MANIPULATOR_X_TELEOP_HPP_
#define OPEN_MANIPULATOR_X_TELEOP__OPEN_MANIPULATOR_X_TELEOP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <string>
#include <memory>

#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72
#define KEYCODE_O 0x6F
#define KEYCODE_P 0x70
#define KEYCODE_ESC 0x1B
#define KEYCODE_9 0x39
#define KEYCODE_0 0x30

const char ARM_JOINT_TOPIC[] = "/servo_node/delta_joint_cmds";
const size_t ROS_QUEUE_SIZE = 10;
const char BASE_FRAME_ID[] = "link1";
const double ARM_JOINT_VEL = 3.0;

class KeyboardReader
{
public:
  KeyboardReader();
  void readOne(char * c);
  void shutdown();
private:
  int kfd;
  struct termios cooked;
};

class KeyboardServo
{
public:
  KeyboardServo();
  ~KeyboardServo();
  int keyLoop();
  void connect_moveit_servo();
  void start_moveit_servo();
  void stop_moveit_servo();
  void send_goal(float position);
private:
  void pub();
  void send_gradual_goal(float start_pos, float end_pos, int steps, int delay_ms);
  void goal_result_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>::WrappedResult & result);

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_stop_client_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr client_;
  rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client_;
  rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client_;
  rclcpp::executors::SingleThreadedExecutor executor_; // เพิ่ม executor

  control_msgs::msg::JointJog joint_msg_;
  bool publish_joint_;
  bool publish_gripper_;
  bool object_attached_;
};

KeyboardReader input;

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  KeyboardServo keyboard_servo;
  signal(SIGINT, quit);
  int rc = keyboard_servo.keyLoop();
  input.shutdown();
  rclcpp::shutdown();
  return rc;
}

#endif  // OPEN_MANIPULATOR_X_TELEOP__OPEN_MANIPULATOR_X_TELEOP_HPP_
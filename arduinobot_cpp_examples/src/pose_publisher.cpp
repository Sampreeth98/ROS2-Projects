// #include "rclcpp/rclcpp.hpp"
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <memory>
// #include <geometry_msgs/msg/pose.hpp>

// #include <chrono>
// using namespace std::chrono_literals;
// //using rclcpp::executors::MultiThreadedExecutor;

// class PosePublisher : public rclcpp::Node // MODIFY NAME
// {
// public:
//     PosePublisher() : Node("pose_publisher") // MODIFY NAME
//     {   
//         std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pose_publisher");

//         // geometry_msgs::msg::Pose current_pose = arm_move_group.getCurrentPose().pose;
//         ppublish = create_publisher<geometry_msgs::msg::Pose>("current_pose",10);
//         timer_ = create_wall_timer(0.0333s, [this, node](){ timerCallback(node); });
//         RCLCPP_INFO(get_logger(), "Publishing current pose");
//     }
    
//     void timerCallback(const std::shared_ptr<rclcpp::Node> node)
//     {   
//         auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm");
//         arm_move_group ->setEndEffectorLink("rgb_camera");
//         geometry_msgs::msg::Pose current_pose = arm_move_group.getCurrentPose().pose;
//         ppublish ->publish(current_pose);
//     }
// private:
//     rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ppublish;
//     rclcpp::TimerBase::SharedPtr timer_;
// };
 
// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<PosePublisher>(); // MODIFY NAME
//     // Create a MultiThreadedExecutor with 2 threads
//     auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
//         rclcpp::ExecutorOptions(), 1);

//     // Add the node to the executor
//     executor->add_node(node);

//     // Start spinning the executor
//     executor->spin();

//     // rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

/**
* Program to print end-effector pose
*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Declare Node
  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("pose_publisher",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
        true));

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  moveit::planning_interface::MoveGroupInterface move_group_interface =
    moveit::planning_interface::MoveGroupInterface(node, "arm");

  // print current pose
  geometry_msgs::msg::PoseStamped current_pose =
    move_group_interface.getCurrentPose("rgb_camera");

  // Print the current pose of the end effector
  RCLCPP_INFO(node->get_logger(), "Current pose: %f %f %f %f %f %f %f",
    current_pose.pose.position.x,
    current_pose.pose.position.y,
    current_pose.pose.position.z,
    current_pose.pose.orientation.x,
    current_pose.pose.orientation.y,
    current_pose.pose.orientation.z,
    current_pose.pose.orientation.w);

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
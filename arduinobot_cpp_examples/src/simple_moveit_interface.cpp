#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <geometry_msgs/msg/pose.hpp>
#include <arduinobot_msgs/msg/joint_pub.hpp>
#include <rclcpp/create_subscription.hpp>

using namespace std::placeholders;

class SimpleMoveitInterface : public rclcpp::Node
{
public:


// void move_robot(const std::shared_ptr<rclcpp::Node> node)
// {

//     js_sub = create_subscription<arduinobot_msgs::msg::JointPub>("to_move_joints", 10, std::bind(&SimpleSubscriber::moveit_now, this, _1, _2));

// }

  SimpleMoveitInterface() : Node("simple_moveit_interface")
  {
    js_sub = create_subscription<arduinobot_msgs::msg::JointPub>(
        "to_move_joints", 10, std::bind(&SimpleMoveitInterface::moveit_now, this, _1));
  }

private:

    rclcpp::Subscription<arduinobot_msgs::msg::JointPub>::SharedPtr js_sub;
    void moveit_now(const arduinobot_msgs::msg::JointPub &msg)
    {   
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_moveit_interface");
        auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm");
        auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(node, "gripper");
        float j1 = msg.state_of_joint[0];
        float j2 = msg.state_of_joint[1];
        float j3 = msg.state_of_joint[2];
        std::vector<double> arm_joint_goal {j1, j2, j3};
        std::vector<double> gripper_joint_goal {0.0, 0.0};

        bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received joint values: j1 = %.2f, j2 = %.2f, j3 = %.2f", j1, j2, j3);

        bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);

        if (!arm_within_bounds | !gripper_within_bounds)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target Joint Position were outside the limits");
            return;
        }

        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;

        bool arm_plan_success = arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS;
        bool gripper_plan_success = gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS;

        if(arm_plan_success && gripper_plan_success)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planner succeeded, Moving the arm and gripper ... Lets goooo");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MOVING TO values: j1 = %.2f, j2 = %.2f, j3 = %.2f", j1, j2, j3);
            arm_move_group.move();
            gripper_move_group.move();

        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "One or more planners failed");
            return;
        }
    }

};


int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_moveit_interface");
   
    // move_robot(node);

    rclcpp::spin(std::make_shared<SimpleMoveitInterface>());
    rclcpp::shutdown();

    return 0;
}
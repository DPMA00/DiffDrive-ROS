#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <my_robot_interfaces/action/move_to.hpp>
#include <algorithm>
#include <cmath>

using MoveTo = my_robot_interfaces::action::MoveTo;
using namespace std::placeholders;
using TargetHandlingGoalHandle = rclcpp_action::ServerGoalHandle<MoveTo>;

class TargetHandlingServer : public rclcpp::Node
{
    public:
    TargetHandlingServer() : Node("target_handling_actionserver")
    {
        target_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("diffdrive/target_pos",10);
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("diffdrive/odometry",
            10, std::bind(&TargetHandlingServer::OdometryCallback, this, std::placeholders::_1));
        
        target_handling_server = rclcpp_action::create_server<MoveTo>(
            this,
            "move_to",
            std::bind(&TargetHandlingServer::goal_callback, this, _1, _2),
            std::bind(&TargetHandlingServer::cancel_callback, this, _1),
            std::bind(&TargetHandlingServer::handle_accepted_callback, this, _1)

        );

        RCLCPP_INFO(this->get_logger(), "Target handling server started");
    }

    private:

    double current_x {0};
    double current_y {0};
    double current_theta {0};

    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveTo::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<TargetHandlingGoalHandle> goal_handle)
    {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(const std::shared_ptr<TargetHandlingGoalHandle> goal_handle)
    {
        execute_goal(goal_handle);
    }

    void execute_goal(const std::shared_ptr<TargetHandlingGoalHandle> goal_handle)
    {
        // Get request from goal
        auto target_x = goal_handle->get_goal()->x;
        auto target_y = goal_handle->get_goal()->y;

        publish_target_data(target_x, target_y);
        auto result = std::make_shared<MoveTo::Result>();
        double pos_error = sqrt(pow((target_x-current_x), 2) + pow((target_y-current_y), 2));

        if (pos_error >=0.1)
        {
            result->end_x = target_x;
            result->end_y = target_y;

            goal_handle->succeed(result);
        }
        
        else
        {
            result->end_x = current_x;
            result->end_y = current_y;

            goal_handle->is_active();
        }


    }

    void publish_target_data(double target_x, double target_y)
    {
        auto target_data = geometry_msgs::msg::Vector3();
        target_data.x = target_x;
        target_data.y = target_y;

        target_publisher->publish(target_data);
    }


    void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr Odom)
    {
        current_x = Odom->pose.pose.position.x;
        current_y = Odom->pose.pose.position.y;
        current_theta = Odom->pose.pose.orientation.w;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp_action::Server<MoveTo>::SharedPtr target_handling_server;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr target_publisher;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetHandlingServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
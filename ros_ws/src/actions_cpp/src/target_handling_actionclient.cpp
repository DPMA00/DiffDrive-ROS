#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <my_robot_interfaces/action/move_to.hpp>
#include <cmath>


using MoveTo = my_robot_interfaces::action::MoveTo;
using MoveToGoalHandle = rclcpp_action::ClientGoalHandle<MoveTo>;
using namespace std::placeholders;


class TargetHandlingClient : public rclcpp::Node
{
public:
    TargetHandlingClient() : Node("target_handling_actionclient")
    {
        target_handling_client = rclcpp_action::create_client<MoveTo>(this, "move_to");

    }

    void send_goal(double x, double y, double theta)
    {
        target_handling_client->wait_for_action_server();
        auto goal = MoveTo::Goal();

        goal.x = x;
        goal.y = y;
        goal.theta = theta;

        auto options = rclcpp_action::Client<MoveTo>::SendGoalOptions();
        options.result_callback = std::bind(&TargetHandlingClient::goal_result_callback, this, _1);
        options.goal_response_callback = std::bind(&TargetHandlingClient::goal_response_callback, this, _1);

        RCLCPP_INFO(this->get_logger(), "Sending goal");
        target_handling_client->async_send_goal(goal);
    }

private:
    //Callback to receive result once the goal is done
    void goal_result_callback(const MoveToGoalHandle::WrappedResult &result)
    {
        auto status = result.code;
        if (status==rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Succeeded");
        }
        else if (status == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_ERROR(this->get_logger(), "Aborted");
        }

        double reached_x = result.result->end_x;
        double reached_y = result.result->end_y;
        double reached_theta = result.result->end_theta;

        RCLCPP_INFO(this->get_logger(),  "x: %f | y: %f | theta(degrees): %f", reached_x, reached_y, reached_theta);
    }

    //callback accept or rejected goal
    void goal_response_callback(const MoveToGoalHandle::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_WARN(this->get_logger(), "Goal was rejected");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Goal was accepted");
        }
    }

    rclcpp_action::Client<MoveTo>::SharedPtr target_handling_client;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetHandlingClient>();
    node->send_goal(4.0, 2.0, 0.1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
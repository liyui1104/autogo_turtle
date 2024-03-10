#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/set_pen.hpp"

using namespace std::chrono_literals;

class AutoGoTurtle : public rclcpp::Node
{
public:
    AutoGoTurtle() : Node("autogo_turtle_waypoints_node")
    {
        client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&AutoGoTurtle::pose_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(10ms, std::bind(&AutoGoTurtle::timer_callback, this));

        read_path_from_file(file_names_.front());
        is_drawing_ = true;
        start_draw(0, 162, 232, 6, 0);
        RCLCPP_INFO(this->get_logger(), "Executing path from file: %s",
            file_names_.front().c_str());
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_theta_ = msg->theta;
        //RCLCPP_INFO(this->get_logger(), "current (x,y)=(%lf,%lf)", current_x_, current_y_);
    }

    void timer_callback()
    {
        if (!path_points_.empty())
        {
            move_to_next_point();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Finished executing current path.");
            if (!file_names_.empty())
            {
                is_drawing_ = false;
                stop_draw();
                file_names_.erase(file_names_.begin());
                if (!file_names_.empty())
                {
                    read_path_from_file(file_names_.front());
                    RCLCPP_INFO(this->get_logger(), "Executing path from file: %s",
                        file_names_.front().c_str());
                }
            }
            else
            {
                stop_turtle();
            }
        }
    }

    void move_to_next_point()
    {
        auto vel = geometry_msgs::msg::Twist();
        auto target_x = path_points_.front().first;
        auto target_y = path_points_.front().second;

        double dx = target_x - current_x_;
        double dy = target_y - current_y_;

        double distance = std::sqrt(dx * dx + dy * dy);
        //RCLCPP_INFO(this->get_logger(), "distance: %lf", distance);

        double target_angle = std::atan2(dy, dx);
        double diff_angle = target_angle - current_theta_;
        //RCLCPP_INFO(this->get_logger(), "diff_angle: %lf", diff_angle);

        // Limit diff_angle to between pi and -pi
        if (diff_angle > 3.14)
        {
            diff_angle -= 2 * 3.14;
        }
        else if (diff_angle < -3.14)
        {
            diff_angle += 2 * 3.14;
        }

        // Adjust orientation first, then tune it when the turtle going straight
        if (std::abs(diff_angle) > 0.1)
        {
            vel.linear.x = 0;
            vel.angular.z = diff_angle * 1; // Adjust this factor as needed
        }
        else if (std::abs(distance) > 0.05)
        {
            vel.linear.x = distance * 1;      // Adjust this factor as needed
            vel.angular.z = diff_angle * 0.5; // Adjust this factor as needed
        }
        else
        {
            if (!is_drawing_)
            {
                is_drawing_ = true;
                start_draw(0, 162, 232, 6, 0);
            }

            // Reached the target point, remove it from the path
            path_points_.erase(path_points_.begin());
            return;
        }

        cmd_vel_pub_->publish(vel);
    }

    void read_path_from_file(const std::string &file_name)
    {
        std::ifstream file(file_name);
        if (!file.is_open())
        {
            std::cerr << "Error opening file " << file_name << ": ";
            std::perror("");
            return;
        }

        path_points_.clear();
        double x, y;
        char comma; // Assuming the values are separated by commas

        while (file >> x >> comma >> y)
        {
            if (file.fail())
            {
                // Failed to read x or y
                std::cerr << "Error reading data from file: " << file_name << std::endl;
                break;
            }
            path_points_.emplace_back(x, y);
        }

        file.close();
    }

    void stop_turtle()
    {
        auto vel = geometry_msgs::msg::Twist();
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
        cmd_vel_pub_->publish(vel);
        RCLCPP_INFO(this->get_logger(), "Turtle stopped.");
    }

    void start_draw(uint8_t r, uint8_t g, uint8_t b,
                    uint8_t width,
                    bool off)
    {
        // Wait for the server to come online
        while (!client_->wait_for_service(1s))
        {
            // Check the status of the rclcpp while waiting
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        // Construct request message
        auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
        request->r = r;
        request->g = g;
        request->b = b;
        request->width = width;
        request->off = off;

        client_->async_send_request(request);
    }

    void stop_draw()
    {
        // Wait for the server to come online
        while (!client_->wait_for_service(1s))
        {
            // Check the status of the rclcpp while waiting
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        // Construct request message
        auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
        request->off = 1;

        client_->async_send_request(request);
    }

private:
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool is_drawing_ = false;
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_theta_ = 0.0;

    std::vector<std::pair<double, double>> path_points_;
    std::vector<std::string> file_names_ = {"/opt/ros/ros2_ws/src/autogo_turtle/src/A.csv",
                                            "/opt/ros/ros2_ws/src/autogo_turtle/src/u.csv",
                                            "/opt/ros/ros2_ws/src/autogo_turtle/src/t.csv",
                                            "/opt/ros/ros2_ws/src/autogo_turtle/src/o1.csv",
                                            "/opt/ros/ros2_ws/src/autogo_turtle/src/G.csv",
                                            "/opt/ros/ros2_ws/src/autogo_turtle/src/o2.csv"};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoGoTurtle>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

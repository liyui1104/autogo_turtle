#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

using namespace std::chrono_literals;

class TurtlePoseSubscriber : public rclcpp::Node
{
public:
    TurtlePoseSubscriber() : Node("turtle_pose_subscriber")
    {
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&TurtlePoseSubscriber::topic_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(20ms, std::bind(&TurtlePoseSubscriber::timer_callback, this));
    }

private:
    void topic_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_x_ = msg->x;
        current_y_ = msg->y;
    }

    void timer_callback()
    {
        if (current_x_ != previous_x_ || current_y_ != previous_y_)
        {
            write_to_file(current_x_, current_y_);
            previous_x_ = current_x_;
            previous_y_ = current_y_;
        }
    }

    void write_to_file(float x, float y)
    {
        std::ofstream file("data.csv", std::ios::app);
        if (file.is_open())
        {
            file << x << "," << y << "\n";
            file.close();
        }
        else
        {
            std::cerr << "Unable to open file" << std::endl;
        }
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    float current_x_ = 0.0;
    float current_y_ = 0.0;
    float previous_x_ = 0.0;
    float previous_y_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtlePoseSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
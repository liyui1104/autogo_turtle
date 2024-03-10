#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"

using namespace std::chrono_literals;

class AutoGoTurtle : public rclcpp::Node
{
public:
  AutoGoTurtle() : Node("autogo_turtle_geometry_node")
  {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
    client_ = this->create_client<turtlesim::srv::SetPen>("/turtle2/set_pen");

    // Control turtle to draw "AutoGo"
    control_turtle();
  }

private:
  void control_turtle()
  {
    draw_A();  // Draw 'A'
    draw_u();  // Draw 'u'
    draw_t();  // Draw 't'
    draw_o1(); // Draw 'o'
    draw_G();  // Draw 'G'
    draw_o2(); // Draw 'o'
  }

  void draw_A()
  {
    auto cmd_msg = geometry_msgs::msg::Twist();

    start_draw(0, 162, 232, 6, 0);

    // Turn left with angular velocity 1.178 rad/s (about 67.5 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = 1.178;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 1.3 m/s
    cmd_msg.linear.x = 1.3;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn right with angular velocity 2.36 rad/s (about 135 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = -2.36;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);
    
    // Move forward with linear velocity 1.3 m/s
    cmd_msg.linear.x = 1.3;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn right with angular velocity 3.14 rad/s (about 180 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = -3.14;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 0.65 m/s
    cmd_msg.linear.x = 0.65;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn left with angular velocity 1.178 rad/s (about 67.5 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = 1.178;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 0.5 m/s
    cmd_msg.linear.x = 0.5;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    stop_draw();

    // Turn right with angular velocity 3.14 rad/s (about 180 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = -3.14;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 1.2 m/s
    cmd_msg.linear.x = 1.2;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);
  }

  void draw_u()
  {
    auto cmd_msg = geometry_msgs::msg::Twist();
    
    start_draw(0, 162, 232, 6, 0);

    // Turn right with angular velocity 1.57 rad/s (about 90 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = -1.57;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 0.35 m/s
    cmd_msg.linear.x = 0.35;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn a half circle with a diameter of 0.5 m
    cmd_msg.linear.x = 3.14*0.5/2;
    cmd_msg.angular.z = 3.14;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);
    
    // Move forward with linear velocity 0.35 m/s
    cmd_msg.linear.x = 0.35;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn right with angular velocity 3.14 rad/s (about 180 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = -3.14;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 0.35 m/s
    cmd_msg.linear.x = 0.35;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn a quarter circle 0.5 meters in diameter
    cmd_msg.linear.x = 3.14*0.5/2/2;
    cmd_msg.angular.z = 1.57;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    stop_draw();

    // Move forward with linear velocity 0.95 m/s
    cmd_msg.linear.x = 0.95;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn left with angular velocity 1.57 rad/s (about 90 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = 1.57;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 0.25 m/s
    cmd_msg.linear.x = 0.25;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn right with angular velocity 1.57 rad/s (about 90 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = -1.57;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);
  }

  void draw_t()
  {
    auto cmd_msg = geometry_msgs::msg::Twist();

    start_draw(0, 162, 232, 6, 0);

    // Turn right with angular velocity 1.57 rad/s (about 90 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = -1.57;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn a half circle with a diameter of 0.5 m
    cmd_msg.linear.x = 3.14*0.5/2;
    cmd_msg.angular.z = -3.14;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 0.45 m/s
    cmd_msg.linear.x = 0.45;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn right with angular velocity 3.14 rad/s (about 180 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = -3.14;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 0.28 m/s
    cmd_msg.linear.x = 0.28;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn right with angular velocity 1.57 rad/s (about 90 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = -1.57;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 0.25 m/s
    cmd_msg.linear.x = 0.25;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn right with angular velocity 3.14 rad/s (about 180 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = -3.14;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 0.5 m/s
    cmd_msg.linear.x = 0.5;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    stop_draw();

    // Turn right with angular velocity 1.57 rad/s (about 90 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = -1.57;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 0.427 m/s
    cmd_msg.linear.x = 0.427;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn left with angular velocity 1.57 rad/s (about 90 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = 1.57;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 0.95 m/s
    cmd_msg.linear.x = 0.95;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);
  }

  void draw_o1()
  {
    auto cmd_msg = geometry_msgs::msg::Twist();

    start_draw(0, 162, 232, 6, 0);

    // Turn a circle with a diameter of 0.6 meters
    cmd_msg.linear.x = 3.14*0.6;
    cmd_msg.angular.z = 6.28;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    stop_draw();

    // Move forward with linear velocity 1.3 m/s
    cmd_msg.linear.x = 1.3;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn left with angular velocity 1.57 rad/s (about 90 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = 1.57;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 1.2 m/s
    cmd_msg.linear.x = 1.2;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn right with angular velocity 1.57 rad/s (about 90 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = -1.57;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);
  }

  void draw_G()
  {
    auto cmd_msg = geometry_msgs::msg::Twist();

    start_draw(0, 162, 232, 6, 0);

    // Turn left with angular velocity 3.14 rad/s (about 180 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = 3.14;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn a half circle with a diameter of 1.2 m
    cmd_msg.linear.x = 3.14*1.2/2;
    cmd_msg.angular.z = 3.14;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn a quarter circle 0.6 meters in diameter
    cmd_msg.linear.x = 3.14*0.6/2/2;
    cmd_msg.angular.z = 1.57;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn right with angular velocity 1.57 rad/s (about 90 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = -1.57;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 0.3 m/s
    cmd_msg.linear.x = 0.3;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn right with angular velocity 3.14 rad/s (about 180 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = -3.14;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 0.6 m/s
    cmd_msg.linear.x = 0.6;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn right with angular velocity 3.14 rad/s (about 180 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = -3.14;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 0.3 m/s
    cmd_msg.linear.x = 0.3;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Turn right with angular velocity 1.57 rad/s (about 90 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = -1.57;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 0.3 m/s
    cmd_msg.linear.x = 0.3;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    stop_draw();

    // Turn left with angular velocity 1.57 rad/s (about 90 degrees)
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = 1.57;                  
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);

    // Move forward with linear velocity 1 m/s
    cmd_msg.linear.x = 1;
    cmd_msg.angular.z = 0;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);
  }

  void draw_o2()
  {
    auto cmd_msg = geometry_msgs::msg::Twist();

    start_draw(0, 162, 232, 6, 0);

    // Turn a circle with a diameter of 0.6 meters
    cmd_msg.linear.x = 3.14*0.6;
    cmd_msg.angular.z = 6.28;
    cmd_pub_->publish(cmd_msg);
    rclcpp::sleep_for(1s);
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
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoGoTurtle>());
  rclcpp::shutdown();
  return 0;
}

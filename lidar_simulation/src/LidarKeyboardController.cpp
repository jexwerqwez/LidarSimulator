
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

class KeyboardControlNode : public rclcpp::Node {
public:
  KeyboardControlNode() : Node("lidar_keyboard_controller") {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/lidar_control", 10);
    RCLCPP_INFO(this->get_logger(), "Use WASD to move the lidar (q to quit)");
    loop();
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

  void loop() {
    while (rclcpp::ok()) {
      char c = getChar();
      geometry_msgs::msg::Twist msg;

      if (c == 'w') msg.linear.x = 0.2;
      else if (c == 's') msg.linear.x = -0.2;
      else if (c == 'a') msg.linear.y = 0.2;
      else if (c == 'd') msg.linear.y = -0.2;
      else if (c == 'q') break;

      pub_->publish(msg);
      usleep(100000);
    }
  }

  char getChar() {
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class BoolPublisher2 : public rclcpp::Node
{
public:
  BoolPublisher2() : Node("bool_publisher"), value_(false)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Bool>("NOT_EvSystemsChecksOK", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&BoolPublisher2::timer_callback, this));
  }

private:
  void timer_callback()
  {
    std_msgs::msg::Bool msg;
    msg.data = value_;
    RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data ? "true" : "false");
    publisher_->publish(msg);
    value_ = !value_;  // alterna entre true e false
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool value_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoolPublisher2>());
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"



class PosePublisher : public rclcpp::Node {
  public:
    PosePublisher() : Node("Pose_publisher"){
      sub_pose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>
                ("input_pose", 10, std::bind(&PosePublisher::PoseCallback, this, std::placeholders::_1));
      pub_pose = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>
                ("output_pose", 10);
  }

  private:
    void PoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg){
      if(first_pose){
        initial_pose = *pose_msg;
        first_pose = !first_pose;
      }
      geometry_msgs::msg::PoseWithCovarianceStamped corrected_pose = *pose_msg;
      corrected_pose.pose.pose.position.x -= initial_pose.pose.pose.position.x;
      corrected_pose.pose.pose.position.y -= initial_pose.pose.pose.position.y;
      corrected_pose.pose.pose.position.z -= initial_pose.pose.pose.position.z;

      pub_pose->publish(corrected_pose);
    }
    
    bool first_pose = true;
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosePublisher>());
  rclcpp::shutdown();
  return 0;
}
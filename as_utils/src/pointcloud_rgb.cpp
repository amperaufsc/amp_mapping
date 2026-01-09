#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "fs_msgs/msg/track.hpp"
#include "fs_msgs/msg/cone.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("Track_Pointcloud")
    {
        subscription_ = this->create_subscription<fs_msgs::msg::Track>(
        "/track", 10, std::bind(&MinimalSubscriber::track_callback, this, _1));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud", 10);
        this->declare_parameter<std::string>("frame_id", "/map");
    }

  private:
    void track_callback(const fs_msgs::msg::Track & msg) const
    {
        // RCLCPP_INFO(this->get_logger(), "There is '%ld' cones inside the message", msg.track.size());
        sensor_msgs::msg::PointCloud2 pointcloudmsg;

        pointcloudmsg.header.stamp = now();
        pointcloudmsg.header.frame_id = this->get_parameter("frame_id").as_string();;
        pointcloudmsg.height = 1;
        pointcloudmsg.width = msg.track.size();
        pointcloudmsg.is_dense = false;
        pointcloudmsg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(pointcloudmsg);
        modifier.setPointCloud2Fields(4,
                                  "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                  "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);

        pointcloudmsg.point_step = 16;  // 4 fields * 4 bytes (x, y, z, rgb)
        pointcloudmsg.row_step = pointcloudmsg.point_step * pointcloudmsg.width;

        pointcloudmsg.data.resize(pointcloudmsg.row_step * pointcloudmsg.height);

        for (size_t i = 0; i < msg.track.size(); i++)
        {
            /* code */
            float x,y,z;
            int16_t r,g,b;
            x = msg.track[i].location.x;
            y = msg.track[i].location.y;
            z = msg.track[i].location.z;

            switch (msg.track[i].color)
            {
            case 0:
                r = 0, g = 0, b = 255;
                break;
            case 1:
                r = 255, g = 255, b = 0;
                break;
            case 2:
                r = 255, g = 165, b = 0;
                break;
            case 3:
                r = 255, g = 165, b = 0;
                break;
            case 4:
                r = 0, g = 0, b = 0;
                break;
            default:
                r = 0, g = 0, b = 0;
                break;
            }

            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

            // Copy x, y, z, and rgb to the point cloud message data
            int index = i * pointcloudmsg.point_step;
            memcpy(&pointcloudmsg.data[index], &x, sizeof(float));
            memcpy(&pointcloudmsg.data[index + 4], &y, sizeof(float));
            memcpy(&pointcloudmsg.data[index + 8], &z, sizeof(float));
            memcpy(&pointcloudmsg.data[index + 12], &rgb, sizeof(uint32_t));

            RCLCPP_INFO(this->get_logger(), "Cone position: x = %f, y = %f, z = %f\n Cone color: [%d,%d,%d]", msg.track[i].location.x, 
                                                                                        msg.track[i].location.y,
                                                                                         msg.track[i].location.z,
                                                                                         r,g,b);
        }
        publisher_->publish(pointcloudmsg);
        
    }
    rclcpp::Subscription<fs_msgs::msg::Track>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
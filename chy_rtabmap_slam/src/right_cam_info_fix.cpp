//把 P[3] 改成 –baseline·fx   sdf文件自动是0的，需要手动改成实际值

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

class RightCamInfoFix : public rclcpp::Node
{
public:
  RightCamInfoFix()
  : Node("right_cam_info_fix")
  {
    // 参数
    baseline_ = this->declare_parameter("baseline", 0.1);   // 10 cm
    sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/stereo/right/camera_info", rclcpp::QoS(1),
        std::bind(&RightCamInfoFix::info_cb, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/stereo/right/camera_info_fixed", rclcpp::QoS(1));
  }

private:
  void info_cb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    // 深度拷贝
    auto out = std::make_shared<sensor_msgs::msg::CameraInfo>(*msg);
    // 只在为 0 时修正
    if (std::abs(out->p[3]) < 1e-6)
    {
      const double fx = out->p[0];          // P(0,0)
      out->p[3] = -baseline_ * fx;          // Tx = -baseline*fx
    }
    pub_->publish(*out);
  }

  double baseline_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RightCamInfoFix>());
  rclcpp::shutdown();
  return 0;
}


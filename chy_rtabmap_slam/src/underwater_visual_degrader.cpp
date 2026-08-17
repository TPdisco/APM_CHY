/**
 * @file underwater_visual_degrader.cpp
 * @brief 订阅 Gazebo 双目原始图像，实时叠加水下光学退化效果
 *
 * 模拟的水下光学退化效应：
 *   1. 颜色吸收（红色衰减，蓝绿色增强）— 模拟不同深度水体对光谱的选择性吸收
 *   2. 浑浊度（高斯模糊）— 模拟悬浮颗粒物导致的图像模糊
 *   3. 后向散射（加性噪声）— 模拟水中微粒对光线的散射
 *   4. 光照衰减（亮度降低）— 模拟随深度增加的光衰减
 *
 * 同时处理左右目，确保双目保持一致的退化效果。
 * 所有效果强度可通过 ROS2 参数动态调节。
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <random>
#include <string>

class UnderwaterVisualDegrader : public rclcpp::Node
{
public:
  UnderwaterVisualDegrader()
  : Node("underwater_visual_degrader"), rng_(std::random_device{}())
  {
    // ========== 声明参数 ==========
    this->declare_parameter<double>("turbidity", 0.3);       // 浑浊度 [0, 1]
    this->declare_parameter<double>("depth", 0.5);           // 深度因子 [0, 1]
    this->declare_parameter<double>("backscatter", 0.1);     // 后向散射强度 [0, 1]
    this->declare_parameter<bool>("enable", true);           // 是否启用退化

    // ========== 左目 ==========
    this->declare_parameter<std::string>("left_input_topic", "/stereo/left/image_raw");
    this->declare_parameter<std::string>("left_output_topic", "/stereo/left/underwater");

    sub_left_ = this->create_subscription<sensor_msgs::msg::Image>(
      this->get_parameter("left_input_topic").as_string(),
      rclcpp::QoS(10).best_effort(),
      std::bind(&UnderwaterVisualDegrader::leftCallback, this, std::placeholders::_1));

    pub_left_ = this->create_publisher<sensor_msgs::msg::Image>(
      this->get_parameter("left_output_topic").as_string(), rclcpp::QoS(10));

    // ========== 右目 ==========
    this->declare_parameter<std::string>("right_input_topic", "/stereo/right/image_raw");
    this->declare_parameter<std::string>("right_output_topic", "/stereo/right/underwater");

    sub_right_ = this->create_subscription<sensor_msgs::msg::Image>(
      this->get_parameter("right_input_topic").as_string(),
      rclcpp::QoS(10).best_effort(),
      std::bind(&UnderwaterVisualDegrader::rightCallback, this, std::placeholders::_1));

    pub_right_ = this->create_publisher<sensor_msgs::msg::Image>(
      this->get_parameter("right_output_topic").as_string(), rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(),
      "水下视觉退化节点已启动\n"
      "  左目: %s → %s\n"
      "  右目: %s → %s\n"
      "  浑浊度: %.2f  深度因子: %.2f  后向散射: %.2f",
      this->get_parameter("left_input_topic").as_string().c_str(),
      this->get_parameter("left_output_topic").as_string().c_str(),
      this->get_parameter("right_input_topic").as_string().c_str(),
      this->get_parameter("right_output_topic").as_string().c_str(),
      this->get_parameter("turbidity").as_double(),
      this->get_parameter("depth").as_double(),
      this->get_parameter("backscatter").as_double());
  }

private:
  // ========== 左目回调 ==========
  void leftCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto out = processImage(msg);
    if (out) pub_left_->publish(*out);
  }

  // ========== 右目回调 ==========
  void rightCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto out = processImage(msg);
    if (out) pub_right_->publish(*out);
  }

  /**
   * @brief 通用图像处理：转换 → 退化 → 输出
   */
  sensor_msgs::msg::Image::SharedPtr processImage(
    const sensor_msgs::msg::Image::SharedPtr msg)
  {
    bool enable = this->get_parameter("enable").as_bool();
    if (!enable) return msg;  // 直通

    double turbidity = this->get_parameter("turbidity").as_double();
    double depth = this->get_parameter("depth").as_double();
    double backscatter = this->get_parameter("backscatter").as_double();

    cv::Mat img;
    try {
      img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
      return nullptr;
    }

    cv::Mat degraded = applyUnderwaterDegradation(img, turbidity, depth, backscatter);
    return cv_bridge::CvImage(msg->header, "bgr8", degraded).toImageMsg();
  }

  /**
   * @brief 对输入图像叠加水下光学退化效果
   */
  cv::Mat applyUnderwaterDegradation(
    const cv::Mat & src,
    double turbidity,
    double depth,
    double backscatter)
  {
    cv::Mat result = src.clone();

    // ---- 1. 颜色吸收：红色衰减，蓝绿色增强 ----
    if (depth > 0.0) {
      std::vector<cv::Mat> channels(3);
      cv::split(result, channels);

      // 红色通道衰减最多（水体对长波吸收强）
      double red_scale = 1.0 - depth * 0.8;   // depth=1 → 红通道剩 20%
      // 蓝绿通道衰减较少
      double green_scale = 1.0 - depth * 0.3;
      double blue_scale = 1.0 - depth * 0.1;

      channels[2] = channels[2] * red_scale;    // BGR: R=2
      channels[1] = channels[1] * green_scale;   // BGR: G=1
      channels[0] = channels[0] * blue_scale;    // BGR: B=0

      cv::merge(channels, result);
    }

    // ---- 2. 浑浊度：高斯模糊 ----
    if (turbidity > 0.0) {
      int kernel_size = static_cast<int>(1 + turbidity * 20);  // 1~21
      if (kernel_size % 2 == 0) kernel_size++;                  // 必须为奇数
      if (kernel_size > 1) {
        cv::GaussianBlur(result, result, cv::Size(kernel_size, kernel_size), 0);
      }
    }

    // ---- 3. 后向散射：加性高斯噪声 ----
    if (backscatter > 0.0) {
      std::normal_distribution<double> dist(0.0, backscatter * 30.0);
      for (int y = 0; y < result.rows; ++y) {
        for (int x = 0; x < result.cols; ++x) {
          cv::Vec3b & pixel = result.at<cv::Vec3b>(y, x);
          for (int c = 0; c < 3; ++c) {
            int val = pixel[c] + static_cast<int>(dist(rng_));
            pixel[c] = static_cast<uchar>(std::clamp(val, 0, 255));
          }
        }
      }
    }

    // ---- 4. 光照衰减：整体变暗 ----
    if (depth > 0.0) {
      double brightness = 1.0 - depth * 0.5;
      result = result * brightness;
    }

    return result;
  }

  // ========== 成员变量 ==========
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_left_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_right_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_;
  std::mt19937 rng_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UnderwaterVisualDegrader>());
  rclcpp::shutdown();
  return 0;
}
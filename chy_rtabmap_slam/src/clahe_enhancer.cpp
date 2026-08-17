/**
 * @file clahe_enhancer.cpp
 * @brief 订阅双目图像，使用 CLAHE 算法增强水下低对比度图像
 *
 * CLAHE (Contrast Limited Adaptive Histogram Equalization)：
 *   将图像分块，对每个小块独立做直方图均衡化，同时限制对比度放大倍数，
 *   避免噪声过度放大。适合水下图像因散射/吸收导致的低对比度问题。
 *
 * 同时处理左右目，共享同一套参数。
 * 所有参数可通过 ROS2 动态调节。
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

class CLAHEEnhancer : public rclcpp::Node
{
public:
  CLAHEEnhancer()
  : Node("clahe_enhancer")
  {
    // ========== 声明参数 ==========
    this->declare_parameter<double>("clip_limit", 2.0);       // 对比度限制 [0.5, 5.0]
    this->declare_parameter<int>("tile_size", 8);             // 分块大小 [4, 32]
    this->declare_parameter<bool>("enable", true);            // 是否启用
    this->declare_parameter<bool>("lab_mode", true);          // LAB模式：只在L通道做CLAHE，保留色彩

    // ========== 左目 ==========
    this->declare_parameter<std::string>("left_input_topic", "/stereo/left/underwater");
    this->declare_parameter<std::string>("left_output_topic", "/stereo/left/enhanced");

    sub_left_ = this->create_subscription<sensor_msgs::msg::Image>(
      this->get_parameter("left_input_topic").as_string(),
      rclcpp::QoS(10).best_effort(),
      std::bind(&CLAHEEnhancer::leftCallback, this, std::placeholders::_1));

    pub_left_ = this->create_publisher<sensor_msgs::msg::Image>(
      this->get_parameter("left_output_topic").as_string(), rclcpp::QoS(10));

    // ========== 右目 ==========
    this->declare_parameter<std::string>("right_input_topic", "/stereo/right/underwater");
    this->declare_parameter<std::string>("right_output_topic", "/stereo/right/enhanced");

    sub_right_ = this->create_subscription<sensor_msgs::msg::Image>(
      this->get_parameter("right_input_topic").as_string(),
      rclcpp::QoS(10).best_effort(),
      std::bind(&CLAHEEnhancer::rightCallback, this, std::placeholders::_1));

    pub_right_ = this->create_publisher<sensor_msgs::msg::Image>(
      this->get_parameter("right_output_topic").as_string(), rclcpp::QoS(10));

    updateCLAHE();  // 初始化 CLAHE 对象

    RCLCPP_INFO(this->get_logger(),
      "CLAHE 图像增强节点已启动\n"
      "  左目: %s → %s\n"
      "  右目: %s → %s\n"
      "  clip_limit: %.1f  tile_size: %ld  lab_mode: %s",
      this->get_parameter("left_input_topic").as_string().c_str(),
      this->get_parameter("left_output_topic").as_string().c_str(),
      this->get_parameter("right_input_topic").as_string().c_str(),
      this->get_parameter("right_output_topic").as_string().c_str(),
      this->get_parameter("clip_limit").as_double(),
      this->get_parameter("tile_size").as_int(),
      this->get_parameter("lab_mode").as_bool() ? "true" : "false");
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
   * @brief 更新 CLAHE 对象（参数变化时重建）
   */
  void updateCLAHE()
  {
    double clip = this->get_parameter("clip_limit").as_double();
    int tile = this->get_parameter("tile_size").as_int();
    if (tile % 2 == 0) tile++;  // 确保奇数
    clahe_ = cv::createCLAHE(clip, cv::Size(tile, tile));
  }

  /**
   * @brief 通用图像处理：CLAHE 增强
   */
  sensor_msgs::msg::Image::SharedPtr processImage(
    const sensor_msgs::msg::Image::SharedPtr msg)
  {
    bool enable = this->get_parameter("enable").as_bool();
    if (!enable) return msg;  // 直通

    // 参数变化时重建 CLAHE
    updateCLAHE();

    cv::Mat img;
    try {
      img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
      return nullptr;
    }

    cv::Mat enhanced;

    bool lab_mode = this->get_parameter("lab_mode").as_bool();
    if (lab_mode) {
      // LAB 模式：只对亮度通道 L 做 CLAHE，保留原始色彩
      cv::Mat lab;
      cv::cvtColor(img, lab, cv::COLOR_BGR2Lab);
      std::vector<cv::Mat> lab_channels(3);
      cv::split(lab, lab_channels);
      clahe_->apply(lab_channels[0], lab_channels[0]);  // 只在 L 通道做 CLAHE
      cv::merge(lab_channels, lab);
      cv::cvtColor(lab, enhanced, cv::COLOR_Lab2BGR);
    } else {
      // BGR 模式：每个通道独立做 CLAHE
      std::vector<cv::Mat> bgr_channels(3);
      cv::split(img, bgr_channels);
      for (int c = 0; c < 3; ++c) {
        clahe_->apply(bgr_channels[c], bgr_channels[c]);
      }
      cv::merge(bgr_channels, enhanced);
    }

    return cv_bridge::CvImage(msg->header, "bgr8", enhanced).toImageMsg();
  }

  // ========== 成员变量 ==========
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_left_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_right_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_;
  cv::Ptr<cv::CLAHE> clahe_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CLAHEEnhancer>());
  rclcpp::shutdown();
  return 0;
}
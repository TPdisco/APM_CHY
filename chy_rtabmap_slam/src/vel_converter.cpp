#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class Nav2ToAPMBridge : public rclcpp::Node
{
public:
  Nav2ToAPMBridge() : Node("nav2_to_apm_bridge")
  {
    // APM 混合坐标系：X前Y右Z上，Yaw右为正
    // Nav2（FLU：X前、Y左、Z上、Yaw左为正）
    this->declare_parameter("flip_y", true);       // Y: 左->右
    this->declare_parameter("flip_z", false);      // Z: 同向（上都为正）
    this->declare_parameter("flip_yaw", false);    // Yaw: 左->右
    this->declare_parameter("nav2_timeout_sec", 0.5);  // Nav2 超时后停止发送速度命令

    rclcpp::QoS qos_nav2(10);
    qos_nav2.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    rclcpp::QoS qos_apm(10);
    qos_apm.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", qos_nav2,
      std::bind(&Nav2ToAPMBridge::cmdVelCallback, this, std::placeholders::_1));

    apm_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/ap/cmd_vel", qos_apm);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Nav2ToAPMBridge::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Nav2 -> APM Bridge (改进版: 只在Nav2导航时发送速度)");
    RCLCPP_INFO(this->get_logger(), "APM coords: X前, Y右, Z上, Yaw右");
    RCLCPP_INFO(this->get_logger(), "Converting: Yflip=ON, Zflip=OFF, Yawflip=ON");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    has_new_cmd_ = true;
    last_cmd_time_ = this->now();

    auto apm_cmd = geometry_msgs::msg::TwistStamped();
    apm_cmd.header.stamp = this->now();
    apm_cmd.header.frame_id = "base_link";

    bool flip_y = this->get_parameter("flip_y").as_bool();
    bool flip_z = this->get_parameter("flip_z").as_bool();
    bool flip_yaw = this->get_parameter("flip_yaw").as_bool();

    // 坐标转换
    apm_cmd.twist.linear.x = msg->linear.x;                           // 前
    apm_cmd.twist.linear.y = flip_y ? -msg->linear.y : msg->linear.y; // 右（取反）
    apm_cmd.twist.linear.z = flip_z ? -msg->linear.z : msg->linear.z; // 上（同向）

    apm_cmd.twist.angular.x = msg->angular.x;
    apm_cmd.twist.angular.y = msg->angular.y;
    apm_cmd.twist.angular.z = flip_yaw ? -msg->angular.z : msg->angular.z; // 右（取反）

    apm_vel_pub_->publish(apm_cmd);
  }

  void timerCallback()
  {
    // 检查 Nav2 是否超时（超过 nav2_timeout_sec 没有发送速度命令）
    if (has_new_cmd_) {
      double nav2_timeout = this->get_parameter("nav2_timeout_sec").as_double();
      auto now = this->now();
      double elapsed = (now - last_cmd_time_).seconds();

      if (elapsed > nav2_timeout) {
        // Nav2 超时，停止发送速度命令（不再发送零速度）
        has_new_cmd_ = false;
        RCLCPP_DEBUG(this->get_logger(), "Nav2 cmd_vel timeout (%.2fs), stop sending velocity", elapsed);
      }
    }
    // 注意：不再在没有 Nav2 命令时发送零速度
    // 这样 ArduPilot 的 GUIDED 模式可以正常控制无人机起飞和悬停
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr apm_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool has_new_cmd_ = false;
  rclcpp::Time last_cmd_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2ToAPMBridge>());
  rclcpp::shutdown();
  return 0;
}
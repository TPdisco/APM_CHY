#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <ardupilot_msgs/srv/mode_switch.hpp>
#include <ardupilot_msgs/srv/arm_motors.hpp>

#include <chrono>

class CopterRunNode : public rclcpp::Node
{
public:
  CopterRunNode()
  : Node("copter_run_node")
  {
    // 声明参数
    this->declare_parameter("forward_speed", 0.5);
    this->declare_parameter("publish_rate", 30.0);
    this->declare_parameter("mode_switch_timeout", 10.0);
    this->declare_parameter("arm_timeout", 10.0);

    forward_speed_ = this->get_parameter("forward_speed").as_double();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    double mode_timeout = this->get_parameter("mode_switch_timeout").as_double();
    double arm_timeout = this->get_parameter("arm_timeout").as_double();

    // 创建服务客户端
    mode_switch_client_ = this->create_client<ardupilot_msgs::srv::ModeSwitch>("/ap/mode_switch");
    arm_motors_client_ = this->create_client<ardupilot_msgs::srv::ArmMotors>("/ap/arm_motors");

    // 创建速度命令发布者
    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/ap/velocity_cmd", 10);

    RCLCPP_INFO(this->get_logger(), "=== 无人机运动控制节点启动 ===");

    // 等待服务就绪
    RCLCPP_INFO(this->get_logger(), "等待 ModeSwitch 服务...");
    if (!mode_switch_client_->wait_for_service(std::chrono::duration<double>(mode_timeout))) {
      RCLCPP_ERROR(this->get_logger(), "ModeSwitch 服务超时！");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "ModeSwitch 服务就绪");

    RCLCPP_INFO(this->get_logger(), "等待 ArmMotors 服务...");
    if (!arm_motors_client_->wait_for_service(std::chrono::duration<double>(arm_timeout))) {
      RCLCPP_ERROR(this->get_logger(), "ArmMotors 服务超时！");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "ArmMotors 服务就绪");

    // 步骤1: 切换到 GUIDED 模式
    if (!switchToGuided()) {
      RCLCPP_ERROR(this->get_logger(), "切换 GUIDED 模式失败！");
      rclcpp::shutdown();
      return;
    }

    // 步骤2: 解锁电机
    if (!armMotors()) {
      RCLCPP_ERROR(this->get_logger(), "电机解锁失败！");
      rclcpp::shutdown();
      return;
    }

    // 步骤3: 等待解锁完成
    RCLCPP_INFO(this->get_logger(), "等待电机解锁完成 (2秒)...");
    rclcpp::sleep_for(std::chrono::seconds(2));

    // 步骤4: 先发送零速度命令，触发 VelAccel 子模式切换
    RCLCPP_INFO(this->get_logger(), "发送零速度命令，切换到 VelAccel 子模式...");
    publishVelocity(0.0, 0.0, 0.0, 0.0);
    rclcpp::sleep_for(std::chrono::seconds(1));

    // 步骤5: 开始持续发送前进速度命令
    RCLCPP_INFO(this->get_logger(), "开始发送前进速度命令: %.1f m/s", forward_speed_);

    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
    timer_ = this->create_wall_timer(period, std::bind(&CopterRunNode::timerCallback, this));
  }

private:
  bool switchToGuided()
  {
    auto request = std::make_shared<ardupilot_msgs::srv::ModeSwitch::Request>();
    request->mode = 4;  // AP_MODE_GUIDED

    RCLCPP_INFO(this->get_logger(), "发送模式切换: GUIDED (mode=4)...");
    auto future = mode_switch_client_->async_send_request(request);

    auto status = future.wait_for(std::chrono::seconds(10));
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "模式切换超时");
      return false;
    }

    auto result = future.get();
    bool success = result->status || result->curr_mode == 4;
    RCLCPP_INFO(this->get_logger(), "模式切换: %s (curr_mode=%d)",
      success ? "成功" : "失败", result->curr_mode);
    return success;
  }

  bool armMotors()
  {
    auto request = std::make_shared<ardupilot_msgs::srv::ArmMotors::Request>();
    request->arm = true;

    RCLCPP_INFO(this->get_logger(), "发送电机解锁命令...");
    auto future = arm_motors_client_->async_send_request(request);

    auto status = future.wait_for(std::chrono::seconds(10));
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "解锁超时");
      return false;
    }

    auto result = future.get();
    RCLCPP_INFO(this->get_logger(), "解锁结果: %s", result->result ? "成功" : "失败");
    return result->result;
  }

  void publishVelocity(double x, double y, double z, double yaw)
  {
    auto cmd = geometry_msgs::msg::TwistStamped();
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = x;
    cmd.twist.linear.y = y;
    cmd.twist.linear.z = z;
    cmd.twist.angular.z = yaw;
    vel_pub_->publish(cmd);
  }

  void timerCallback()
  {
    publishVelocity(forward_speed_, 0.0, 0.0, 0.0);
  }

  rclcpp::Client<ardupilot_msgs::srv::ModeSwitch>::SharedPtr mode_switch_client_;
  rclcpp::Client<ardupilot_msgs::srv::ArmMotors>::SharedPtr arm_motors_client_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double forward_speed_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CopterRunNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
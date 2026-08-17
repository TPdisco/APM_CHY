#include <rclcpp/rclcpp.hpp>
#include "mavros_msgs/srv/command_bool.hpp"

class ArmCommander : public rclcpp::Node
{
public:
  ArmCommander()
  : Node("arm_commander")
  {
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");


    // 定时器：节点启动 1 s 后自动尝试解锁
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ArmCommander::send_arm_command, this));

    RCLCPP_INFO(this->get_logger(), "Telemetry reader ready!");
  }

private:
  void send_arm_command()
  {
    // 只调用一次
    timer_->cancel();

    // 等待服务上线
    while (!arming_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) return;
      RCLCPP_INFO(this->get_logger(), "waiting for /mavros/cmd/arming ...");
    }

    // 构造请求
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = false;

    // 异步调用 + 简单 lambda 处理返回
    arming_client_->async_send_request(
      req,
      [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
        if (future.get()->success)
          RCLCPP_INFO(this->get_logger(), "****  Pixhawk UNLOCKED !  ****");
        else
          RCLCPP_WARN(this->get_logger(), "Unlock rejected by FCU (check safety switch / GPS)");
      });
  }

  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmCommander>());
  rclcpp::shutdown();
  return 0;
}
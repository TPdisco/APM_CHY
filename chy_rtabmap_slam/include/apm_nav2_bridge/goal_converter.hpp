// include/apm_nav2_bridge/goal_converter.hpp
#ifndef APM_NAV2_BRIDGE__GOAL_CONVERTER_HPP_
#define APM_NAV2_BRIDGE__GOAL_CONVERTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ardupilot_msgs/msg/global_position.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <ardupilot_msgs/srv/arm_motors.hpp>
#include <ardupilot_msgs/srv/mode_switch.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <memory>
#include <string>
#include <atomic>

namespace apm_nav2_bridge
{

class GoalConverter : public rclcpp::Node
{
public:
    explicit GoalConverter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~GoalConverter() = default;

private:
    // 回调函数
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // 服务调用
    void switchToGuidedMode();
    void armMotors();
    
    // 坐标转换
    void enuToGps(double x, double y, double z, 
                  double & lat, double & lon, double & alt);
    
    // 参数
    double origin_lat_{40.08370};
    double origin_lon_{-105.21740};
    double origin_alt_{1630.0};
    double goal_altitude_{20.0};
    bool auto_guided_mode_{true};
    bool auto_arm_{false};
    double max_goal_distance_{100.0};  // 最大目标距离(米)
    
    // 状态
    std::atomic<bool> is_guided_mode_{false};
    std::atomic<bool> is_armed_{false};
    geometry_msgs::msg::Pose current_pose_;
    
    // 发布者
    rclcpp::Publisher<ardupilot_msgs::msg::GlobalPosition>::SharedPtr global_pos_pub_;
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr geo_pose_pub_;
    
    // 订阅者
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // 服务客户端
    rclcpp::Client<ardupilot_msgs::srv::ArmMotors>::SharedPtr arm_client_;
    rclcpp::Client<ardupilot_msgs::srv::ModeSwitch>::SharedPtr mode_client_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr status_timer_;
    void statusCallback();


    rclcpp::Publisher<ardupilot_msgs::msg::GlobalPosition>::SharedPtr global_pos_pub_2;
    ardupilot_msgs::msg::GlobalPosition global_msg_2;  // 用于调试的副本
};

}  // namespace apm_nav2_bridge

#endif  // APM_NAV2_BRIDGE__GOAL_CONVERTER_HPP_
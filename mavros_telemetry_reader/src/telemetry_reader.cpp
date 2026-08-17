#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>   // <-- 新增
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>

using std::placeholders::_1;

class TelemetryReader : public rclcpp::Node
{
public:
  TelemetryReader()
  : Node("telemetry_reader")
  {

    
    // 1. IMU：三轴角速度、线加速度、姿态四元数
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/mavros/imu/data", rclcpp::QoS(10).best_effort(),
      std::bind(&TelemetryReader::imuCallback, this, _1));

    // 2. 电池电压、电流、剩余电量
    batt_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "/mavros/battery", rclcpp::QoS(10).best_effort(),
      std::bind(&TelemetryReader::batteryCallback, this, _1));

    // 3. 全局位置（WGS84 经纬度+高度）
    global_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global", rclcpp::QoS(10).best_effort(),
      std::bind(&TelemetryReader::globalCallback, this, _1));

    // 4. 本地位置（ENU，单位 m）
    local_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::QoS(10).best_effort(),
      std::bind(&TelemetryReader::localCallback, this, _1));

    // 5. 飞控连接状态、当前模式、上锁状态
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      std::bind(&TelemetryReader::stateCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Telemetry reader ready!");
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),"****************************************");
    RCLCPP_INFO(this->get_logger(),
                "IMU  ang_vel  x:%+7.3f y:%+7.3f z:%+7.3f   lin_acc  x:%+7.3f y:%+7.3f z:%+7.3f",
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z,
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z);
  }

  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
                "Battery  voltage:%6.3f V   current:%+7.3f A   percentage:%3.0f %%",
                msg->voltage,
                msg->current,
                msg->percentage * 100);
  }

  void globalCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
                "Global   lat:%+11.7f  lon:%+11.7f  alt:%+7.3f m",
                msg->latitude,
                msg->longitude,
                msg->altitude);
  }

  void localCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
                "Local    x:%+7.3f  y:%+7.3f  z:%+7.3f",
                msg->pose.position.x,
                msg->pose.position.y,
                msg->pose.position.z);
  }

  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
                "State    connected:%d  armed:%d  guided:%d  mode:%s",
                msg->connected,
                msg->armed,
                msg->guided,
                msg->mode.c_str());
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr batt_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelemetryReader>());
  rclcpp::shutdown();
  return 0;
}
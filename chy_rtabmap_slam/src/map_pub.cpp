#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <mutex>

class GridMapThrottler : public rclcpp::Node
{
public:
    GridMapThrottler() : Node("grid_map_throttler")
    {
        // 声明参数
        this->declare_parameter<double>("publish_rate", 2.0);  // 默认2Hz
        this->declare_parameter<std::string>("input_topic", "/map");
        this->declare_parameter<std::string>("output_topic", "/map_fix");
        
        double publish_rate = this->get_parameter("publish_rate").as_double();
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();

        // 创建订阅者 - 接收高频地图消息
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            input_topic,
            rclcpp::QoS(10).best_effort(),  // 使用best_effort处理高频数据
            std::bind(&GridMapThrottler::mapCallback, this, std::placeholders::_1));

        // 创建发布者 - 降频发布
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            output_topic,
            rclcpp::QoS(10).reliable());

        // 创建定时器 - 固定频率发布
        auto period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&GridMapThrottler::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), 
            "GridMapThrottler started: subscribing to '%s', publishing to '%s' at %.1f Hz",
            input_topic.c_str(), output_topic.c_str(), publish_rate);
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_map_ = msg;
        has_new_data_ = true;
    }

    void timerCallback()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!latest_map_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "No map data received yet...");
            return;
        }

        // 复制消息并更新时间戳
        auto msg_to_publish = *latest_map_;
        msg_to_publish.header.stamp = this->now();
        
        // 可选：添加降频标记
        msg_to_publish.header.frame_id = latest_map_->header.frame_id;
        
        publisher_->publish(msg_to_publish);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Published map at %.3f (original stamp: %.3f)", 
            rclcpp::Time(msg_to_publish.header.stamp).seconds(),
            rclcpp::Time(latest_map_->header.stamp).seconds());
        
        has_new_data_ = false;
    }

    // 成员变量
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    bool has_new_data_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GridMapThrottler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
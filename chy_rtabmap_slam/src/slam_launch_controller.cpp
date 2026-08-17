// SLAM 启动控制节点
// 订阅 ardupilot_msgs::MyVec3 消息，根据 z 值控制 launch 文件的启停：
//   z ≈ 0 ：停止所有 launch 文件
//   z ≈ 1 ：运行 slam.launch.py + rtabmap_nav2_bringup.launch.py
//   z ≈ 2 ：仅运行 slam.launch.py

#include <rclcpp/rclcpp.hpp>
#include <ardupilot_msgs/msg/my_vec3.hpp>

// POSIX 进程管理头文件，用于 fork/exec/kill/waitpid 控制子进程
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <chrono>
#include <thread>

class SlamLaunchController : public rclcpp::Node
{
public:
  SlamLaunchController()
  : Node("slam_launch_controller"),
    slam_pid_(-1),       // -1 表示无活跃子进程
    nav2_pid_(-1),
    current_mode_(0)     // 初始模式为 0（全部停止）
  {
    // ---- 声明可配置参数 ----
    this->declare_parameter<std::string>("topic_name", "slam_control_cmd");
    this->declare_parameter<std::string>("launch_package", "chy_rtabmap_slam");
    this->declare_parameter<std::string>("slam_launch_file", "slam.launch.py");
    this->declare_parameter<std::string>("nav2_launch_file", "rtabmap_nav2_bringup.launch.py");
    this->declare_parameter<double>("mode_transition_delay_sec", 1.0);

    // ---- 读取参数 ----
    topic_name_ = this->get_parameter("topic_name").as_string();
    launch_package_ = this->get_parameter("launch_package").as_string();
    slam_launch_file_ = this->get_parameter("slam_launch_file").as_string();
    nav2_launch_file_ = this->get_parameter("nav2_launch_file").as_string();
    mode_transition_delay_sec_ = this->get_parameter("mode_transition_delay_sec").as_double();

    // ---- 订阅 MyVec3 话题，回调中根据 z 值切换模式 ----
    sub_ = this->create_subscription<ardupilot_msgs::msg::MyVec3>(
      topic_name_, rclcpp::QoS(10),
      std::bind(&SlamLaunchController::vec3_callback, this, std::placeholders::_1));

    // ---- 定时器：每 2 秒检查子进程是否意外退出 ----
    check_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&SlamLaunchController::check_processes, this));

    RCLCPP_INFO(
      this->get_logger(),
      "SlamLaunchController started on topic '%s' | modes: z=0 stop all, z=1 slam+nav2, z=2 slam only",
      topic_name_.c_str());
  }

  // 析构时确保所有子进程被终止，防止孤儿进程
  ~SlamLaunchController()
  {
    stop_process(slam_pid_, "SLAM");
    stop_process(nav2_pid_, "Nav2");
  }

private:
  // ========================================================================
  // 消息回调：解析 z 值 → 模式，按需启停 launch 进程
  // ========================================================================
  void vec3_callback(const ardupilot_msgs::msg::MyVec3::SharedPtr msg)
  {
    // 将 float 的 z 值量化为整数模式
    // z > 1.5 → mode 2（仅 SLAM）
    // z > 0.5 → mode 1（SLAM + Nav2）
    // 其他    → mode 0（全部停止）
    int new_mode = 0;
    if (msg->z > 1.5f) {
      new_mode = 2;
    } else if (msg->z > 0.5f) {
      new_mode = 1;
    } else {
      new_mode = 0;
    }

    // 模式未变化，无需操作
    if (new_mode == current_mode_) {
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Mode change: %d -> %d (z=%.2f)", current_mode_, new_mode, msg->z);

    // 计算新模式需要哪些进程
    bool need_slam = (new_mode == 1 || new_mode == 2);  // mode 1 和 2 都需要 SLAM
    bool need_nav2 = (new_mode == 1);                    // 仅 mode 1 需要 Nav2

    // 查询当前哪些进程正在运行（PID > 0 表示活跃）
    bool slam_active = (slam_pid_ > 0);
    bool nav2_active = (nav2_pid_ > 0);

    // ---- 第一步：停止不再需要的进程 ----

    // SLAM 正在运行但新模式不需要 → 停止 SLAM
    if (slam_active && !need_slam) {
      stop_process(slam_pid_, "SLAM");
      slam_pid_ = -1;
    }

    // Nav2 正在运行但新模式不需要 → 停止 Nav2
    if (nav2_active && !need_nav2) {
      stop_process(nav2_pid_, "Nav2");
      nav2_pid_ = -1;
    }

    // ---- 第二步：启动需要的进程 ----

    // 如果 SLAM 已在运行且仍需要，且 Nav2 不活跃但需要 → 仅启动 Nav2
    // （例如 mode 2→1：SLAM 保持，追加 Nav2）
    if (slam_active && need_slam && !nav2_active && need_nav2) {
      // SLAM already running, just start Nav2
    } else if (need_slam && !slam_active) {
      // SLAM 不在运行但需要 → 启动前等待一小段时间（让之前的进程完全退出）
      if (mode_transition_delay_sec_ > 0) {
        std::this_thread::sleep_for(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(mode_transition_delay_sec_)));
      }
      slam_pid_ = start_process(slam_launch_file_, "SLAM");
    }

    // Nav2 不在运行但需要 → 启动 Nav2
    if (need_nav2 && !nav2_active) {
      nav2_pid_ = start_process(nav2_launch_file_, "Nav2");
    }

    // 更新当前模式
    current_mode_ = new_mode;
  }

  // ========================================================================
  // 启动一个 launch 文件作为独立子进程
  // 返回子进程 PID（父进程中），或在 fork/execlp 失败时返回 -1
  // ========================================================================
  pid_t start_process(const std::string & launch_file, const std::string & label)
  {
    // fork() 创建子进程：
    //   返回 0  → 子进程分支
    //   返回 >0 → 父进程分支，返回值为子进程的 PID
    //   返回 -1 → 失败
    pid_t pid = fork();
    if (pid == 0) {
      // ---- 子进程分支 ----

      // 关闭从父进程继承的所有文件描述符（fd 0/1/2 为标准 I/O 保留）
      // 防止子进程占用父进程的 DDS socket、管道等，避免 ROS2 通信冲突
      for (int fd = 3; fd < 1024; ++fd) {
        close(fd);
      }

      // 将子进程放入新的独立进程组（PGID = 子进程自身 PID）
      // 后续 kill(-pgid, signal) 可一次性终止整个进程组
      // （包括 ros2 launch 产生的 rtabmap、stereo_odometry 等孙进程）
      setpgid(0, 0);

      // execlp 通过 PATH 搜索并执行 "ros2 launch <package> <file>"
      // 参数格式：程序名, argv[0], argv[1], argv[2], argv[3], ..., NULL
      // execlp 成功时不返回（进程映像被替换），失败时返回 -1
      execlp(
        "ros2", "ros2", "launch",
        launch_package_.c_str(), launch_file.c_str(),
        static_cast<char *>(nullptr));

      // 若 execlp 返回说明执行失败
      // 使用 _exit 而非 exit，避免刷新父进程的 stdio 缓冲区
      // 127 是惯例的"命令未找到"退出码
      _exit(127);
    } else if (pid > 0) {
      // ---- 父进程分支：fork 成功 ----

      // 在父进程中也调用 setpgid，与子进程中的 setpgid 形成双保险
      // 避免竞态条件导致子进程仍留在父进程的进程组中
      setpgid(pid, pid);

      RCLCPP_INFO(
        this->get_logger(),
        "%s launch started (PID: %d, PGID: %d, file: %s)",
        label.c_str(), pid, pid, launch_file.c_str());

      // 返回子进程 PID 供后续 waitpid/kill 使用
      return pid;
    } else {
      // ---- fork 失败 ----
      RCLCPP_ERROR(this->get_logger(), "fork() failed for %s", label.c_str());
      return -1;
    }
  }

  // ========================================================================
  // 停止一个 launch 子进程（通过引用修改调用者的 pid 变量）
  // 流程：SIGTERM 优雅终止 → 等待 5 秒 → SIGKILL 强制终止
  // ========================================================================
  void stop_process(pid_t & pid, const std::string & label)
  {
    // PID 无效，无需操作
    if (pid <= 0) {
      return;
    }

    // 向整个进程组发送 SIGTERM（负 PID 表示向进程组发信号）
    // 这样 ros2 launch 产生的所有孙节点都会收到终止信号
    kill(-pid, SIGTERM);

    // 轮询等待子进程退出，最多等待 50 × 100ms = 5 秒
    for (int i = 0; i < 50; ++i) {
      int status;
      // WNOHANG：非阻塞模式，子进程未退出时立即返回 0
      pid_t ret = waitpid(pid, &status, WNOHANG);
      if (ret == pid) {
        // waitpid 返回子进程 PID → 子进程已正常退出（SIGTERM 生效）
        RCLCPP_INFO(this->get_logger(), "%s stopped gracefully (PID: %d)", label.c_str(), pid);
        pid = -1;
        return;
      }
      if (ret == -1 && errno == ECHILD) {
        // ECHILD：子进程已不存在（可能被外部信号提前终止）
        RCLCPP_INFO(this->get_logger(), "%s process already gone (PID: %d)", label.c_str(), pid);
        pid = -1;
        return;
      }
      // 子进程尚未退出，等待 100ms 后再次检查
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 5 秒超时，子进程仍未退出 → SIGKILL 强制终止（不可被捕获或忽略）
    kill(-pid, SIGKILL);
    int status;
    // 阻塞等待子进程被回收，防止产生僵尸进程
    waitpid(pid, &status, 0);
    RCLCPP_WARN(this->get_logger(), "%s force-killed (PID: %d)", label.c_str(), pid);
    pid = -1;
  }

  // ========================================================================
  // 定时健康检查：检测子进程是否意外退出，并自动降级 current_mode_
  // ========================================================================
  void check_processes()
  {
    // 逐个检查 SLAM 和 Nav2 进程
    check_one_process(slam_pid_, "SLAM");
    check_one_process(nav2_pid_, "Nav2");

    // 根据进程存活状态自动降级模式
    bool slam_active = (slam_pid_ > 0);
    bool nav2_active = (nav2_pid_ > 0);

    if (current_mode_ == 1 && !slam_active && !nav2_active) {
      // mode 1 但两个进程都挂了 → 降级到 0
      current_mode_ = 0;
    } else if (current_mode_ == 2 && !slam_active) {
      // mode 2 但 SLAM 挂了 → 降级到 0
      current_mode_ = 0;
    } else if (current_mode_ == 1 && slam_active && !nav2_active) {
      // mode 1 但 Nav2 挂了（SLAM 还在）→ 降级到 2
      current_mode_ = 2;
    }
  }

  // ========================================================================
  // 检查单个子进程是否意外退出（非阻塞 waitpid）
  // ========================================================================
  void check_one_process(pid_t & pid, const std::string & label)
  {
    if (pid <= 0) {
      return;
    }

    int status;
    // WNOHANG 非阻塞：返回 0 表示仍在运行，返回 pid 表示已退出
    pid_t ret = waitpid(pid, &status, WNOHANG);
    if (ret == pid) {
      // 子进程已退出但未被回调处理 → 异常退出（crash 或被外部 kill）
      // WEXITSTATUS 提取子进程退出码
      RCLCPP_WARN(
        this->get_logger(),
        "%s process exited unexpectedly (PID: %d, status: %d)",
        label.c_str(), pid, WEXITSTATUS(status));
      pid = -1;
    }
    // ret == 0 → 子进程仍在运行，无需处理
  }

  // ========================================================================
  // 成员变量
  // ========================================================================

  // ROS2 订阅和定时器
  rclcpp::Subscription<ardupilot_msgs::msg::MyVec3>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr check_timer_;

  // 子进程 PID（-1 表示无活跃进程）
  pid_t slam_pid_;   // slam.launch.py 进程 PID
  pid_t nav2_pid_;   // rtabmap_nav2_bringup.launch.py 进程 PID

  // 当前运行模式：0=全部停止, 1=SLAM+Nav2, 2=仅SLAM
  int current_mode_;

  // 可配置参数缓存
  std::string topic_name_;                  // 订阅话题名
  std::string launch_package_;              // launch 文件所在 ROS2 包名
  std::string slam_launch_file_;            // SLAM launch 文件名
  std::string nav2_launch_file_;            // Nav2 launch 文件名
  double mode_transition_delay_sec_;        // 模式切换时启动前的等待秒数
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamLaunchController>());
  rclcpp::shutdown();
  return 0;
}

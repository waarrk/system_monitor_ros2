#include <sys/statvfs.h>  // statvfsのために追加
#include <sys/sysinfo.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <system_monitor_ros2/msg/system_monitor.hpp>
#include <thread>
#include <vector>

class SystemMonitorNode : public rclcpp::Node {
 public:
  SystemMonitorNode() : Node("system_monitor_node") {
    // Publisherを作成
    system_pub_ =
        this->create_publisher<system_monitor_ros2::msg::SystemMonitor>(
            "system_monitor", 10);

    // タイマーで定期的に状態を取得
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SystemMonitorNode::get_system_state, this));
  }

 private:
  void get_system_state() {
    auto msg = system_monitor_ros2::msg::SystemMonitor();

    // Time
    msg.time = this->now().seconds();

    // CPU情報の取得
    msg.cpu_count = sysconf(_SC_NPROCESSORS_ONLN);
    msg.cpu_percent = get_cpu_usage();
    msg.current_cpu_freq = get_cpu_freq();

    RCLCPP_INFO(this->get_logger(), "CPU Cores: %d", msg.cpu_count);
    RCLCPP_INFO(this->get_logger(), "CPU Usage: %.2f%%", msg.cpu_percent);
    RCLCPP_INFO(this->get_logger(), "CPU Frequency: %.2f MHz",
                msg.current_cpu_freq);

    // メモリ情報の取得
    struct sysinfo mem_info;
    sysinfo(&mem_info);
    msg.memory_total = mem_info.totalram / (1024 * 1024);  // MBに変換
    msg.memory_percent =
        100.0f * (mem_info.totalram - mem_info.freeram) / mem_info.totalram;

    RCLCPP_INFO(this->get_logger(), "Total Memory: %.2f MB", msg.memory_total);
    RCLCPP_INFO(this->get_logger(), "Memory Usage: %.2f%%", msg.memory_percent);

    // ディスク使用量の取得
    msg.disk_usage_percent = get_disk_usage("/");

    RCLCPP_INFO(this->get_logger(), "Disk Usage: %.2f%%",
                msg.disk_usage_percent);

    // センサー情報（例としてCPU温度）
    msg.sensors_temperatures = get_cpu_temperature();

    RCLCPP_INFO(this->get_logger(), "CPU Temperature: %.2f °C",
                msg.sensors_temperatures);

    // バッテリー情報（ここでは仮の値として0.0を使用）
    msg.sensors_battery = 0.0;

    // ノードリストの取得
    msg.rosnode_list = get_rosnode_list();

    // ノードリストを表示
    for (const auto &node_name : msg.rosnode_list) {
      RCLCPP_INFO(this->get_logger(), "ROS Node: %s", node_name.c_str());
    }

    // メッセージをパブリッシュ
    system_pub_->publish(msg);
  }

  float get_cpu_usage() {
    // CPU使用率を取得するためのメソッド
    std::ifstream stat_file("/proc/stat");
    std::string line;
    std::getline(stat_file, line);
    std::istringstream iss(line);
    std::string cpu;
    long user, nice, system, idle;
    iss >> cpu >> user >> nice >> system >> idle;
    return 100.0f * (user + nice + system) / (user + nice + system + idle);
  }

  float get_cpu_freq() {
    // CPU周波数の取得
    std::ifstream freq_file(
        "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq");
    float freq;
    if (freq_file >> freq) {
      return freq / 1000.0f;  // kHzからMHzに変換
    }
    return 0.0f;
  }

  float get_disk_usage(const std::string &path) {
    // ディスク使用率の取得
    struct statvfs stat;  // statvfs構造体
    if (statvfs(path.c_str(), &stat) != 0) {
      return 0.0f;  // エラーの場合は0
    }
    return 100.0f * (1.0f - (float)stat.f_bfree / (float)stat.f_blocks);
  }

  float get_cpu_temperature() {
    // CPU温度の取得（Raspberry Piなどの例として）
    std::ifstream temp_file("/sys/class/thermal/thermal_zone0/temp");
    float temp;
    temp_file >> temp;
    return temp / 1000.0f;  // ミリ度単位を度に変換
  }

  std::vector<std::string> get_rosnode_list() {
    std::vector<std::string> node_list;
    auto node_names = this->get_node_names();
    for (const auto &name : node_names) {
      node_list.push_back(name);
    }
    return node_list;
  }

  rclcpp::Publisher<system_monitor_ros2::msg::SystemMonitor>::SharedPtr
      system_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SystemMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

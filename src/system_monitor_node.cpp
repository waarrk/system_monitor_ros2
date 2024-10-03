#include <nvml.h>
#include <sys/statvfs.h>
#include <sys/sysinfo.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
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

    // NVMLの初期化
    nvmlInit();

    // タイマーで定期的に状態を取得
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SystemMonitorNode::get_system_state, this));
  }

  ~SystemMonitorNode() {
    // NVMLの終了
    nvmlShutdown();
  }

 private:
  void get_system_state() {
    auto msg = system_monitor_ros2::msg::SystemMonitor();

    // Time
    msg.time = this->now().seconds();

    // CPU情報の取得
    msg.cpu_name = get_cpu_name();
    msg.cpu_count = sysconf(_SC_NPROCESSORS_ONLN);
    msg.cpu_percent = get_cpu_usage();
    msg.current_cpu_freq = get_cpu_freq();

    RCLCPP_INFO(this->get_logger(), "CPU Name: %s", msg.cpu_name.c_str());
    RCLCPP_INFO(this->get_logger(), "CPU Cores: %d", msg.cpu_count);
    RCLCPP_INFO(this->get_logger(), "CPU Usage: %.2f%%", msg.cpu_percent);
    RCLCPP_INFO(this->get_logger(), "CPU Frequency: %.2f MHz",
                msg.current_cpu_freq);

    // メモリ情報の取得
    struct sysinfo mem_info;
    sysinfo(&mem_info);
    msg.memory_total = mem_info.totalram / (1024 * 1024);  // MB
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

    // バッテリー情報の取得
    msg.sensors_battery = get_battery_level();

    RCLCPP_INFO(this->get_logger(), "Battery Level: %.2f%%",
                msg.sensors_battery);

    // GPU情報の取得
    msg.gpu_name = get_gpu_name();
    msg.gpu_usage_percent = get_gpu_usage();
    msg.gpu_temperature = get_gpu_temperature();

    RCLCPP_INFO(this->get_logger(), "GPU Name: %s", msg.gpu_name.c_str());
    RCLCPP_INFO(this->get_logger(), "GPU Usage: %.2f%%", msg.gpu_usage_percent);
    RCLCPP_INFO(this->get_logger(), "GPU Temperature: %.2f °C",
                msg.gpu_temperature);

    // ノードリストの取得
    msg.rosnode_list = get_rosnode_list();

    // ノードリストを表示
    for (const auto &node_name : msg.rosnode_list) {
      RCLCPP_INFO(this->get_logger(), "ROS Node: %s", node_name.c_str());
    }

    // メッセージをパブリッシュ
    system_pub_->publish(msg);
  }

  std::string get_cpu_name() {
    // CPUの名前を取得
    std::ifstream cpuinfo_file("/proc/cpuinfo");
    std::string line;
    std::string cpu_name;
    while (std::getline(cpuinfo_file, line)) {
      if (line.find("model name") != std::string::npos) {
        cpu_name = line.substr(line.find(":") + 2);
        break;
      }
    }
    return cpu_name;
  }

  float get_cpu_usage() {
    // CPU使用率を取得
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
      return freq / 1000.0f;  // kHz to MHz
    }
    return 0.0f;
  }

  float get_disk_usage(const std::string &path) {
    // ディスク使用率の取得
    struct statvfs stat;
    if (statvfs(path.c_str(), &stat) != 0) {
      return 0.0f;
    }
    return 100.0f * (1.0f - (float)stat.f_bfree / (float)stat.f_blocks);
  }

  float get_battery_level() {
    std::ifstream battery_file("/sys/class/power_supply/BAT0/capacity");
    float battery_level;
    if (battery_file >> battery_level) {
      return battery_level;
    }
    return 0.0f;
  }

  float get_cpu_temperature() {
    // CPU温度の取得
    std::ifstream temp_file("/sys/class/thermal/thermal_zone0/temp");
    float temp;
    temp_file >> temp;
    return temp / 1000.0f;
  }

  std::string get_gpu_name() {
    // NVMLを使ってGPUの名前を取得
    nvmlDevice_t device;
    char gpu_name[NVML_DEVICE_NAME_BUFFER_SIZE];
    nvmlDeviceGetHandleByIndex(0, &device);
    nvmlDeviceGetName(device, gpu_name, NVML_DEVICE_NAME_BUFFER_SIZE);
    return std::string(gpu_name);
  }

  float get_gpu_usage() {
    // NVMLを使ってGPU使用率を取得
    nvmlDevice_t device;
    nvmlUtilization_t utilization;
    nvmlDeviceGetHandleByIndex(0, &device);
    nvmlDeviceGetUtilizationRates(device, &utilization);
    return utilization.gpu;
  }

  float get_gpu_temperature() {
    // NVMLを使ってGPU温度を取得
    nvmlDevice_t device;
    unsigned int temperature;
    nvmlDeviceGetHandleByIndex(0, &device);
    nvmlDeviceGetTemperature(device, NVML_TEMPERATURE_GPU, &temperature);
    return static_cast<float>(temperature);
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

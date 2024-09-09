# system_monitor_ros2

このリポジトリは，システム状態の監視を目的としたROS2ノードです．

[haruyama8940/system_monitor_ros](https://github.com/haruyama8940/system_monitor_ros/)
（Copyright (c) 2023, haruyama kenta）をベースにしています．

## 実行方法

```bash
colcon build
ros2 run system_monitor_ros2 system_monitor_ros2
source install/setup.bash
```

## 実行結果

```bash
ros2 topic echo /system_monitor
```

```bash
---
time: 1725868541.2260795
cpu_count: 16
cpu_percent: 5.904842376708984
current_cpu_freq: 400.0
memory_total: 31749.0
memory_percent: 66.86810302734375
disk_usage_percent: 4.939990997314453
sensors_temperatures: 51.0
sensors_battery: 90.0
rosnode_list:
- /system_monitor_node
- /_ros2cli_70084
- /_ros2cli_daemon_30_03e87bc0a76542f6ae2693fcbdf7f869
---
time: 1725868542.2261872
cpu_count: 16
cpu_percent: 5.903729438781738
current_cpu_freq: 400.0
memory_total: 31749.0
memory_percent: 66.42782592773438
disk_usage_percent: 4.939990997314453
sensors_temperatures: 51.0
sensors_battery: 90.0
rosnode_list:
- /system_monitor_node
- /_ros2cli_70084
- /_ros2cli_daemon_30_03e87bc0a76542f6ae2693fcbdf7f869
```
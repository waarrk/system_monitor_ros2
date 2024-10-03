# Time information
float64 time

# CPU Information
string cpu_name
int8 cpu_count           # Number of CPU cores
float32 cpu_percent      # System-wide CPU utilization as a percentage
float32 current_cpu_freq # Current CPU frequency

# GPU Information
string gpu_name
float32 gpu_usage_percent
float32 gpu_temperature

# Memory Information
float32 memory_total     # Total memory available (MB)
float32 memory_percent   # Percentage of memory usage

# Disk Information
float32 disk_usage_percent # Disk usage as a percentage

# Sensor Information
float32 sensors_temperatures # Sensor temperatures
float32 sensors_battery      # Battery status (percentage)

# ROS Node List
string[] rosnode_list     # List of active ROS nodes

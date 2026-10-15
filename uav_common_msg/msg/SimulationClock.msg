# Gazebo's simultaneous simulation/system timestamps; header is bridge ROS receipt.
# Does not change /clock or any node's use_sim_time setting.
std_msgs/Header header
builtin_interfaces/Time sim_time
builtin_interfaces/Time system_time

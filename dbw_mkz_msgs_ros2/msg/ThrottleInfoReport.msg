std_msgs/Header header

# Throttle Pedal
float32 throttle_pc   # Throttle pedal percent, range 0 to 1
float32 throttle_rate # Throttle pedal change per second (1/s)
dbw_mkz_msgs_ros2/QualityFactor throttle_pedal_qf # Non-zero is limp-home

# Engine
float32 engine_rpm    # Engine Speed (rpm)

# Gear Num
dbw_mkz_msgs_ros2/GearNum gear_num      # Gear Number

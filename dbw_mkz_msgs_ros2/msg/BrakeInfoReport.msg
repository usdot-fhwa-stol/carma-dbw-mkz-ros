std_msgs/Header header

# Wheel torques (Nm)
float32 brake_torque_request
float32 brake_torque_actual
float32 wheel_torque_actual

# Vehicle Acceleration (m/s^2)
float32 accel_over_ground

# Brake Pedal
dbw_mkz_msgs_ros2/QualityFactor brake_pedal_qf

# Hill Start Assist
dbw_mkz_msgs_ros2/HillStartAssist hsa

# Anti-lock Brakes
bool abs_active
bool abs_enabled

# Stability Control
bool stab_active
bool stab_enabled

# Traction Control
bool trac_active
bool trac_enabled

# Parking Brake
dbw_mkz_msgs_ros2/ParkingBrake parking_brake

# Misc
bool stationary

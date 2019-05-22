#include <sensor_msgs/Imu.h>
#include <dataspeed_rostest/MsgRx.h>
#include <dbw_mkz_can/dispatch.h>
#include <dbw_mkz_can/PlatformMap.h>

/*
 * Message type templated class for ros CAN message subscription wrapper with analytics used for testing
 */
class MsgRxIMU : public dataspeed_rostest::MsgRx<sensor_msgs::Imu> {
public:
  MsgRxIMU() {};
  MsgRxIMU(ros::NodeHandle &nh, std::string topic_name, const float thresh = maxDuration()) : MsgRx(nh, topic_name, thresh) { };

  /*
   * Version
   */
  bool validData() const {
    return valid_orientation_w
      && valid_orientation_x
      && valid_orientation_y
      && valid_orientation_z
      && valid_angular_velocity_x
      && valid_angular_velocity_y
      && valid_angular_velocity_z
      && valid_linear_acceleration_x
      && valid_linear_acceleration_y
      && valid_linear_acceleration_z;
  }

protected:
  bool valid_orientation_w = false;
  bool valid_orientation_x = false;
  bool valid_orientation_y = false;
  bool valid_orientation_z = false;
  bool valid_angular_velocity_x = false;
  bool valid_angular_velocity_y = false;
  bool valid_angular_velocity_z = false;
  bool valid_linear_acceleration_x = false;
  bool valid_linear_acceleration_y = false;
  bool valid_linear_acceleration_z = false;

  void recv(const sensor_msgs::Imu::ConstPtr& msg) {
    msg_ = *msg;
    stamp_ = ros::Time::now();

    valid_orientation_w = valid_orientation_w || (msg->orientation.w != 0);
    valid_orientation_x = valid_orientation_x || (msg->orientation.x != 0);
    valid_orientation_y = valid_orientation_y || (msg->orientation.y != 0);
    valid_orientation_z = valid_orientation_z || (msg->orientation.z != 0);
    valid_angular_velocity_x = valid_angular_velocity_x || (msg->angular_velocity.x != 0);
    valid_angular_velocity_y = valid_angular_velocity_y || (msg->angular_velocity.y != 0);
    valid_angular_velocity_z = valid_angular_velocity_z || (msg->angular_velocity.z != 0);
    valid_linear_acceleration_x = valid_linear_acceleration_x || (msg->linear_acceleration.x != 0);
    valid_linear_acceleration_y = valid_linear_acceleration_y || (msg->linear_acceleration.y != 0);
    valid_linear_acceleration_y = valid_linear_acceleration_y || (msg->linear_acceleration.z != 0);
  }
};

std::function<bool(const std::shared_ptr<MsgRxIMU>)> imuValid = [](const std::shared_ptr<MsgRxIMU> t) -> bool { return t->validData(); };
#include <can_msgs/Frame.h>
#include <dataspeed_rostest/MsgRx.h>
#include <dbw_mkz_can/dispatch.h>
#include <dbw_mkz_can/PlatformMap.h>

/*
 * Message type templated class for ros CAN message subscription wrapper with analytics used for testing
 */
class MsgRxCAN : public dataspeed_rostest::MsgRx<can_msgs::Frame> {
public:
  MsgRxCAN() {};
  MsgRxCAN(ros::NodeHandle &nh, std::string topic_name, const float thresh = maxDuration()) : MsgRx(nh, topic_name, thresh) { };

  /*
   * Version
   */
  bool freshVersion(ros::Duration delta) const {
    return ageVersion() < delta;
  }
  bool freshVersion() const {
    return freshVersion(dur_);
  }
  const dbw_mkz_can::PlatformMap getVersions() const {
    return platform_map_;
  }
  ros::Duration ageVersion() const {
    return !stamp_version_.isZero() ? ros::Time::now() - stamp_version_ : ros::DURATION_MAX;
  }
  const ros::Time& stampVersion() const {
    return stamp_version_;
  }

protected:
  // Use PlatformMap
  // Insert to overwrite
  // All 0's == non-existant
  dbw_mkz_can::PlatformMap platform_map_;
  ros::Time stamp_version_;

  void recv(const can_msgs::Frame::ConstPtr& msg) {
    msg_ = *msg;
    stamp_ = ros::Time::now();

    if (!msg->is_rtr && !msg->is_error && !msg->is_extended) {
      switch (msg->id) {
        case dbw_mkz_can::ID_VERSION:
          if (msg->dlc >= sizeof(dbw_mkz_can::MsgVersion)) {
            dbw_mkz_can::MsgVersion msg_version;
            memcpy(&msg_version, msg->data.elems, sizeof(msg_version));
            platform_map_.insert(dbw_mkz_can::PlatformVersion((dbw_mkz_can::Platform)msg_version.platform, (dbw_mkz_can::Module)msg_version.module,
              msg_version.major, msg_version.minor, msg_version.build));
            stamp_version_ = ros::Time::now();
          }
          break;
        default:
          break;
      }
    }
  }
};

std::function<bool(const std::shared_ptr<MsgRxCAN>)> versionReady    = [](const std::shared_ptr<MsgRxCAN> t) -> bool { return t->freshVersion(); };
std::function<bool(const std::shared_ptr<MsgRxCAN>)> versionNotReady = [](const std::shared_ptr<MsgRxCAN> t) -> bool { return !t->freshVersion(); };
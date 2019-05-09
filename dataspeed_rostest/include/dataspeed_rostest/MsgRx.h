/*********************************************************************
 * Software License Agreement (Proprietary and Confidential)
 *
 *  Copyright (c) 2017-2018, Dataspeed Inc.
 *  All rights reserved.
 *
 *  NOTICE:  All information contained herein is, and remains the
 *  property of Dataspeed Inc. The intellectual and technical concepts
 *  contained herein are proprietary to Dataspeed Inc. and may be
 *  covered by U.S. and Foreign Patents, patents in process, and are
 *  protected by trade secret or copyright law. Dissemination of this
 *  information or reproduction of this material is strictly forbidden
 *  unless prior written permission is obtained from Dataspeed Inc.
 *********************************************************************/

#ifndef _DATASPEED_ROSTEST_MSG_RX_H_
#define _DATASPEED_ROSTEST_MSG_RX_H_

#include <ros/ros.h>

namespace dataspeed_rostest {

/*
 * Base class for ros message subscription wrapper
 */
class MsgRxb {
 public:
  virtual ~MsgRxb() {};
  virtual bool fresh(ros::Duration delta) const = 0;
  virtual bool fresh() const = 0;
  virtual ros::Duration age() const = 0;
  virtual const ros::Time& stamp() const = 0;
  virtual uint32_t getNumPublishers() const = 0;
  virtual std::string topic() const = 0;
  std::string name() { return topic(); } // generic version for service compatibility
};

/*
 * Message type templated class for ros message subscription wrapper with analytics used for testing
 */
template <typename MsgT>
class MsgRx : public MsgRxb {
public:
  /*
   * Unitialized subscription wrapper
   */
  MsgRx() {};
  /*
   * Create a ros message subscriber on @topic_name with testing analytics. @thresh specifies the expected message period.
   * If thresh is unspecified we want fresh() to report true once one message is received. Setting thresh = big number != ros::DURATION_MAX
   * allows this to work for practical testing.
   */ 
  MsgRx(ros::NodeHandle &nh, std::string topic_name, const float thresh = maxDuration()) :
    sub_(nh.subscribe<MsgT>(topic_name, 2, &MsgRx<MsgT>::recv, this)),
    dur_(thresh)
  {
  };
  MsgRx(ros::NodeHandle &nh, std::string topic_name, const float thresh, const MsgT& msg) :
    MsgRx(nh, topic_name, thresh)
  {
    set(msg);
  };
  /*
   * Was a message received recently?
   */
  bool fresh(ros::Duration delta) const {
    return age() < delta;
  }
  bool fresh() const {
    return fresh(dur_);
  }
  /*
   * Age of the last message
   */
  ros::Duration age() const {
    return !stamp_.isZero() ? ros::Time::now() - stamp_ : ros::DURATION_MAX;
  }
  /*
   * Get the latest message
   */
  const MsgT& get() const {
    return msg_;
  }
  /*
   * Timestamp of the latest message
   */
  const ros::Time& stamp() const {
    return stamp_;
  }
  /*
   * Number of publishers connected
   *
   * can be used to detect if the publishing node is up and usable
   */
  uint32_t getNumPublishers() const { return sub_.getNumPublishers(); }
  /*
   * Topic name
   */
  std::string topic() const { return sub_.getTopic(); }
protected:
  virtual void recv(const typename MsgT::ConstPtr& msg) { msg_ = *msg; stamp_ = ros::Time::now(); }
  ros::Subscriber sub_;
  ros::Duration dur_;
  ros::Time stamp_;
  MsgT msg_;
  static float maxDuration() {
    return (ros::DURATION_MAX * 0.5).toSec(); // not a constant at compile time :/
  }
};

} // namespace dataspeed_rostest

#endif // _DATASPEED_ROSTEST_MSG_RX_H_

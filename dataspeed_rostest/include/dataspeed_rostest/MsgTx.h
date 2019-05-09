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

#ifndef _DATASPEED_ROSTEST_MSG_TX_H_
#define _DATASPEED_ROSTEST_MSG_TX_H_

#include <ros/ros.h>

namespace dataspeed_rostest {

/*
 * Base class for ros message publisher wrapper
 */
class MsgTxb {
 public:
  virtual ~MsgTxb() {};
  virtual void clr() = 0;
  virtual void enable() = 0;
  virtual void disable() = 0;
  virtual bool enabled() = 0;
  virtual uint32_t getNumSubscribers() const = 0;
  virtual std::string topic() const = 0;
  std::string name() { return topic(); }
};

/*
 * Message type templated class for ros message publisher wrapper with analytics used for testing
 */
template <typename MsgT>
class MsgTx : public MsgTxb {
public:
  /*
   * Unitialized publisher wrapper
   */
  MsgTx() : enable_(false) {}

  /*
   * Create a ros message publisher on @topic_name with testing analytics.
   */
  MsgTx(ros::NodeHandle &nh, std::string topic_name, const bool latched = false) :
    enable_(false),
    pub_(nh.advertise<MsgT>(topic_name, 2, latched))
  {
  }
  MsgTx(ros::NodeHandle &nh, std::string topic_name, const bool latched, const MsgT& msg) :
    enable_(false),
    pub_(nh.advertise<MsgT>(topic_name, 2, latched))
  {
    set(msg);
  };
  /*
   * Get the latest message
   */
  MsgT& get() {
    return msg_;
  }
  /*
   * Number of subscribers connected
   *
   * can be used to detect if the subscribing node is up and usable
   */
  uint32_t getNumSubscribers() const {
    return pub_.getNumSubscribers();
  }
  /*
   * Published topic name
   */
  std::string topic() const {
    return pub_.getTopic();
  }
  /*
   * Set the message to publish
   */
  void set(const MsgT& msg) {
    msg_ = msg;
    enable_ = true;
  }
  /*
   * Clear the message and disable the publisher
   */
  void clr() {
    msg_ = MsgT();
    disable();
  }
  /*
   * Enable publisher
   */
  void enable() {
    enable_ = true;
  }
  /*
   * Disable publisher
   */
  void disable() {
    enable_ = false;
  }
  /*
   * Publisher enabled?
   */
  bool enabled() {
    return enable_;
  }
  /*
   * Send the current message
   */
  void send() {
    if (enable_) {
      pub_.publish(msg_);
    }
  }
  /*
   * Publish a message
   */
  void publish(const MsgT& msg) {
    set(msg);
    send();
  }
private:
  bool enable_;
  ros::Publisher pub_;
  MsgT msg_;
};

} // namespace dataspeed_rostest

#endif // _DATASPEED_ROSTEST_MSG_TX_H_

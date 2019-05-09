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

#ifndef _WAIT_FOR_TOPICS_H_
#define _WAIT_FOR_TOPICS_H_

#include "MsgRx.h"
#include "MsgTx.h"
#include "Srv.h"

#include <ros/ros.h>
#include <functional> // std::function
#include <vector>
#include <memory> // std::shared_ptr
#include <sstream>

///@TODO: Add unit tests

namespace dataspeed_rostest {

/*
 * Wait for a condition to be true for @duration_s
 */
bool waitFor(std::function<bool ()> cond, double duration_s)
{
  ros::WallDuration dur(duration_s);
  const ros::WallTime start = ros::WallTime::now();
  while (true) {
    if(cond()) {
      return true;
    }
    if ((ros::WallTime::now() - start) > dur) {
      return false;
    }
    ros::WallDuration(0.001).sleep();
    ros::spinOnce();
  }
}
/*
 * Wait for a condition (@cond) to be true for a list of topics for @duration_s
 * debug adds information on topics that cause problems 
 */
template <typename T>
bool waitFor(std::function<bool (const std::shared_ptr<T>)> cond, std::vector<std::shared_ptr<T>> &topics, double duration_s, bool debug = false)
{
  ros::WallDuration dur(duration_s);
  const ros::WallTime start = ros::WallTime::now();
  while (true) {
    bool all = true;
    for(const std::shared_ptr<T> topic : topics) {
      all &= cond(topic);
    }
    if(all) {
      return true;
    }
    if ((ros::WallTime::now() - start) > dur) {
      if (debug) {
        std::stringstream ss;
        for(const std::shared_ptr<T> topic : topics) {
          if (!cond(topic)) {
            ss << topic->name() << ", ";
          }
        }
        ROS_WARN_STREAM(ss.str() << " don't match condition");
      }
      return false;
    }
    ros::WallDuration(0.001).sleep();
    ros::spinOnce();
  }
}
/*
 * Wait for a condition (@cond) to be true for a topic for @duration_s
 */
template <typename T>
  bool waitFor(std::function<bool (const std::shared_ptr<T>)> cond, std::shared_ptr<T> topic, double duration_s) {
  std::vector<std::shared_ptr<T>> topics = {topic};
  return waitFor(cond, topics, duration_s, false);
}

// helper functions for common conditions
std::function<bool(const std::shared_ptr<MsgRxb>)> subReady     = [](const std::shared_ptr<MsgRxb> t) -> bool { return t->getNumPublishers() == 1; };
std::function<bool(const std::shared_ptr<MsgRxb>)> subNotReady  = [](const std::shared_ptr<MsgRxb> t) -> bool { return t->getNumPublishers() == 0; };
std::function<bool(const std::shared_ptr<MsgTxb>)> pubReady     = [](const std::shared_ptr<MsgTxb> t) -> bool { return t->getNumSubscribers() == 1; };
std::function<bool(const std::shared_ptr<MsgTxb>)> pubNotReady  = [](const std::shared_ptr<MsgTxb> t) -> bool { return t->getNumSubscribers() == 0; };
std::function<bool(const std::shared_ptr<MsgRxb>)> dataReady    = [](const std::shared_ptr<MsgRxb> t) -> bool { return t->fresh(); };
std::function<bool(const std::shared_ptr<MsgRxb>)> dataNotReady = [](const std::shared_ptr<MsgRxb> t) -> bool { return !t->fresh(); };
std::function<bool(const std::shared_ptr<Srvb>)>   srvReady     = [](const std::shared_ptr<Srvb> t)   -> bool { return t->exists(); };
std::function<bool(const std::shared_ptr<Srvb>)>   srvNotReady  = [](const std::shared_ptr<Srvb> t)   -> bool { return !t->exists(); };

} // namespace dataspeed_rostest

#endif // _WAIT_FOR_TOPICS_H_

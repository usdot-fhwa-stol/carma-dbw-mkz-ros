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

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// ROS messages
#include <std_msgs/Empty.h>

// ROS services
#include <std_srvs/Trigger.h>

ros::NodeHandle *nh;

ros::ServiceServer srv_enable_pubs;
ros::ServiceServer srv_disable_pubs;
ros::ServiceServer srv_run_pubs;
ros::ServiceServer srv_stop_pubs;
ros::ServiceServer srv_enable_subs;
ros::ServiceServer srv_disable_subs;
ros::ServiceServer srv_enable_srvs;
ros::ServiceServer srv_disable_srvs;

ros::Publisher pub1, pub2;
ros::Subscriber sub1, sub2;
ros::Timer timer_pub;
const ros::Duration PUB_DUR(0.1);

bool enablePubs(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &) {
  pub1 = nh->advertise<std_msgs::Empty>("pub1", 2);
  pub2 = nh->advertise<std_msgs::Empty>("pub2", 2);
  return true;
}
bool disablePubs(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &) {
  pub1.shutdown();
  pub2.shutdown();
  return true;
}

static void subCb(const std_msgs::Empty::ConstPtr&) { }
bool enableSubs(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &) {
  sub1 = nh->subscribe("sub1", 2, subCb);
  sub2 = nh->subscribe("sub2", 2, subCb);
  return true;
}
bool disableSubs(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &) {
  sub1.shutdown();
  sub2.shutdown();
  return true;
}

static void pubCb(const ros::TimerEvent &) {
  std_msgs::Empty msg;
  if (pub1 && pub2) {
    pub1.publish(msg);
    pub2.publish(msg);
  }
}
bool runPubs(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &) {
  timer_pub = nh->createTimer(PUB_DUR, pubCb);
  return true;
}
bool stopPubs(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &) {
  timer_pub.stop();
  return true;
}

bool enableSrvs(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &) {
  srv_enable_pubs  = nh->advertiseService("enable_pubs",  enablePubs);
  srv_disable_pubs = nh->advertiseService("disable_pubs", disablePubs);
  srv_run_pubs     = nh->advertiseService("run_pubs",     runPubs);
  srv_stop_pubs    = nh->advertiseService("stop_pubs",    stopPubs);
  srv_enable_subs  = nh->advertiseService("enable_subs",  enableSubs);
  srv_disable_subs = nh->advertiseService("disable_subs", disableSubs);
  return true;
}
bool disableSrvs(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &) {
  srv_enable_pubs.shutdown();
  srv_disable_pubs.shutdown();
  srv_run_pubs.shutdown();
  srv_stop_pubs.shutdown();
  srv_enable_subs.shutdown();
  srv_disable_subs.shutdown();
  return true;
}

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "topics");
  nh = new ros::NodeHandle();

  // These services need to be always available
  srv_enable_srvs  = nh->advertiseService("enable_srvs",  enableSrvs);
  srv_disable_srvs = nh->advertiseService("disable_srvs", disableSrvs);

  ros::spin();

  nh->~NodeHandle();

  return 0;
}

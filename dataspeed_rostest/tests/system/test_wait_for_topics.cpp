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

// Google Test
#include <gtest/gtest.h>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <dataspeed_rostest/WaitForTopics.h>

// ROS messages
#include <std_msgs/Empty.h>

// ROS services
#include <std_srvs/Trigger.h>

using namespace dataspeed_rostest;

struct Topics {
  // Publishers
  std::shared_ptr<MsgTx<std_msgs::Empty> > pub1;
  std::shared_ptr<MsgTx<std_msgs::Empty> > pub2;
  std::vector<std::shared_ptr<MsgTxb> >    pubs { pub1, pub2 };
  // Subscribers
  std::shared_ptr<MsgRx<std_msgs::Empty> > sub1;
  std::shared_ptr<MsgRx<std_msgs::Empty> > sub2;
  std::vector<std::shared_ptr<MsgRxb> >    subs { sub1, sub2 };
  // Services
  std::shared_ptr<Srv<std_srvs::Trigger> > srv_enable_pubs;
  std::shared_ptr<Srv<std_srvs::Trigger> > srv_disable_pubs;
  std::shared_ptr<Srv<std_srvs::Trigger> > srv_run_pubs;
  std::shared_ptr<Srv<std_srvs::Trigger> > srv_stop_pubs;
  std::shared_ptr<Srv<std_srvs::Trigger> > srv_enable_subs;
  std::shared_ptr<Srv<std_srvs::Trigger> > srv_disable_subs;
  std::shared_ptr<Srv<std_srvs::Trigger> > srv_enable_srvs;
  std::shared_ptr<Srv<std_srvs::Trigger> > srv_disable_srvs;
  std::vector<std::shared_ptr<Srvb> >      srvs { srv_enable_pubs, srv_disable_pubs, srv_run_pubs, srv_stop_pubs, srv_enable_subs, srv_disable_subs };

  std::vector<std::shared_ptr<Srvb> >      base_srvs { srv_enable_srvs, srv_disable_srvs };

  Topics () { }
  Topics (ros::NodeHandle &nh) :
    pub1(std::make_shared<MsgTx<std_msgs::Empty> > (nh, "sub1")),
    pub2(std::make_shared<MsgTx<std_msgs::Empty> > (nh, "sub2")),
    sub1(std::make_shared<MsgRx<std_msgs::Empty> > (nh, "pub1", 0.25)),
    sub2(std::make_shared<MsgRx<std_msgs::Empty> > (nh, "pub2", 0.25)),
    srv_enable_pubs (std::make_shared<Srv<std_srvs::Trigger> > (nh, "enable_pubs")),
    srv_disable_pubs(std::make_shared<Srv<std_srvs::Trigger> > (nh, "disable_pubs")),
    srv_run_pubs    (std::make_shared<Srv<std_srvs::Trigger> > (nh, "run_pubs")),
    srv_stop_pubs   (std::make_shared<Srv<std_srvs::Trigger> > (nh, "stop_pubs")),
    srv_enable_subs (std::make_shared<Srv<std_srvs::Trigger> > (nh, "enable_subs")),
    srv_disable_subs(std::make_shared<Srv<std_srvs::Trigger> > (nh, "disable_subs")),
    srv_enable_srvs (std::make_shared<Srv<std_srvs::Trigger> > (nh, "enable_srvs")),
    srv_disable_srvs(std::make_shared<Srv<std_srvs::Trigger> > (nh, "disable_srvs"))
  {
  }
};
Topics g_t;
TEST(Unit, Services) {
  std_srvs::Trigger rr;
  ASSERT_TRUE(waitFor(srvReady, g_t.base_srvs, 3.0));
  ASSERT_FALSE(waitFor(srvReady, g_t.srvs, 1.0));
  ASSERT_TRUE(g_t.srv_enable_srvs->call(rr));
  ASSERT_TRUE(waitFor(srvReady, g_t.srvs, 1.0));
  ASSERT_TRUE(g_t.srv_disable_srvs->call(rr));
  ASSERT_TRUE(waitFor(srvNotReady, g_t.srvs, 1.0));
  ASSERT_TRUE(g_t.srv_enable_srvs->call(rr));
  ASSERT_FALSE(waitFor(srvNotReady, g_t.srvs, 1.0));

  ASSERT_TRUE(g_t.srv_disable_srvs->call(rr));
}
TEST(Unit, Subscribers) {
  std_msgs::Empty msg;
  std_srvs::Trigger rr;

  ASSERT_TRUE(g_t.srv_enable_srvs->call(rr));
  ASSERT_TRUE(waitFor(srvReady, g_t.srvs, 1.0));

  // check subReady/subNotReady correspond to topic advertisement
  EXPECT_TRUE(g_t.srv_disable_pubs->call(rr));
  EXPECT_FALSE(waitFor(subReady, g_t.subs, 1.0));
  EXPECT_TRUE(g_t.srv_enable_pubs->call(rr));
  EXPECT_TRUE(waitFor(subReady, g_t.subs, 1.0));
  EXPECT_TRUE(g_t.srv_disable_pubs->call(rr));
  EXPECT_TRUE(waitFor(subNotReady, g_t.subs, 1.0));
  EXPECT_TRUE(g_t.srv_enable_pubs->call(rr));
  EXPECT_TRUE(waitFor(subReady, g_t.subs, 1.0)); // wait for small transition delay to check inverse case
  EXPECT_FALSE(waitFor(subNotReady, g_t.subs, 1.0));

  // check dataReady/dataNotReady are independent of whether a topic is advertised
  EXPECT_TRUE(g_t.srv_disable_pubs->call(rr));
  EXPECT_TRUE(g_t.srv_stop_pubs->call(rr));
  EXPECT_TRUE(waitFor(dataNotReady, g_t.subs, 1.0));
  EXPECT_FALSE(waitFor(dataReady, g_t.subs, 1.0));
  EXPECT_TRUE(g_t.srv_enable_pubs->call(rr));
  EXPECT_TRUE(waitFor(dataNotReady, g_t.subs, 1.0));
  EXPECT_FALSE(waitFor(dataReady, g_t.subs, 1.0));

  // check dataReady/dataNotReady correspond to periodically published messages
  EXPECT_TRUE(g_t.srv_enable_pubs->call(rr));
  EXPECT_TRUE(g_t.srv_run_pubs->call(rr));
  EXPECT_TRUE(waitFor(dataReady, g_t.subs, 1.0));
  EXPECT_TRUE(g_t.srv_stop_pubs->call(rr));
  EXPECT_TRUE(waitFor(dataNotReady, g_t.subs, 1.0));
  EXPECT_TRUE(g_t.srv_run_pubs->call(rr));
  EXPECT_TRUE(waitFor(dataReady, g_t.subs, 1.0)); // wait for small transition delay to check inverse case
  EXPECT_FALSE(waitFor(dataNotReady, g_t.subs, 1.0));

  // check subReady/subNotReady are independent of publishing
  EXPECT_TRUE(waitFor(subReady, g_t.subs, 1.0));
  EXPECT_FALSE(waitFor(subNotReady, g_t.subs, 1.0));

  ASSERT_TRUE(g_t.srv_disable_srvs->call(rr));
}
TEST(Unit, Publishers) {
  std_msgs::Empty msg;
  std_srvs::Trigger rr;

  ASSERT_TRUE(g_t.srv_enable_srvs->call(rr));
  ASSERT_TRUE(waitFor(srvReady, g_t.srvs, 1.0));

  EXPECT_TRUE(g_t.srv_disable_subs->call(rr));
  EXPECT_TRUE(waitFor(pubNotReady, g_t.pubs, 1.0));
  EXPECT_FALSE(waitFor(pubReady, g_t.pubs, 1.0));
  EXPECT_TRUE(g_t.srv_enable_subs->call(rr));
  EXPECT_TRUE(waitFor(pubReady, g_t.pubs, 1.0));
  EXPECT_FALSE(waitFor(pubNotReady, g_t.pubs, 1.0));

  ASSERT_TRUE(g_t.srv_disable_srvs->call(rr));
}

int main(int argc, char **argv) {
  // Initialize Google Tests
  testing::InitGoogleTest(&argc, argv);

  // Initialize ROS
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  g_t = Topics(nh);

  // Setup Spinner
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Run all the tests that were declared with TEST()
  int result = RUN_ALL_TESTS();

  // Cleanup
  spinner.stop();
  nh.shutdown();

  // Return test result
  return result;
}

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

// ROS Messages
#include <std_msgs/String.h>

// rostest Helper classes
#include <dataspeed_rostest/MsgRx.h>
#include <dataspeed_rostest/MsgTx.h>
#include <dataspeed_rostest/Srv.h>
using namespace dataspeed_rostest;

// Regex
#include <regex>

// Parameters
bool        g_recording = false; // keep track of when we have an extra subscriber (rosbag record)

// Cancel all tests on critical error
bool g_exit = false;

// Topics
std::shared_ptr<MsgRx<std_msgs::String> > sub_vin;
std::shared_ptr<MsgRx<dbw_mkz_msgs::BrakeReport> > sub_brake;

/*
 * Test rig setup and configuration tests
 * These are run before any of the actual tests.
 * If any of these tests fail, the failure is "critical" and no other tests get run.
 */
class System : public testing::Test {
  // Cancel all tests on critical error
  virtual void SetUp() {
    ASSERT_FALSE(g_exit);
  }
  // Any error in a system test is critical!
  virtual void TearDown() {
    if (::testing::Test::HasFailure()) {
      g_exit = true;
    }
  }
};
class BasicTest : public testing::Test {
  // Cancel all tests on critical error
  virtual void SetUp() {
    ASSERT_FALSE(g_exit);
  }
};

void setupTopics(ros::NodeHandle nh) {
	sub_vin = std::make_shared<MsgRx<std_msgs::String> > (nh, "vin", 1.0);
}

// Transmit periodic messages
void timerCallback(__attribute__((unused)) const ros::TimerEvent& event) {
  //g_t.bpec.pub_cmd->send();
  //g_t.bpet.pub_cmd->send();
  //g_t.tpec.pub_cmd->send();
  //g_t.tpet.pub_cmd->send();
}

// Tests
TEST_F(System, Params)
{
  //None
}
TEST_F(System, Firmware)
{
  // Wait for subscribers to finish setup.
  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_vin, 5.0));
  // which subscriber isn't ready?
  EXPECT_EQ(1, sub_vin->getNumPublishers());

}
TEST_F(BasicTest, Vin)
{
  // Wait for subscribers to finish setup.
  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_vin, 5.0));
  // which subscriber isn't ready?
  EXPECT_EQ(1, sub_vin->getNumPublishers());

  // Ensure the VIN is valid.
  ros::WallDuration(0.3).sleep(); // Faults and commands are disabled for the first ~50ms at startup
  ASSERT_TRUE(sub_vin->fresh());
  ASSERT_NE(sub_vin->get().data.c_str(), nullptr);
}
//TEST_F(BasicTest, License)
//{
//  // Wait for subscribers to finish setup.
//  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_vin, 5.0));
//  // which subscriber isn't ready?
//  EXPECT_EQ(1, sub_vin->getNumPublishers());
//}
TEST_F(BasicTest, BrakeFaults)
{
  // Wait for subscribers to finish setup.
  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_brake, 5.0));
  // which subscriber isn't ready?
  EXPECT_EQ(1, sub_brake->getNumPublishers());

  // Ensure there are no brake faults.
  ros::WallDuration(0.3).sleep(); // Faults and commands are disabled for the first ~50ms at startup
  ASSERT_TRUE(sub_brake->fresh());
  ASSERT_FALSE(sub_brake->get().fault_ch1);
  ASSERT_FALSE(sub_brake->get().fault_ch2);
  ASSERT_FALSE(sub_brake->get().fault_power);
}
//TEST_F(System, License)
//{
//  // Wait for subscribers to finish setup.
//  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_vin, 5.0));
//  // which subscriber isn't ready?
//  // Check inputs and faults
//  ros::WallDuration(0.3).sleep(); // Faults and commands are disabled for the first ~50ms at startup
//  ASSERT_TRUE(g_t.tpec.sub_report->fresh());
//  ASSERT_FALSE(g_t.tpec.sub_report->get().fault_power);
//}
//TEST_F(System, License)
//{
//  // Wait for subscribers to finish setup.
//  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_vin, 5.0));
//  // which subscriber isn't ready?
//  EXPECT_EQ(1, sub_vin->getNumPublishers());
//}
//TEST_F(System, License)
//{
//  // Wait for subscribers to finish setup.
//  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_vin, 5.0));
//  // which subscriber isn't ready?
//  EXPECT_EQ(1, sub_vin->getNumPublishers());
//}
//TEST_F(System, License)
//{
//  // Wait for subscribers to finish setup.
//  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_vin, 5.0));
//  // which subscriber isn't ready?
//  EXPECT_EQ(1, sub_vin->getNumPublishers());
//}

int main(int argc, char **argv) {
  // Initialize Google Tests
  testing::InitGoogleTest(&argc, argv);

  // Initialize ROS
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // Parameters
  nh_priv.getParam("recording", g_recording);
  if (g_recording) { ROS_ERROR("recording"); }
  std::string platform_str;
  nh_priv.getParam("platform", platform_str);

  // Setup Timer
  ros::Timer timer = nh.createTimer(ros::Duration(0.02), &timerCallback);

  // Create ROS topics
  setupTopics(nh);

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

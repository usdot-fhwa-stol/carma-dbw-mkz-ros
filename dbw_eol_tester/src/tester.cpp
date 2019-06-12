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
#include <dbw_mkz_msgs/BrakeCmd.h>
#include <dbw_mkz_msgs/ThrottleCmd.h>
#include <dbw_mkz_msgs/SteeringCmd.h>
#include <dbw_mkz_msgs/GearCmd.h>
#include <dbw_mkz_msgs/BrakeInfoReport.h>
#include <dbw_mkz_msgs/BrakeReport.h>
#include <dbw_mkz_msgs/Gear.h>
#include <dbw_mkz_msgs/GearReport.h>
#include <dbw_mkz_msgs/ParkingBrake.h>
#include <dbw_mkz_msgs/SteeringReport.h>
#include <dbw_mkz_msgs/SurroundReport.h>
#include <dbw_mkz_msgs/ThrottleInfoReport.h>
#include <dbw_mkz_msgs/ThrottleReport.h>
#include <dbw_mkz_msgs/TirePressureReport.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

// rostest Helper classes
#include <dataspeed_rostest/MsgRx.h>
#include <dataspeed_rostest/MsgTx.h>
#include <dataspeed_rostest/Srv.h>
#include "MsgRxCAN.h"
#include "MsgRxIMU.h"
using namespace dataspeed_rostest;

#include <dbw_mkz_can/PlatformMap.h>

// Regex
#include <regex>

// Parameters
bool        g_recording = false; // keep track of when we have an extra subscriber (rosbag record)

// Cancel all tests on critical error
bool g_critical_fail = false;
bool g_actuate_fail = false;

// Topics
std::shared_ptr<MsgRxCAN> sub_can;
std::shared_ptr<MsgRxIMU> sub_imu;

std::shared_ptr<MsgRx<std_msgs::String> > sub_vin;
std::shared_ptr<MsgRx<dbw_mkz_msgs::BrakeReport> > sub_brake_report;
std::shared_ptr<MsgRx<dbw_mkz_msgs::ThrottleReport> > sub_throttle_report;
std::shared_ptr<MsgRx<dbw_mkz_msgs::SteeringReport> > sub_steering_report;
std::shared_ptr<MsgRx<dbw_mkz_msgs::SurroundReport> > sub_surround;
std::shared_ptr<MsgRx<dbw_mkz_msgs::BrakeInfoReport> > sub_brake_info_report;
std::shared_ptr<MsgRx<dbw_mkz_msgs::ThrottleInfoReport> > sub_throttle_info_report;
std::shared_ptr<MsgRx<dbw_mkz_msgs::GearReport> > sub_gear_report;
std::shared_ptr<MsgRx<dbw_mkz_msgs::TirePressureReport> > sub_tire;
std::shared_ptr<MsgRx<sensor_msgs::NavSatFix> > sub_gps_fix;
std::shared_ptr<MsgRx<geometry_msgs::TwistStamped> > sub_gps_vel;
std::shared_ptr<MsgRx<sensor_msgs::TimeReference> > sub_gps_time;

std::shared_ptr<MsgTx<std_msgs::Empty> > pub_dbw_enable;

std::shared_ptr<MsgTx<dbw_mkz_msgs::BrakeCmd> > pub_brake_cmd;
std::shared_ptr<MsgTx<dbw_mkz_msgs::ThrottleCmd> > pub_throttle_cmd;
std::shared_ptr<MsgTx<dbw_mkz_msgs::SteeringCmd> > pub_steer_cmd;
std::shared_ptr<MsgTx<dbw_mkz_msgs::GearCmd> > pub_gear_cmd;

dbw_mkz_can::PlatformMap FIRMWARE_LATEST({
  {dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_CD4, dbw_mkz_can::M_BPEC,  dbw_mkz_can::ModuleVersion(2,1,2))},
  //{dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_CD4, dbw_mkz_can::M_BPEC,  dbw_mkz_can::ModuleVersion(2,1,0))},
  {dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_CD4, dbw_mkz_can::M_TPEC,  dbw_mkz_can::ModuleVersion(2,1,2))},
  //{dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_CD4, dbw_mkz_can::M_TPEC,  dbw_mkz_can::ModuleVersion(2,1,0))},
  {dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_CD4, dbw_mkz_can::M_STEER, dbw_mkz_can::ModuleVersion(2,1,1))},
  //{dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_CD4, dbw_mkz_can::M_STEER, dbw_mkz_can::ModuleVersion(2,1,99))},
  {dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_CD4, dbw_mkz_can::M_SHIFT, dbw_mkz_can::ModuleVersion(2,1,2))},
  {dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_CD4, dbw_mkz_can::M_SHIFT, dbw_mkz_can::ModuleVersion(2,1,0))},
  {dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_P5,  dbw_mkz_can::M_TPEC,  dbw_mkz_can::ModuleVersion(1,0,2))},
  {dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_P5,  dbw_mkz_can::M_STEER, dbw_mkz_can::ModuleVersion(1,0,2))},
  {dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_P5,  dbw_mkz_can::M_SHIFT, dbw_mkz_can::ModuleVersion(1,0,2))},
  {dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_P5,  dbw_mkz_can::M_ABS,   dbw_mkz_can::ModuleVersion(1,0,2))},
  {dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_P5,  dbw_mkz_can::M_BOO,   dbw_mkz_can::ModuleVersion(1,0,2))},
  {dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_C1,  dbw_mkz_can::M_TPEC,  dbw_mkz_can::ModuleVersion(0,0,1))},
  {dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_C1,  dbw_mkz_can::M_STEER, dbw_mkz_can::ModuleVersion(0,0,1))},
  {dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_C1,  dbw_mkz_can::M_SHIFT, dbw_mkz_can::ModuleVersion(0,0,1))},
  {dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_C1,  dbw_mkz_can::M_ABS,   dbw_mkz_can::ModuleVersion(0,0,1))},
  {dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_C1,  dbw_mkz_can::M_BOO,   dbw_mkz_can::ModuleVersion(0,0,1))},
  {dbw_mkz_can::PlatformVersion(dbw_mkz_can::P_FORD_C1,  dbw_mkz_can::M_EPS,   dbw_mkz_can::ModuleVersion(0,0,1))},
});

/*
 * Test rig setup and configuration tests
 * These are run before any of the actual tests.
 * If any of these tests fail, the failure is "critical" and no other tests get run.
 */
class System : public testing::Test {
  // Cancel all tests on critical error
  virtual void SetUp() {
    ASSERT_FALSE(g_critical_fail) << "A critical failure has occurred in a previous system test.";
  }
  // Any error in a system test is critical!
  virtual void TearDown() {
    if (::testing::Test::HasFailure()) {
      g_critical_fail = true;
    }
  }
};
class BasicTest : public testing::Test {
  // Cancel all tests on critical error
  virtual void SetUp() {
    ASSERT_FALSE(g_critical_fail) << "A critical failure has occurred in a previous system test.";
  }
};
class CAN_DBW : public BasicTest { };
class CAN_HS1 : public BasicTest { };
class CAN_HS2 : public BasicTest { };
class CAN_PSCM : public BasicTest { };
class Actuate : public testing::Test {
  // Cancel all tests on critical error
  virtual void SetUp() {
    ASSERT_FALSE(g_critical_fail) << "A critical failure has occurred in a previous system test.";
    ASSERT_FALSE(g_actuate_fail) << "A critical failure has occurred in a previous actuation test.";
  }
  // Any error in an actuation test should prevent future tests.
  virtual void TearDown() {
    if (::testing::Test::HasFailure()) {
      g_actuate_fail = true;
    }
  }
};

void setupTopics(ros::NodeHandle nh) {
  // TO-DO: Make these namespaces more flexible. See other launch files?
  sub_can = std::make_shared<MsgRxCAN> (nh, "/can_bus_dbw/can_rx", 10.0);
  sub_imu = std::make_shared<MsgRxIMU> (nh, "/vehicle/imu/data_raw", 10.0);

  sub_vin = std::make_shared<MsgRx<std_msgs::String> > (nh, "/vehicle/vin", 10.0);
  sub_brake_report = std::make_shared<MsgRx<dbw_mkz_msgs::BrakeReport> > (nh, "/vehicle/brake_report", 10.0);
  sub_throttle_report = std::make_shared<MsgRx<dbw_mkz_msgs::ThrottleReport> > (nh, "/vehicle/throttle_report", 10.0);
  sub_surround = std::make_shared<MsgRx<dbw_mkz_msgs::SurroundReport> > (nh, "/vehicle/surround_report", 10.0);
  sub_steering_report = std::make_shared<MsgRx<dbw_mkz_msgs::SteeringReport> > (nh, "/vehicle/steering_report", 10.0);
  sub_brake_info_report = std::make_shared<MsgRx<dbw_mkz_msgs::BrakeInfoReport> > (nh, "/vehicle/brake_info_report", 10.0);
  sub_throttle_info_report = std::make_shared<MsgRx<dbw_mkz_msgs::ThrottleInfoReport> > (nh, "/vehicle/throttle_info_report", 10.0);
  sub_gear_report = std::make_shared<MsgRx<dbw_mkz_msgs::GearReport> > (nh, "/vehicle/gear_report", 10.0);
  sub_tire = std::make_shared<MsgRx<dbw_mkz_msgs::TirePressureReport> > (nh, "/vehicle/tire_pressure_report", 10.0);
  sub_gps_fix = std::make_shared<MsgRx<sensor_msgs::NavSatFix> > (nh, "/vehicle/gps/fix", 10.0);
  sub_gps_vel = std::make_shared<MsgRx<geometry_msgs::TwistStamped> > (nh, "/vehicle/gps/vel", 10.0);
  sub_gps_time = std::make_shared<MsgRx<sensor_msgs::TimeReference> > (nh, "/vehicle/gps/time", 10.0);

  pub_dbw_enable = std::make_shared<MsgTx<std_msgs::Empty> > (nh, "/vehicle/enable");

  pub_brake_cmd = std::make_shared<MsgTx<dbw_mkz_msgs::BrakeCmd> > (nh, "/vehicle/brake_cmd");
  pub_throttle_cmd = std::make_shared<MsgTx<dbw_mkz_msgs::ThrottleCmd> > (nh, "/vehicle/throttle_cmd");
  pub_steer_cmd = std::make_shared<MsgTx<dbw_mkz_msgs::SteeringCmd> > (nh, "/vehicle/steering_cmd");
  pub_gear_cmd = std::make_shared<MsgTx<dbw_mkz_msgs::GearCmd> > (nh, "/vehicle/gear_cmd");
}

// Transmit periodic command messages
void timerCallback(__attribute__((unused)) const ros::TimerEvent& event) {
  pub_brake_cmd->send();
  pub_steer_cmd->send();
  pub_throttle_cmd->send();
  pub_gear_cmd->send();
}

// Tests
TEST_F(System, Params)
{
  // No parameters

  // Call the unused dispatchAssertSizes() function to get full test coverage
  dbw_mkz_can::dispatchAssertSizes();
}

TEST_F(System, CAN)
{
  // Wait for subscribers to finish setup.
  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_can, 5.0)) << "Could not connect to CAN topic. Is the DBW system powered on and connected?";
  // Which subscriber isn't ready?
  ASSERT_GE(sub_can->getNumPublishers(), 1) << "CAN topic has no publishers.";

  // Ensure the CAN data is received.
  EXPECT_TRUE(waitFor<MsgRxb>(dataReady, sub_can, 5.0)) << "Timeout expired waiting for CAN from the vehicle.";
  ASSERT_TRUE(sub_can->fresh()) << "Did not receive a CAN message from the vehicle.";

}

TEST_F(CAN_DBW, FirmwareVersion)
{
  // Wait for all the version messages to be sent.
  ros::Duration(3.0).sleep();

  // Ensure version data is received.
  EXPECT_TRUE(waitFor<MsgRxCAN>(versionReady, sub_can, 5.0)) << "Timeout expired waiting for version from the vehicle.";
  ASSERT_TRUE(sub_can->freshVersion()) << "Did not receive a version message from the vehicle.";

  std::vector<dbw_mkz_can::Platform> versionPlatforms = sub_can->getVersions().listPlatforms();
  ASSERT_GT(versionPlatforms.size(), 0) << "ERROR: No module versions received.";
  EXPECT_EQ(versionPlatforms.size(), 1) << "WARNING: " << versionPlatforms.size() << " platforms detected.";

  for (size_t i = 0; i < versionPlatforms.size(); i++) {
    dbw_mkz_can::Platform currentPlatform = versionPlatforms[i];
    bool ignoreSHIFT = (currentPlatform == dbw_mkz_can::P_FORD_CD4);

    std::vector<dbw_mkz_can::Module> knownModules = FIRMWARE_LATEST.listModules(currentPlatform);
    std::vector<dbw_mkz_can::Module> versionModules = sub_can->getVersions().listModules(currentPlatform);

    if (ignoreSHIFT) {
      // It's fine if knownModules == versionModules OR there is one less module present.
      if (versionModules.size() != knownModules.size() - 1) {
        ASSERT_EQ(knownModules.size(), versionModules.size()) << "ERROR: Expected " << knownModules.size() << " modules, found " << versionModules.size();
      }
    } else {
      ASSERT_EQ(knownModules.size(), versionModules.size()) << "ERROR: Expected " << knownModules.size() << " modules, found " << versionModules.size();
    }

    for (size_t j = 0; j < knownModules.size(); j++) {
      dbw_mkz_can::Module currentModule = knownModules[j]; 
      
      bool isSHIFT = (currentModule == dbw_mkz_can::M_SHIFT);

      if (ignoreSHIFT && isSHIFT) {
        // Doesn't matter if the module doesn't exist, but enforce it if it does.
        if (sub_can->getVersions().findModule(currentPlatform, currentModule).valid()) {
          // If the module exists, make sure the version isn't old.
          
          // Fetch the platform version from CAN.
          dbw_mkz_can::PlatformVersion platformVersion = sub_can->getVersions().findPlatform(currentPlatform, currentModule);
          // Fetch the static platform version.
          dbw_mkz_can::PlatformVersion platformLatest = FIRMWARE_LATEST.findPlatform(currentPlatform, currentModule);
  
          //Compare the static and CAN platform versions.
          EXPECT_TRUE(platformVersion == platformLatest)
            << "ERROR: Module " << moduleToString(currentModule) << " has unsupported version "
            << platformVersion.v.major() << "." << platformVersion.v.minor() << "." << platformVersion.v.build()
            << ", switch to "
            << platformLatest.v.major() << "." << platformLatest.v.minor() << "." << platformLatest.v.build();
        }
        // If the module isn't valid, no big deal.
        continue;
      } else {
        // Enforce existance and version of module.
        EXPECT_TRUE(sub_can->getVersions().findModule(currentPlatform, currentModule).valid()) << "ERROR: Module " << moduleToString(currentModule)
          << "(" << currentModule << ") has no valid version.";

        // Fetch the platform version from CAN.
        dbw_mkz_can::PlatformVersion platformVersion = sub_can->getVersions().findPlatform(currentPlatform, currentModule);
        // Fetch the static platform version.
        dbw_mkz_can::PlatformVersion platformLatest = FIRMWARE_LATEST.findPlatform(currentPlatform, currentModule);

        //Compare the static and CAN platform versions.
        EXPECT_TRUE(platformVersion == platformLatest)
          << "ERROR: Module " << moduleToString(currentModule) << "has unsupported version "
          << platformVersion.v.major() << "." << platformVersion.v.minor() << "." << platformVersion.v.build()
          << ", switch to "
          << platformLatest.v.major() << "." << platformLatest.v.minor() << "." << platformLatest.v.build();
      }

    }
  }
}

TEST_F(BasicTest, License)
{
  // Ensure license data is received.
  EXPECT_TRUE(waitFor<MsgRxCAN>(licenseReady, sub_can, 5.0)) << "Timeout expired waiting for license from the vehicle.";
  ASSERT_TRUE(sub_can->freshLicense()) << "Did not receive a license message from the vehicle.";
}

TEST_F(CAN_HS1, Vin)
{
  // Wait for subscribers to finish setup.
  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_vin, 5.0)) << "Could not connect to VIN topic.";
  // Which subscriber isn't ready?
  ASSERT_GE(sub_vin->getNumPublishers(), 1) << "VIN topic has no publishers.";

  // Ensure the VIN is valid.
  EXPECT_TRUE(waitFor<MsgRxb>(dataReady, sub_vin, 5.0)) << "Timeout expired waiting for VIN from the vehicle.";
  ASSERT_TRUE(sub_vin->fresh()) << "Did not receive a VIN from the vehicle.";
  ASSERT_NE(sub_vin->get().data.c_str(), nullptr) << "VIN received from the vehicle was empty."; // Is string not null?
}

TEST_F(CAN_HS1, TirePressure)
{
   // Wait for subscribers to finish setup.
  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_tire, 5.0)) << "Could not connect to Tire Pressure data topic.";
  // Which subscriber isn't ready?
  ASSERT_GE(sub_tire->getNumPublishers(), 1) << "Tire Pressure data topic has no publishers.";

  // Ensure the VIN is valid.
  EXPECT_TRUE(waitFor<MsgRxb>(dataReady, sub_tire, 5.0)) << "Timeout expired waiting for Tire Pressure data from the vehicle.";
  ASSERT_TRUE(sub_tire->fresh()) << "Did not receive Tire Pressure data from the vehicle.";
  ASSERT_GT(sub_tire->get().front_left, 0) << "Got bad Tire Pressure data: Front Left";
  ASSERT_GT(sub_tire->get().front_right, 0) << "Got bad Tire Pressure data: Front Right";
  ASSERT_GT(sub_tire->get().rear_left, 0) << "Got bad Tire Pressure data: Rear Left";
  ASSERT_GT(sub_tire->get().rear_right, 0) << "Got bad Tire Pressure data: Rear Right";
}

TEST_F(CAN_HS1, Surround)
{
   // Wait for subscribers to finish setup.
  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_surround, 5.0)) << "Could not connect to Surround data topic.";
  // Which subscriber isn't ready?
  ASSERT_GE(sub_surround->getNumPublishers(), 1) << "Surround data topic has no publishers.";

  // Ensure the Surround data is received.
  EXPECT_TRUE(waitFor<MsgRxb>(dataReady, sub_surround, 5.0)) << "Timeout expired waiting for Surround data from the vehicle.";
  ASSERT_TRUE(sub_surround->fresh()) << "Did not receive Tire Pressure data from the vehicle.";
}

TEST_F(BasicTest, BrakeFaults)
{
  // Wait for subscribers to finish setup.
  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_brake_report, 5.0)) << "Could not connect to BrakeReport topic.";
  // Which subscriber isn't ready?
  ASSERT_GE(sub_brake_report->getNumPublishers(), 1) << "BrakeReport topic has no publishers.";

  // Ensure there are no brake faults.
  EXPECT_TRUE(waitFor<MsgRxb>(dataReady, sub_brake_report, 5.0)) << "Timeout expired waiting for BrakeReport from the vehicle.";
  ASSERT_FALSE(sub_brake_report->get().fault_ch1) << "Vehicle reported brake fault: Channel 1";
  ASSERT_FALSE(sub_brake_report->get().fault_ch2) << "Vehicle reported brake fault: Channel 2";
  ASSERT_FALSE(sub_brake_report->get().fault_power) << "Vehicle reported brake fault: Power";
}

TEST_F(BasicTest, ThrottleFaults)
{
  // Wait for subscribers to finish setup.
  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_throttle_report, 5.0)) << "Could not connect to ThrottleReport topic.";
  // Which subscriber isn't ready?
  ASSERT_GE(sub_throttle_report->getNumPublishers(), 1) << "ThrottleReport topic has no publishers.";

  // Ensure there are no throttle faults.
  EXPECT_TRUE(waitFor<MsgRxb>(dataReady, sub_throttle_report, 5.0)) << "Timeout expired waiting for ThrottleReport from the vehicle.";
  ASSERT_FALSE(sub_throttle_report->get().fault_ch1) << "Vehicle reported throttle fault: Channel 1";
  ASSERT_FALSE(sub_throttle_report->get().fault_ch2) << "Vehicle reported throttle fault: Channel 2";
  ASSERT_FALSE(sub_throttle_report->get().fault_power) << "Vehicle reported throttle fault: Power";
}

TEST_F(CAN_HS2, SteeringFaults)
{
  // Wait for subscribers to finish setup.
  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_steering_report, 5.0)) << "Could not connect to SteeringReport topic.";
  // Which subscriber isn't ready?
  ASSERT_GE(sub_steering_report->getNumPublishers(), 1) << "SteeringReport topic has no publishers.";

  // Ensure there are no steering faults.
  EXPECT_TRUE(waitFor<MsgRxb>(dataReady, sub_steering_report, 5.0));
  ASSERT_FALSE(sub_steering_report->get().fault_bus1) << "Vehicle reported steering fault: Bus 1";
  ASSERT_FALSE(sub_steering_report->get().fault_bus2) << "Vehicle reported steering fault: Bus 2";
  ASSERT_FALSE(sub_steering_report->get().fault_calibration) << "Vehicle reported steering fault: Calibration";
  ASSERT_FALSE(sub_steering_report->get().fault_power) << "Vehicle reported steering fault: Power";
}

TEST_F(BasicTest, ShiftingFaults)
{
  // Wait for subscribers to finish setup.
  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_gear_report, 5.0)) << "Could not connect to GearReport topic.";
  // Which subscriber isn't ready?
  ASSERT_GE(sub_gear_report->getNumPublishers(), 1) << "GearReport topic has no publishers.";

  // Ensure there are no steering faults.
  EXPECT_TRUE(waitFor<MsgRxb>(dataReady, sub_gear_report, 5.0));
  ASSERT_FALSE(sub_gear_report->get().fault_bus) << "Vehicle reported gear shifting fault.";
}

TEST_F(CAN_HS1, IMU)
{
  // Wait for subscribers to finish setup.
  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_imu, 5.0)) << "Could not connect to IMU topic.";
  // Which subscriber isn't ready?
  ASSERT_GE(sub_imu->getNumPublishers(), 1) << "IMU topic has no publishers.";

  // Ensure IMU data is non-zero. Each field should be non-zero in some message.
  // If correct data is being received, no value should be exactly 0.
  EXPECT_TRUE(waitFor<MsgRxIMU>(imuValid, sub_imu, 5.0));
}

TEST_F(CAN_HS1, GPS)
{
  // Wait for subscribers to finish setup.
  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_gps_fix, 5.0)) << "Could not connect to GPS fix data topic.";
  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_gps_vel, 5.0)) << "Could not connect to GPS velocity data topic.";
  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_gps_time, 5.0)) << "Could not connect to GPS time data topic.";
  // Which subscriber isn't ready?
  ASSERT_GE(sub_gps_fix->getNumPublishers(), 1) << "GPS fix data topic has no publishers.";
  ASSERT_GE(sub_gps_vel->getNumPublishers(), 1) << "GPS velocity data topic has no publishers.";
  ASSERT_GE(sub_gps_time->getNumPublishers(), 1) << "GPS time data topic has no publishers.";

  // Ensure the GPS data is received.
  EXPECT_TRUE(waitFor<MsgRxb>(dataReady, sub_gps_fix, 5.0)) << "Timeout expired waiting for GPS fix data from the vehicle.";
  EXPECT_TRUE(waitFor<MsgRxb>(dataReady, sub_gps_vel, 5.0)) << "Timeout expired waiting for GPS velocity data from the vehicle.";
  EXPECT_TRUE(waitFor<MsgRxb>(dataReady, sub_gps_time, 5.0)) << "Timeout expired waiting for GPS time data from the vehicle.";
  ASSERT_TRUE(sub_gps_fix->fresh()) << "Did not receive GPS fix data from the vehicle.";
  ASSERT_TRUE(sub_gps_vel->fresh()) << "Did not receive GPS velocity data from the vehicle.";
  ASSERT_TRUE(sub_gps_time->fresh()) << "Did not receive GPS time data from the vehicle.";
}

TEST_F(Actuate, ActuateInit)
{
  // Wait for subscribers to finish setup.
  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_brake_info_report, 5.0)) << "Could not connect to BrakeInfoReport topic.";
  EXPECT_TRUE(waitFor<MsgRxb>(subReady, sub_gear_report, 5.0)) << "Could not connect to GearReport topic.";
  // Which subscriber isn't ready?
  ASSERT_GE(sub_brake_info_report->getNumPublishers(), 1) << "BrakeInfoReport topic has no publishers.";
  ASSERT_GE(sub_gear_report->getNumPublishers(), 1) << "GearReport topic has no publishers.";

  // Ensure the parking brake is on.
  EXPECT_TRUE(waitFor<MsgRxb>(dataReady, sub_brake_info_report, 5.0));
  ASSERT_EQ(sub_brake_info_report->get().parking_brake.status, dbw_mkz_msgs::ParkingBrake::ON) << "WARNING: Parking brake is not enabled. Skipping actuation test.";

  // Ensure the vehicle is in park.
  EXPECT_TRUE(waitFor<MsgRxb>(dataReady, sub_gear_report, 5.0));
  ASSERT_EQ(sub_gear_report->get().state.gear, dbw_mkz_msgs::Gear::PARK) << "WARNING: Vehicle not in park. Skipping actuation test.";


  // Wait for publishers to finish setup.
  EXPECT_TRUE(waitFor<MsgTxb>(pubReady, pub_dbw_enable, 5.0)) << "Could not connect to DBW enable topic.";
  EXPECT_EQ(g_recording ? 2 : 1, pub_dbw_enable->getNumSubscribers()) << "DBW enable has no subscribers.";

  // Enable DBW.
  pub_dbw_enable->enable();
  pub_dbw_enable->send();
}

TEST_F(Actuate, ActuateBrakes)
{
  // Wait for publishers to finish setup.
  EXPECT_TRUE(waitFor<MsgTxb>(pubReady, pub_brake_cmd, 5.0));
  // Which publisher isn't ready?
  ASSERT_GE(pub_brake_cmd->getNumSubscribers(), 1);

  pub_brake_cmd->get().pedal_cmd = 0.3;
  pub_brake_cmd->get().pedal_cmd_type = dbw_mkz_msgs::BrakeCmd::CMD_PEDAL;
  pub_brake_cmd->get().enable = true;
  pub_brake_cmd->enable();
  ros::WallDuration(3).sleep();

  EXPECT_TRUE(sub_brake_report->fresh());
  EXPECT_TRUE(sub_brake_report->get().enabled);
  // Is the pedal cmd within 0.01 of 0.3?
  EXPECT_TRUE(abs(sub_brake_report->get().pedal_cmd - 0.3) < 0.1);

  pub_brake_cmd->get().pedal_cmd = 0;
  pub_brake_cmd->get().enable = false;
}

TEST_F(Actuate, ActuateThrottle)
{
  // Wait for publishers to finish setup.
  EXPECT_TRUE(waitFor<MsgTxb>(pubReady, pub_throttle_cmd, 5.0));
  // Which publisher isn't ready?
  ASSERT_GE(pub_throttle_cmd->getNumSubscribers(), 1);

  pub_throttle_cmd->get().pedal_cmd = 0.5;
  pub_throttle_cmd->get().pedal_cmd_type = dbw_mkz_msgs::ThrottleCmd::CMD_PEDAL;
  pub_throttle_cmd->get().enable = true;
  pub_throttle_cmd->enable();
  ros::WallDuration(3).sleep();

  EXPECT_TRUE(sub_throttle_report->fresh());
  EXPECT_TRUE(sub_throttle_report->get().enabled);
    // Is the pedal cmd within 0.01 of 0.3?
  EXPECT_TRUE(abs(sub_throttle_report->get().pedal_cmd - 0.5) < 0.1);

  pub_throttle_cmd->get().pedal_cmd = 0;
  pub_throttle_cmd->get().enable = false;
}

TEST_F(Actuate, ActuateSteer)
{
  // Wait for publishers to finish setup.
  EXPECT_TRUE(waitFor<MsgTxb>(pubReady, pub_steer_cmd, 5.0));
  // Which publisher isn't ready?
  ASSERT_GE(pub_steer_cmd->getNumSubscribers(), 1);

  pub_steer_cmd->get().steering_wheel_angle_cmd = 0.5;
  pub_steer_cmd->get().cmd_type = dbw_mkz_msgs::SteeringCmd::CMD_ANGLE;
  pub_steer_cmd->get().enable = true;
  pub_steer_cmd->enable();
  ros::WallDuration(3).sleep();

  EXPECT_TRUE(sub_steering_report->fresh());

  EXPECT_TRUE(sub_steering_report->get().enabled);
  // Is the pedal cmd within 0.01 of 0.3?
  EXPECT_TRUE(abs(sub_steering_report->get().steering_wheel_cmd - 0.5) < 0.1);

  pub_steer_cmd->get().steering_wheel_angle_cmd = 0;
  pub_steer_cmd->get().enable = false;
}

TEST_F(Actuate, ActuateShift)
{
  // Wait for publishers to finish setup.
  EXPECT_TRUE(waitFor<MsgTxb>(pubReady, pub_brake_cmd, 5.0));
  EXPECT_TRUE(waitFor<MsgTxb>(pubReady, pub_gear_cmd, 5.0));
  // Which publisher isn't ready?
  ASSERT_GE(pub_brake_cmd->getNumSubscribers(), 1);
  ASSERT_GE(pub_gear_cmd->getNumSubscribers(), 1);

  // Apply the parking brake.
  pub_brake_cmd->get().pedal_cmd = 0.5;
  pub_brake_cmd->get().pedal_cmd_type = dbw_mkz_msgs::BrakeCmd::CMD_PEDAL;
  pub_brake_cmd->get().enable = true;
  pub_brake_cmd->enable();
  ros::WallDuration(3).sleep();

  EXPECT_TRUE(sub_brake_report->fresh());
  EXPECT_TRUE(sub_brake_report->get().enabled);
  // Is the pedal cmd within 0.01 of 0.3?
  ASSERT_TRUE(abs(sub_brake_report->get().pedal_cmd - 0.5) < 0.1);

  pub_gear_cmd->get().cmd.gear = dbw_mkz_msgs::Gear::NEUTRAL;
  pub_gear_cmd->enable();
  ros::WallDuration(3).sleep();

  ASSERT_TRUE(sub_steering_report->fresh());
  // Ensure the vehicle is in neutral.
  EXPECT_TRUE(waitFor<MsgRxb>(dataReady, sub_gear_report, 5.0));
  ASSERT_EQ(sub_gear_report->get().state.gear, dbw_mkz_msgs::Gear::NEUTRAL) << "WARNING: Vehicle not shifted to neutral.";

  // Display additional info.
  if (sub_gear_report->get().state.gear != dbw_mkz_msgs::Gear::NEUTRAL) {
    switch (sub_gear_report->get().reject.value) {
      case dbw_mkz_msgs::GearReject::NONE:
        printf("Shift rejection reason: None\n");
      case dbw_mkz_msgs::GearReject::SHIFT_IN_PROGRESS:
        printf("Shift rejection reason: Shift in Progress\n");
      case dbw_mkz_msgs::GearReject::OVERRIDE:
        printf("Shift rejection reason: Override\n");
      case dbw_mkz_msgs::GearReject::ROTARY_LOW:
        printf("Shift rejection reason: Rotary Low\n");
      case dbw_mkz_msgs::GearReject::ROTARY_PARK:
        printf("Shift rejection reason: Rotary Park\n");
      case dbw_mkz_msgs::GearReject::VEHICLE:
        printf("Shift rejection reason: Vehicle\n");
    }
  }

  pub_gear_cmd->get().cmd.gear = dbw_mkz_msgs::Gear::PARK;
  pub_gear_cmd->enable();
  ros::WallDuration(3).sleep();

  ASSERT_TRUE(sub_steering_report->fresh());
  // Ensure the vehicle is back in park.
  EXPECT_TRUE(waitFor<MsgRxb>(dataReady, sub_gear_report, 5.0));
  ASSERT_EQ(sub_gear_report->get().state.gear, dbw_mkz_msgs::Gear::PARK) << "WARNING: Vehicle not shifted to park.";

  // Display additional info.
  if (sub_gear_report->get().state.gear != dbw_mkz_msgs::Gear::PARK) {
    switch (sub_gear_report->get().reject.value) {
      case dbw_mkz_msgs::GearReject::NONE:
        printf("Shift rejection reason: None\n");
      case dbw_mkz_msgs::GearReject::SHIFT_IN_PROGRESS:
        printf("Shift rejection reason: Shift in Progress\n");
      case dbw_mkz_msgs::GearReject::OVERRIDE:
        printf("Shift rejection reason: Override\n");
      case dbw_mkz_msgs::GearReject::ROTARY_LOW:
        printf("Shift rejection reason: Rotary Low\n");
      case dbw_mkz_msgs::GearReject::ROTARY_PARK:
        printf("Shift rejection reason: Rotary Park\n");
      case dbw_mkz_msgs::GearReject::VEHICLE:
        printf("Shift rejection reason: Vehicle\n");
    }
  }

  pub_brake_cmd->get().pedal_cmd = 0;
  pub_brake_cmd->get().enable = false;
}

// Shift actuation requires brake pressed (do this with DBW CMD)

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

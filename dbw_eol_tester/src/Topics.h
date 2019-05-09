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

// Undefine GNU C system macros that we use for other purposes
#undef major
#undef minor

// ROS messages
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <hil_msgs/BrakeCmd.h>
#include <hil_msgs/BrakeReport.h>
#include <hil_msgs/BrakeTestData.h>
#include <hil_msgs/ThrottleCmd.h>
#include <hil_msgs/ThrottleReport.h>
#include <hil_msgs/ThrottleTestData.h>
#include <dbw_dev_msgs/Uut.h>
#include <dbw_dev_msgs/Version.h>

// ROS services
#include <std_srvs/Trigger.h>
#include <dbw_dev_usb_msgs/GetInputs.h>
#include <dbw_dev_usb_msgs/SetVccLatch.h>
#include <dbw_dev_usb_msgs/GetFeatures.h>

// rostest Helper classes
#include <dataspeed_rostest/MsgRx.h>
#include <dataspeed_rostest/MsgTx.h>
#include <dataspeed_rostest/Srv.h>
using namespace dataspeed_rostest;

/*
 * Simple averaging filter
 */
class AvgFilter {
private:
  size_t ptr_ = 0;
  std::vector<double> samples_;
public:
  AvgFilter (size_t num_samples) : samples_(num_samples) { }
  void put (double sample) {
    samples_[ptr_] = sample;
    ptr_ = (ptr_ + 1) % samples_.size();
  }
  double avg () {
    double sum = 0;
    for (double sample : samples_) {
      sum += sample;
    }
    return sum / samples_.size();
  }
};

/*
 * Filter values from throttle report
 */
class MsgRxThrottleReport : public MsgRx<hil_msgs::ThrottleReport> {
public:
  MsgRxThrottleReport(ros::NodeHandle &nh, std::string topic_name, const float thresh, uint8_t num_samples = 4) :
    MsgRx(nh, topic_name, thresh),
    pedal_input_(num_samples),
    pedal_output_(num_samples)
  {
  }
  double getAvgPedalOutput () { return pedal_output_.avg(); }
  double getAvgPedalInput ()  { return pedal_input_.avg(); }
protected:
  void recv(const hil_msgs::ThrottleReport::ConstPtr& msg) {
    MsgRx::recv(msg);
    pedal_input_.put(msg->pedal_input);
    pedal_output_.put(msg->pedal_output);
  }
private:
  AvgFilter pedal_input_, pedal_output_;
};

/*
 * Filter values from Throttle Test unit's report
 */
class MsgRxThrottleTestData : public MsgRx<hil_msgs::ThrottleTestData> {
public:
  MsgRxThrottleTestData(ros::NodeHandle &nh, std::string topic_name, const float thresh, uint8_t num_samples = 4) :
    MsgRx(nh, topic_name, thresh),
    pedal_ch1_(num_samples),
    pedal_ch2_(num_samples)
  {
  }
  double getAvgPedalCh1 () { return pedal_ch1_.avg(); }
  double getAvgPedalCh2 () { return pedal_ch2_.avg(); }
protected:
  void recv(const hil_msgs::ThrottleTestData::ConstPtr& msg) {
    MsgRx::recv(msg);
    pedal_ch1_.put(msg->pedal_ch1);
    pedal_ch2_.put(msg->pedal_ch2);
  }
private:
  AvgFilter pedal_ch1_, pedal_ch2_;
};

template<typename T>
std::vector<T> operator+(const std::vector<T>& v1, const std::vector<T>& v2){
  std::vector<T> vr(std::begin(v1), std::end(v1));
  vr.insert(std::end(vr), std::begin(v2), std::end(v2));
  return vr;
}

/*
 * Basic cmd/report/version info for each module used in tests 
 *
 * Each of the modules has a command(cmd), report, and version message
 * this templated class acts as the base class for each modules topics.
 * Publishers and subscribers are grouped together to simplify waiting for ready handling.
 */
template<class CmdMsgT, class MsgRxReportMsgT>
class ModuleTopics {
public:
  std::shared_ptr<MsgTx<CmdMsgT> >                   pub_cmd;
  std::shared_ptr<MsgRxReportMsgT>                   sub_report;
  std::shared_ptr<MsgRx<dbw_dev_msgs::Version> > sub_version;
  std::vector<std::shared_ptr<MsgTxb> > pubs = { pub_cmd };
  std::vector<std::shared_ptr<MsgRxb> > subs = { sub_report, sub_version};
  ModuleTopics() { }
  ModuleTopics(ros::NodeHandle nh) :
    pub_cmd(    std::make_shared<MsgTx<CmdMsgT> >                   (nh, "can/cmd")),
    sub_report( std::make_shared<MsgRxReportMsgT>                   (nh, "can/report",  0.25)),
    sub_version(std::make_shared<MsgRx<dbw_dev_msgs::Version> > (nh, "can/version", 2.5))
  {
  }
};

/*
 * Services for commands modules with special test interface
 *
 * Each of the command modules has an additional test interface over usb that supports extra
 * features. This adds these features via exposed services.
 * Like services are grouped together to simplify wait for ready handling.
 */
class CmdModuleServices {
public:
  // Services
  std::shared_ptr<Srv<dbw_dev_usb_msgs::SetVccLatch> >        srv_set_vcc_latch;
  std::shared_ptr<Srv<dbw_dev_usb_msgs::GetInputs> >          srv_get_inputs;
  std::shared_ptr<Srv<std_srvs::Trigger> >                 srv_reset_params;

  CmdModuleServices() { }
  CmdModuleServices(ros::NodeHandle nh) :
    srv_set_vcc_latch(std::make_shared<Srv<dbw_dev_usb_msgs::SetVccLatch> >        (nh, "set_vcc_latch")),
    srv_get_inputs(   std::make_shared<Srv<dbw_dev_usb_msgs::GetInputs> >          (nh, "get_inputs")),
    srv_reset_params( std::make_shared<Srv<std_srvs::Trigger> >                 (nh, "reset_params"))
  {
  }

  std::vector<std::shared_ptr<Srvb> > srvs_io      { srv_set_vcc_latch, srv_get_inputs };
  std::vector<std::shared_ptr<Srvb> > srvs_params  { srv_reset_params };

  bool readInputs (dbw_dev_usb_msgs::GetInputs::Response &resp) {
    dbw_dev_usb_msgs::GetInputs::Request req;
    return srv_get_inputs->call(req, resp);
  }
  bool setVccLatch (bool latch) {
    dbw_dev_usb_msgs::SetVccLatch rr;
    rr.request.vcc_latch = latch;
    bool success = srv_set_vcc_latch->call(rr);
    if (success) {
      ROS_DEBUG_STREAM("set vcc latch on " << module_ << " to " << (latch ? "ON" : "OFF"));
    }
    return success;
  }
  bool resetParams() {
    std_srvs::Trigger rr;
    return srv_reset_params->call(rr);
  }
private:
  std::string module_;
};

// Each of the modules has a slightly different configuration since different command and report messages are used for every module
class BrakeTestModuleTopics : public ModuleTopics<hil_msgs::BrakeCmd, MsgRx<hil_msgs::BrakeTestData> > {
public:
  std::shared_ptr<MsgTx<dbw_dev_msgs::Uut> >         pub_uut;
  BrakeTestModuleTopics() { }
  BrakeTestModuleTopics(ros::NodeHandle nh) : ModuleTopics(nh),
    pub_uut(std::make_shared<MsgTx<dbw_dev_msgs::Uut> > (nh, "set_uut"))
  {
    pubs.push_back(pub_uut);
  }
};
typedef ModuleTopics<hil_msgs::ThrottleCmd, MsgRxThrottleTestData > ThrottleTestModuleTopics;
class BrakeCmdModuleServices : public CmdModuleServices, public ModuleTopics<hil_msgs::BrakeCmd, MsgRx<hil_msgs::BrakeReport> > {
public:
  BrakeCmdModuleServices() { }
  BrakeCmdModuleServices(ros::NodeHandle nh) : CmdModuleServices(nh), ModuleTopics(nh) { }
};
class ThrottleCmdModuleServices : public CmdModuleServices, public ModuleTopics<hil_msgs::ThrottleCmd, MsgRxThrottleReport> {
public:
  ThrottleCmdModuleServices() { }
  ThrottleCmdModuleServices(ros::NodeHandle nh) : CmdModuleServices(nh), ModuleTopics(nh) { }
};

/*
 * Combined topic/service handling for all modules and topics used in the test
 *
 * Acts as the global ros interface for all of the tests.
 * Like publishers and subscribers across modules are grouped together to simplify wait for ready handling.
 */
struct Topics {
public:
  // Modules
  BrakeCmdModuleServices     bpec;
  BrakeTestModuleTopics      bpet;
  ThrottleCmdModuleServices  tpec;
  ThrottleTestModuleTopics   tpet;

  // Publishers
  std::shared_ptr<MsgTx<std_msgs::Byte> >                pub_relay1;
  std::shared_ptr<MsgTx<std_msgs::Byte> >                pub_relay2;
  std::vector<std::shared_ptr<MsgTxb> > pubs_relay { pub_relay1, pub_relay2 };
  std::vector<std::shared_ptr<MsgTxb> > pubs_bpe;
  std::vector<std::shared_ptr<MsgTxb> > pubs_tpe;
  std::vector<std::shared_ptr<MsgTxb> > pubs;

  // Subscribers
  std::shared_ptr<MsgRx<std_msgs::Bool> > sub_relay1;
  std::shared_ptr<MsgRx<std_msgs::Bool> > sub_relay2;
  std::vector<std::shared_ptr<MsgRxb> > subs_relay  { sub_relay1, sub_relay2 };
  std::vector<std::shared_ptr<MsgRxb> > subs_bpe;
  std::vector<std::shared_ptr<MsgRxb> > subs_tpe;
  std::vector<std::shared_ptr<MsgRxb> > subs_report;
  std::vector<std::shared_ptr<MsgRxb> > subs_throttle_report;
  std::vector<std::shared_ptr<MsgRxb> > subs_tester;
  std::vector<std::shared_ptr<MsgRxb> > subs_uut;
  std::vector<std::shared_ptr<MsgRxb> > subs_version;
  std::vector<std::shared_ptr<MsgRxb> > subs;

  // Setup Subscribers
  Topics() { } // allow uninitialized state
  Topics(ros::NodeHandle &nh) :
    bpec(ros::NodeHandle("bpec")), bpet(ros::NodeHandle("bpet")),
    tpec(ros::NodeHandle("tpec")), tpet(ros::NodeHandle("tpet")),
    pub_relay1(std::make_shared<MsgTx<std_msgs::Byte> > (nh, "relay1/relay_cmd", true)),
    pub_relay2(std::make_shared<MsgTx<std_msgs::Byte> > (nh, "relay2/relay_cmd", true)),
    sub_relay1(std::make_shared<MsgRx<std_msgs::Bool> > (nh, "relay1/ready",     1.0)),
    sub_relay2(std::make_shared<MsgRx<std_msgs::Bool> > (nh, "relay2/ready",     1.0))
  {
    // Publishers
    pubs_bpe = bpec.pubs + bpet.pubs;
    pubs_tpe = tpec.pubs + tpet.pubs;
    pubs =     pubs_bpe + pubs_tpe + pubs_relay;
    // Subscribers
    subs_bpe = bpec.subs + bpet.subs;
    subs_tpe = tpec.subs + tpet.subs;
    subs_throttle_report = { tpec.sub_report, tpet.sub_report };
    subs_report = { bpec.sub_report, bpet.sub_report, tpec.sub_report, tpet.sub_report };
    subs_version = { bpec.sub_version, bpet.sub_version, tpec.sub_version, tpet.sub_version };
    subs_tester = { bpet.sub_report, bpet.sub_version, tpet.sub_report, tpet.sub_version };
    subs_uut = { bpec.sub_report, bpec.sub_version, tpec.sub_report, tpec.sub_version };
    subs = subs_relay + subs_bpe + subs_tpe;
  }
};

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

#ifndef _DATASPEED_ROSTEST_SRV_H_
#define _DATASPEED_ROSTEST_SRV_H_

#include <ros/ros.h>

namespace dataspeed_rostest {

/*
 * Base class for ros service client wrapper
 */
class Srvb {
 public:
  virtual ~Srvb() {};
  virtual bool exists() = 0;
  virtual std::string name() = 0;
};

/*
 * Service type templated class for ros service handling
 */
template <typename Service>
class Srv : public Srvb {
private:
  ros::ServiceClient srv_;
public:
  /*
   * Unitialized service wrapper
   */
  Srv() {};
  /*
   * Create a ros service client for @name
   */
  Srv(ros::NodeHandle &nh, std::string name) :
    srv_(nh.serviceClient<Service>(name))
  {
  };
  /*
   * Service name
   */
  std::string name() {
    return srv_.getService();
  }
  /*
   * Service available?
   */
  bool exists () {
    return srv_.exists();
  }
  /*
   * Call service
   */
  bool call (Service &service) {
    return srv_.call(service);
  }
  bool call (const typename Service::Request &req, typename Service::Response &resp) {
    return srv_.call(req, resp);
  }
};

} // namespace dataspeed_rostest

#endif // _DATASPEED_ROSTEST_SRV_H_

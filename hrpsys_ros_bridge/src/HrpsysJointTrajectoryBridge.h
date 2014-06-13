// -*- C++ -*-
/*!
 * @file  HrpsysJointTrajectoryBridge.h * @brief hrpsys setJointAngle - ros joint trajectory bridge * @date  $Date$ 
 *
 * $Id$ 
 *
 # Software License Agreement (BSD License)
 #
 # Copyright (c) 2013, JSK Lab, University of Tokyo All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above
 #    copyright notice, this list of conditions and the following
 #    disclaimer in the documentation and/or other materials provided
 #    with the distribution.
 #  * Neither the name of JSK Lab, University of Tokyo. nor the
 #    names of its contributors may be used to endorse or promote products
 #    derived from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 # FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 # COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 # BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 # CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 # LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 # ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 # POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef HRPSYSJOINTTRAJECTORYBRIDGE_H
#define HRPSYSJOINTTRAJECTORYBRIDGE_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/Manager.h>
#include <rtm/CorbaNaming.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

// hrp
#include "hrpsys_ros_bridge/idl/HRPDataTypes.hh"
#include <hrpCorba/ModelLoader.hh>
#include <hrpModel/Body.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/Link.h>
#include <hrpModel/ModelLoaderUtil.h>

// ros
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/server/simple_action_server.h"
#include "pr2_controllers_msgs/JointTrajectoryAction.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"

#include "hrpsys_ros_bridge/idl/SequencePlayerServiceStub.h"

using namespace RTC;

class HrpsysJointTrajectoryBridge : public RTC::DataFlowComponentBase
{
public:
  HrpsysJointTrajectoryBridge(RTC::Manager* manager);
  ~HrpsysJointTrajectoryBridge();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry()
  RTC::ReturnCode_t onInitialize();
  RTC::ReturnCode_t onFinalize();
  RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
  RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  class jointTrajectoryActionObj
  {
  protected:
    HrpsysJointTrajectoryBridge *parent;

    ros::Publisher joint_controller_state_pub;

    boost::shared_ptr<actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> > joint_trajectory_server;
    boost::shared_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> > follow_joint_trajectory_server;

    std::string controller_name;
    std::string groupname;
    std::vector<std::string> joint_list;
    bool interpolationp;

  public:
    typedef boost::shared_ptr<jointTrajectoryActionObj> Ptr;
    jointTrajectoryActionObj(HrpsysJointTrajectoryBridge *ptr, std::string &cname, std::string &gname,
                             std::vector<std::string> &jlist);
    ~jointTrajectoryActionObj();

    void onJointTrajectory(trajectory_msgs::JointTrajectory trajectory);
    void onJointTrajectoryActionGoal();
    void onJointTrajectoryActionPreempt();
    void onFollowJointTrajectoryActionGoal();
    void onFollowJointTrajectoryActionPreempt();

    void proc();
    void restart();
  };

protected:
  RTC::CorbaPort m_SequencePlayerServicePort;
  RTC::CorbaConsumer<OpenHRP::SequencePlayerService> m_service0;

protected:
  hrp::BodyPtr body;
  OpenHRP::BodyInfo_var bodyinfo;

  ros::NodeHandle nh;
  std::vector<jointTrajectoryActionObj::Ptr> trajectory_actions;
  coil::TimeMeasure tm;
  coil::Mutex m_mutex;
  std::string nameserver;

private:

};

extern "C"
{
DLL_EXPORT void HrpsysJointTrajectoryBridgeInit(RTC::Manager* manager);
}
;

#endif // HRPSYSJOINTTRAJECTORYBRIDGE_H


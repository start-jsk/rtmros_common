// -*- C++ -*-
/*!
 * @file  HrpsysSeqStateROSBridge.h * @brief hrpsys seq state - ros bridge * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef HRPSYSSEQSTATEROSBRIDGE_H
#define HRPSYSSEQSTATEROSBRIDGE_H

#include "HrpsysSeqStateROSBridgeImpl.h"

// rtm
#include <rtm/CorbaNaming.h>

// hrp
#include <hrpCorba/ModelLoader.hh>
#include <hrpModel/Link.h>
#include <hrpModel/ModelLoaderUtil.h>

// ros
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "actionlib/server/simple_action_server.h"
#include "pr2_controllers_msgs/JointTrajectoryAction.h"

extern const char* hrpsysseqstaterosbridgeimpl_spec[];

class HrpsysSeqStateROSBridge  : public HrpsysSeqStateROSBridgeImpl
{
 public:
  HrpsysSeqStateROSBridge(RTC::Manager* manager) ;
  ~HrpsysSeqStateROSBridge();

  RTC::ReturnCode_t onInitialize();
  RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void onJointTrajectoryAction(const pr2_controllers_msgs::JointTrajectoryGoalConstPtr& goal);

 private:
  hrp::BodyPtr body;

  ros::NodeHandle nh;
  ros::Publisher joint_state_pub;
  actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> server;

  coil::Mutex m_mutex;
};


extern "C"
{
  DLL_EXPORT void HrpsysSeqStateROSBridgeInit(RTC::Manager* manager);
};

#endif // HRPSYSSEQSTATEROSBRIDGE_H


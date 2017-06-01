// -*- C++ -*-
/*!
 * @file  HrpsysJointTrajectoryBridge.cpp * @brief hrpsys setJointAngle - ros joint trajectory bridge * $Date$ 
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
#include "HrpsysJointTrajectoryBridge.h"

#include "rtm/idl/RTC.hh"
#include "hrpsys_ros_bridge/idl/ExecutionProfileService.hh"
#include "hrpsys_ros_bridge/idl/RobotHardwareService.hh"

// Module specification
// <rtc-template block="module_spec">
static const char* hrpsysjointtrajectorybridge_spec[] = {"implementation_id", "HrpsysJointTrajectoryBridge",
                                                         "type_name", "HrpsysJointTrajectoryBridge", "description",
                                                         "hrpsys setJointAngle - ros joint trajectory bridge",
                                                         "version", "1.0", "vendor", "Yohei Kakiuchi", "category",
                                                         "example", "activity_type", "SPORADIC", "kind",
                                                         "DataFlowComponent", "max_instance", "10", "language", "C++",
                                                         "lang_type", "compile",
                                                         // Configuration variables
                                                         ""};
// </rtc-template>

HrpsysJointTrajectoryBridge::HrpsysJointTrajectoryBridge(RTC::Manager* manager)
// <rtc-template block="initializer">
:
    RTC::DataFlowComponentBase(manager), m_SequencePlayerServicePort("SequencePlayerService")
// </rtc-template>
{
}

HrpsysJointTrajectoryBridge::~HrpsysJointTrajectoryBridge()
{
}

RTC::ReturnCode_t HrpsysJointTrajectoryBridge::onInitialize()
{
  m_SequencePlayerServicePort.registerConsumer("service0", "SequencePlayerService", m_service0);

  addPort(m_SequencePlayerServicePort);

  RTC::Properties& prop = getProperties();

  body = hrp::BodyPtr(new hrp::Body());

  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0)
  {
    comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  std::string modelfile = m_pManager->getConfig()["model"];

  if (!loadBodyFromModelLoader(body, modelfile.c_str(), CosNaming::NamingContext::_duplicate(naming.getRootContext())))
  {
    std::cerr << "[HrpsysJointTrajectoryBridge] failed to load model[" << prop["model"] << "]" << std::endl;
  }
  bool ret = false;
  while (!ret)
  {
    try
    {
      bodyinfo = hrp::loadBodyInfo(modelfile.c_str(), CosNaming::NamingContext::_duplicate(naming.getRootContext()));
      ret = loadBodyFromBodyInfo(body, bodyinfo);
    }
    catch (CORBA::SystemException& ex)
    {
      std::cerr << "[HrpsysJointTrajectoryBridge] loadBodyInfo(): CORBA::SystemException " << ex._name() << std::endl;
      sleep(1);
    }
    catch (...)
    {
      std::cerr << "[HrpsysJointTrajectoryBridge] loadBodyInfo(): failed to load model[" << modelfile << "]"
          << std::endl;
      ;
      sleep(1);
    }
  }

  if (body == NULL)
  {
    return RTC::RTC_ERROR;
  }
  body->calcForwardKinematics();

  ROS_INFO_STREAM("[HrpsysJointTrajectoryBridge] @Initilize name : " << getInstanceName() << " done");

  return RTC::RTC_OK;
}

RTC::ReturnCode_t HrpsysJointTrajectoryBridge::onActivated(RTC::UniqueId ec_id)
{
  // ROS_INFO("ON_ACTIVATED");
  std::string config_name;
  config_name = nh.resolveName("controller_configuration");
  if (nh.hasParam(config_name))
  {
    XmlRpc::XmlRpcValue param_val;
    nh.getParam(config_name, param_val);
    if (param_val.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < param_val.size(); i++)
      {
        if (param_val[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
          XmlRpc::XmlRpcValue gval = param_val[i]["group_name"];
          XmlRpc::XmlRpcValue cval = param_val[i]["controller_name"];
          XmlRpc::XmlRpcValue lval = param_val[i]["joint_list"];

          std::string gname = gval;
          std::string cname = cval;
          std::vector<std::string> jlst;
          if (lval.getType() == XmlRpc::XmlRpcValue::TypeArray)
          {
            for (int s = 0; s < lval.size(); s++)
            {
              jlst.push_back(lval[s]);
            }
          }
          if (gname.length() == 0 && cname.length() > 0)
          {
            gname = cname;
          }
          if (gname.length() > 0 && cname.length() == 0)
          {
            cname = gname;
          }
          if (gname.length() > 0 && cname.length() > 0 && jlst.size() > 0)
          {
            std::stringstream ss;
            for (size_t s = 0; s < jlst.size(); s++)
            {
              ss << " " << jlst[s];
            }
            ROS_INFO_STREAM("ADD_GROUP: " << gname << " (" << cname << ")");
            ROS_INFO_STREAM("    JOINT:" << ss.str());
            jointTrajectoryActionObj::Ptr tmp(new jointTrajectoryActionObj(this, cval, gval, jlst));
            trajectory_actions.push_back(tmp);
          }
        }
      }
    }
    else
    {
      ROS_WARN_STREAM("param: " << config_name << ", configuration is not an array.");
    }
  }
  else
  {
    ROS_WARN_STREAM("param: " << config_name << ", param does not exist.");
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t HrpsysJointTrajectoryBridge::onFinalize()
{
  // delete objs
  for (size_t i = 0; i < trajectory_actions.size(); i++)
  {
    trajectory_actions[i].reset();
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t HrpsysJointTrajectoryBridge::onExecute(RTC::UniqueId ec_id)
{
  ros::spinOnce();
  for (size_t i = 0; i < trajectory_actions.size(); i++)
  {
    trajectory_actions[i]->proc();
  }

  return RTC::RTC_OK;
}

HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::jointTrajectoryActionObj(HrpsysJointTrajectoryBridge *ptr,
                                                                                std::string &cname, std::string &gname,
                                                                                std::vector<std::string> &jlist)
{
  parent = ptr;
  controller_name = cname;
  groupname = gname;
  joint_list = jlist;

  joint_trajectory_server.reset(
      new actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction>(
          parent->nh, controller_name + "/joint_trajectory_action", false));
  follow_joint_trajectory_server.reset(
      new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(
          parent->nh, controller_name + "/follow_joint_trajectory_action", false));
  trajectory_command_sub = parent->nh.subscribe(controller_name + "/command", 1, &HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::onTrajectoryCommandCB, this);


  joint_trajectory_server->registerGoalCallback(
      boost::bind(&HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::onJointTrajectoryActionGoal, this));
  joint_trajectory_server->registerPreemptCallback(
      boost::bind(&HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::onJointTrajectoryActionPreempt, this));
  follow_joint_trajectory_server->registerGoalCallback(
      boost::bind(&HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::onFollowJointTrajectoryActionGoal, this));
  follow_joint_trajectory_server->registerPreemptCallback(
      boost::bind(&HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::onFollowJointTrajectoryActionPreempt, this));

  joint_controller_state_pub = parent->nh.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>(
      controller_name + "/state", 1);

  if (groupname.length() > 0)
  {
    OpenHRP::SequencePlayerService::StrSequence jnames;
    jnames.length(joint_list.size());
    for (size_t i = 0; i < joint_list.size(); i++)
    {
      jnames[i] = joint_list[i].c_str();
    }
    try
    {
      parent->m_service0->addJointGroup(groupname.c_str(), jnames);
    }
    catch (CORBA::SystemException& ex)
    {
      std::cerr << "[HrpsysJointTrajectoryBridge] addJointGroup(" << groupname << "), CORBA::SystemException "
          << ex._name() << std::endl;
      sleep(1);
    }
    catch (...)
    {
      std::cerr << "[HrpsysJointTrajectoryBridge] addJointGroup(" << groupname << "), failed to addJointGroup["
          << groupname.c_str() << "]" << std::endl;
      ;
      sleep(1);
    }
  }

  interpolationp = false;

  joint_trajectory_server->start();
  follow_joint_trajectory_server->start();
}

HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::~jointTrajectoryActionObj()
{
  ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @~jointTrajectoryActionObj (" << this->groupname);
  if (joint_trajectory_server->isActive())
  {
    joint_trajectory_server->setPreempted();
  }

  if (follow_joint_trajectory_server->isActive())
  {
    follow_joint_trajectory_server->setPreempted();
  }

  joint_trajectory_server->shutdown();
  follow_joint_trajectory_server->shutdown();
}

void HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::proc()
{
  // finish interpolation
  if (joint_trajectory_server->isActive() && interpolationp == true && parent->m_service0->isEmpty() == true)
  {
    pr2_controllers_msgs::JointTrajectoryResult result;
    joint_trajectory_server->setSucceeded(result);
    interpolationp = false;
    ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @proc joint_trajectory_server->setSucceeded()");
  }
  if (follow_joint_trajectory_server->isActive() && interpolationp == true && parent->m_service0->isEmpty() == true)
  {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    follow_joint_trajectory_server->setSucceeded(result);
    interpolationp = false;
    ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @proc follow_joint_trajectory_server->setSucceeded()");
  }

  ros::Time tm_on_execute = ros::Time::now();

  // FIXME: need to set actual informatoin, currently we set dummy information
  trajectory_msgs::JointTrajectoryPoint commanded_joint_trajectory_point, error_joint_trajectory_point;
  commanded_joint_trajectory_point.positions.resize(joint_list.size());
  commanded_joint_trajectory_point.velocities.resize(joint_list.size());
  commanded_joint_trajectory_point.accelerations.resize(joint_list.size());
  commanded_joint_trajectory_point.effort.resize(joint_list.size());
  for (unsigned int j = 0; j < joint_list.size(); j++)
    {
      commanded_joint_trajectory_point.positions[j]     = parent->body->link(joint_list[j])->q;
      commanded_joint_trajectory_point.velocities[j]    = parent->body->link(joint_list[j])->dq;
      commanded_joint_trajectory_point.accelerations[j] = parent->body->link(joint_list[j])->ddq;
      commanded_joint_trajectory_point.effort[j]        = parent->body->link(joint_list[j])->u;
    }
  error_joint_trajectory_point.positions.resize(joint_list.size());
  error_joint_trajectory_point.velocities.resize(joint_list.size());
  error_joint_trajectory_point.accelerations.resize(joint_list.size());
  error_joint_trajectory_point.effort.resize(joint_list.size());

  if ( joint_trajectory_server->isActive() ) {
    pr2_controllers_msgs::JointTrajectoryFeedback joint_trajectory_feedback;
    joint_trajectory_server->publishFeedback(joint_trajectory_feedback);
  }

  if ( follow_joint_trajectory_server->isActive() ) {
    control_msgs::FollowJointTrajectoryFeedback follow_joint_trajectory_feedback;
    follow_joint_trajectory_feedback.header.stamp = tm_on_execute;
    follow_joint_trajectory_feedback.joint_names = joint_list;

    follow_joint_trajectory_feedback.desired = commanded_joint_trajectory_point;
    follow_joint_trajectory_feedback.actual  = commanded_joint_trajectory_point;
    follow_joint_trajectory_feedback.error   = error_joint_trajectory_point;

    follow_joint_trajectory_server->publishFeedback(follow_joint_trajectory_feedback);
  }

  pr2_controllers_msgs::JointTrajectoryControllerState joint_controller_state;
  joint_controller_state.joint_names = joint_list;

  joint_controller_state.desired = commanded_joint_trajectory_point;
  joint_controller_state.actual = commanded_joint_trajectory_point;
  joint_controller_state.error = error_joint_trajectory_point;

  joint_controller_state_pub.publish(joint_controller_state);
}

void HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::restart()
{
  parent->m_service0->removeJointGroup(groupname.c_str());
  sleep(0.1);
  if (groupname.length() > 0)
  {
    OpenHRP::SequencePlayerService::StrSequence jnames;
    jnames.length(joint_list.size());
    for (size_t i = 0; i < joint_list.size(); i++)
    {
      jnames[i] = joint_list[i].c_str();
    }
    try
    {
      parent->m_service0->addJointGroup(groupname.c_str(), jnames);
    }
    catch (CORBA::SystemException& ex)
    {
      std::cerr << "[HrpsysJointTrajectoryBridge] addJointGroup(" << groupname << "), CORBA::SystemException "
          << ex._name() << std::endl;
      sleep(1);
    }
    catch (...)
    {
      std::cerr << "[HrpsysJointTrajectoryBridge] addJointGroup(" << groupname << "), failed to addJointGroup["
          << groupname.c_str() << "]" << std::endl;
      ;
    }
  }
}

void HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::onJointTrajectory(
    trajectory_msgs::JointTrajectory trajectory)
{
  parent->m_mutex.lock();

  ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onJointTrajectoryAction (" << this->groupname << ")");
  // TODO: check size and joint names

  OpenHRP::dSequenceSequence angles;
  OpenHRP::dSequence duration;

  angles.length(trajectory.points.size());
  duration.length(trajectory.points.size());

  std::vector<std::string> joint_names = trajectory.joint_names;
  if (joint_names.size() < joint_list.size())
  {
    ROS_ERROR_STREAM(
        "[" << parent->getInstanceName() << "] @onJointTrajectoryAction / Error : " << "required joint_names.size() = " << joint_names.size() << " < joint_list.size() = " << joint_list.size());
    return;
  }
  for (unsigned int i = 0; i < joint_list.size(); i++)
  {
    if (count(joint_names.begin(), joint_names.end(), joint_list[i]) != 1)
    {
      ROS_ERROR_STREAM(
          "[" << parent->getInstanceName() << "] @onJointTrajectoryAction / Error : " << "joint : " << joint_list[i] << " did not exist in the required trajectory.");
      return;
    }
  }

  ROS_INFO_STREAM(
      "[" << parent->getInstanceName() << "] @onJointTrajectoryAction (" << this->groupname << ") : trajectory.points.size() " << trajectory.points.size());
  for (unsigned int i = 0; i < trajectory.points.size(); i++)
  {
    angles[i].length(joint_names.size());

    trajectory_msgs::JointTrajectoryPoint point = trajectory.points[i];
    for (unsigned int j = 0; j < joint_names.size(); j++)
    {
      parent->body->link(joint_names[j])->q = point.positions[j];
    }

    parent->body->calcForwardKinematics();

    std::stringstream ss;
    for (unsigned int j = 0; j < joint_list.size(); j++)
    {
      angles[i][j] = parent->body->link(joint_list[j])->q;
      ss << " " << point.positions[j];
    }
    ROS_INFO_STREAM(
        "[" << parent->getInstanceName() << "] @onJointTrajectoryAction (" << this->groupname << ") : time_from_start " << trajectory.points[i].time_from_start.toSec());
    ROS_INFO_STREAM("[" << parent->getInstanceName() << "] " << ss.str());

    if (i > 0)
    {
      duration[i] = trajectory.points[i].time_from_start.toSec() - trajectory.points[i - 1].time_from_start.toSec();
    }
    else
    { // if i == 0
      duration[i] = trajectory.points[i].time_from_start.toSec();
    }
  }

  parent->m_mutex.unlock();
  if (duration.length() == 1)
  {
    if (groupname.length() > 0)
    { // group
      ROS_INFO_STREAM("[" << parent->getInstanceName() << "] setJointAnglesOfGroup: " << groupname);
      parent->m_service0->setJointAnglesOfGroup(groupname.c_str(), angles[0], duration[0]);
    }
    else
    { // fullbody
      parent->m_service0->setJointAngles(angles[0], duration[0]);
    }
  }
  else
  {
    if (groupname.length() > 0)
    { // group
      ROS_INFO_STREAM("[" << parent->getInstanceName() << "] playPatternGroup: " << groupname);
      parent->m_service0->playPatternOfGroup(groupname.c_str(), angles, duration);
    }
    else
    {
      OpenHRP::dSequenceSequence rpy, zmp;
      parent->m_service0->playPattern(angles, rpy, zmp, duration);
    }
  }

  interpolationp = true;
}

void HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::onJointTrajectoryActionGoal()
{
  ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onJointTrajectoryActionGoal");
  pr2_controllers_msgs::JointTrajectoryGoalConstPtr goal = joint_trajectory_server->acceptNewGoal();
  onJointTrajectory(goal->trajectory);
}

void HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::onFollowJointTrajectoryActionGoal()
{
  ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onFollowJointTrajectoryActionGoal()");
  control_msgs::FollowJointTrajectoryGoalConstPtr goal = follow_joint_trajectory_server->acceptNewGoal();
  onJointTrajectory(goal->trajectory);
}

void HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::onJointTrajectoryActionPreempt()
{
  ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onJointTrajectoryActionPreempt()");
  joint_trajectory_server->setPreempted();
}

void HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::onFollowJointTrajectoryActionPreempt()
{
  ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onFollowJointTrajectoryActionPreempt()");
  follow_joint_trajectory_server->setPreempted();
}

void HrpsysJointTrajectoryBridge::jointTrajectoryActionObj::onTrajectoryCommandCB(const trajectory_msgs::JointTrajectoryConstPtr& msg)
{
  ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onTrajectoryCommandCB()");
  onJointTrajectory(*msg);
}

extern "C"
{

void HrpsysJointTrajectoryBridgeInit(RTC::Manager* manager)
{
  coil::Properties profile(hrpsysjointtrajectorybridge_spec);
  manager->registerFactory(profile, RTC::Create<HrpsysJointTrajectoryBridge>, RTC::Delete<HrpsysJointTrajectoryBridge>);
}

}
;

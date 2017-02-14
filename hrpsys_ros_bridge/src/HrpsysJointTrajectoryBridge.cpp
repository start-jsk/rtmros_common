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

// rtm
#include "rtm/idl/RTC.hh"
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
#include "hrpsys_ros_bridge/idl/ExecutionProfileService.hh"
#include "hrpsys_ros_bridge/idl/RobotHardwareService.hh"
#include <hrpCorba/ModelLoader.hh>
#include <hrpModel/Body.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/Link.h>

// http://stackoverflow.com/questions/2941491/compare-versions-as-strings
void Parse(int result[3], const std::string& input)
{
  std::istringstream parser(input);
  parser >> result[0];
  for(int idx = 1; idx < 3; idx++)
  {
    parser.get(); //Skip period
    parser >> result[idx];
  }
}

bool LessThanVersion(const std::string& a,const std::string& b)
{
  int parsedA[3], parsedB[3];
  try {
    Parse(parsedA, a);
    Parse(parsedB, b);
    return std::lexicographical_compare(parsedA, parsedA + 4, parsedB, parsedB + 4);
  } catch (...) {
    ROS_ERROR_STREAM("LessThanVersion failed " << a << " < " << b <<", force return false");
    return false;
  }
}

//
HrpsysJointTrajectoryAction::HrpsysJointTrajectoryAction(HrpsysSeqStateROSBridge *ptr,
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
  trajectory_command_sub = parent->nh.subscribe(controller_name + "/command", 1, &HrpsysJointTrajectoryAction::onTrajectoryCommandCB, this);


  joint_trajectory_server->registerGoalCallback(
      boost::bind(&HrpsysJointTrajectoryAction::onJointTrajectoryActionGoal, this));
  joint_trajectory_server->registerPreemptCallback(
      boost::bind(&HrpsysJointTrajectoryAction::onJointTrajectoryActionPreempt, this));
  follow_joint_trajectory_server->registerGoalCallback(
      boost::bind(&HrpsysJointTrajectoryAction::onFollowJointTrajectoryActionGoal, this));
  follow_joint_trajectory_server->registerPreemptCallback(
      boost::bind(&HrpsysJointTrajectoryAction::onFollowJointTrajectoryActionPreempt, this));

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

HrpsysJointTrajectoryAction::~HrpsysJointTrajectoryAction()
{
  ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @~HrpsysJointTrajectoryAction (" << this->groupname);
  if (joint_trajectory_server->isActive() || follow_joint_trajectory_server->isActive()) {
    onJointTrajectoryCancel();
  }

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

void HrpsysJointTrajectoryAction::proc()
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
  trajectory_msgs::JointTrajectoryPoint commanded_joint_trajectory_point, desired_joint_trajectory_point, error_joint_trajectory_point;
  commanded_joint_trajectory_point.positions.resize(joint_list.size());
  commanded_joint_trajectory_point.velocities.resize(joint_list.size());
  commanded_joint_trajectory_point.accelerations.resize(joint_list.size());
  commanded_joint_trajectory_point.effort.resize(joint_list.size());
  desired_joint_trajectory_point.positions.resize(joint_list.size());
  desired_joint_trajectory_point.velocities.resize(joint_list.size());
  desired_joint_trajectory_point.accelerations.resize(joint_list.size());
  desired_joint_trajectory_point.effort.resize(joint_list.size());
  error_joint_trajectory_point.positions.resize(joint_list.size());
  error_joint_trajectory_point.velocities.resize(joint_list.size());
  error_joint_trajectory_point.accelerations.resize(joint_list.size());
  error_joint_trajectory_point.effort.resize(joint_list.size());
  for (unsigned int j = 0; j < joint_list.size(); j++)
    {
      commanded_joint_trajectory_point.positions[j]     = parent->body->link(joint_list[j])->q;
      commanded_joint_trajectory_point.velocities[j]    = parent->body->link(joint_list[j])->dq;
      commanded_joint_trajectory_point.accelerations[j] = parent->body->link(joint_list[j])->ddq;
      commanded_joint_trajectory_point.effort[j]        = parent->body->link(joint_list[j])->u;
      if ( parent->desired_joint_values.find(joint_list[j]) == parent->desired_joint_values.end() ) {
        ROS_WARN_STREAM("desired_joint_values " << joint_list[j] << "is not found");
      }
      desired_joint_trajectory_point.positions[j]       = parent->desired_joint_values[joint_list[j]];
      error_joint_trajectory_point.positions[j] = (commanded_joint_trajectory_point.positions[j]-desired_joint_trajectory_point.positions[j]);
    }

  if ( joint_trajectory_server->isActive() ) {
    pr2_controllers_msgs::JointTrajectoryFeedback joint_trajectory_feedback;
    joint_trajectory_server->publishFeedback(joint_trajectory_feedback);
  }

  if ( follow_joint_trajectory_server->isActive() ) {
    control_msgs::FollowJointTrajectoryFeedback follow_joint_trajectory_feedback;
    follow_joint_trajectory_feedback.header.stamp = tm_on_execute;
    follow_joint_trajectory_feedback.joint_names = joint_list;

    follow_joint_trajectory_feedback.desired = desired_joint_trajectory_point;
    follow_joint_trajectory_feedback.actual  = commanded_joint_trajectory_point;
    follow_joint_trajectory_feedback.error   = error_joint_trajectory_point;

    follow_joint_trajectory_server->publishFeedback(follow_joint_trajectory_feedback);
  }

  pr2_controllers_msgs::JointTrajectoryControllerState joint_controller_state;
  joint_controller_state.joint_names = joint_list;

  joint_controller_state.desired = desired_joint_trajectory_point;
  joint_controller_state.actual = commanded_joint_trajectory_point;
  joint_controller_state.error = error_joint_trajectory_point;

  joint_controller_state_pub.publish(joint_controller_state);
}

void HrpsysJointTrajectoryAction::restart()
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

void HrpsysJointTrajectoryAction::onJointTrajectory(
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
  // if number of joint_names (within trajectory command) is larger than number of joint_list (set from configuration or body mdoel), something wrong
  if (joint_names.size() > joint_list.size())
  {
    ROS_ERROR_STREAM(
        "[" << parent->getInstanceName() << "] @onJointTrajectoryAction / Error : " << "required joint_names.size() = " << joint_names.size() << " < joint_list.size() = " << joint_list.size());
    return;
  }
  for (unsigned int i = 0; i < joint_list.size(); i++)
  {
    if (count(joint_names.begin(), joint_names.end(), joint_list[i]) != 1)
    {
      ROS_WARN_STREAM(
          "[" << parent->getInstanceName() << "] @onJointTrajectoryAction / Error : " << "joint : " << joint_list[i] << " did not exist in the required trajectory.");
    }
  }
  for (unsigned int i = 0; i < joint_names.size(); i++)
  {
    if (count(joint_list.begin(), joint_list.end(), joint_names[i]) != 1)
    {
      ROS_ERROR_STREAM(
          "[" << parent->getInstanceName() << "] @onJointTrajectoryAction / Error : " << "joint : " << joint_list[i] << " did not exist in the robot model (or limbs).");
      return;
    }
  }

  ROS_INFO_STREAM(
      "[" << parent->getInstanceName() << "] @onJointTrajectoryAction (" << this->groupname << ") : trajectory.points.size() " << trajectory.points.size());
  for (unsigned int i = 0; i < trajectory.points.size(); i++)
  {
    angles[i].length(joint_list.size());

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
      if (trajectory.points.size() == 1)
      {
        duration[i] = trajectory.points[i].time_from_start.toSec();
      }
      else
      { // 0.2 is magic number writtein in roseus/euslisp/robot-interface.l
        duration[i] = trajectory.points[i].time_from_start.toSec() - 0.2;
      }
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
      // hrpsys < 315.5.0 does not have setJointAnglesSequenceOfGroup,  so need to use old API
      if (LessThanVersion(parent->hrpsys_version, std::string("315.5.0"))) {
        ROS_INFO_STREAM("[" << parent->getInstanceName() << "] playPatternGroup: " << groupname);
        parent->m_service0->playPatternOfGroup(groupname.c_str(), angles, duration);
      }else{
        ROS_INFO_STREAM("[" << parent->getInstanceName() << "] setJointAnglesSequenceOfGroup: " << groupname);
        parent->m_service0->setJointAnglesSequenceOfGroup(groupname.c_str(), angles, duration);
      }
    }
    else
    { // fullbody
      // hrpsys < 315.5.0 does not have setJointAnglesSequence,  so need to use old API
      if (LessThanVersion(parent->hrpsys_version, std::string("315.5.0"))) {
        OpenHRP::dSequenceSequence rpy, zmp;
        parent->m_service0->playPattern(angles, rpy, zmp, duration);
      }else{
        parent->m_service0->setJointAnglesSequence(angles, duration);
      }
    }
  }

  interpolationp = true;
}

void HrpsysJointTrajectoryAction::onJointTrajectoryActionGoal()
{
  ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onJointTrajectoryActionGoal");
  pr2_controllers_msgs::JointTrajectoryGoalConstPtr goal = joint_trajectory_server->acceptNewGoal();
  onJointTrajectory(goal->trajectory);
}

void HrpsysJointTrajectoryAction::onFollowJointTrajectoryActionGoal()
{
  ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onFollowJointTrajectoryActionGoal()");
  control_msgs::FollowJointTrajectoryGoalConstPtr goal = follow_joint_trajectory_server->acceptNewGoal();
  onJointTrajectory(goal->trajectory);
}


void HrpsysJointTrajectoryAction::onJointTrajectoryCancel()
{
  if (groupname.length() > 0)
  { // group
      // hrpsys < 315.5.0 does not have clearJointAnglesOfGroup,  so need to use old API
    if (LessThanVersion(parent->hrpsys_version, std::string("315.5.0"))) {
      ROS_INFO_STREAM("[" << parent->getInstanceName() << "] clearOfGroup: " << groupname);
      parent->m_service0->clearOfGroup(groupname.c_str(), 0.05);
      OpenHRP::dSequence angles;
      angles.length(joint_list.size());
      for (unsigned int i = 0; i < joint_list.size(); i++)
      {
        angles[i] = parent->body->link(joint_list[i])->q;
      }
      parent->m_service0->setJointAnglesOfGroup(groupname.c_str(), angles, 0.05);
    } else {
      ROS_INFO_STREAM("[" << parent->getInstanceName() << "] clearJointAnglesOfGroup: " << groupname);
      parent->m_service0->clearJointAnglesOfGroup(groupname.c_str());
    }
  }
  else
  { // fullbody
      // hrpsys < 315.5.0 does not have clearJointAngles,  so need to use old API
    if (LessThanVersion(parent->hrpsys_version, std::string("315.5.0"))) {
      ROS_INFO_STREAM("[" << parent->getInstanceName() << "] clear ");
      parent->m_service0->clear();
      OpenHRP::dSequence angles;
      angles.length(joint_list.size());
      for (unsigned int i = 0; i < joint_list.size(); i++)
      {
        angles[i] = parent->body->link(joint_list[i])->q;
      }
      parent->m_service0->setJointAngles(angles, 0.05);
    } else {
      ROS_INFO_STREAM("[" << parent->getInstanceName() << "] clearJointAngles ");
      parent->m_service0->clearJointAngles();
    }
  }
}

void HrpsysJointTrajectoryAction::onJointTrajectoryActionPreempt()
{
  ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onJointTrajectoryActionPreempt()");
  joint_trajectory_server->setPreempted();
  onJointTrajectoryCancel();
}

void HrpsysJointTrajectoryAction::onFollowJointTrajectoryActionPreempt()
{
  ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onFollowJointTrajectoryActionPreempt()");
  follow_joint_trajectory_server->setPreempted();
  onJointTrajectoryCancel();
}

void HrpsysJointTrajectoryAction::onTrajectoryCommandCB(const trajectory_msgs::JointTrajectoryConstPtr& msg)
{
  ROS_INFO_STREAM("[" << parent->getInstanceName() << "] @onTrajectoryCommandCB()");
  onJointTrajectory(*msg);
}


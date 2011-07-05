// -*- C++ -*-
/*!
 * @file  Navigation.h
 * @brief compornent
 * @date  $Date$
 *
 * $Id$
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>

#include "intellirobotStub.h"

using namespace RTC;

class Navigation
  : public RTC::DataFlowComponentBase
{
 public:
  Navigation(RTC::Manager* manager);
  ~Navigation();
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

 protected:
  double m_judge_radius;
  double m_max_vel;
  
  IIS::TimedPose2D m_position;
  InPort<IIS::TimedPose2D> m_positionIn;

  IIS::TimedPath2DSeq m_min_path;
  InPort<IIS::TimedPath2DSeq> m_min_pathIn;

  Path m_path;
  OutPort<Path> m_pathOut;
  
  // add:s 2010.08.17 by rtc-center
  TimedState m_status;
  OutPort<TimedState> m_statusOut;
  // add:e 2010.08.17 by rtc-center

 private:
  int dummy;
  int state;
  int current_id;
  int path_num;
};


extern "C"
{
  void NavigationInit(RTC::Manager* manager);
};

#endif // NAVIGATION_H

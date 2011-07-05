// -*- C++ -*-
/*!
 * @file  PathPlanning.h
 * @brief compornent
 * @date  $Date$
 *
 * $Id$
 */

#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>

#include "intellirobotStub.h"
#include "planner.h"

// </rtc-template>

using namespace RTC;

class PathPlanning
  : public RTC::DataFlowComponentBase
{
 public:
  PathPlanning(RTC::Manager* manager);
  ~PathPlanning();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);


 protected:
  std::string m_map_file;
  
  IIS::TimedPose2D m_current_position;
  InPort<IIS::TimedPose2D> m_current_positionIn;
  IIS::TimedPose2D m_goal;
  InPort<IIS::TimedPose2D> m_goalIn;
  
  IIS::TimedPath2DSeq m_min_path;
  OutPort<IIS::TimedPath2DSeq> m_min_pathOut;
  

 private:
  int dummy;
  PathMapPtr all_path;
  PathMapPtr minimum_path;

  int map_loaded;
};


extern "C"
{
  void PathPlanningInit(RTC::Manager* manager);
};

#endif // PATHPLANNING_H

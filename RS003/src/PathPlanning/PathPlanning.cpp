// -*- C++ -*-
/*!
 * @file  PathPlanning.cpp
 * @brief compornent
 * $Date$
 *
 * $Id$
 */

#include "PathPlanning.h"
#include "planner.h"
#include <math.h>
#include "PathType.h"

void path2pathseq(PathMapPtr a_path, IIS::TimedPath2DSeq *out_path);

// Module specification
// <rtc-template block="module_spec">
static const char* pathplanning_spec[] =
  {
    "implementation_id", "PathPlanning",
    "type_name",         "PathPlanning",
    "description",       "compornent",
    "version",           "1.0",
    "vendor",            "MyName",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.map_file", "route_map",

    ""
  };
// </rtc-template>

PathPlanning::PathPlanning(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_current_positionIn("current_position", m_current_position),
    m_goalIn("goal", m_goal),
    m_min_pathOut("min_path", m_min_path)
    
{
  registerInPort("current_position", m_current_positionIn);
  registerInPort("goal", m_goalIn);

  registerOutPort("min_path", m_min_pathOut);

  map_loaded = 0;
}

PathPlanning::~PathPlanning()
{
}


RTC::ReturnCode_t PathPlanning::onInitialize()
{
  bindParameter("map_file", m_map_file, "route_map");
 
  return RTC::RTC_OK;
}


RTC::ReturnCode_t PathPlanning::onActivated(RTC::UniqueId ec_id)
{
  all_path = make_path(); 
  if(load_path_file((const char*)m_map_file.c_str(), all_path)) 
    map_loaded = 1;
  return RTC::RTC_OK;
}



RTC::ReturnCode_t PathPlanning::onDeactivated(RTC::UniqueId ec_id)
{
  printf("deactivate\n");
  if(map_loaded)delete_path(all_path);
  map_loaded = 0;

  return RTC::RTC_OK;
}



RTC::ReturnCode_t PathPlanning::onExecute(RTC::UniqueId ec_id)
{
  while(!m_current_positionIn.isEmpty())m_current_positionIn.read();

  if(map_loaded){
    if(!m_goalIn.isEmpty()){
    
      while(!m_goalIn.isEmpty())m_goalIn.read();

      /*input current position*/
      set_start_position(all_path, m_current_position.data.position.x, m_current_position.data.position.y);
      printf("%f %f\n",m_current_position.data.position.x, m_current_position.data.position.y);

      /*input target  position*/
      set_goal_position(all_path, m_goal.data.position.x, m_goal.data.position.y);
      printf("%f %f\n",m_goal.data.position.x, m_goal.data.position.y);

      /*solve*/
      minimum_path=solve_minimum_path(all_path);
      draw_path(all_path,minimum_path);
  
      /**/
      path2pathseq(minimum_path,&m_min_path);
      if(minimum_path)delete_path(minimum_path);
			int pointnum = m_min_path.pose.length();

		  m_min_path.id.length(pointnum+1);
		  m_min_path.pose.length(pointnum+1);
  		m_min_path.velocity.length(pointnum+1);
  		m_min_path.error.length(pointnum+1);

			m_min_path.pose[pointnum].position.x = m_goal.data.position.x;
			m_min_path.pose[pointnum].position.y = m_goal.data.position.y;
			m_min_path.pose[pointnum].heading = m_goal.data.heading;

      m_min_pathOut.write();
    } 
  }
  return RTC::RTC_OK;
}


extern "C"
{
 
  void PathPlanningInit(RTC::Manager* manager)
  {
    RTC::Properties profile(pathplanning_spec);
    manager->registerFactory(profile,
                             RTC::Create<PathPlanning>,
                             RTC::Delete<PathPlanning>);
  }
  
};


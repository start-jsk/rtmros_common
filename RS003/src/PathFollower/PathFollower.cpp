// -*- C++ -*-
/*!
 * @file  PathFollower.cpp
 * @brief compornent
 * $Date$
 *
 * $Id$
 */

#include "PathFollower.h"
#include "PathType.h"
// Module specification
// <rtc-template block="module_spec">
static const char* pathfollower_spec[] =
  {
    "implementation_id", "PathFollower",
    "type_name",         "PathFollower",
    "description",       "compornent",
    "version",           "1.0",
    "vendor",            "MyName",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.max_v", "1.0",
    "conf.default.max_w", "0.3",
    "conf.default.max_acc_v", "0.1",
    "conf.default.max_acc_w", "0.1",
    "conf.default.line_C1", "0.01",
    "conf.default.line_K1", "800",
    "conf.default.line_K2", "300",
    "conf.default.line_K3", "400",
    "conf.default.line_Dist", "0.05",
    "conf.default.control_cycle", "0.02",

    ""
  };
// </rtc-template>

PathFollower::PathFollower(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_positionIn("position", m_position),
    m_target_pathIn("target_path", m_target_path),
    m_velocityOut("velocity", m_velocity),
    
    // </rtc-template>
	dummy(0)
{
  registerInPort("position", m_positionIn);
  registerInPort("target_path", m_target_pathIn);
  
  // Set OutPort buffer
  registerOutPort("velocity", m_velocityOut);

  initPathFollower();
}

PathFollower::~PathFollower()
{
}


RTC::ReturnCode_t PathFollower::onInitialize()
{
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("max_v", m_max_v, "1.0");
  bindParameter("max_w", m_max_w, "0.3");
  bindParameter("max_acc_v", m_max_acc_v, "0.1");
  bindParameter("max_acc_w", m_max_acc_w, "0.1");
  bindParameter("line_C1", m_line_C1, "0.01");
  bindParameter("line_K1", m_line_K1, "800");
  bindParameter("line_K2", m_line_K2, "300");
  bindParameter("line_K3", m_line_K3, "400");
  bindParameter("line_Dist", m_line_Dist, "0.05");
  bindParameter("control_cycle", m_control_cycle, "0.02");
  
  // </rtc-template>
  return RTC::RTC_OK;
}



RTC::ReturnCode_t PathFollower::onActivated(RTC::UniqueId ec_id)
{
  run_mode = RUN_STOP;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t PathFollower::onExecute(RTC::UniqueId ec_id)
{
  /*next_path*/
  if(!m_target_pathIn.isEmpty()){
    m_target_pathIn.read();
    path_x = m_target_path.x;
    path_y = m_target_path.y;
    path_theta = m_target_path.theta;
    path_v = m_target_path.v;
    path_w = m_target_path.w;
    run_mode = m_target_path.type;
    printf("%d %f %f %f %f %f\n",run_mode,path_x,path_y,path_theta,path_v,path_w);
  }

  /*input odometry */
  while(!m_positionIn.isEmpty())m_positionIn.read();

  now_x = m_position.data.position.x;
  now_y = m_position.data.position.y;
  now_theta = m_position.data.heading;

  /*path following*/
  path_following(now_x,now_y,now_theta);

  /*output velocity*/
  m_velocity.data.vx = target_v*1.5;
  m_velocity.data.vy = 0;
  m_velocity.data.va = target_w*1.5;
  m_velocityOut.write();

  return RTC::RTC_OK;
}


extern "C"
{

  void PathFollowerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(pathfollower_spec);
    manager->registerFactory(profile,
                             RTC::Create<PathFollower>,
                             RTC::Delete<PathFollower>);
  }

};



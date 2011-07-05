// -*- C++ -*-
/*!
 * @file  PathFollower.h
 * @brief compornent
 * @date  $Date$
 *
 * $Id$
 */

#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include "intellirobotStub.h"

// </rtc-template>

using namespace RTC;

class PathFollower
  : public RTC::DataFlowComponentBase
{
 public:
  PathFollower(RTC::Manager* manager);
  ~PathFollower();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry()
 virtual RTC::ReturnCode_t onInitialize();
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  double m_max_v;
  double m_max_w;
  double m_max_acc_v;
  double m_max_acc_w;
  double m_line_C1;
  double m_line_K1;
  double m_line_K2;
  double m_line_K3;
  double m_line_Dist;
  double m_control_cycle;
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  IIS::TimedPose2D m_position;
  InPort<IIS::TimedPose2D> m_positionIn;
  Path m_target_path;
  InPort<Path> m_target_pathIn;

  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  IIS::TimedVelocity2D m_velocity;
  OutPort<IIS::TimedVelocity2D> m_velocityOut;
  
 private:
  int dummy;
  double now_x;
  double now_y;
  double now_theta;
  double target_v;
  double target_w;

  int run_mode;

  double path_x;
  double path_y;
  double path_theta;
  double path_v;
  double path_w;
  double path_d;

  int do_control;


/*±ß¸ÌÄÉ½¾*/
double circle_follow(double x,double y, double theta,
		     double cx, double cy, double cradius,
		     double v_max);

/*Ä¾ÀþÄÉ½¾*/
double line_follow(double x,double y,double theta,
		   double cx, double cy, double ctheta,
		   double v_max);

/*µ°À×ÄÉ½¾¥ì¥®¥å¥ì¡¼¥¿*/
  double regurator(double d, double q, double r,  double v_max);

/*²óÅ¾*/
  double spin(double theta, double target_theta, double w_max);
/*ÅÀ¤Þ¤Ç¤Îµ÷Î¥*/
  double dist_pos(double x,double y, double tx, double ty);


/*Ä¾Àþ¤ÎÃ¼ÅÀ¤Þ¤Ç°ÜÆ°¤·»ß¤Þ¤ë*/
  int to_point(double x, double y,double theta,
	       double tx, double ty, double ttheta,double max_vel);
/**/
  int robot_speed_smooth(double v, double w);
  void path_following(double x, double y, double theta);
  void initPathFollower(void);
  void finalizePathFollower(void);


};


extern "C"
{
  void PathFollowerInit(RTC::Manager* manager);
};

#endif // PATHFOLLOWER_H

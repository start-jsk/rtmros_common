// -*- C++ -*-
/*!
 * @file  Navigation.cpp
 * @brief compornent
 * $Date$
 *
 * $Id$
 */

#include "Navigation.h"
#include "PathType.h"
#include<math.h>

// Module specification
// <rtc-template block="module_spec">
static const char* navigation_spec[] =
  {
    "implementation_id", "Navigation",
    "type_name",         "Navigation",
    "description",       "compornent",
    "version",           "1.0",
    "vendor",            "MyName",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.judge_radius", "0.5",
    "conf.default.max_vel", "0.3",

    ""
  };
// </rtc-template>


enum{
  FINISH = 0,
  ANGLE_CHECK,
  TURNING,
  TURNING2,
  GOAL_CHECK,
  SPEED_DOWN,
  STOPPING
};

Navigation::Navigation(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_positionIn("position", m_position),
    m_min_pathIn("min_path", m_min_path),
    m_pathOut("path", m_path),
    // add:s 2010.08.17 by rtc-center
    m_statusOut("status", m_status),
    // add:e 2010.08.17 by rtc-center
    // </rtc-template>
	dummy(0)
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  registerInPort("position", m_positionIn);
  registerInPort("min_path", m_min_pathIn);
  
  // Set OutPort buffer
  registerOutPort("path", m_pathOut);
  // add:s 2010.08.17 by rtc-center
  registerOutPort("status", m_statusOut);
  // add:e 2010.08.17 by rtc-center
}

Navigation::~Navigation()
{
}


RTC::ReturnCode_t Navigation::onInitialize()
{
  bindParameter("judge_radius", m_judge_radius, "0.5");
  bindParameter("max_vel", m_max_vel, "0.3");

  printf("Navigation Component ver.0.1\n Initialised\n\n");  
  // </rtc-template>
  return RTC::RTC_OK;
}



RTC::ReturnCode_t Navigation::onActivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}

double angle_diff(double theta, double target){
  double theta_diff;
  theta_diff = theta-target;

  while(theta_diff < -M_PI)theta_diff+=2*M_PI;
  while(theta_diff > M_PI)theta_diff-=2*M_PI;

  return fabs(theta_diff);
}

double distance(double x, double y, double tx,double ty){
  return sqrt((x-tx)*(x-tx)+(y-ty)*(y-ty));
}

double over_line(double x,double y, double tx, double ty,double ttheta){
  double rx,ry;

  rx = (x-tx)*cos(-ttheta)-(y-ty)*sin(-ttheta);
  ry = (x-tx)*sin(-ttheta)+(y-ty)*cos(-ttheta);

  return rx;
}

RTC::ReturnCode_t Navigation::onExecute(RTC::UniqueId ec_id)
{
int i;


 if(!m_min_pathIn.isEmpty()){
   while(!m_min_pathIn.isEmpty())
     m_min_pathIn.read();
   
    path_num = m_min_path.pose.length();
    state = ANGLE_CHECK;
    current_id = -1;//H.T 20100824
		for(i = 0;i < path_num;i++){
      printf("%d %f %f %f\n",i, 
	     m_min_path.pose[i].position.x,
	     m_min_path.pose[i].position.y,
	     m_min_path.pose[i].heading);
    }
  }

  // H.T 20100824 現在位置・姿勢をもとにパスの初期点を設定し直す．
	// 現状ではm_positionInに位置情報が来ない限り動作が始まらないことになっている．
  if(current_id == -1){
    while(!m_positionIn.isEmpty())
      m_positionIn.read();
		m_min_path.pose[0].position.x = m_position.data.position.x;
		m_min_path.pose[0].position.y = m_position.data.position.y;
		m_min_path.pose[0].heading = m_position.data.heading;
		current_id = 0;
	}

  /*If odometry is updated*/
  if(!m_positionIn.isEmpty()){
    while(!m_positionIn.isEmpty())
      m_positionIn.read();

	
    switch(state){
    case ANGLE_CHECK:
      if(current_id != path_num-1){
				if(angle_diff(m_min_path.pose[current_id].heading,m_position.data.heading)> M_PI/6
					 /*&& current_id != 0*/){
					m_path.type = RUN_SPIN;
					m_path.theta = m_min_path.pose[current_id].heading;
					m_path.w = 1;
					m_pathOut.write();
					state = TURNING;
					printf("%d turn\n",current_id);
				}else{
					state = GOAL_CHECK;
					printf("%d no turn\n",current_id);
				}
      }else{/*last*/
	m_path.type = RUN_SPIN;
	m_path.theta = m_min_path.pose[current_id].heading;
	m_path.w = 1;
	m_pathOut.write();

	state = TURNING2;
      }
      break;

    case TURNING:
      if(angle_diff(m_min_path.pose[current_id].heading,m_position.data.heading)< M_PI/20){
	printf("%d turned\n",current_id);

	state = GOAL_CHECK;
      }
      break;

    case TURNING2:
      if(angle_diff(m_min_path.pose[current_id].heading,m_position.data.heading)< 0.02){
	m_path.type = RUN_STOP;
	m_pathOut.write();
	state = FINISH;

	printf("%d goal\n",current_id);
      }
      break;

    case GOAL_CHECK:
       if(/*1 || */distance(m_min_path.pose[current_id].position.x, //H.T 20100824 1をコメントアウト
			m_min_path.pose[current_id].position.y,
			m_position.data.position.x, m_position.data.position.y)> 0.05){ //H.T 20100825 0.3 -> 0.05
	m_path.type = RUN_LINEFOLLOW;
	m_path.x = m_min_path.pose[current_id].position.x;
	m_path.y = m_min_path.pose[current_id].position.y;
	m_path.theta = m_min_path.pose[current_id].heading;
	m_path.v = m_max_vel;
	m_pathOut.write();
	state = SPEED_DOWN;
	printf("%d line follow\n",current_id);
      } else{
	m_path.type = RUN_STOP;
	m_pathOut.write();
	state = ANGLE_CHECK;
	current_id++;
      }
	break;

    case SPEED_DOWN:
      //       if(distance(m_min_path.path_list[current_id].x, m_min_path.path_list[current_id].y,
      //	 m_position.x, m_position.y)< 1.2){
      if(over_line(m_position.data.position.x, m_position.data.position.y,
		   m_min_path.pose[current_id].position.x, 
		   m_min_path.pose[current_id].position.y,
		   m_min_path.pose[current_id].heading)>-m_judge_radius){
	m_path.type = RUN_LINEFOLLOW;
	m_path.x = m_min_path.pose[current_id].position.x;
	m_path.y = m_min_path.pose[current_id].position.y;
	 m_path.theta = m_min_path.pose[current_id].heading;
	 m_path.v = 0.1;
	 m_pathOut.write();
	 state = STOPPING;
	 printf("%d stopping\n",current_id);
      }
      break;

    case STOPPING:
	if(over_line(m_position.data.position.x,m_position.data.position.y,
		     m_min_path.pose[current_id].position.x, 
		     m_min_path.pose[current_id].position.y,
		     m_min_path.pose[current_id].heading)>0){
	  
	m_path.type = RUN_STOP;
	m_pathOut.write();
	state = ANGLE_CHECK;
	current_id++;
	printf("%d stop\n",current_id);
      }else{
	printf("%d reach %f\n",current_id,distance(m_min_path.pose[current_id].position.x, m_min_path.pose[current_id].position.y,
		  m_position.data.position.x, m_position.data.position.y));    
      }
      break;

    case FINISH:

      break;
    }
    //    printf(".\n");
  }

  //add:s 2010.08.17 by rtc-center
  m_status.data = state;
  m_statusOut.write();
  //add:e 2010.08.17 by rtc-center

  return RTC::RTC_OK;
}


extern "C"
{

  void NavigationInit(RTC::Manager* manager)
  {
    RTC::Properties profile(navigation_spec);
    manager->registerFactory(profile,
                             RTC::Create<Navigation>,
                             RTC::Delete<Navigation>);
  }

};



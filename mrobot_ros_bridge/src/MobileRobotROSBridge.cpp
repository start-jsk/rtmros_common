// -*- C++ -*-
/*!
 * @file  MobileRobotROSBridge.cpp * @brief MobileRobot-ROS bridge * $Date$ 
 *
 * $Id$ 
 */
#include "MobileRobotROSBridge.h"

// Module specification
// <rtc-template block="module_spec">
static const char* mobilerobotrosbridge_spec[] =
  {
    "implementation_id", "MobileRobotROSBridge",
    "type_name",         "MobileRobotROSBridge",
    "description",       "MobileRobot-ROS bridge",
    "version",           "1.0",
    "vendor",            "Kei Okada",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

MobileRobotROSBridge::MobileRobotROSBridge(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_inIn("in", m_in),
    m_outOut("out", m_out)
    // </rtc-template>
{
}

MobileRobotROSBridge::~MobileRobotROSBridge()
{
}


RTC::ReturnCode_t MobileRobotROSBridge::onInitialize()
{
  std::cerr << "@Initilize name : " << getInstanceName() << std::endl;
  odometry_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
  velocity_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &MobileRobotROSBridge::velocityCB, this);
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("in", m_inIn);

  // Set OutPort buffer
  addOutPort("out", m_outOut);

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t MobileRobotROSBridge::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MobileRobotROSBridge::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MobileRobotROSBridge::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MobileRobotROSBridge::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MobileRobotROSBridge::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t MobileRobotROSBridge::onExecute(RTC::UniqueId ec_id)
{
  //std::cerr << "@Execute name : " << getInstanceName() << std::endl;
  if ( m_inIn.isNew() ) {
    m_inIn.read();
    std::cerr << "x = " << m_in.x << ", y = " << m_in.y << ", theta = " << m_in.theta << std::endl;
    //
    m_out.vx = 0.5;
    m_out.vy = 0.0;
    m_out.w  = 0.0;
    m_outOut.write();
    //
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "";
    odom.pose.pose.position.x = m_in.x;
    odom.pose.pose.position.y = m_in.y;
    odom.pose.pose.orientation.w = atan(m_in.theta/2.0);
    odom.pose.pose.orientation.z = sqrt(1.0 - odom.pose.pose.orientation.w*odom.pose.pose.orientation.w);
    odometry_pub.publish(odom);
    ros::spinOnce();
  }
  return RTC::RTC_OK;
}

void MobileRobotROSBridge::velocityCB(const geometry_msgs::TwistConstPtr& velocity) {
  std::cerr << "[velocity] x = " << velocity->linear.x << ", y = " << velocity->linear.y << ", z = " << velocity->angular.z << std::endl;
}
/*
RTC::ReturnCode_t MobileRobotROSBridge::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MobileRobotROSBridge::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MobileRobotROSBridge::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MobileRobotROSBridge::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MobileRobotROSBridge::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void MobileRobotROSBridgeInit(RTC::Manager* manager)
  {
    coil::Properties profile(mobilerobotrosbridge_spec);
    manager->registerFactory(profile,
                             RTC::Create<MobileRobotROSBridge>,
                             RTC::Delete<MobileRobotROSBridge>);
  }
  
};




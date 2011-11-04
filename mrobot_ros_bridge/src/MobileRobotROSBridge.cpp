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
    ros::param::param<double>("x_coe",x_coe_,1.0);
    ros::param::param<double>("y_coe",y_coe_,1.0);
    ros::param::param<double>("z_coe",z_coe_,1.0);
}

MobileRobotROSBridge::~MobileRobotROSBridge()
{
}


RTC::ReturnCode_t MobileRobotROSBridge::onInitialize()
{
  std::cerr << "@onInitilize name : " << getInstanceName() << std::endl;
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
  //std::cerr << "@onExecute name : " << getInstanceName() << std::endl;
  if ( m_inIn.isNew() ) {
    m_inIn.read();
    std::cerr << "[odometry] x = " << m_in.x << ", y = " << m_in.y << ", theta = " << m_in.theta << std::endl;
    //
    if((ros::Time::now() - latest_v).toSec() < 0.5) {
      m_out.vx = velocity.linear.x * x_coe_;
      m_out.vy = velocity.linear.y * y_coe_;
      m_out.w  = velocity.angular.z * z_coe_;
    } else {
      m_out.vx = m_out.vy = m_out.w  = 0;
    }
    m_outOut.write();
    //
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "base_footprint";
    odom.pose.pose.position.x = m_in.x;
    odom.pose.pose.position.y = m_in.y;
    odom.pose.pose.orientation.w = cos(m_in.theta/2.0);
    odom.pose.pose.orientation.z = sin(m_in.theta/2.0);
    odometry_pub.publish(odom);

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(m_in.x, m_in.y, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0,
					  odom.pose.pose.orientation.z,
					  odom.pose.pose.orientation.w) );
    tf_pub.sendTransform(tf::StampedTransform(transform, odom.header.stamp, "/odom", "/base_link"));

    //calling all callbacks
    while(!ros::getGlobalCallbackQueue()->isEmpty())
      ros::spinOnce();
  }
  return RTC::RTC_OK;
}

void MobileRobotROSBridge::velocityCB(const geometry_msgs::TwistConstPtr& msg) {
  latest_v = ros::Time::now();
  velocity = geometry_msgs::Twist(*msg);
  std::cerr << "[velocity] x = " << velocity.linear.x << ", y = " << velocity.linear.y << ", z = " << velocity.angular.z << std::endl;
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




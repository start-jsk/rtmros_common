// -*- C++ -*-
/*!
 * @file  RangeSensorROSBridge.cpp * @brief RangeSensor-ROS bridge * $Date$ 
 *
 * $Id$ 
 */
#include "RangeSensorROSBridge.h"

// Module specification
// <rtc-template block="module_spec">
static const char* rangesensorrosbridge_spec[] =
  {
    "implementation_id", "RangeSensorROSBridge",
    "type_name",         "RangeSensorROSBridge",
    "description",       "MobileRobot-ROS bridge",
    "version",           "1.0",
    "vendor",            "Manabu Saito",
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

RangeSensorROSBridge::RangeSensorROSBridge(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_inIn("in", m_in)
    // </rtc-template>
{
}

RangeSensorROSBridge::~RangeSensorROSBridge()
{
}


RTC::ReturnCode_t RangeSensorROSBridge::onInitialize()
{
  std::cerr << "@onInitilize name : " << getInstanceName() << std::endl;
  scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("in", m_inIn);

  // Set OutPort buffer
  // addOutPort("out", m_outOut);

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
RTC::ReturnCode_t RangeSensorROSBridge::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RangeSensorROSBridge::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RangeSensorROSBridge::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RangeSensorROSBridge::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RangeSensorROSBridge::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t RangeSensorROSBridge::onExecute(RTC::UniqueId ec_id)
{
  //std::cerr << "@onExecute name : " << getInstanceName() << std::endl;
  if ( m_inIn.isNew() ) {
    m_inIn.read();
    //m_in.tm;   // Simulation Time
    //m_in.data; // sequence<double>
    sensor_msgs::LaserScan scan;
    //scan.header.stamp = ros::Time(m_in.tm.sec, m_in.tm.nsec);
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "laser_link";
    scan.angle_min = -M_PI/2;
    scan.angle_max =  M_PI/2;
    scan.angle_increment = 0.02;
    scan.time_increment = 0.0; // simulation?
    scan.scan_time = 0.1;
    scan.range_min = 0.0;
    scan.range_max = 10.0;

    scan.ranges.resize(m_in.data.length());
    for(int i=0; i<m_in.data.length(); i++)
      scan.ranges[i] = m_in.data[i];

    scan_pub.publish(scan);
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RangeSensorROSBridge::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RangeSensorROSBridge::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RangeSensorROSBridge::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RangeSensorROSBridge::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RangeSensorROSBridge::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void RangeSensorROSBridgeInit(RTC::Manager* manager)
  {
    coil::Properties profile(rangesensorrosbridge_spec);
    manager->registerFactory(profile,
                             RTC::Create<RangeSensorROSBridge>,
                             RTC::Delete<RangeSensorROSBridge>);
  }
  
};




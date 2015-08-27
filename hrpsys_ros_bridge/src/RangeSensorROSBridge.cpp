// -*- C++ -*-
/*!
 * @file  RangeSensorROSBridge.cpp * @brief openhrp image - ros bridge * $Date$ 
 *
 * $Id$ 
 */
#include "RangeSensorROSBridge.h"

// Module specification
// <rtc-template block="module_spec">
static const char* imagesensorrosbridge_spec[] =
  {
    "implementation_id", "RangeSensorROSBridge",
    "type_name",         "RangeSensorROSBridge",
    "description",       "rtm range data - ros bridge",
    "version",           "1.0",
    "vendor",            "JSK",
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
    m_rangeIn("range", m_range)
{
}

RangeSensorROSBridge::~RangeSensorROSBridge()
{
}


RTC::ReturnCode_t RangeSensorROSBridge::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("range", m_rangeIn);

  // Set OutPort buffer

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  range_pub = node.advertise<sensor_msgs::LaserScan>("range", 1);

  // initialize
  ROS_INFO_STREAM("[RangeSensorROSBridge] @Initilize name : " << getInstanceName());

  pair_id = 0;
  ros::param::param<std::string>("~frame_id", _frame_id, "range");
  use_intensities = false;
  if(ros::param::has("~intensity")) {
    use_intensities = true;
    ros::param::get("~intensity", initial_intensity);
  }
  tm.tick();
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
  //capture_time = ros::Time::now();

  if (m_rangeIn.isNew()){ // m_range

    ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecute ec_id : " << ec_id << ", image:" << m_rangeIn.isNew ());

    m_rangeIn.read();
    //
    sensor_msgs::LaserScan ls;
    //m_range.tm;
    ls.header.frame_id = _frame_id;
    ls.header.stamp = ros::Time(m_range.tm.sec, m_range.tm.nsec);
    ls.header.seq = pair_id; pair_id++;
    //m_range.geometry;
    ls.angle_min = m_range.config.minAngle;
    ls.angle_max = m_range.config.maxAngle;
    ls.angle_increment = m_range.config.angularRes;
    //m_range.config.rangeRes; ??
    //ls.time_increment = 1 / m_range.config.frequency * (ls.angle_max - ls.angle_min) / PI / m_range.ranges.length(); // ???
    ls.time_increment = 0.0;
    ls.scan_time = 1 / m_range.config.frequency;
    ls.range_min = m_range.config.minRange;
    ls.range_max = m_range.config.maxRange;
    ls.ranges.resize(m_range.ranges.length());
    if (use_intensities) {
      ls.intensities.resize(m_range.ranges.length());
    }
    for(int i = 0; i < ls.ranges.size(); i++) {
      ls.ranges[i] = m_range.ranges[i];
      if (use_intensities) {
        ls.intensities[i] = initial_intensity;
      }
    }
    range_pub.publish(ls);

    static int count = 0;
    tm.tack();
    if ( tm.interval() > 1 ) {
      ROS_INFO_STREAM("[" << getInstanceName() << "] @onExecutece " << ec_id << " is working at " << count << "[Hz]");
      tm.tick();
      count = 0;
    }
    count ++;
  } else {  // m_range
    double interval = 5;
    tm.tack();
    if ( tm.interval() > interval ) {
      ROS_WARN_STREAM("[" << getInstanceName() << "] @onExecutece " << ec_id << " is not executed last " << interval << "[sec]");
      tm.tick();
    }
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
    coil::Properties profile(imagesensorrosbridge_spec);
    manager->registerFactory(profile,
                             RTC::Create<RangeSensorROSBridge>,
                             RTC::Delete<RangeSensorROSBridge>);
  }
  
};

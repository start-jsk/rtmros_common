// -*- C++ -*-
/*!
 * @file  HrpsysStatePublisher.h * @brief HrpsysState component * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef HRPSYSSTATEPUBLISHER_H
#define HRPSYSSTATEPUBLISHER_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaNaming.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

// hrp
#include <hrpCorba/ModelLoader.hh>
#include <hrpModel/Link.h>
#include <hrpModel/ModelLoaderUtil.h>


// ros
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class HrpsysStatePublisher  : public RTC::DataFlowComponentBase
{
 public:
  HrpsysStatePublisher(RTC::Manager* manager);
  ~HrpsysStatePublisher();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry() 
 virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  // virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  // virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  // virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_in_rsangle;
  InPort<TimedDoubleSeq> m_in_rsangleIn;
  TimedDoubleSeq m_in_mcangle;
  InPort<TimedDoubleSeq> m_in_mcangleIn;
  TimedDoubleSeq m_in_rsrfsensor;
  InPort<TimedDoubleSeq> m_in_rsrfsensorIn;
  TimedDoubleSeq m_in_rslfsensor;
  InPort<TimedDoubleSeq> m_in_rslfsensorIn;
  TimedDoubleSeq m_in_rsrhsensor;
  InPort<TimedDoubleSeq> m_in_rsrhsensorIn;
  TimedDoubleSeq m_in_rslhsensor;
  InPort<TimedDoubleSeq> m_in_rslhsensorIn;
  TimedDoubleSeq m_in_gsensor;
  InPort<TimedDoubleSeq> m_in_gsensorIn;
  TimedDoubleSeq m_in_gyrometer;
  InPort<TimedDoubleSeq> m_in_gyrometerIn;

  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">

  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">

  // </rtc-template>

 private:

  hrp::BodyPtr body;

  ros::NodeHandle nh;
  ros::Publisher joint_state_pub;
};


extern "C"
{
  DLL_EXPORT void HrpsysStatePublisherInit(RTC::Manager* manager);
};

#endif // HRPSYSSTATEPUBLISHER_H


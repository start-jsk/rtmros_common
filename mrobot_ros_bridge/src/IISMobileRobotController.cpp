// -*- C++ -*-
/*!
 * @file  IISMobileRobotController.cpp * @brief IIS Mobile Robot Controller * $Date$ 
 *
 * $Id$ 
 */
#include "IISMobileRobotController.h"

// Module specification
// <rtc-template block="module_spec">
static const char* iismobilerobotcontroller_spec[] =
  {
    "implementation_id", "IISMobileRobotController",
    "type_name",         "IISMobileRobotController",
    "description",       "IIS Mobile Robot Controller",
    "version",           "1.0.0",
    "vendor",            "JSK",
    "category",          "OpenHRP Controller",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

IISMobileRobotController::IISMobileRobotController(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_angleIn("angle", m_angle),
    m_velocityIn("velocity", m_velocity),
    m_torqueOut("torque", m_torque),
    m_outOut("out", m_out),
    m_inIn("in", m_in)

    // </rtc-template>
{
}

IISMobileRobotController::~IISMobileRobotController()
{
}


RTC::ReturnCode_t IISMobileRobotController::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("angle", m_angleIn);
  addInPort("velocity", m_velocityIn);
  addInPort("in", m_inIn);

  // Set OutPort buffer
  addOutPort("torque", m_torqueOut);
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
RTC::ReturnCode_t IISMobileRobotController::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t IISMobileRobotController::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t IISMobileRobotController::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t IISMobileRobotController::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t IISMobileRobotController::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t IISMobileRobotController::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t IISMobileRobotController::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t IISMobileRobotController::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t IISMobileRobotController::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t IISMobileRobotController::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t IISMobileRobotController::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void IISMobileRobotControllerInit(RTC::Manager* manager)
  {
    coil::Properties profile(iismobilerobotcontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<IISMobileRobotController>,
                             RTC::Delete<IISMobileRobotController>);
  }
  
};




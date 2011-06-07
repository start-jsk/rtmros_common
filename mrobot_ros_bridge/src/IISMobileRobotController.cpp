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
    m_inIn("in", m_in),
    m_outOut("out", m_out)

    // </rtc-template>
{
}

IISMobileRobotController::~IISMobileRobotController()
{
}


RTC::ReturnCode_t IISMobileRobotController::onInitialize()
{
  std::cerr << "@Initilize name : " << getInstanceName() << std::endl;
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

  // initialize
  m_angle.data.length(1);
  m_angle.data[0] = 0.0;
  m_velocity.data.length(2);
  m_velocity.data[0] = m_velocity.data[1] = 0.0;
  m_torque.data.length(4);
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

RTC::ReturnCode_t IISMobileRobotController::onExecute(RTC::UniqueId ec_id)
{
  //std::cerr << "@onExecute name : " << getInstanceName() << std::endl;
  // read data from simulated robot
  if ( m_angleIn.isNew() &&
       m_velocityIn.isNew() ) {
    m_angleIn.read();
    m_velocityIn.read();
    double steerAngle = m_angle.data[0];
    double steerVel = m_velocity.data[0];
    double tireVel = m_velocity.data[1];
    fprintf(stderr, "[simulate] steer angle = %.3f, steer vel = %.3f, tire vel = %.3f\n", steerAngle, steerVel, tireVel);


    double steerCommandAngle = 0;
    double tireCommandVel = 0;
    if ( m_inIn.isNew() ) {

      // read data from client
      m_inIn.read();

      //
      steerCommandAngle = m_in.w ;//* 180.0 / 3.14156;
      tireCommandVel = (m_in.vx + abs(m_in.w));
      fprintf(stderr, "[command]  x = %.3f y = %.3f, w = %.3f\n", m_in.vx, m_in.vy, m_in.w);
    }
    double steerCommandTorque = 4*(steerCommandAngle - steerAngle) - 2.0 * steerVel;
    double tireCommandTorque = (tireCommandVel * 10 - tireVel);

    // write to simulated robot
    m_torque.data[0] = steerCommandTorque;
    m_torque.data[1] = tireCommandTorque;
    m_torque.data[2] = tireCommandTorque;
    m_torque.data[3] = tireCommandTorque;
    m_torqueOut.write();

    m_out.x = tireVel;
    m_out.y = 0;
    m_out.theta = steerAngle;
    m_outOut.write();
  }

  return RTC::RTC_OK;
}

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




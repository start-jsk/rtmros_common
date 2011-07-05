// -*- C++ -*-
/*!
 * @file  PositionInput.cpp * @brief compornent * $Date$ 
 *
 * $Id$ 
 */
#include "PositionInput.h"

// Module specification
// <rtc-template block="module_spec">
static const char* positioninput_spec[] =
  {
    "implementation_id", "PositionInput",
    "type_name",         "PositionInput",
    "description",       "compornent",
    "version",           "1.0",
    "vendor",            "MyName",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.01_x", "3.4",
    "conf.default.02_y", "7.7",
    "conf.default.03_th", "-1.57",
    ""
  };
// </rtc-template>

PositionInput::PositionInput(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_positionOut("position", m_position)

    // </rtc-template>
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer
  registerOutPort("position", m_positionOut);

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

}

PositionInput::~PositionInput()
{
}


RTC::ReturnCode_t PositionInput::onInitialize()
{
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("01_x", m_01_x, "3.4");
  bindParameter("02_y", m_02_y, "7.7");
  bindParameter("03_th", m_03_th, "-1.57");
  // </rtc-template>
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t PositionInput::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t PositionInput::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t PositionInput::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t PositionInput::onActivated(RTC::UniqueId ec_id)
{
  m_position.data.position.x = m_01_x;
  m_position.data.position.y = m_02_y;
  m_position.data.heading    = m_03_th;
  RTC_INFO(("input goal position"))
  RTC_INFO(("%lf %lf %lf",m_position.data.position.x, m_position.data.position.y, m_position.data.heading))
  m_positionOut.write();
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PositionInput::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t PositionInput::onExecute(RTC::UniqueId ec_id)
{
  //printf("input position(x y theta)>");
  //scanf("%lf %lf %lf",&m_position.data.position.x,&m_position.data.position.y,&m_position.data.heading);
  //m_positionOut.write();

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PositionInput::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t PositionInput::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t PositionInput::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t PositionInput::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t PositionInput::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void PositionInputInit(RTC::Manager* manager)
  {
    RTC::Properties profile(positioninput_spec);
    manager->registerFactory(profile,
                             RTC::Create<PositionInput>,
                             RTC::Delete<PositionInput>);
  }
  
};




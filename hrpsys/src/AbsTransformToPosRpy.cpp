// -*- C++ -*-
/*!
 * @file  AbsTransformToPosRpy.cpp * @brief ABS_TRANSFORM from openhrp_controller_bridge to basePosIn and baseRpyIn of StateHolder * $Date$ 
 *
 * $Id$ 
 */
#include "AbsTransformToPosRpy.h"
#include <hrpUtil/Tvmet3d.h>


// Module specification
// <rtc-template block="module_spec">
static const char* abstransformtoposrpy_spec[] =
  {
    "implementation_id", "AbsTransformToPosRpy",
    "type_name",         "AbsTransformToPosRpy",
    "description",       "ABS_TRANSFORM from openhrp_controller_bridge to basePosIn and baseRpyIn of StateHolder",
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

AbsTransformToPosRpy::AbsTransformToPosRpy(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_prIn("pr", m_pr),
    m_basePosOut("basePos", m_basePos),
    m_baseRpyOut("baseRpy", m_baseRpy)

    // </rtc-template>
{
}

AbsTransformToPosRpy::~AbsTransformToPosRpy()
{
}


RTC::ReturnCode_t AbsTransformToPosRpy::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("pr", m_prIn);

  // Set OutPort buffer
  addOutPort("basePos", m_basePosOut);
  addOutPort("baseRpy", m_baseRpyOut);

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>

  tm.tick();
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t AbsTransformToPosRpy::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AbsTransformToPosRpy::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AbsTransformToPosRpy::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AbsTransformToPosRpy::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AbsTransformToPosRpy::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t AbsTransformToPosRpy::onExecute(RTC::UniqueId ec_id)
{
  if ( m_prIn.isNew () ) {
    //
    std::cout << "[" << getInstanceName() << "] @onExecutece " << ec_id << ", pr:" << m_prIn.isNew () << std::endl;
    //
    m_prIn.read();
    //
    for ( unsigned int i = 0; i < m_pr.data.length() ; i++ ) std::cout << std::setw(4) << m_pr.data[i] << " "; std::cout << std::endl;
  }else {
    double interval = 5;
    tm.tack();
    if ( tm.interval() > interval ) {
      std::cout << "[" << getInstanceName() << "] @onExecutece " << ec_id << " is not executed last " << interval << "[sec]" << std::endl;
      tm.tick();
    }
    return RTC::RTC_OK;
  }
  m_basePos.data.x = m_pr.data[0];
  m_basePos.data.y = m_pr.data[1];
  m_basePos.data.z = m_pr.data[2];
  hrp::Matrix33 R;
  hrp::getMatrix33FromRowMajorArray(R, m_pr.data, 3);
  hrp::Vector3 rpy = hrp::rpyFromRot(R);
  m_baseRpy.data.r = rpy[0];
  m_baseRpy.data.p = rpy[1];
  m_baseRpy.data.y = rpy[2];

  // output
  m_basePosOut.write();
  m_baseRpyOut.write();

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t AbsTransformToPosRpy::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AbsTransformToPosRpy::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AbsTransformToPosRpy::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AbsTransformToPosRpy::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AbsTransformToPosRpy::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void AbsTransformToPosRpyInit(RTC::Manager* manager)
  {
    coil::Properties profile(abstransformtoposrpy_spec);
    manager->registerFactory(profile,
                             RTC::Create<AbsTransformToPosRpy>,
                             RTC::Delete<AbsTransformToPosRpy>);
  }
  
};




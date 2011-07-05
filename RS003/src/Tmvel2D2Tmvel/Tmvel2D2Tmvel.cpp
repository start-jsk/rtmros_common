// -*- C++ -*-
/*!
 * @file  Tmvel2D2Tmvel.cpp
 * @brief DataConversionRTC
 * @date $Date$
 *
 * $Id$
 */

#include "Tmvel2D2Tmvel.h"

// Module specification
// <rtc-template block="module_spec">
static const char* Tmvel2D2Tmvel_spec[] =
  {
    "implementation_id", "Tmvel2D2Tmvel",
    "type_name",         "Tmvel2D2Tmvel",
    "description",       "DataConversionRTC",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "tool",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "5",
    "language",          "C++",
    "lang_type",         "compile",
    //"exec_cxt.periodic.rate", "1.0",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
Tmvel2D2Tmvel::Tmvel2D2Tmvel(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_inIn("in", m_in),
    m_outOut("out", m_out)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
Tmvel2D2Tmvel::~Tmvel2D2Tmvel()
{
}



RTC::ReturnCode_t Tmvel2D2Tmvel::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  registerInPort("in", m_inIn);
  
  // Set OutPort buffer
  registerOutPort("out", m_outOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Tmvel2D2Tmvel::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Tmvel2D2Tmvel::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Tmvel2D2Tmvel::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Tmvel2D2Tmvel::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Tmvel2D2Tmvel::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t Tmvel2D2Tmvel::onExecute(RTC::UniqueId ec_id)
{
	while(!m_inIn.isEmpty())
		m_inIn.read();
		
	m_out.vx = m_in.data.vx;
	m_out.vy = m_in.data.vy;
	m_out.w = m_in.data.va;

	m_outOut.write();

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Tmvel2D2Tmvel::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Tmvel2D2Tmvel::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Tmvel2D2Tmvel::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Tmvel2D2Tmvel::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Tmvel2D2Tmvel::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void Tmvel2D2TmvelInit(RTC::Manager* manager)
  {
    coil::Properties profile(Tmvel2D2Tmvel_spec);
    manager->registerFactory(profile,
                             RTC::Create<Tmvel2D2Tmvel>,
                             RTC::Delete<Tmvel2D2Tmvel>);
  }
  
};



// -*- C++ -*-
/*!
 * @file  SimRangeUrg.cpp
 * @brief DataConversionRTC
 * @date $Date$
 *
 * $Id$
 */

#include "SimRangeUrg.h"

// Module specification
// <rtc-template block="module_spec">
static const char* SimRangeUrg_spec[] =
  {
    "implementation_id", "SimRangeUrg",
    "type_name",         "SimRangeUrg",
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
SimRangeUrg::SimRangeUrg(RTC::Manager* manager)
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
SimRangeUrg::~SimRangeUrg()
{
}



RTC::ReturnCode_t SimRangeUrg::onInitialize()
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


RTC::ReturnCode_t SimRangeUrg::onFinalize()
{
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t SimRangeUrg::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimRangeUrg::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimRangeUrg::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimRangeUrg::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t SimRangeUrg::onExecute(RTC::UniqueId ec_id)
{

	if(m_inIn.isNew()){
		m_inIn.read();
		
		for(int i=0; i<(int)m_in.data.length(); i++)
      m_out.data[i] = (long int)(m_in.data[i]*1000);
    printf("DataLen=%d\n", m_in.data.length());
	 	m_outOut.write();
	}

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SimRangeUrg::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimRangeUrg::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimRangeUrg::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimRangeUrg::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimRangeUrg::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void SimRangeUrgInit(RTC::Manager* manager)
  {
    coil::Properties profile(SimRangeUrg_spec);
    manager->registerFactory(profile,
                             RTC::Create<SimRangeUrg>,
                             RTC::Delete<SimRangeUrg>);
  }
  
};



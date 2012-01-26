// -*- C++ -*-
/*!
 * @file  HiroStateConverter.cpp * @brief hiro - ros bridge * $Date$ 
 *
 * $Id$ 
 */
#include "HiroStateConverter.h"

// Module specification
// <rtc-template block="module_spec">
static const char* hirostateconverter_spec[] =
  {
    "implementation_id", "HiroStateConverter",
    "type_name",         "HiroStateConverter",
    "description",       "hiro - ros bridge",
    "version",           "1.0",
    "vendor",            "Kei Okada",
    "category",          "example",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

HiroStateConverter::HiroStateConverter(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_angleOut("angle", m_angle),
    m_rsangleOut("rsangle", m_rsangle),
    m_mcangleOut("mcangle", m_mcangle),
    m_rsrfsensorOut("rsrfsensor", m_rsrfsensor),
    m_rslfsensorOut("rslfsensor", m_rslfsensor),
    m_rsrhsensorOut("rsrhsensor", m_rsrhsensor),
    m_rslhsensorOut("rslhsensor", m_rslhsensor),
    m_gsensorOut("gsensor", m_gsensor),
    m_gyrometerOut("gyrometer", m_gyrometer),
    m_poseOut("pose", m_pose),
    m_jointDataIn("jointData", m_jointData)

    // </rtc-template>
{
}

HiroStateConverter::~HiroStateConverter()
{
}


RTC::ReturnCode_t HiroStateConverter::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("jointData", m_jointDataIn);

  // Set OutPort buffer
  addOutPort("angle", m_angleOut);
  addOutPort("rsangle", m_rsangleOut);
  addOutPort("mcangle", m_mcangleOut);
  addOutPort("rsrfsensor", m_rsrfsensorOut);
  addOutPort("rslfsensor", m_rslfsensorOut);
  addOutPort("rsrhsensor", m_rsrhsensorOut);
  addOutPort("rslhsensor", m_rslhsensorOut);
  addOutPort("gsensor", m_gsensorOut);
  addOutPort("gyrometer", m_gyrometerOut);
  addOutPort("pose", m_poseOut);

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
RTC::ReturnCode_t HiroStateConverter::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HiroStateConverter::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HiroStateConverter::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HiroStateConverter::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HiroStateConverter::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t HiroStateConverter::onExecute(RTC::UniqueId ec_id)
{
  if ( m_jointDataIn.isNew() ) {
    m_jointDataIn.read();
    int k = 0;
    for (int i = 0; i < m_jointData.id.length(); i++ ) {
      k+= m_jointData.qCommand[i].length();
    }
    m_angle.data.length(k);
    k = 0;
    for (int i = 0; i < m_jointData.id.length(); i++ ) {
      for (int j = 0; j < m_jointData.qCommand[i].length(); j++ ) {
	m_angle.data[k] = m_jointData.qCommand[i][j];
      }
    }

    m_angleOut.write();
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t HiroStateConverter::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HiroStateConverter::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HiroStateConverter::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HiroStateConverter::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HiroStateConverter::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void HiroStateConverterInit(RTC::Manager* manager)
  {
    coil::Properties profile(hirostateconverter_spec);
    manager->registerFactory(profile,
                             RTC::Create<HiroStateConverter>,
                             RTC::Delete<HiroStateConverter>);
  }
  
};




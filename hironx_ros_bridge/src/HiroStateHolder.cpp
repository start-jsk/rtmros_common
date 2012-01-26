// -*- C++ -*-
/*!
 * @file  HiroStateHolder.cpp * @brief hiro - ros bridge * $Date$ 
 *
 * $Id$ 
 */
#include "HiroStateHolder.h"
#include <hrpUtil/Tvmet3d.h>


// Module specification
// <rtc-template block="module_spec">
static const char* hirostateconverter_spec[] =
  {
    "implementation_id", "HiroStateHolder",
    "type_name",         "HiroStateHolder",
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

HiroStateHolder::HiroStateHolder(RTC::Manager* manager)
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

    m_jointDataIn("jointData", m_jointData),
    m_basePosIn("basePos", m_basePos),
    m_baseRpyIn("baseRpy", m_baseRpy)
    // </rtc-template>
{
}

HiroStateHolder::~HiroStateHolder()
{
}


RTC::ReturnCode_t HiroStateHolder::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("jointData", m_jointDataIn);
  addInPort("basePos", m_basePosIn);
  addInPort("baseRpy", m_baseRpyIn);

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
RTC::ReturnCode_t HiroStateHolder::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HiroStateHolder::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HiroStateHolder::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HiroStateHolder::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HiroStateHolder::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t HiroStateHolder::onExecute(RTC::UniqueId ec_id)
{
  if ( m_jointDataIn.isNew() ) {
    m_jointDataIn.read();
    int k = 0;
    for (int i = 0; i < m_jointData.id.length(); i++ ) {
      k+= m_jointData.qCommand[i].length();
    }
    m_angle.data.length(k);
    m_rsangle.data.length(k);
    m_mcangle.data.length(k);
    k = 0;
    for (int i = 0; i < m_jointData.id.length(); i++ ) {
      for (int j = 0; j < m_jointData.qCommand[i].length(); j++ ) {
	m_angle.data[k]   = m_jointData.qState[i][j];
	m_mcangle.data[k] = m_jointData.qCommand[i][j];
	m_rsangle.data[k] = m_jointData.qState[i][j];
	k++;
      }
    }

    m_angleOut.write();
    m_mcangleOut.write();
    m_rsangleOut.write();
  }

  if ( m_basePosIn.isNew() && m_baseRpyIn.isNew() ) {
    m_basePosIn.read(); m_baseRpyIn.read();
    m_pose.data.position.x = m_basePos.data[0];
    m_pose.data.position.y = m_basePos.data[1];
    m_pose.data.position.z = m_basePos.data[2];
    hrp::Matrix33 R;
    hrp::getMatrix33FromRowMajorArray(R, m_baseRpy.data, 3);
    hrp::Vector3 rpy = hrp::rpyFromRot(R);
    m_pose.data.orientation.r = rpy[0];
    m_pose.data.orientation.p = rpy[1];
    m_pose.data.orientation.y = rpy[2];
    m_poseOut.write();
    }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t HiroStateHolder::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HiroStateHolder::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HiroStateHolder::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HiroStateHolder::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HiroStateHolder::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void HiroStateHolderInit(RTC::Manager* manager)
  {
    coil::Properties profile(hirostateconverter_spec);
    manager->registerFactory(profile,
                             RTC::Create<HiroStateHolder>,
                             RTC::Delete<HiroStateHolder>);
  }
  
};




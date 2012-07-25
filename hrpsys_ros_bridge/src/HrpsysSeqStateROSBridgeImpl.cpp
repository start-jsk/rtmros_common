// -*- C++ -*-
/*!
 * @file  HrpsysSeqStateROSBridgeImpl.cpp * @brief hrpsys seq state - ros bridge * $Date$ 
 *
 * $Id$ 
 */
#include "HrpsysSeqStateROSBridgeImpl.h"

// Module specification
// <rtc-template block="module_spec">
static const char* hrpsysseqstaterosbridgeimpl_spec[] =
  {
    "implementation_id", "HrpsysSeqStateROSBridgeImpl",
    "type_name",         "HrpsysSeqStateROSBridgeImpl",
    "description",       "hrpsys seq state - ros bridge",
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

HrpsysSeqStateROSBridgeImpl::HrpsysSeqStateROSBridgeImpl(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_rsangleIn("rsangle", m_rsangle),
    m_mcangleIn("mcangle", m_mcangle),
    m_rsJointTemperatureIn("rsJointTemperature", m_rsJointTemperature),
    m_rsrfsensorIn("rsrfsensor", m_rsrfsensor),
    m_rslfsensorIn("rslfsensor", m_rslfsensor),
    m_rsrhsensorIn("rsrhsensor", m_rsrhsensor),
    m_rslhsensorIn("rslhsensor", m_rslhsensor),
    m_gsensorIn("gsensor", m_gsensor),
    m_gyrometerIn("gyrometer", m_gyrometer),
    m_baseTformIn("baseTform", m_baseTform),
    m_rstorqueIn("rstorque", m_rstorque),
    m_servoStateIn("servoState", m_servoState),
    m_mctorqueOut("mctorque", m_mctorque),
    m_SequencePlayerServicePort("SequencePlayerService")

    // </rtc-template>
{
}

HrpsysSeqStateROSBridgeImpl::~HrpsysSeqStateROSBridgeImpl()
{
}


RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("rsangle", m_rsangleIn);
  addInPort("mcangle", m_mcangleIn);
  addInPort("rsJointTemperature", m_rsJointTemperatureIn);
  addInPort("rsrfsensor", m_rsrfsensorIn);
  addInPort("rslfsensor", m_rslfsensorIn);
  addInPort("rsrhsensor", m_rsrhsensorIn);
  addInPort("rslhsensor", m_rslhsensorIn);
  addInPort("gsensor", m_gsensorIn);
  addInPort("gyrometer", m_gyrometerIn);
  addInPort("baseTform", m_baseTformIn);
  addInPort("rstorque", m_rstorqueIn);
  addInPort("servoState", m_servoStateIn);

  // Set OutPort buffer
  addOutPort("mctorque", m_mctorqueOut);

  // Set service provider to Ports

  // Set service consumers to Ports
  m_SequencePlayerServicePort.registerConsumer("service0", "SequencePlayerService", m_service0);

  // Set CORBA Service Ports
  addPort(m_SequencePlayerServicePort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void HrpsysSeqStateROSBridgeImplInit(RTC::Manager* manager)
  {
    coil::Properties profile(hrpsysseqstaterosbridgeimpl_spec);
    manager->registerFactory(profile,
                             RTC::Create<HrpsysSeqStateROSBridgeImpl>,
                             RTC::Delete<HrpsysSeqStateROSBridgeImpl>);
  }
  
};




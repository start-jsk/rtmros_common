// -*- C++ -*-
/*!
 * @file  SwitchInputRTC.cpp * @brief Swtich input component * $Date$ 
 *
 * $Id$ 
 */
#include "SwitchInputRTC.h"

// Module specification
// <rtc-template block="module_spec">
static const char* switchinputrtc_spec[] =
  {
    "implementation_id", "SwitchInputRTC",
    "type_name",         "SwitchInputRTC",
    "description",       "Swtich input component",
    "version",           "1.0",
    "vendor",            "AIST",
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

SwitchInputRTC::SwitchInputRTC(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_outVelOut("outVel", m_outVel),
    m_inVelJoyIn("inVelJoy", m_inVelJoy),
    m_inVelAutoIn("inVelAuto", m_inVelAuto)

    // </rtc-template>
{
}

SwitchInputRTC::~SwitchInputRTC()
{
}


RTC::ReturnCode_t SwitchInputRTC::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("inVelJoy", m_inVelJoyIn);
  addInPort("inVelAuto", m_inVelAutoIn);

  // Set OutPort buffer
  addOutPort("outVel", m_outVelOut);

  tv0.tv_sec = 0;
  tv1.tv_sec = 0;
  tv0.tv_usec = 0;
  tv1.tv_usec = 0;

  return RTC::RTC_OK;
}


RTC::ReturnCode_t SwitchInputRTC::onExecute(RTC::UniqueId ec_id)
{
  if (m_inVelJoyIn.isNew())
    {
      m_inVelJoyIn.read();
      gettimeofday(&tv0,0);
      std::cout << "JoyStick vx:" << m_inVelJoy.data.vx << "vy:" << m_inVelJoy.data.vy << "TimeStamp: " << tv0.tv_sec << "[s] " << tv0.tv_usec << "[usec]" << std::endl;
      m_outVel.data.vx = m_inVelJoy.data.vx;
      m_outVel.data.vy = m_inVelJoy.data.vy;
      m_outVel.data.va = m_inVelJoy.data.va;
      m_outVelOut.write();
    }
  else if (m_inVelAutoIn.isNew())
    {
      m_inVelAutoIn.read();
      gettimeofday(&tv1,0);

      dsec = tv1.tv_sec - tv0.tv_sec;
      dusec = tv1.tv_usec - tv0.tv_usec;
      dusec = dsec*1000*1000+dusec;
      //printf("--------------- %d\n", dusec);

      if(dusec>500000){
        std::cout << "AUTO  vx:" << m_inVelAuto.data.vx << "vy:" << m_inVelAuto.data.vy << "TimeStamp: " << tv1.tv_sec << "[s] " << tv1.tv_usec << "[usec]" << std::endl;
        m_outVel.data.vx = m_inVelAuto.data.vx;
        m_outVel.data.vy = m_inVelAuto.data.vy;
        m_outVel.data.va = m_inVelAuto.data.va;
        m_outVelOut.write();
      }
    }
  usleep(50000);

  return RTC::RTC_OK;
}


extern "C"
{
 
  void SwitchInputRTCInit(RTC::Manager* manager)
  {
    coil::Properties profile(switchinputrtc_spec);
    manager->registerFactory(profile,
                             RTC::Create<SwitchInputRTC>,
                             RTC::Delete<SwitchInputRTC>);
  }
  
};


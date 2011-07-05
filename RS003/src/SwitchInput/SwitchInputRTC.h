// -*- C++ -*-
/*!
 * @file  SwitchInputRTC.h * @brief Swtich input component * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef SWITCHINPUTRTC_H
#define SWITCHINPUTRTC_H


#include <stdio.h>
#include <sys/time.h>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/RingBuffer.h>

#include "intellirobotStub.h"

using namespace RTC;

class SwitchInputRTC  : public RTC::DataFlowComponentBase
{
 public:
  SwitchInputRTC(RTC::Manager* manager);
  ~SwitchInputRTC();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

 protected:

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  IIS::TimedVelocity2D m_inVelJoy;
  InPort<IIS::TimedVelocity2D> m_inVelJoyIn;
  IIS::TimedVelocity2D m_inVelAuto;
  InPort<IIS::TimedVelocity2D> m_inVelAutoIn;

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  IIS::TimedVelocity2D m_outVel;
  OutPort<IIS::TimedVelocity2D> m_outVelOut;

 private:
  struct timeval tv0, tv1;
  unsigned long dsec, dusec;

};


extern "C"
{
  DLL_EXPORT void SwitchInputRTCInit(RTC::Manager* manager);
};

#endif // SWITCHINPUTRTC_H


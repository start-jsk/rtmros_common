/************************************************************************************
GamePad RT-Component
Copyright (c) 2010, Segway-Japan, Ltd.
All rights reserved.

Contact us if you use this software for sell.
If you use this software not for sell, you can use this software under BSD lisence.
See the files LICENSE.TXT and LICENSE-BSD.TXT for more details.                     
************************************************************************************/

#ifndef GAMEPAD_H
#define GAMEPAD_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>


//If you use windows, you should add additional include path to srcwin
#include "intellirobotStub.h"

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;


/*!
  @brief GamePad RTM-Module class\n
         outport TimedULong 'Buttons','Pushed Buttons' have 1 bit info for each button in 32bit value\n
         outport TimedFloat  'Stick*'  value was adjusted to [-1.0 - 1.0] * coefficient[x,y]\n
         outport TimedDouble 'Stick*d' value was adjusted to [-1.0 - 1.0] * coefficient[x,y]\n
         outport TimedVelocity 'Velocity' v = -StickLYd * coefficienty,  w = -StickLXd * coefficientx \n
*/
class GamePad
  : public RTC::DataFlowComponentBase
{
 public:
  GamePad(RTC::Manager* manager);
  ~GamePad();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry()
 virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  // virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);

  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * 
   * - Name:  Klx
   * - DefaultValue: 1.0
   */
  float m_Klx;
  /*!
   * 
   * - Name:  Kly
   * - DefaultValue: 1.0
   */
  float m_Kly;
  /*!
   * 
   * - Name:  Krx
   * - DefaultValue: 1.0
   */
  float m_Krx;
  /*!
   * 
   * - Name:  Kry
   * - DefaultValue: 1.0
   */
  float m_Kry;
  /*!
   * 
   * - Name:  str_port
   * - DefaultValue: FTDI
   */
  std::string m_str_port;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedULong m_Button;
  /*!
   */
  OutPort<RTC::TimedULong> m_ButtonOut;
  RTC::TimedFloat m_StickLX;
  /*!
   */
  OutPort<RTC::TimedFloat> m_StickLXOut;
  RTC::TimedFloat m_StickLY;
  /*!
   */
  OutPort<RTC::TimedFloat> m_StickLYOut;
  RTC::TimedFloat m_StickRX;
  /*!
   */
  OutPort<RTC::TimedFloat> m_StickRXOut;
  RTC::TimedFloat m_StickRY;
  /*!
   */
  OutPort<RTC::TimedFloat> m_StickRYOut;
  RTC::TimedDouble m_StickLXd;
  /*!
   */
  OutPort<RTC::TimedDouble> m_StickLXdOut;
  RTC::TimedDouble m_StickLYd;
  /*!
   */
  OutPort<RTC::TimedDouble> m_StickLYdOut;
  RTC::TimedDouble m_StickRXd;
  /*!
   */
  OutPort<RTC::TimedDouble> m_StickRXdOut;
  RTC::TimedDouble m_StickRYd;
  /*!
   */
  OutPort<RTC::TimedDouble> m_StickRYdOut;
  RTC::TimedVelocity m_Velocity;
  /*!
   */
  OutPort<RTC::TimedVelocity> m_VelocityOut;


  IIS::TimedVelocity m_VelocityIIS;
  /*!
   */
  OutPort<IIS::TimedVelocity> m_VelocityIISOut;

  IIS::TimedVelocity2D m_Velocity2DIIS;
  /*!
   */
  OutPort<IIS::TimedVelocity2D> m_Velocity2DIISOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  int dummy;

};


extern "C"
{
  void GamePadInit(RTC::Manager* manager);
};

#endif // GAMEPAD_H

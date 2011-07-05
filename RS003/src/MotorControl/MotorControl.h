// -*- C++ -*-
/*!
 * @file  MotorControl.h
 * @brief MotorControl
 * @date  $Date$
 *
 * @author Yusuke Nakajima (y.nakajima@aist.go.jp)
 *
 * Copyright (c) 2008, National Institute of Advanced Industrial Science and Tec
 * hnology (AIST). All rights reserved. This program is made available under the te
 * rms of the Eclipse Public License v1.0 which accompanies this distribution, and 
 * is available at http://www.eclipse.org/legal/epl-v10.html
 *
 * $Id$
 */

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
//#include <rtm/idl/IIS.h>

//--- [add] ---------------
#include "IISStub.h"
#include "IISSkel.h"
#include "IIS.hh"

#define LOG 0
#define DEBUG 0
#define CALLBACK 0
#define CALLBACK_DEBUG 0
#include <iostream>
#include <fstream>
#include <math.h>

#if CALLBACK		
#include <rtm/RingBuffer.h>
//-CallBackObject--
class Debug
	: public RTC::OnWriteConvert<RTC::TimedDoubleSeq>
{
	//ofstream ofsCallBack;
	public:
		Debug(void) {	
		  //ofsCallBack.open("data/callback.dat");  
		};
		RTC::TimedDoubleSeq operator()(const RTC::TimedDoubleSeq& value)
		{
			RTC::TimedDoubleSeq ret(value);
#if CALLBACK_DEBUG	
			double dummy_sec = value.tm.sec + 1.0e-9*value.tm.nsec;
			printf("--[CallBack_TIME(CURRENT)]: %lf\t[L]=%lf\t[R]=%lf\n",dummy_sec, value.data[0], value.data[1]);
#endif
			return ret;
		};

};
//--
#endif
//--- [add] ---------------

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "MotorControlProSVC_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;
using namespace IIS;

/*!
 * @class MotorControl
 * @brief MotorControl
 *
 * (1)getting target trans/rot Velocity of robot's center data from DriveControl
 * RTC
 * (2)calculating Torque values
 * (3)sending Torque data to Simulator or real Motor
 * (4)getting current Angle data (encoder dadta) from Simulator or Real Robot
 * (5)sending current Angle data  to DriveControlRTC and OdometryRTC
 *
 * [INPUT]
 *   (1) target Trans/Rot Velocity data
 *   (2) current Angle data (encoder data)
 * [OUTPUT]
 *   (1) Torque data
 *   (2) current Angle data
 *
 * [Calculating Torque values using current/target angle/AngVel values]
 *   [Torque] = Pgain * ([TargetAngle] - [CurrentAngle]) * Dgain * ([TargetAngVel] 
 * - [CurrentAngVel])
 *
 */
class MotorControl
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  MotorControl(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~MotorControl();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   * set Configuration parameters
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * @pre none
   * @post none
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   * 
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   * (1) update INPORT's data
   * (2) open files for debugging
   * (3) initialize
   * (4) get robot's parameter
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * @pre none
   * @post none
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   * (1)close files for debugging
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * @pre none
   * @post none
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   * (1)getting target trans/rot Velocity of robot's center data from DriveContr
   * olRTC
   * (2)calculating Torque values
   * (3)sending Torque data to Simulator or real Motor
   * (4)getting current Angle data (encoder dadta) from Simulator or Real Robot
   * (5)sending current Angle data  to DriveControlRTC or OdometryRTC
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * @pre none
   * @post none
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * Left Wheel's P gain value of PD control
   * - Name: PGainL PGainL
   * - DefaultValue: 5.0
   */
  double m_PGainL;
  /*!
   * Left Wheel's D gain value of PD control
   * - Name: DGainL DGainL
   * - DefaultValue: 2.0
   */
  double m_DGainL;
  /*!
   * Right Wheel's P gain value of PD control
   * - Name: PGainR PGainR
   * - DefaultValue: 5.0
   */
  double m_PGainR;
  /*!
   * Right Wheel's D gain value of PD control
   * - Name: DGainR DGainR
   * - DefaultValue: 2.0
   */
  double m_DGainR;
  /*!
   * distance from CenterOfBody to CenterOfAxle of Model
   * - Name: SimulatedOffsetX SimulatedOffsetX
   * - DefaultValue: 0.0
   */
  double m_SimulatedOffsetX;
  /*!
   * distance from CenterOfBody to CenterOfAxle of Model
   * - Name: SimulatedOffsetY SimulatedOffsetY
   * - DefaultValue: 0.0
   */
  double m_SimulatedOffsetY;
  /*!
   * offset for adjustment of initial posture angle
   * - Name: SimulatedOffsetAngle SimulatedOffsetAngle
   * - DefaultValue: 0.0
   */
  double m_SimulatedOffsetAngle;
  /*!
   * ID number of left wheel of mobile robot.
   * - Name: leftWheelID leftWheelID
   * - DefaultValue: 0
   * - Unit: none
   * - Constraint: none
   */
  int m_leftWheelID;
  /*!
   * ID number of right wheel of mobile robot.
   * - Name: rightWheelID rightWheelID
   * - DefaultValue: 1
   * - Unit: none
   * - Constraint: none
   */
  int m_rightWheelID;
  /*!
   * radius of left wheel of mobile robot
   * - Name: radiusOfLeftWheel radiusOfLeftWheel
   * - DefaultValue: 0.1
   * - Unit: [m]
   * - Constraint: none
   */
  double m_radiusOfLeftWheel;
  /*!
   * radius of right wheel of mobile robot
   * - Name: radiusOfRightWheel radiusOfRightWheel
   * - DefaultValue: 0.1
   * - Unit: [m]
   * - Constraint: none
   */
  double m_radiusOfRightWheel;
  /*!
   * length of axle (from left wheel to right wheel)
   * - Name: lengthOfAxle lengthOfAxle
   * - DefaultValue: 0.441
   * - Unit: [m]
   * - Constraint: none
   */
  double m_lengthOfAxle;
  /*!
   * length of body radius (from center of Axle to corner of Body)
   * - Name: radiusOfBodyArea radiusOfBodyArea
   * - DefaultValue: 0.45
   * - Unit: [m]
   */
  double m_radiusOfBodyArea;
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedVelocity m_TargetVelocity;
  /*!
   * target Trans/Rot Velocity of robot's center data for PD control
   * - Type: Trans/Rot velocity data: double,    timeStamp:tm
   * - Number: Trans velocity data:2, Rot velocity data:1,  timeStamp:1
   * - Semantics: target Trans/Rot Velocity of robot's center data for PD contro
   *              l
   *                [Trans Velocity]: translational velocity  of robot's center [m/sec]
   *                [Rot Velocity]: rotation velocity (swing velocity) of robot's center [rad/sec]
   * - Unit: Trans velocity [m/s],  Rot velocity [rad/s],  timeStamp: [sec],[nan
   *         osec]
   */
  InPort<TimedVelocity> m_TargetVelocityIn;
  TimedDoubleSeq m_InCurrentWheelAngle;
  /*!
   * current Wheel Angle data for PD control
   * - Type: angle data: double,    timeStamp:tm
   * - Number: angle data:2,  timeStamp:1
   * - Semantics: current Wheel Angle data for PD control
   *               [0]: left wheel's current data
   *               [1]: right wheel's current data
   * - Unit: angle data : [rad],  timeStamp: [sec],[nanosec]
   */
  InPort<TimedDoubleSeq> m_InCurrentWheelAngleIn;
  TimedDoubleSeq m_InSimulatedPosition;
  /*!
   * current Simulated Position data
   * - Type: double
   * - Number: 12 (x,y,z, posture angle matrix:9)
   * - Semantics: current Simulated Position data (the position of center of rob
   *              ots model)
   *               [0]: X data
   *               [1]: Y data
   *               [2]: Z data
   *               [3]-[11]: 3x3 Matrix data (posture angle)
   * - Unit: X,Y,Z data : [m]
   */
  InPort<TimedDoubleSeq> m_InSimulatedPositionIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_OutSimulatedPositionToInventGUI;
  /*!
   * current Simulated Position data
   * - Type: simulated position data: double,    timeStamp:tm
   * - Number: simulated position data:4 (x,y,z, posture angle ) ,  timeStamp:1
   * - Semantics: current Simulated Position data (the position of center of rob
   *              ots model)
   *               [0]: X data
   *               [1]: Y data
   *               [2]: Z data
   *               [3]: THETA (posture angle)
   * - Unit: X,Y,Z data : [m], THETA[rad] ,  timeStamp: [sec],[nanosec]
   */
  OutPort<TimedDoubleSeq> m_OutSimulatedPositionToInventGUIOut;
  TimedDoubleSeq m_Torque;
  /*!
   * Torque data
   * - Type: Torque data: double,    timeStamp:tm
   * - Number: Torque data:2,  timeStamp:1
   * - Semantics: Torque data
   *               [0]: left wheel's torque data
   *               [1]: right wheel's torque data
   * - Unit: torque data : [],  timeStamp: [sec],[nanosec]
   */
  OutPort<TimedDoubleSeq> m_TorqueOut;
  TimedDoubleSeq m_OutCurrentWheelAngle;
  /*!
   * current Wheel Angle data for sending to [OdometryRTC]
   * - Type: angle data: double,    timeStamp:tm
   * - Number: angle data:2,  timeStamp:1
   * - Semantics: current Wheel Angle data for PD control
   *               [0]: left wheel's current data
   *               [1]: right wheel's current data
   * - Unit: angle data : [rad],  timeStamp: [sec],[nanosec]
   */
  OutPort<TimedDoubleSeq> m_OutCurrentWheelAngleOut;
  TimedDoubleSeq m_OutSimulatedPositionToLocalization;
  /*!
   * current Simulated Position data
   * - Type: simulated position data: double,    timeStamp:tm
   * - Number: simulated position data:4 (x,y,z, posture angle ) ,  timeStamp:1
   * - Semantics: current Simulated Position data (the position of center of rob
   *              ots model)
   *               [0]: X data
   *               [1]: Y data
   *               [2]: Z data
   *               [3]: THETA (posture angle)
   * - Unit: X,Y,Z data : [m], THETA[rad] ,  timeStamp: [sec],[nanosec]
   */
  OutPort<TimedDoubleSeq> m_OutSimulatedPositionToLocalizationOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  /*!
   * for InventGUIRTC
   * Interface: [Provider]
   *             (1) for changing GAIN parameters
   *             (2) for receiving start/finish etc.. commands
   */
  RTC::CorbaPort m_InventGUIProvPort;
  /*!
   * for BumpDetectionRTC
   * Interface: (1) for receiving bump command
   */
  RTC::CorbaPort m_BumpProvPort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  /*!
   * [Provider]
   *  (1) for changing GAIN parameters
   *  (2) for receiving start/finish etc.. commands
   * - Argument:      setPDGain()
   *                    PGainL : P gain (LeftWheel) of PD control
   *                    PGainR : P gain (RightWheel) of PD control
   *                    DGainL : D gain (LeftWheel) of PD control
   *                    DGainR : D gain (RightWheel) of PD control
   * - Return Value:  void
   */
  MotorSVC_impl m_InventGUIMotor;
  /*!
   * If get BumpDetected information,
   * set 0.0 values to Torque data 
   * for stopping MOTOR. 
   * - Argument:      void
   * - Return Value:  void
   */
  MotorSVC_impl m_BumpMotor;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>

//---[add]------------------------
	const static int Dof = 2;            //!< Degree of Freedom = 2(left/right wheel)
	//const static int WheelLeftId = 0;       //!< jointID of left wheel
	//const static int WheelRightId = 1;       //!< jointID of right wheel

	double timeStep;   //!< timeStep([sec])
	double TimeStamp;  //!< time stamp (sec + 1.0e-9*nsec);
	double TimeStampOld;  //!< previous time stamp
	unsigned long sec;   //!< time [sec]
	unsigned long nsec; //!< time  [nano sec]
	int compSwitch;  //!< FLAG (if inputed NweData from service Port)
	double qOld[Dof]; //!< for holding previous wheel angle's data 
	bool InitializeFlag;   //!< FLAG (initialize flag of Wheel Angle)

	//! structure parameter related to BODY data
	struct BODY {
		int leftID;    //!< jointID of left wheel
		int rightID;   //!< jointID of right wheel
		double length;      //!< [the distance from the center of BODY to the center of WHEEL]
		double wheelRadiusLeft;   //!< the RADIUS of wheel (Left)
		double wheelRadiusRight;   //!< the RADIUS of wheel (Right)
		double bodyRadius;   //!<  length from center of Axle to corner of Body
	}Body;

#if DEBUG	
	std::ofstream ofs;   // TODO:someday REMOVE ( for output the SimulatedPosition&Angle data to DEBUG file) 
#endif

#if CALLBACK	
	//-- callback object
	Debug m_debug;  //!<  ( for CallBack debug)
#endif
//---[add]------------------------

};


extern "C"
{
  DLL_EXPORT void MotorControlInit(RTC::Manager* manager);
};

#endif // MOTORCONTROL_H

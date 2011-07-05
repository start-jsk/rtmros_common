// -*- C++ -*-
/*!
 * @file  Odometry.h
 * @brief detect position by Odometry
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

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>

//---[add]--------------------------------
#include "intellirobotStub.h"
#include "intellirobotSkel.h"
#include "intellirobot.hh"

//#include <rtm/RingBuffer.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>
#define LOG 0
#define DEBUG 0
#define DEBUG2 0
#define CALLBACK 0
#define CALLBACK_DEBUG 0

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
			printf("--[CallBack_TIME(WheelAngle)]: %lf\t[L]=%lf\t[R]=%lf\n",dummy_sec, value.data[0], value.data[1]);
			//printf("--[CallBack_TIME(LocalizedPosition)]: %lf\tx=%lf\ty=%lf\ttheta=%lf\n",dummy_sec, value.data[0], value.data[1], value.data[2]);
#endif
			return ret;
		};

};
//--
#endif
using namespace std;
//---[add]--------------------------------

using namespace RTC;
using namespace IIS;

/*!
 * @class Odometry
 * @brief detect position by Odometry
 *
 * (1)get current Wheel Angle data from MotorControlRTC
 * (2)get current Robot's position data from LocalizationRTC
 * (3)calculate current position by Odometry
 * (4)send current odometry position data to LocalizationRTC
 *
 * [INPUT]
 *   (1) current Angle data (encoder data)
 *   (2) current position data
 * [OUTPUT]
 *   (1) Odometry's position data
 *
 */
class Odometry
  : public RTC::DataFlowComponentBase
{
 public:
  Odometry(RTC::Manager* manager);
  ~Odometry();

   virtual RTC::ReturnCode_t onInitialize();
  /***
   * (1) update INPORT's data
   * (2) open files for debugging
   * (3) initialize
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
   * (1)get current Wheel Angle data from MotorControlRTC
   * (2)get current Robot's position data from LocalizationRTC
   * (3)calculate current position by Odometry
   * (4)send current odometry position data to LocalizationRTC
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


//---[add]--------------------------------
	/*!
		@brief  get Localized Position data from INPORT
		@return void				
	*/
	virtual void getLocalizedPosition();

	/*!
		@brief  get Current wheel's Angle data from INPORT
		@return bool (true:OK, false:NG)				
	*/
	virtual bool getWheelAngleData();

	/*!
		@brief  calc Current Position by Odometry
		@return void				
	*/
	virtual void calcOdometry();
	/*!
		@brief  set current data to previous
		@return void				
	*/
	virtual void setCurrentToOld();

	/*!
		@brief  set DATA and write to OUTPORT
		@return void				
	*/
	virtual void outputData();
//---[add]--------------------------------


 protected:
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
  TimedDoubleSeq m_CurrentWheelAngle;
  /*!
   * current Wheel Angle data for Odometry
   * - Type: angle data: double,    timeStamp:tm
   * - Number: angle data:2,  timeStamp:1
   * - Semantics: current Wheel Angle data for Odometry
   *               [0]: left wheel's current data
   *               [1]: right wheel's current data
   * - Unit: angle data : [rad],  timeStamp: [sec],[nanosec]
   */
  InPort<TimedDoubleSeq> m_CurrentWheelAngleIn;
  IIS::TimedPose2D m_LocalizedPosition;

  /*!
   * current Localized Position data
   * - Type: localized Position data: double,    timeStamp:tm
   * - Number: 4 (x,y, posture angle) ,  timeStamp:1 ,id[] ,error[]
   * - Semantics: current Localized Position data (the position of center of rob
   *              ots model)
   *              [0]: X data
   *              [1]: Y data
   *              [2]: posture angle[rad]
   * - Unit: X,Y data : [m],  Angle : [rad], timeStamp: [sec],[nanosec]
   */
  InPort<IIS::TimedPose2D> m_LocalizedPositionIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  IIS::TimedPose2D m_OdometryPosition;
  /*!
   * current Odometry Position data
   * - Type: Odometry Position data: double,    timeStamp:tm
   * - Number: 4 (x,y, posture angle) ,  timeStamp:1 ,id[] ,error[]
   * - Semantics: current Odometry Position data (the position of center of robo
   *              ts model)
   *               [0]: X data
   *               [1]: Y data
   *               [2]: posture angle[rad]
   * - Unit: X,Y data : [m],  Angle : [rad], timeStamp: [sec],[nanosec]
   */
  OutPort<IIS::TimedPose2D> m_OdometryPositionOut;  

 private:

//---[add]----------------------------------------
	//--static PARAMETER
	const static int Dof = 2;              //!< Degree of Freedom = 2(left/right wheel)
	//double Pi;        //!< the ratio of the circumference of a circle to its diameter
	double RotationAngleBorder; //!< the border value used by approximation calculation
	double RhoBorder;   //!< if rho > RhoBorder   ->  staraight line

	//! structure parameter related to BODY data
	struct BODY {
		int leftID;    //!< jointID of left wheel
		int rightID;   //!< jointID of right wheel
		double length;      //!< [the distance from the center of BODY to the center of WHEEL]
		double wheelRadiusLeft;   //!< the RADIUS of wheel (Left)
		double wheelRadiusRight;   //!< the RADIUS of wheel (Right)
		double bodyRadius;   //!<  length from center of Axle to corner of Body
	}Body;

	//! structure parameter related to CURRENT data
	struct CURRENT {
		double BodyVel;   //!< current Body Velocity
		double RightVel;  //!< current right wheel Trans Velocity
		double LeftVel;   //!< current left wheel Trans Velocity
		double BodyAccel; //!< current body Acceleration
		double BodyOmega; //!< current rotation angular velocity of body
		double Rho;      //!< current curvature radius
//		double sumLength;  //!< the sum of distance which robot moved
		double BodyVelOld;   //!< previous Body Velocity
		double BodyOmegaOld; //!< previous rotation angular velocity of body
		double RhoOld;       //!< previous curvature radius
		double Ang[Dof];     //!< current wheel angle data
		double AngVel[Dof];  //!< current wheel anglular velocity data
		double AngOld[Dof];  //!< previous wheel angle data
		double AngVelOld[Dof]; //!< previous wheel anglular velocity data
		double x;    //!< current position X 
		double y;    //!< current position Y
		double theta;    //!< current posture angle
		double xOld;    //!< previous position X
		double yOld;    //!< previous position X
		double thetaOld;   //!< previous posture angle
//		double distanceToGoal;  //! the distance from current position to GOAL
		unsigned long sec;  //!< time [sec]
		unsigned long nsec;  //!< time [nano sec]
		unsigned long secOld;  //!< time [sec]
		unsigned long nsecOld;  //!< time [nano sec]
		double Time;          //!< time [sec] (Current.sec + 1.0e-9*Current.nsec;)
		double TimeOld;       //!< time [sec]
	}Current;

	//--others
	double timeStep;   //!< timeStep([sec])
	bool InitialPositionFlag; //!< InitialFlag [false:current position have not been initialized yet][true:current position have been already initialized]
	bool InitialAngleFlag; //!< InitialFlag [false:current wheel angle have not been initialized yet][true:current wheel angle have been already initialized]

#if DEBUG		
	//-- output file pointer for debugging
	ofstream ofs;   //!< TODO:[someday REMOVE] ( for output the [CurrentPosition by Odometry] data to DEBUG file)
#endif

#if CALLBACK
	//-- callback object
	Debug m_debug;  //!< TODO:[someday REMOVE] ( for CallBack debug)
#endif

#if DEBUG2
	long counter;
	ofstream ofs2;   //!<  ( for output the [runtime] data to DEBUG file)
	ofstream ofs3;   //!<  ( for output the [runtime2] data to DEBUG file)
	//--for TimeDebug
	struct timeval TotalTime;  //!< 
	struct timeval GetTime;  		//!< 
	struct timeval SetTime;  		//!< 
#endif

//---[add]----------------------------------------

};


extern "C"
{
  DLL_EXPORT void OdometryInit(RTC::Manager* manager);
};

#endif // ODOMETRY_H

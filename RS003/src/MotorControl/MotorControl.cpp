// -*- C++ -*-
/*!
 * @file  MotorControl.cpp
 * @brief MotorControl
 * @date $Date$
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

#include "MotorControl.h"

//--- add---------------------------------------
#include <stdio.h>
#include <iostream>
//----------------------------------------------

// Module specification
// <rtc-template block="module_spec">
static const char* motorcontrol_spec[] =
  {
    "implementation_id", "MotorControl",
    "type_name",         "MotorControl",
    "description",       "MotorControl",
    "version",           "4.0.0",
    "vendor",            "AIST INVENT",
    "category",          "Controller",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    "exec_cxt.periodic.rate", "1000.0",
    // Configuration variables
    "conf.default.PGainL", "5.0",
    "conf.default.DGainL", "2.0",
    "conf.default.PGainR", "5.0",
    "conf.default.DGainR", "2.0",
    "conf.default.SimulatedOffsetX", "0.0",
    "conf.default.SimulatedOffsetY", "0.0",
    "conf.default.SimulatedOffsetAngle", "0.0",
    "conf.default.leftWheelID", "0",
    "conf.default.rightWheelID", "1",
    "conf.default.radiusOfLeftWheel", "0.1",
    "conf.default.radiusOfRightWheel", "0.1",
    "conf.default.lengthOfAxle", "0.441",
    "conf.default.radiusOfBodyArea", "0.45",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
MotorControl::MotorControl(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_TargetVelocityIn("TargetVelocity", m_TargetVelocity),
    m_InCurrentWheelAngleIn("InCurrentWheelAngle", m_InCurrentWheelAngle),
    m_InSimulatedPositionIn("InSimulatedPosition", m_InSimulatedPosition),
    m_OutSimulatedPositionToInventGUIOut("OutSimulatedPositionToInventGUI", m_OutSimulatedPositionToInventGUI),
    m_TorqueOut("Torque", m_Torque),
    m_OutCurrentWheelAngleOut("OutCurrentWheelAngle", m_OutCurrentWheelAngle),
    m_OutSimulatedPositionToLocalizationOut("OutSimulatedPositionToLocalization", m_OutSimulatedPositionToLocalization),
    m_InventGUIProvPort("InventGUIProv"),
    m_BumpProvPort("BumpProv")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
MotorControl::~MotorControl()
{
	RTC_INFO(("on Destructer"));
	//std::cout << "on Destructer" << std::endl;
}


/*!
 * set Configuration parameters
 */
RTC::ReturnCode_t MotorControl::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  registerInPort("TargetVelocity", m_TargetVelocityIn);
  registerInPort("InCurrentWheelAngle", m_InCurrentWheelAngleIn);
  registerInPort("InSimulatedPosition", m_InSimulatedPositionIn);
  
  // Set OutPort buffer
  registerOutPort("OutSimulatedPositionToInventGUI", m_OutSimulatedPositionToInventGUIOut);
  registerOutPort("Torque", m_TorqueOut);
  registerOutPort("OutCurrentWheelAngle", m_OutCurrentWheelAngleOut);
  registerOutPort("OutSimulatedPositionToLocalization", m_OutSimulatedPositionToLocalizationOut);
  
  // Set service provider to Ports
  m_InventGUIProvPort.registerProvider("InventGUIMotor", "Motor", m_InventGUIMotor);
  m_BumpProvPort.registerProvider("BumpMotor", "Motor", m_BumpMotor);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  registerPort(m_InventGUIProvPort);
  registerPort(m_BumpProvPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("PGainL", m_PGainL, "5.0");
  bindParameter("DGainL", m_DGainL, "2.0");
  bindParameter("PGainR", m_PGainR, "5.0");
  bindParameter("DGainR", m_DGainR, "2.0");
  bindParameter("SimulatedOffsetX", m_SimulatedOffsetX, "0.0");
  bindParameter("SimulatedOffsetY", m_SimulatedOffsetY, "0.0");
  bindParameter("SimulatedOffsetAngle", m_SimulatedOffsetAngle, "0.0");
  bindParameter("leftWheelID", m_leftWheelID, "0");
  bindParameter("rightWheelID", m_rightWheelID, "1");
  bindParameter("radiusOfLeftWheel", m_radiusOfLeftWheel, "0.1");
  bindParameter("radiusOfRightWheel", m_radiusOfRightWheel, "0.1");
  bindParameter("lengthOfAxle", m_lengthOfAxle, "0.441");
  bindParameter("radiusOfBodyArea", m_radiusOfBodyArea, "0.45");
  
  // </rtc-template>

//---[add]---------------------------------
  RTC_INFO(("on Constructer"));
  //std::cout << "on Constructer" << std::endl;

  // set Dimension(PORT length) 
  m_Torque.data.length(Dof);  // left and right wheel (no casters)     
  m_OutCurrentWheelAngle.data.length(Dof);
//  m_OutSimulatedPositionToInventGUI.data.length(12);  //for output SimulatedPosition&angle data to InventGUI
//  m_OutSimulatedPositionToLocalization.data.length(12);  //for output SimulatedPosition&angle data to Localization  
  m_OutSimulatedPositionToInventGUI.data.length(4);  //for output SimulatedPosition&angle data to InventGUI
  m_OutSimulatedPositionToLocalization.data.length(4);  //for output SimulatedPosition&angle data to Localization  
 
#if CALLBACK
	//-- callback object
//	m_InCurrentWheelAngleIn.setOnWriteConvert(&m_debug);
#endif  

//---[add]---------------------------------

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t MotorControl::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MotorControl::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MotorControl::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*!
 * (1) update INPORT's data
 * (2) open files for debugging
 * (3) initialize
 * (4) get robot's parameter
 */

RTC::ReturnCode_t MotorControl::onActivated(RTC::UniqueId ec_id)
{
//---[add]---------------------------------
	RTC_INFO(("on Activated"));
	//std::cout << "on Activated" << std::endl;

	// Update INPORT 
	m_InCurrentWheelAngleIn.update();
	m_TargetVelocityIn.update();
	m_InSimulatedPositionIn.update();

//--get Model Parameter from Configulation
	Body.leftID = m_leftWheelID; // jointID of left wheel
	Body.rightID = m_rightWheelID; // jointID of right wheel
	Body.wheelRadiusLeft = m_radiusOfLeftWheel; // Wheel Radius(Left)
	Body.wheelRadiusRight = m_radiusOfRightWheel; // Wheel Radius(Right)
	Body.bodyRadius = 0.5 * m_lengthOfAxle; // Body Radius (half of length of Wheel Axle)
	Body.length = m_radiusOfBodyArea; // length from center of Body to corner of Body

	RTC_DEBUG(("Body.leftID: %d   Body.rightID: %d", Body.leftID, Body.rightID));	
	RTC_DEBUG(("Body.wheelRadius[L]: %f Body.wheelRadius[R]: %f   Body.bodyRadius: %f",Body.wheelRadiusLeft, Body.wheelRadiusRight, Body.bodyRadius));	
#if DEBUG	
	//std::cout << "Body.leftID: " << Body.leftID << "   Body.rightID: " << Body.rightID << std::endl;	
	//std::cout << "Body.wheelRadius[L]: " << Body.wheelRadiusLeft << "Body.wheelRadius[R]: " << Body.wheelRadiusRight << "   Body.bodyRadius: " << Body.bodyRadius << std::endl;	

	ofs.open("SimulatePosition.dat");
#endif


  // set previous wheel angular velocity's data 
	for(int i=0; i < Dof; i++)  	qOld[i] = 0.0;  // initialize to current wheel's angle value

  // initialize SWITCH
	compSwitch = -1;
	InitializeFlag = false;

	//-- initialize Time data
	sec = nsec = 0;	
	TimeStampOld = TimeStamp = 0.0;

//---[add]---------------------------------

  return RTC::RTC_OK;
}

/*!
 * (1)close files for debugging
 */

RTC::ReturnCode_t MotorControl::onDeactivated(RTC::UniqueId ec_id)
{
//---[add]---------------------------------
	// close file
#if DEBUG	
	ofs.close();
#endif
//---[add]---------------------------------
  return RTC::RTC_OK;
}

/*!
 * (1)getting target trans/rot Velocity of robot's center data from DriveControl
 * RTC
 * (2)calculating Torque values
 * (3)sending Torque data to Simulator or real Motor
 * (4)getting current Angle data (encoder dadta) from Simulator or Real Robot
 * (5)sending current Angle data  to DriveControlRTC or OdometryRTC
 */

RTC::ReturnCode_t MotorControl::onExecute(RTC::UniqueId ec_id)
{
//---[add]---------------------------------
//  std::cout << " ==================================== [onExecute] ====================================" << std::endl;

/*********************************************************************************************************/
//  [START][FINISH][CLEAR] 
/*********************************************************************************************************/

	//--- set startFlag if [start()] was called
	if(m_InventGUIMotor.m_StartFlag){  
		m_InventGUIMotor.m_StartFlag = false;
		compSwitch = 1;
		RTC_INFO(("====== [start() was called from InventGUIComp] ======================"));
		//std::cout << "====== [start() was called from InventGUIComp] ======================" << std::endl;
	//--- set finishFlag if [finish()] was called
	}else if(m_InventGUIMotor.m_FinishFlag){  
		m_InventGUIMotor.m_FinishFlag = false;
		compSwitch = 2;
		RTC_INFO(("====== [finish() was called from InventGUIComp] ======================"));
		//std::cout << "====== [finish() was called from InventGUIComp] ======================" << std::endl;
	//--- set clearFlag if [clear()] was called
	}else if(m_InventGUIMotor.m_ClearFlag){  
		m_InventGUIMotor.m_ClearFlag = false;
		compSwitch = 3;
		//TODO: add CLEAR_ACTION 
		RTC_INFO(("====== [clear() was called from InventGUIComp] ======================"));
		//std::cout << "====== [clear() was called from InventGUIComp] ======================" << std::endl;
	//--- set bumpDetectFlag if [Stop()] was called
	}else if(m_BumpMotor.m_StopFlag){  
		m_BumpMotor.m_StopFlag = false;
		compSwitch = 4;
		//TODO: add Bump Detect_ACTION 
		RTC_INFO(("====== [Stop() was called from BumpDetectionComp] ======================"));
	}

	

/*********************************************************************************************************/
//  if [New GainDatas] are given -> update ConfigValue
/*********************************************************************************************************/
	if(m_InventGUIMotor.m_GainFlag){  // if [New GainDatas] are given
		m_PGainL = m_InventGUIMotor.m_PGainL;
		m_PGainR = m_InventGUIMotor.m_PGainR;
		m_DGainL = m_InventGUIMotor.m_DGainL;
		m_DGainR = m_InventGUIMotor.m_DGainR;
		m_InventGUIMotor.m_GainFlag = false;
		RTC_INFO(("== [GAIN] change GAIN value"));
		//std::cout << "== [GAIN] change GAIN value" << std::endl;
	}


/*********************************************************************************************************/
//  main action
/*********************************************************************************************************/

	if (m_InCurrentWheelAngleIn.isNew()) {

		//-- get Previous Wheel Angle Data 
		m_InCurrentWheelAngleIn.read();

		double dummy_sec =  m_InCurrentWheelAngle.tm.sec + 1.0e-9*m_InCurrentWheelAngle.tm.nsec;
		RTC_DEBUG(("--[TIME]: %f nsec = %ld", dummy_sec, m_InCurrentWheelAngle.tm.nsec));
#if DEBUG	
		//double dummy_sec =  m_InCurrentWheelAngle.tm.sec + 1.0e-9*m_InCurrentWheelAngle.tm.nsec;
		//std::cout << "--[TIME]: " << dummy_sec << " nsec =  "  << m_InCurrentWheelAngle.tm.nsec << "-------------------------------------------" << std::endl;
#endif
	 
		//-- set Previous Wheel Angle Data
		double q[Dof];
		q[Body.leftID] = m_InCurrentWheelAngle.data[Body.leftID];
		q[Body.rightID] = m_InCurrentWheelAngle.data[Body.rightID];

		//-- get time stamp data 
		sec = m_InCurrentWheelAngle.tm.sec;
		nsec = m_InCurrentWheelAngle.tm.nsec;
		TimeStamp = sec + 1.0e-9*nsec;

		//-- initialize 
		if (!InitializeFlag) {
			InitializeFlag = true;
		} else {
			//-- calc timestep value
			timeStep = TimeStamp - TimeStampOld;  

			//-- calc Wheel Angular Velocity's Datas by previous angle data
			double dq[Dof];
			dq[Body.leftID] = (q[Body.leftID] - qOld[Body.leftID]) / timeStep;
			dq[Body.rightID] = (q[Body.rightID] - qOld[Body.rightID]) / timeStep;

			//-- calc TORQUE  & set data to OUTPORT
			double q_ref[Dof], dq_ref[Dof];

			//-- action if Target Velocity datas are given 
			if (m_TargetVelocityIn.isNew()) {

				//*** read Target Velocity data
				m_TargetVelocityIn.read();

				double TransVel,RotVel,Rho;
				TransVel = m_TargetVelocity.vx;   // [m/s]
				RotVel = m_TargetVelocity.w;    //  [rad/s]

				if (RotVel == 0.0) { 
					//-- calc [Target] Angular Velocity Dat
					dq_ref[Body.leftID] = TransVel / Body.wheelRadiusLeft;
					dq_ref[Body.rightID] = TransVel / Body.wheelRadiusRight;
				} else {
					Rho = TransVel / RotVel;        // curvature radius
					//-- calc [Target] Angular Velocity Dat
					dq_ref[Body.leftID] = (Rho - Body.bodyRadius) * RotVel / Body.wheelRadiusLeft;
					dq_ref[Body.rightID] = (Rho + Body.bodyRadius) * RotVel / Body.wheelRadiusRight;
				}

				//-- calc [Target] Angle Data
				q_ref[Body.leftID] = q[Body.leftID] + 0.5 * (dq_ref[Body.leftID] + dq[Body.leftID]) * timeStep;
				q_ref[Body.rightID] = q[Body.rightID] + 0.5 * (dq_ref[Body.rightID] + dq[Body.rightID]) * timeStep;

				//-- calc TORQUE value    
				m_Torque.data[Body.leftID] = (q_ref[Body.leftID] - q[Body.leftID]) * m_PGainL + (dq_ref[Body.leftID] - dq[Body.leftID]) * m_DGainL;
				m_Torque.data[Body.rightID] = (q_ref[Body.rightID] - q[Body.rightID]) * m_PGainR + (dq_ref[Body.rightID] - dq[Body.rightID]) * m_DGainR;

				RTC_DEBUG((" Pgain(L): %f Pgain(R): %f Dgain(L): %f Dgain(R): %f", m_PGainL, m_PGainR, m_DGainL, m_DGainR));;
				RTC_DEBUG((" TransVel: %f RotVel: %f", TransVel, RotVel));
				RTC_DEBUG((" q_ref(L): %f q(L): %f [q_ref-q](L): %f dq_ref(L): %f dq(L): %f [dq_ref-dq](L): %f", q_ref[Body.leftID], q[Body.leftID], q_ref[Body.leftID] - q[Body.leftID], dq_ref[Body.leftID], dq[Body.leftID], dq_ref[Body.leftID] - dq[Body.leftID]));	
				RTC_DEBUG((" q_ref(R): %f q(R): %f [q_ref-q](R): %f dq_ref(R): %f dq(R): %f [dq_ref-dq](R): %f", q_ref[Body.rightID], q[Body.rightID], q_ref[Body.rightID] - q[Body.rightID], dq_ref[Body.rightID], dq[Body.rightID], dq_ref[Body.rightID] - dq[Body.rightID]));;	
				RTC_DEBUG((" Ptorque(L): %f Dtorque(L): %f", (q_ref[Body.leftID] - q[Body.leftID]) * m_PGainL, (dq_ref[Body.leftID] - dq[Body.leftID]) * m_DGainL));
				RTC_DEBUG((" Ptorque(R): %f Dtorque(R): %f", (q_ref[Body.rightID] - q[Body.rightID]) * m_PGainR, (dq_ref[Body.rightID] - dq[Body.rightID]) * m_DGainR));
#if DEBUG
				//std::cout << " Pgain(L): " << m_PGainL << " Pgain(R): " << m_PGainR << " Dgain(L): " << m_DGainL << " Dgain(R): " << m_DGainR << std::endl;
				//std::cout << " TransVel: " << TransVel << " RotVel: " << RotVel  << std::endl;
				//std::cout << " q_ref(L): " << q_ref[Body.leftID] << " q(L): " << q[Body.leftID] << " [q_ref-q](L): " << q_ref[Body.leftID] - q[Body.leftID] << " dq_ref(L): " << dq_ref[Body.leftID] << " dq(L): " << dq[Body.leftID] << " [dq_ref-dq](L): " << dq_ref[Body.leftID] - dq[Body.leftID] << std::endl;	
				//std::cout << " q_ref(R): " << q_ref[Body.rightID] << " q(R): " << q[Body.rightID] << " [q_ref-q](R): " << q_ref[Body.rightID] - q[Body.rightID] << " dq_ref(R): " << dq_ref[Body.rightID] << " dq(R): " << dq[Body.rightID] << " [dq_ref-dq](R): " << dq_ref[Body.rightID] - dq[Body.rightID] << std::endl;	
				//std::cout << " Ptorque(L): " << (q_ref[Body.leftID] - q[Body.leftID]) * m_PGainL << " Dtorque(L): " << (dq_ref[Body.leftID] - dq[Body.leftID]) * m_DGainL << std::endl;
				//std::cout << " Ptorque(R): " << (q_ref[Body.rightID] - q[Body.rightID]) * m_PGainR << " Dtorque(R): " << (dq_ref[Body.rightID] - dq[Body.rightID]) * m_DGainR << std::endl;
#endif

			} else {  
#if DEBUG	
//				std::cout << "   ***[Skip][NoTarget] can't get new TARGET Velocity data from [DriveControl]***" << std::endl;	
#endif
				m_Torque.data[Body.leftID] = 0.0;
				m_Torque.data[Body.rightID] = 0.0;	
			}

	RTC_DEBUG(("[MotorControl Torque] Time = %f[s]  Torque(L,R) = (%f, %f)", TimeStamp, m_Torque.data[Body.leftID], m_Torque.data[Body.rightID]));
#if LOG
	//std::cout <<  "[MotorControl Torque] Time = " << TimeStamp << "[s]  Torque(L,R) =  (" << m_Torque.data[Body.leftID]  << " , " << m_Torque.data[Body.rightID] <<  ") "  << std::endl;
#endif

			// set TIME
			m_Torque.tm.sec = sec;
			m_Torque.tm.nsec = nsec;
			//*** set Torque data to OutPort
			m_TorqueOut.write();			  

		} // enod of if (initialize)

		//set Current Wheel Angle Data for output to DriveControl and Odometry
	 	m_OutCurrentWheelAngle.data[Body.leftID] = m_InCurrentWheelAngle.data[Body.leftID];
		m_OutCurrentWheelAngle.data[Body.rightID] = m_InCurrentWheelAngle.data[Body.rightID];
		// set TIME
		m_OutCurrentWheelAngle.tm.sec = sec;
		m_OutCurrentWheelAngle.tm.nsec = nsec;
		//*** set Current Wheel angle's data to OUTPORT
		m_OutCurrentWheelAngleOut.write();


	RTC_DEBUG(("  wheel[L] = %f ,  wheel[R] = %f TIME1= %ld TIME2= %ld", m_OutCurrentWheelAngle.data[Body.leftID], m_OutCurrentWheelAngle.data[Body.rightID], m_OutCurrentWheelAngle.tm.sec, m_OutCurrentWheelAngle.tm.nsec));
#if DEBUG
	//std::cout << "  wheel[L] = " << m_OutCurrentWheelAngle.data[Body.leftID]  << " ,  wheel[R] = " << m_OutCurrentWheelAngle.data[Body.rightID]   << " TIME1= " << m_OutCurrentWheelAngle.tm.sec << " TIME2=" << m_OutCurrentWheelAngle.tm.nsec << std::endl;
#endif

	//*** set Current data -> Previous data
		qOld[Body.leftID] = q[Body.leftID];
		qOld[Body.rightID] = q[Body.rightID];
		TimeStampOld = TimeStamp;

	//*** get Simulated Position data and send them to InventGUIRTC
		if (m_InSimulatedPositionIn.isNew()) {
			m_InSimulatedPositionIn.read();

			//calc collect simulated position values
			double simu_x,simu_y,simu_theta,simu_fai;
			double length = sqrt(m_SimulatedOffsetX * m_SimulatedOffsetX  +  m_SimulatedOffsetY * m_SimulatedOffsetY);
//			simu_theta = atan2(m_InSimulatedPosition.data[3], m_InSimulatedPosition.data[4]);  // (0,0) -> (0,1) : theta = 90[deg]
			simu_theta = -1.0*atan2(m_InSimulatedPosition.data[4], m_InSimulatedPosition.data[3]);  // (0,0) -> (0,1) : theta = 90[deg]
			simu_fai = atan2(m_SimulatedOffsetY, m_SimulatedOffsetX);    // CenterOfBody(Xb,Yb) -> CenterOfAxle(Xa,Ya) : offsetX = Xa-Xb, offsetY = Ya-Yb
//			simu_x = m_InSimulatedPosition.data[0] + length*cos(simu_theta + simu_fai - 0.5*M_PI);
//			simu_y = m_InSimulatedPosition.data[1] + length*sin(simu_theta + simu_fai - 0.5*M_PI);
			simu_x = m_InSimulatedPosition.data[0] + length*cos(simu_theta + simu_fai);
			simu_y = m_InSimulatedPosition.data[1] + length*sin(simu_theta + simu_fai);

			// Simulated X value 
			m_OutSimulatedPositionToInventGUI.data[0] = simu_x;			m_OutSimulatedPositionToLocalization.data[0] = simu_x;

			// Simulated Y value 
			m_OutSimulatedPositionToInventGUI.data[1] = simu_y;			m_OutSimulatedPositionToLocalization.data[1] = simu_y;

/*
			for(int i=2;i<12;i++){
				m_OutSimulatedPositionToInventGUI.data[i] = m_InSimulatedPosition.data[i];				m_OutSimulatedPositionToLocalization.data[i] = m_InSimulatedPosition.data[i];
			}
*/
			// Simulated Z value
			m_OutSimulatedPositionToInventGUI.data[2] = m_InSimulatedPosition.data[2];			m_OutSimulatedPositionToLocalization.data[2] = m_InSimulatedPosition.data[2];

			// Simulated PostureAngle value
			simu_theta += m_SimulatedOffsetAngle;

			//-- [offset] -180 < dummySimuTheta < 180  
			while (simu_theta <= -1.0*M_PI || simu_theta >= 1.0*M_PI) {
				if( simu_theta <= -1.0*M_PI)			simu_theta += 2.0*M_PI;
				else if ( simu_theta >= 1.0*M_PI)		simu_theta -= 2.0*M_PI;
			}

			m_OutSimulatedPositionToInventGUI.data[3] = simu_theta;			m_OutSimulatedPositionToLocalization.data[3] = simu_theta;

		RTC_DEBUG(("[BEFORE SIMULATED]  [X] = %f ,  [Y] = %f TH[rad]= %f TH[deg]= %f TIME1= %ld TIME2= %ld", m_InSimulatedPosition.data[0], m_InSimulatedPosition.data[1], -1.0*atan2(m_InSimulatedPosition.data[4], m_InSimulatedPosition.data[3]), -180*atan2(m_InSimulatedPosition.data[4], m_InSimulatedPosition.data[3])/M_PI, m_InSimulatedPosition.tm.sec ,m_InSimulatedPosition.tm.nsec));
		RTC_DEBUG(("[AFTER SIMULATED]  [X] = %f ,  [Y] = %f TH[rad]= %f TH[deg]= %f"));

#if DEBUG	
		//for(int i=0;i<12;i++) ofs << m_OutSimulatedPositionToInventGUI.data[i]  << "	  " 	;
		//ofs << std::endl;
		for(int i=0;i<4;i++) ofs << m_OutSimulatedPositionToInventGUI.data[i]  << "	  " 	;
		ofs << std::endl;
		//std::cout << "[BEFORE SIMULATED]  [X] = " << m_InSimulatedPosition.data[0]  << " ,  [Y] = " << m_InSimulatedPosition.data[1]   << " TH[rad]= " << -1.0*atan2(m_InSimulatedPosition.data[4], m_InSimulatedPosition.data[3]) << " TH[deg]=" << -180*atan2(m_InSimulatedPosition.data[4], m_InSimulatedPosition.data[3])/M_PI << " TIME1= " << m_InSimulatedPosition.tm.sec << " TIME2=" << m_InSimulatedPosition.tm.nsec << std::endl;
		//std::cout << "[AFTER SIMULATED]  [X] = " << m_OutSimulatedPositionToInventGUI.data[0]  << " ,  [Y] = " << m_OutSimulatedPositionToInventGUI.data[1]   << " TH[rad]= " << simu_theta << " TH[deg]=" << 180*simu_theta/M_PI  << std::endl << std::endl;
#endif

		// set TIME
			m_OutSimulatedPositionToInventGUI.tm.sec = m_InSimulatedPosition.tm.sec;
			m_OutSimulatedPositionToInventGUI.tm.nsec = m_InSimulatedPosition.tm.nsec;	
			m_OutSimulatedPositionToLocalization.tm.sec = m_InSimulatedPosition.tm.sec;
			m_OutSimulatedPositionToLocalization.tm.nsec = m_InSimulatedPosition.tm.nsec;	
		//*** set Simulated Data to OUTPORT
			m_OutSimulatedPositionToInventGUIOut.write();	
			m_OutSimulatedPositionToLocalizationOut.write();	
		}

	} else {

#if DEBUG	
//	std::cout << "      ---[SKIP][NoCurrentData] can't get new CURRENT angle data ---" << std::endl;		 
#endif

	}

//---[add]---------------------------------

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t MotorControl::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MotorControl::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MotorControl::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MotorControl::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t MotorControl::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void MotorControlInit(RTC::Manager* manager)
  {
    coil::Properties profile(motorcontrol_spec);
    manager->registerFactory(profile,
                             RTC::Create<MotorControl>,
                             RTC::Delete<MotorControl>);
  }
  
};



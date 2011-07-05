// -*- C++ -*-
/*!
 * @file  Odometry.cpp
 * @brief detect position by Odometry
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

#include "Odometry.h"

// Module specification
// <rtc-template block="module_spec">
static const char* odometry_spec[] =
  {
    "implementation_id", "Odometry",
    "type_name",         "Odometry",
    "description",       "detect position by Odometry",
    "version",           "4.0.0",
    "vendor",            "AIST INVENT",
    "category",          "Localization",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    "exec_cxt.periodic.rate", "1000.0",
    // Configuration variables
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
Odometry::Odometry(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_CurrentWheelAngleIn("CurrentWheelAngle", m_CurrentWheelAngle),
    m_LocalizedPositionIn("LocalizedPosition", m_LocalizedPosition),
    m_OdometryPositionOut("OdometryPosition", m_OdometryPosition)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
Odometry::~Odometry()
{
}


/*!
 * set Configuration parameters
 */
RTC::ReturnCode_t Odometry::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  registerInPort("CurrentWheelAngle", m_CurrentWheelAngleIn);
  registerInPort("LocalizedPosition", m_LocalizedPositionIn);
  
  // Set OutPort buffer
  registerOutPort("OdometryPosition", m_OdometryPositionOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("leftWheelID", m_leftWheelID, "0");
  bindParameter("rightWheelID", m_rightWheelID, "1");
  bindParameter("radiusOfLeftWheel", m_radiusOfLeftWheel, "0.1");
  bindParameter("radiusOfRightWheel", m_radiusOfRightWheel, "0.1");
  bindParameter("lengthOfAxle", m_lengthOfAxle, "0.441");
  bindParameter("radiusOfBodyArea", m_radiusOfBodyArea, "0.45");
  
  // </rtc-template>

//--- [add] -------------------------------------
std::cout << "OnInitialize()" << std::endl;

  //-- set STATIC PARAMETER            
	//Pi = 3.1415927;        
	RotationAngleBorder = 3.1415927/180.0; 
	RhoBorder = 10000.0;   


#if CALLBACK
	//-- callback object
//	m_CurrentWheelAngleIn.setOnWriteConvert(&m_debug);
//	m_LocalizedPositionIn.setOnWriteConvert(&m_debug);
#endif  
//--- [add] -------------------------------------

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Odometry::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Odometry::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Odometry::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*!
 * (1) update INPORT's data
 * (2) open files for debugging
 * (3) initialize
 */

RTC::ReturnCode_t Odometry::onActivated(RTC::UniqueId ec_id)
{
//--- [add] -------------------------------------
  std::cout << "on Activated" << std::endl;

//--initialize  TODO:[CHANGE] get CURRENT STATUS DATA, not 0.0！！
	// Body struct
	Body.wheelRadiusLeft = Body.wheelRadiusRight = Body.bodyRadius = Body.length = 0.0;
	Body.leftID = Body.rightID = -1;

	//--Current struct
	Current.BodyVel = Current.RightVel = Current.LeftVel = Current.BodyAccel = Current.Rho = Current.BodyOmega = 0.0;
	Current.x = 0.0;
	Current.y = 0.0;
	Current.xOld = 0.0;          
	Current.yOld = 0.0;
	Current.theta = 0.0;
	Current.thetaOld = 0.0; 
	Current.BodyVelOld = Current.BodyOmegaOld = Current.RhoOld = 0.0;
	Current.Time = Current.TimeOld = 0.0;
	//-- initialize time value for DataPort' Timed
	Current.sec = Current.nsec = Current.secOld = Current.nsecOld = 0;

	//-- initialize Flag
	InitialPositionFlag = InitialAngleFlag = false;

#if DEBUG
	//--[debug] open file 
	ofs.open("odometry.dat");
#endif

//--get Model Parameter from Configulation
	Body.leftID = m_leftWheelID; // jointID of left wheel
	Body.rightID = m_rightWheelID; // jointID of right wheel
	Body.wheelRadiusLeft = m_radiusOfLeftWheel; // Wheel Radius(Left)
	Body.wheelRadiusRight = m_radiusOfRightWheel; // Wheel Radius(Right)
	Body.bodyRadius = 0.5 * m_lengthOfAxle; // Body Radius (half of length of Wheel Axle)
	Body.length = m_radiusOfBodyArea; // length from center of Body to corner of Body

#if DEBUG	
	std::cout << "Body.leftID: " << Body.leftID << "   Body.rightID: " << Body.rightID << std::endl;	
	std::cout << "Body.wheelRadius[L]: " << Body.wheelRadiusLeft << "Body.wheelRadius[R]: " << Body.wheelRadiusRight << "   Body.bodyRadius: " << Body.bodyRadius << std::endl;	
#endif

//-- Update INPORT 
	m_CurrentWheelAngleIn.update();
	m_LocalizedPositionIn.update();  

//-- set previous  data
	for(int i=0; i < Dof; i++){
  	//Current.AngOld[i] = m_CurrentWheelAngle.data[i];  //TODO: [CHANGE] dummy start position  ->  get current status data someday!
		Current.Ang[i] = Current.AngVel[i] = 0.0;
		Current.AngOld[i] = Current.AngVelOld[i] = 0.0;
	}
#if DEBUG2	
	counter = 0;
	ofs2.open("runtime.dat");
	ofs3.open("runtime2.dat");
	// timeval Initialize
	timerclear(&TotalTime);
	// Reset of time
	if(gettimeofday(&TotalTime, NULL)){
		std::cout << "failed to gettimeofday" << std::endl;
	}
#endif

//--- [add] -------------------------------------

  return RTC::RTC_OK;
}

/*!
 * (1)close files for debugging
 */

RTC::ReturnCode_t Odometry::onDeactivated(RTC::UniqueId ec_id)
{
//--- [add] -------------------------------------
	std::cout << "on Deactivated" << std::endl;
	//TODO:add ACTION

	// close file
#if DEBUG	
	ofs.close();
#endif
#if DEBUG2	
	ofs2.close();
	ofs3.close();
#endif

//--- [add] -------------------------------------

  return RTC::RTC_OK;
}

/*!
 * (1)get current Wheel Angle data from MotorControlRTC
 * (2)get current Robot's position data from LocalizationRTC
 * (3)calculate current position by Odometry
 * (4)send current odometry position data to LocalizationRTC
 */

RTC::ReturnCode_t Odometry::onExecute(RTC::UniqueId ec_id)
{
//--- [add] -------------------------------------
//  std::cout << " ==================================== [onExecute] ====================================" << std::endl;
//--- TIME DEBUG ---------------------------------------------------------------------------
#if DEBUG2	
//	std::cout << "" << std::endl;
	if(gettimeofday(&SetTime, NULL)){
		std::cout << "failed to gettimeofday" << std::endl;
	}
	timerclear(&GetTime);
	timersub(&SetTime, &TotalTime, &GetTime);	

	unsigned long  sec_debug_start = GetTime.tv_sec;
	unsigned long nsec_debug_start = GetTime.tv_usec*1000;
	double starttime = sec_debug_start + 1.0e-9*nsec_debug_start;

//	std::cout << "----[onExecute][START]   StartTime [sec] = " << starttime << std::endl;
#endif
//-----------------------------------------------------------------------------------------

	//-- action if the values of INPORT are updated 

	//-- get [LOCALIZED POSITION] data from INPORT	
/*	if (m_LocalizedPositionIn.isNew()) {
		getLocalizedPosition();
	} else {
#if DEBUG
		std::cout << "      ---[SKIP] can't get localized position data from [LocaliationRTC]---" << std::endl;	
#endif
	}*/

	//H.T 20100822 リングバッファの最新の情報を読み取るために上のものを下のように改変
	while (!m_LocalizedPositionIn.isEmpty())
		getLocalizedPosition();

	//-- after getting initialized data
//	if (InitialPositionFlag) {
	if (true) {
		//-- get [WHEEL ANGLE] data from INPORT and calc [ODOMETRY]
		if (m_CurrentWheelAngleIn.isNew()) {
			//-- get Previous wheel angle data from INPORT		
			if (getWheelAngleData()) {
				//-- calc current position and posture angle by odometry
				calcOdometry(); 
				//-- set current data to previous
				setCurrentToOld();
				//-- output
				usleep(10000);
				outputData();
			} 
		} 
else {
#if DEBUG
			std::cout << "      --------[SKIP] can't get wheel angle data from [MotorControlRTC]---" << std::endl;	
#endif
		}
	}

#if DEBUG2	
	if(gettimeofday(&SetTime, NULL)){
		std::cout << "failed to gettimeofday" << std::endl;
	}
	timerclear(&GetTime);
	timersub(&SetTime, &TotalTime, &GetTime);	

	unsigned long  sec_debug_end = GetTime.tv_sec;
	unsigned long nsec_debug_end = GetTime.tv_usec*1000;
	double endtime = sec_debug_end + 1.0e-9*nsec_debug_end;

	double deltaTime = 1.0e3*( endtime - starttime);

	std::cout << "----[RuntTime(" << counter << ")]   StartTime [sec] = " << starttime << "   EndTime [sec] = " << endtime << " [ " << deltaTime << " msec]" << std::endl;
/*	if (deltaTime > 1.0){
	 	std::cout << "----[1msecOver]---" << std::endl;
		ofs3 << counter << "  " << deltaTime  << std::endl;
	}*/
//	std::cout << "----------[END]   EndTime [sec] " << endtime << " [ " << 1.0e3*( endtime - starttime) << " msec]" << std::endl;
	std::cout << std::endl;
	ofs2 << counter++ << " , CurTime= " << Current.Time << " , dTime[ms]= " << deltaTime << std::endl;
#endif

//--- [add] -------------------------------------

  return RTC::RTC_OK;
}



/*********************************************************************************************************/
//-- get LocalizedPosition data 
/*********************************************************************************************************/
inline void Odometry::getLocalizedPosition()
{

	//-- get LocalizedPosition Data
	m_LocalizedPositionIn.read();

	//-- the actions when get first initial position data 
	if (!InitialPositionFlag) {
		//-- initialized position data
		/*
		Current.xOld = m_LocalizedPosition.x;
		Current.yOld = m_LocalizedPosition.y;
		Current.thetaOld = m_LocalizedPosition.theta;
		*/
		Current.xOld = m_LocalizedPosition.data.position.x;
		Current.yOld = m_LocalizedPosition.data.position.y;
		Current.thetaOld = m_LocalizedPosition.data.heading;
		Current.secOld = 0;  //TODO: other value?
		Current.nsecOld = 0;
		Current.TimeOld = Current.secOld + 1.0e-9*Current.nsecOld;
		InitialPositionFlag = true;
		std::cout << " [INITIALIZED] X = " << Current.xOld << " , Y = " << Current.yOld << " , THETA =" << Current.thetaOld << std::endl;	
	} else if ( 1/*(Current.secOld == m_LocalizedPosition.tm.sec) && (Current.nsecOld == m_LocalizedPosition.tm.nsec) */) {  // if correspond to previous TimeStamp
		//-- set localized data
		/*
		Current.xOld = m_LocalizedPosition.x;
		Current.yOld = m_LocalizedPosition.y;
		Current.thetaOld = m_LocalizedPosition.theta;
		*/		
		Current.xOld = m_LocalizedPosition.data.position.x;
		Current.yOld = m_LocalizedPosition.data.position.y;
		Current.thetaOld = m_LocalizedPosition.data.heading;
	} else {
#if DEBUG	
		std::cout << "   -----[NoCorresponding  (LocalizedTime)--(previousTime)] LocalizedSEC = " << m_LocalizedPosition.tm.sec << " ,  LocalizedNSEC = " << m_LocalizedPosition.tm.nsec << " , SecOld =" << Current.secOld << " , NSecOld =" << Current.nsecOld << std::endl;	
#endif
	}

	return ;
}


/*********************************************************************************************************/
//-- get Previous data 
/*********************************************************************************************************/
inline bool Odometry::getWheelAngleData()
{

	//-- get Previous Wheel Angle Data[rad]
	m_CurrentWheelAngleIn.read();

	//-- set Previous Wheel Angle Data
	Current.Ang[Body.leftID] = m_CurrentWheelAngle.data[Body.leftID];
	Current.Ang[Body.rightID] = m_CurrentWheelAngle.data[Body.rightID];

	//-- get Simulated Time
	Current.sec = m_CurrentWheelAngle.tm.sec;
	Current.nsec = m_CurrentWheelAngle.tm.nsec;

	//-- set Current Time (Simulated)
	Current.Time = Current.sec + 1.0e-9*Current.nsec;

	//-- the actions when get first initial angle data 
	if (!InitialAngleFlag) {
		Current.AngOld[Body.leftID] = Current.Ang[Body.leftID];
		Current.AngOld[Body.rightID] = Current.Ang[Body.rightID];
		timeStep = 1.0e5;   // approximate
		InitialAngleFlag = true;
		std::cout << " [INITIALIZED] Angle[L] = " << Current.AngOld[Body.leftID] << " , Angle[R] = " << Current.AngOld[Body.rightID]  << std::endl;
	} else {
		//calc timestep value
		timeStep = Current.Time - Current.TimeOld; 
	}

#if DEBUG	
	std::cout << std::endl;
	std::cout << "--[TIME(sec)]: " << Current.Time << "--------------------------------------" << std::endl;	
	std::cout << "   TimeStep[sec] =  " << timeStep  << std::endl;
	std::cout << "   [Current] currWheelAng[L] = " << m_CurrentWheelAngle.data[Body.leftID] << " / currWheelAng[R] = " << m_CurrentWheelAngle.data[Body.rightID] << " TIME1= " << m_CurrentWheelAngle.tm.sec << " TIME2=" << m_CurrentWheelAngle.tm.nsec << std::endl;
#endif

	if (timeStep <= 0.0) {
		std::cout << "== [ERROR(timeStep)] [Old or Same] timestamp's data were send, so couldn't calculate Odometry position values. " << std::endl;
		return false;
	}

	//-- calc Wheel Angular Velocity's Datas by previous angle data
	Current.AngVel[Body.leftID] = (Current.Ang[Body.leftID] - Current.AngOld[Body.leftID]) / timeStep;
	Current.AngVel[Body.rightID] = (Current.Ang[Body.rightID] - Current.AngOld[Body.rightID]) / timeStep;


	return true;
}


/************************************************************************************************************/
//-- calc Current Position by Odometry 
/************************************************************************************************************/
inline void Odometry::calcOdometry() {

	double dl,dfai,faiDummy;

	Current.LeftVel = Body.wheelRadiusLeft * Current.AngVel[Body.leftID];    	// Trans Velocity of Left Wheel
	Current.RightVel = Body.wheelRadiusRight * Current.AngVel[Body.rightID];  // Trans Velocity of Right Wheel

#if DEBUG	
	std::cout << "----calc CurrentPosition by Odometry-----------" << std::endl;
	std::cout  << "    [odometry] AngularVel[L] = " << Current.AngVel[Body.leftID] << " / AngularVel[R] = " << Current.AngVel[Body.rightID] << std::endl;
	std::cout  << "    [odometry] BodyleftVel = " << Current.LeftVel << " / BodyRightVel = " << Current.RightVel << std::endl;
#endif

	//-- calc velocity/Acceleration of BODY & RotationAngle & RotationVel by velocity of both wheel    
	Current.BodyVel = 0.5 * (Current.RightVel + Current.LeftVel) * 9.0/5.5;              // 係数調整（2011.4.24）vBody = (vRight+vLeft)/2
	Current.BodyAccel = (Current.BodyVel - Current.BodyVelOld) / timeStep;
	Current.BodyOmega = 0.5 * (Current.RightVel - Current.LeftVel) * 9.0/5.5 / Body.bodyRadius;  // 係数調整（2011.4.24）RotAngVel: OMEGA = (vRight-vLeft)/2*Body.bodyRadius (turn anti-clockwise -> +)

	//dfai = 0.5 * (Current.BodyOmega + Current.BodyOmegaOld) * timeStep;        //deltaRotationAng:  AVERAGE:(current+previous)/2
	//dfai = Current.BodyOmega * timeStep;
	dfai = Current.BodyOmega * timeStep * 6.28/(6.28 - 0.2);  //TODO:[CONSIDER] //係数調整

	//--calc current posture angle  TODO:[CHANGE] must PreSet first position
	Current.theta = Current.thetaOld + dfai;

	//-- [offset] -180 < Current.theta < 180  (while roop : If TimeStep is so long -> dfai > 360)
	while (Current.theta <= -1.0*M_PI || Current.theta >= 1.0*M_PI) {
		if( Current.theta <= -1.0*M_PI)			Current.theta += 2.0*M_PI;
		else if ( Current.theta >= 1.0*M_PI)		Current.theta -= 2.0*M_PI;
	}

#if DEBUG			
	std::cout << "    [odometry] BodyVel = " << Current.BodyVel << " / PreBodyVel = " << Current.BodyVelOld << " / BodyAccel = " << Current.BodyAccel << " / dfai[deg] = " << dfai*180/M_PI << " / CurrPostureAng[deg] = " << Current.theta*180/M_PI << " / Current.BodyOmega[deg/s] = " <<Current.BodyOmega*180/M_PI  << " / Current.BodyOmegaOLD[deg/s] = " <<Current.BodyOmegaOld*180/M_PI << std::endl;	
#endif

	//--if difference from LeftWheelAngular Velocity to Right ->  calc [CurvatureRadius]
	if(dfai != 0.0) {                 
		Current.Rho = Body.bodyRadius * (Current.RightVel + Current.LeftVel) / (Current.RightVel - Current.LeftVel);  //   rho = Body.bodyRadius*(vRight+vLeft)/(vRight-vLeft) (turn anti-clockwise -> [+] ) 
		//double rho_ave = 0.5 * (Current.Rho + Current.RhoOld);

#if DEBUG			
		//std::cout << "    [odometry] rho_ave = " << rho_ave << " Current.RhoOld = " << Current.RhoOld <<  " Current.rho = " << Current.Rho << std::endl;	
		std::cout << "    [odometry]  Current.RhoOld = " << Current.RhoOld <<  " Current.rho = " << Current.Rho << std::endl;	
#endif

		//-- modify rho
		if (fabs(Current.Rho) > RhoBorder  &&  Current.Rho >= 0.0 ) {
			Current.Rho = RhoBorder;  //TODO:[CONSIDER]  if  rho > border  ->  almost STRAIGHT Line 
			dl = 0.5 * (Current.BodyVel + Current.BodyVelOld) * timeStep;   //TODO:[CONSIDER]  x = t * ( V(i+1) + V(i) )/2 
		} else if (fabs(Current.Rho) > RhoBorder  &&  Current.Rho < 0.0 ) {
			Current.Rho = -1.0*RhoBorder;
			dl = 0.5 * (Current.BodyVel + Current.BodyVelOld) * timeStep;   //TODO:[CONSIDER]  x = t * ( V(i+1) + V(i) )/2 
		} else if(fabs(dfai) < RotationAngleBorder) {        // TODO:[CONSIDER] RotationAngleBorder -> HOW?
			dl = Current.Rho * dfai;                    // deltaL = {rho}*{deltaFAI}   (if deltaFAI << 1)
			//dl = rho_ave * dfai;
		} else {
			dl = 2.0 * Current.Rho * sin(0.5 * dfai);    // deltaL = {rho}*{deltaFAI}   (if deltaFAI >> 1)	
			//dl = 2.0 * rho_ave * sin(0.5 * dfai); 
		}

	} else {
		Current.Rho = 0.0;  // TODO:[CONSIDER] if STRAIGHT orbit
		dl = 0.5 * (Current.BodyVel + Current.BodyVelOld) * timeStep;   //TODO:[CONSIDER]  x = t * ( V(i+1) + V(i) )/2 
	}

	//-- X{i+1} = X{i} + {deltaL} * cos(theta{i}+{delta_theta}/2) 
	//faiDummy = Current.thetaOld + 0.5*dfai + 0.5*M_PI;  // TODO:[CONSIDER]  because [dfai] is angle from [Yaxis to Y'axis] ->  add 90[deg]  
	faiDummy = Current.thetaOld + 0.5*dfai;//dtの半分の時点での角度を採用？

	//--calc CURRENT POSITION ([forward]: (1)rho>0,dfai>0 --> dl>0 (2) rho<0,dfai<0 --> dl>0  /  [backward]: (1)rho>0,dfai<0 --> dl<0 (2) rho<0,dfai>0 --> dl<0 )
	Current.x = Current.xOld + dl * cos(faiDummy);
	Current.y = Current.yOld + dl * sin(faiDummy);

//#if DEBUG			
	std::cout << "[odometry] Current.x = " << Current.x << " Current.y = " << Current.y <<  " Current.theta = " << Current.theta << " deltaL = " << dl << std::endl;	
//#endif

	return ;
}


/***************************************************************************************************/
//-- set current data to previous
/***************************************************************************************************/
inline void Odometry::setCurrentToOld()
{

	//-- set CURRENT data -> PREVIOUS data 
	Current.xOld = Current.x;
	Current.yOld = Current.y;
	Current.thetaOld = Current.theta;

	Current.BodyVelOld = Current.BodyVel;
	Current.BodyOmegaOld = Current.BodyOmega; 
	Current.RhoOld = Current.Rho;

	Current.AngOld[Body.leftID] = Current.Ang[Body.leftID];
	Current.AngVelOld[Body.leftID] = Current.AngVel[Body.leftID];
	Current.AngOld[Body.rightID] = Current.Ang[Body.rightID];
	Current.AngVelOld[Body.rightID] = Current.AngVel[Body.rightID];			

	Current.secOld = Current.sec;
	Current.nsecOld = Current.nsec;
	Current.TimeOld = Current.Time;
				
	return;
}


/********************************************************************************************************/
//-- set DATA and write to OUTPORT
/*********************************************************************************************************/
inline void Odometry::outputData()
{

	//-- set current position data for OUTPORT to LocalizationRTC
	/*
	m_OdometryPosition.x = Current.x;
	m_OdometryPosition.y = Current.y;
	m_OdometryPosition.theta = Current.theta;
	m_OdometryPosition.tm.sec = Current.sec;
	m_OdometryPosition.tm.nsec = Current.nsec;
	*/
	m_OdometryPosition.data.position.x = Current.x;
	m_OdometryPosition.data.position.y = Current.y;
	m_OdometryPosition.data.heading = Current.theta;
	m_OdometryPosition.tm.sec = Current.sec;
	m_OdometryPosition.tm.nsec = Current.nsec;

#if DEBUG	
	std::cout << "     [output]  currWheelAng[L]: " << Current.Ang[Body.leftID] << ",  currWheelAng[R]:" << Current.Ang[Body.rightID] << "  TIME1= " << Current.sec << " TIME2=" << Current.nsec << std::endl;
#endif

	//--  write to OutPort
	m_OdometryPositionOut.write();

#if DEBUG	
  //-- TODO:[someday REMOVE]  [DEBUG] output current position & posture angle data to file 
	ofs << Current.x  << "	" << Current.y << "	"  << Current.theta << "	"  << 180.0*Current.theta/M_PI << std::endl;
#endif

#if LOG
	std::cout <<  "[Odometry Position] Time = " << Current.Time << "[s]  (X[m],Y[m],Theta[deg]/[rad]) =  (" << Current.x  << " , " << Current.y << " , "  << 180.0*Current.theta/M_PI << "/"  << Current.theta << ") "  << std::endl;
#endif

	return;
}



/*
RTC::ReturnCode_t Odometry::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Odometry::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Odometry::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Odometry::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Odometry::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void OdometryInit(RTC::Manager* manager)
  {
    coil::Properties profile(odometry_spec);
    manager->registerFactory(profile,
                             RTC::Create<Odometry>,
                             RTC::Delete<Odometry>);
  }
  
};



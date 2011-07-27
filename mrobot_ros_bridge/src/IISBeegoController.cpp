// -*- C++ -*-
/*!
 * @file  IISBeegoController.cpp * @brief IIS Beego Controller * $Date$ 
 *
 * $Id$ 
 */
#include "IISBeegoController.h"
#include <math.h> // for M_PI, isfinite

// Module specification
// <rtc-template block="module_spec">
static const char* iisbeegocontroller_spec[] =
  {
    "implementation_id", "IISBeegoController",
    "type_name",         "IISBeegoController",
    "description",       "IIS",
    "version",           "1.0.0",
    "vendor",            "JSK",
    "category",          "OpenHRP Controller",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

IISBeegoController::IISBeegoController(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_angleIn("angle", m_angle),
    m_velocityIn("velocity", m_velocity),
    m_torqueOut("torque", m_torque),
    m_inIn("in", m_in),
    m_outOut("out", m_out)

    // </rtc-template>
{
}

IISBeegoController::~IISBeegoController()
{
}


RTC::ReturnCode_t IISBeegoController::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  addInPort("angle", m_angleIn);
  addInPort("velocity", m_velocityIn);
  addInPort("in", m_inIn);

  // Set OutPort buffer
  addOutPort("torque", m_torqueOut);
  addOutPort("out", m_outOut);

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  m_angle.data.length(2);
  m_angle.data[0] = m_angle.data[0] = 0.0;
  m_velocity.data.length(2);
  m_velocity.data[0] = m_velocity.data[1] = 0.0;
  m_torque.data.length(4);
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t IISBeegoController::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t IISBeegoController::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t IISBeegoController::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t IISBeegoController::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t IISBeegoController::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t IISBeegoController::onExecute(RTC::UniqueId ec_id)
{
  //    static double tiresize = 0.041; // [m] TODO  
    static double tiresize = 0.1; // [m] TODO
  //    static double tread = 0.275; // [m] TODO
    static double tread = 0.4; // [m] TODO
    // add for odometry
    static double prevSec = m_velocity.tm.sec;
    static double prevNsec = m_velocity.tm.nsec;
    static double prev_x = 0.0;
    static double prev_y = 0.0;
    static double prev_theta = 0.0;
    //

    if ( m_angleIn.isNew() &&
         m_velocityIn.isNew() ) {
        m_angleIn.read();
        m_velocityIn.read();
	//       double l_tireVel = m_velocity.data[0] / 180 * M_PI * tiresize;
	//       double r_tireVel = m_velocity.data[1] / 180 * M_PI * tiresize;
	// double l_tireVel = m_velocity.data[2] / 180 * M_PI * tiresize;
	// double r_tireVel = m_velocity.data[3] / 180 * M_PI * tiresize;
	double l_tireVel = m_velocity.data[2] * tiresize;
	double r_tireVel = m_velocity.data[3] * tiresize;

	std::cerr <<
	  " 0 : " << m_velocity.data[0] <<
	  " 1 : " << m_velocity.data[1] <<
	  " 2 : " << m_velocity.data[2] <<
	  " 3 : " << m_velocity.data[3] << std::endl;
        // add for odometry
        double currentSec = m_velocity.tm.sec;
        double currentNsec = m_velocity.tm.nsec;
        double delta_t = (double)( (currentSec * 1.0e9 + currentNsec)
                                   - (prevSec * 1.0e9 + prevNsec) ) / 1.0e9;
        prevSec = currentSec;
        prevNsec = currentNsec;

        double robotVel = (r_tireVel + l_tireVel) / 2.0;
        double robotAngleVel = (r_tireVel - l_tireVel) / (2.0 * tread);
        double current_theta = prev_theta + robotAngleVel * delta_t;
        double current_x = prev_x + robotVel * cos(current_theta) * delta_t;
        double current_y = prev_y + robotVel * sin(current_theta) * delta_t;
        prev_x = current_x;
        prev_y = current_y;
        prev_theta = current_theta;
        //
	std::cerr << "robotvel : " << robotVel <<
	  " current_t : " << current_theta <<
	  " delta_t : " << delta_t << std::endl;


        fprintf(stderr, "[simulate] l tire vel = %.3f, r tire vel = %.3f\n", l_tireVel, r_tireVel);
        if(!isfinite( l_tireVel )) l_tireVel = 0.0;
        if(!isfinite( r_tireVel )) r_tireVel = 0.0;

        static double LeftCommandVel = 0, RightCommandVel = 0;
        if ( m_inIn.isNew() ) {

            // read data from client
            m_inIn.read();

            //
            LeftCommandVel  = m_in.vx - m_in.w*tread/2;
            RightCommandVel = m_in.vx + m_in.w*tread/2;
            fprintf(stderr, "[command]  x = %.3f y = %.3f, w = %.3f\n", m_in.vx, m_in.vy, m_in.w);
        }
        double LeftTireTorque = (LeftCommandVel - l_tireVel) * 100;
        double RightTireTorque = (RightCommandVel - r_tireVel) * 100;
	// if (LeftTireTorque < 0.001)
	//   LeftTireTorque = 0.01;
	// if (RightTireTorque < 0.001)
	//   RightTireTorque = 0.01;

        // write to simulated robot
        m_torque.data[0] = m_torque.data[1] = 0.0;
        m_torque.data[2] = LeftTireTorque;
        m_torque.data[3] = RightTireTorque;
        m_torqueOut.write();

	std::cerr << "torque : " << m_torque.data[0] <<
	  " torque left : " << m_torque.data[2] <<
	  " torque right : " << m_torque.data[3] << std::endl;

	m_out.x = current_x;
	m_out.y = current_y;
	m_out.theta = current_theta;
        m_outOut.write();
    }

    return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t IISBeegoController::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t IISBeegoController::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t IISBeegoController::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t IISBeegoController::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t IISBeegoController::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void IISBeegoControllerInit(RTC::Manager* manager)
  {
    coil::Properties profile(iisbeegocontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<IISBeegoController>,
                             RTC::Delete<IISBeegoController>);
  }
  
};




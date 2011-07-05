/************************************************************************************
GamePad RT-Component
Copyright (c) 2010, Segway-Japan, Ltd.
All rights reserved.

Contact us if you use this software for sell.
If you use this software not for sell, you can use this software under BSD lisence.
See the files LICENSE.TXT and LICENSE-BSD.TXT for more details.                     
************************************************************************************/
#include "GamePad.h"
#include "pad_linux.h"


// Module specification
// <rtc-template block="module_spec">
static const char* gamepad_spec[] =
  {
    "implementation_id", "GamePad",
    "type_name",         "GamePad",
    "description",       "GamePad Component",
    "version",           "0.1.0",
    "vendor",            "SegwayJapan",
    "category",          "Generic",
    "activity_type",     "SPORADIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    //"exec_cxt.periodic.rate", "30.0",
    // Configuration variables
    "conf.default.Klx", "1.0",
    "conf.default.Kly", "1.0",
    "conf.default.Krx", "1.0",
    "conf.default.Kry", "1.0",
    "conf.default.str_port", "/dev/input/js0",

    ""
  };
// </rtc-template>

GamePad::GamePad(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_ButtonOut("Button", m_Button),
    m_StickLXOut("StickLX", m_StickLX),
    m_StickLYOut("StickLY", m_StickLY),
    m_StickRXOut("StickRX", m_StickRX),
    m_StickRYOut("StickRY", m_StickRY),
    m_StickLXdOut("StickLXd", m_StickLXd),
    m_StickLYdOut("StickLYd", m_StickLYd),
    m_StickRXdOut("StickRXd", m_StickRXd),
    m_StickRYdOut("StickRYd", m_StickRYd),
    m_VelocityOut("Velocity", m_Velocity),
    m_VelocityIISOut("VelocityIIS", m_VelocityIIS),
    m_Velocity2DIISOut("Velocity2DIIS", m_Velocity2DIIS),

    // </rtc-template>
	dummy(0)
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  registerOutPort("Button", m_ButtonOut);
  registerOutPort("StickLX", m_StickLXOut);
  registerOutPort("StickLY", m_StickLYOut);
  registerOutPort("StickRX", m_StickRXOut);
  registerOutPort("StickRY", m_StickRYOut);
  registerOutPort("StickLXd", m_StickLXdOut);
  registerOutPort("StickLYd", m_StickLYdOut);
  registerOutPort("StickRXd", m_StickRXdOut);
  registerOutPort("StickRYd", m_StickRYdOut);
  registerOutPort("Velocity", m_VelocityOut);
  registerOutPort("VelocityIIS", m_VelocityIISOut);
  registerOutPort("Velocity2DIIS", m_Velocity2DIISOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>





}

GamePad::~GamePad()
{
}


RTC::ReturnCode_t GamePad::onInitialize()
{
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("Klx", m_Klx, "1.0");
  bindParameter("Kly", m_Kly, "1.0");
  bindParameter("Krx", m_Krx, "1.0");
  bindParameter("Kry", m_Kry, "1.0");
  bindParameter("str_port", m_str_port, "/dev/input/js0");

  
  // </rtc-template>
  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t GamePad::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GamePad::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GamePad::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t GamePad::onActivated(RTC::UniqueId ec_id)
{

  pad_sdl_js_open(m_str_port.c_str());

  return RTC::RTC_OK;
}



RTC::ReturnCode_t GamePad::onDeactivated(RTC::UniqueId ec_id)
{
  pad_sdl_js_close();

  return RTC::RTC_OK;
}


/*!
  @brief polling gamepad status
  @param[in]
  @return
*/
RTC::ReturnCode_t GamePad::onExecute(RTC::UniqueId ec_id)
{

  pad_sdl_js_update();
 

  m_Button.data = 0;
  for (int i = 0; i < 12; i++) {
    if (pad_sdl_js_button(i)) {
      m_Button.data |= (1<<i);
    }
  }

  //printf("button %x \n", (unsigned int)m_Button.data);

  float lx, ly, rx, ry;
  lx = (float)pad_sdl_js_axis(0) / (float)PAD_SDL_JS_AXIS_MAX;
  ly = (float)pad_sdl_js_axis(1) / (float)PAD_SDL_JS_AXIS_MAX;
  rx = (float)pad_sdl_js_axis(2) / (float)PAD_SDL_JS_AXIS_MAX;
  ry = (float)pad_sdl_js_axis(3) / (float)PAD_SDL_JS_AXIS_MAX;

  m_StickLX.data = m_Klx * lx;
  m_StickLY.data = m_Kly * ly;
  m_StickRX.data = m_Krx * rx;
  m_StickRY.data = m_Kry * ry;

  m_StickLXd.data = (double)m_StickLX.data;
  m_StickLYd.data = (double)m_StickLY.data;
  m_StickRXd.data = (double)m_StickRX.data;
  m_StickRYd.data = (double)m_StickRY.data;

  m_Velocity.v = -m_Kly * (double)ly;
  m_Velocity.w = -m_Klx * (double)rx;

  m_VelocityIIS.vx = -m_Kly * (double)ly;
  m_VelocityIIS.vy = -m_Klx * (double)lx;
  m_VelocityIIS.w  = -m_Krx * (double)rx;

  m_Velocity2DIIS.data.vx = -m_Kly * (double)ly;
  m_Velocity2DIIS.data.vy = -m_Klx * (double)lx;
  m_Velocity2DIIS.data.va = -m_Krx * (double)rx;

  m_ButtonOut.write();
  m_StickLXOut.write();
  m_StickLYOut.write();
  m_StickRXOut.write();
  m_StickRYOut.write();

  m_StickLXdOut.write();
  m_StickLYdOut.write();
  m_StickRXdOut.write();
  m_StickRYdOut.write();

  m_VelocityOut.write();
  m_VelocityIISOut.write();
  
  //ysuzuki replace-
  //m_Velocity2DIISOut.write();
  if(pad_sdl_js_isok()==true){
    m_Velocity2DIISOut.write();
  }
  //-replace

  coil::usleep(15000);

  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t GamePad::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GamePad::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GamePad::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GamePad::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GamePad::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void GamePadInit(RTC::Manager* manager)
  {
    RTC::Properties profile(gamepad_spec);
    manager->registerFactory(profile,
                             RTC::Create<GamePad>,
                             RTC::Delete<GamePad>);
  }

};



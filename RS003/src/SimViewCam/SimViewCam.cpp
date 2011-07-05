// -*- C++ -*-
/*!
 * @file  SimViewCam.cpp
 * @brief DataConversionRTC
 * @date $Date$
 *
 * $Id$
 */

#include "SimViewCam.h"

// Module specification
// <rtc-template block="module_spec">
static const char* SimViewCam_spec[] =
  {
    "implementation_id", "SimViewCam",
    "type_name",         "SimViewCam",
    "description",       "DataConversionRTC",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "tool",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "5",
    "language",          "C++",
    "lang_type",         "compile",
    //"exec_cxt.periodic.rate", "1.0",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
SimViewCam::SimViewCam(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_inIn("in", m_in),
    m_outOut("out", m_out)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
SimViewCam::~SimViewCam()
{
}



RTC::ReturnCode_t SimViewCam::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  registerInPort("in", m_inIn);
  
  // Set OutPort buffer
  registerOutPort("out", m_outOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  img_rcv = cvCreateImage(cvSize(320,240), IPL_DEPTH_8U, 3);
  img_tmp = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 3);
  img_snd = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 3);

  windowOn = 0; //1ならSimViewを表示
  if(windowOn==1)
	cvNamedWindow("SimView", CV_WINDOW_AUTOSIZE);

  return RTC::RTC_OK;
}


RTC::ReturnCode_t SimViewCam::onFinalize()
{
  cvReleaseImage(&img_rcv);
  cvReleaseImage(&img_tmp);
  cvReleaseImage(&img_snd);
  if(windowOn==1)
  	cvDestroyWindow("SimView");

  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t SimViewCam::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimViewCam::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimViewCam::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimViewCam::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t SimViewCam::onExecute(RTC::UniqueId ec_id)
{
	int simdata,r,g,b;
	int maskR = 0xff0000;
	int maskG = 0xff00;
	int maskB = 0xff;

	//while(!m_inIn.isEmpty())
	if(m_inIn.isNew()){
		m_inIn.read();
		
		//ビット演算を駆使してビューシミュレーションの画像データからRGB値を取り出す。
		for(int i=0; i<(int)m_in.data.length(); i++){
			simdata = m_in.data[i];
			r = (simdata & maskR) >> 16;
			g = (simdata & maskG) >> 8;
			b = (simdata & maskB);

			//IplImage型のimg_rcvはonInitialize内でcreateしている。サイズはOpenHRP3側の
			//ビューシミュレーションの設定に合わせて320x240に固定してしまっているので注意。
		
			img_rcv->imageData[i*3]   = (uchar) b;
			img_rcv->imageData[i*3+1] = (uchar) g;
			img_rcv->imageData[i*3+2] = (uchar) r; 

		}
		if(windowOn==1){
			cvShowImage("SimView",img_rcv);
			cvWaitKey(5);//ShowImageするなら必須
		}
		//CeilingNavigationで使う画像データの型に変換して送る。
		//CeilingNavigationの側ではどうやら640x480の画像にしか対応していないようなので
		//img_rcvを2倍サイズのimg_sndに変換して送る。
		cvResize(img_rcv, img_tmp);
		cvFlip(img_tmp, img_snd, -1);//画像をフリップ(垂直軸反転)
		m_out.data.length(sizeof(IplImage) + img_snd->imageSize);
		memcpy(&(m_out.data[0]), (unsigned char*)img_snd, sizeof(IplImage));
	   	memcpy(&(m_out.data[sizeof(IplImage)]), img_snd->imageData, img_snd->imageSize);
	 	m_outOut.write();
	}

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SimViewCam::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimViewCam::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimViewCam::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimViewCam::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimViewCam::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void SimViewCamInit(RTC::Manager* manager)
  {
    coil::Properties profile(SimViewCam_spec);
    manager->registerFactory(profile,
                             RTC::Create<SimViewCam>,
                             RTC::Delete<SimViewCam>);
  }
  
};



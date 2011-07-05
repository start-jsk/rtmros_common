// -*- C++ -*-
/*!
 * @file  DispPosition.cpp
 * @brief ${rtcParam.description}
 * @date $Date$
 *
 * $Id$
 */

#include "DispPosition.h"

// Module specification
// <rtc-template block="module_spec">
static const char* dispposition_spec[] =
  {
    "implementation_id", "DispPosition",
    "type_name",         "DispPosition",
    "description",       "${rtcParam.description}",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
DispPosition::DispPosition(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_posIn("position", m_pos)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
DispPosition::~DispPosition()
{
}



RTC::ReturnCode_t DispPosition::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("position", m_posIn);
  
	//地図画像の読み込み
	imOrgmap = cvLoadImage(ORG_MAP,CV_LOAD_IMAGE_COLOR);
  if (!imOrgmap)
  {
    fprintf(stderr, "%s: LOAD ERROR\n", ORG_MAP);
    return RTC::RTC_ERROR;
  }
	imTrjmap = cvCloneImage(imOrgmap);
	imRobmap = cvCloneImage(imOrgmap);
	cvNamedWindow("Display",CV_WINDOW_AUTOSIZE);

	//初期値
	rx = 100; ry = 50;
	rth = 1.0;

	mPerPix = 0.05;
	//m_BlockSize = 64;
	cnt=0;

	for(int i=0; i<5; i++)
		robopt[i] = cvPoint(0.0,0.0);

  return RTC::RTC_OK;
}


RTC::ReturnCode_t DispPosition::onShutdown(RTC::UniqueId ec_id)
{
	cvDestroyWindow("Display");
  return RTC::RTC_OK;
}



RTC::ReturnCode_t DispPosition::onExecute(RTC::UniqueId ec_id)
{
	if(m_posIn.isNew())
	{
		m_posIn.read();
		rx = round(m_pos.data.position.x);
		ry = round(m_pos.data.position.y);
		rth = m_pos.data.heading + M_PI/2.0;
		
		if (rth > M_PI)		/* 180度を越えている場合、マイナスに変更する */
			rth = -M_PI * 2 + rth;
		else if (rth < -M_PI)	/* -180を下回る場合、プラスに変更する */
			rth = M_PI * 2 + rth;
		
		rx = round( m_pos.data.position.x / mPerPix ) + 69;
		ry = - round( m_pos.data.position.y / mPerPix ) + (333.0 - 63.0);

		//ロボット位置の描画
		//位置 (imTrjmapに軌跡として蓄積)
		cvCircle ( imTrjmap, cvPoint(rx,ry), 1, CV_RGB(255,0,0), -1, 4, 0 );
		cvCopy( imTrjmap, imRobmap );

		//方向 (imRobmapで更新)
		//cvCircle ( imRobmap, cvPoint(rx,ry), 3, CV_RGB(0,0,255), -1, 2, 0 );
		cvLine( imRobmap, cvPoint( rx, ry ),cvPoint( rx + sin(rth)*15, ry + cos(rth)*15 ), CV_RGB(0,0,255), 2, 0, 0);
	
		robopt[0] = cvPoint(rx + sin(rth+M_PI/4.0)*10, ry + cos(rth+M_PI/4.0)*10 );
		robopt[1] = cvPoint(rx + sin(rth-M_PI/4.0)*10, ry + cos(rth-M_PI/4.0)*10 );
		robopt[2] = cvPoint(rx - sin(rth+M_PI/4.0)*10, ry - cos(rth+M_PI/4.0)*10 );
		robopt[3] = cvPoint(rx - sin(rth-M_PI/4.0)*10, ry - cos(rth-M_PI/4.0)*10 );
		robopt[4] = cvPoint(rx + sin(rth)*15, ry + cos(rth)*15 );

		cvLine( imRobmap, robopt[0], robopt[4], CV_RGB(0,0,255), 1, 0, 0);
		cvLine( imRobmap, robopt[4], robopt[1], CV_RGB(0,0,255), 1, 0, 0);
		cvLine( imRobmap, robopt[1], robopt[2], CV_RGB(0,0,255), 1, 0, 0);
		cvLine( imRobmap, robopt[2], robopt[3], CV_RGB(0,0,255), 1, 0, 0);
		cvLine( imRobmap, robopt[3], robopt[0], CV_RGB(0,0,255), 1, 0, 0);
				
		cvShowImage("Display", imRobmap);
		key = cvWaitKey(50) & 0xff;
		cnt++;
	}		
	return RTC::RTC_OK;
}


extern "C"
{
 
  void DispPositionInit(RTC::Manager* manager)
  {
    coil::Properties profile(dispposition_spec);
    manager->registerFactory(profile,
                             RTC::Create<DispPosition>,
                             RTC::Delete<DispPosition>);
  }
  
};


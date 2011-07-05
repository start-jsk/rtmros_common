// -*- C++ -*-
/*!
 * @file  CeilingNavigation.cpp
 * @brief CeilingNavigation
 * @date $Date$
 *
 * $Id$
 */

#include "CeilingNavigation.h"
#include "cv.h"
#include "highgui.h"

//#define _USE_SHOW_MAP	1 //現状ではmakeのときに宣言
//#define _OFFLINE_IMAGE	1

#ifdef _USE_SHOW_MAP
#include "Common/sharedmem.h"
robot_status *rstat = NULL;
int count = 130;
int delaycount=0;
#endif

FILE *fp;

// Module specification
// <rtc-template block="module_spec">
static const char* ceilingnavigation_spec[] =
  {
    "implementation_id", "CeilingNavigation",
    "type_name",         "CeilingNavigation",
    "description",       "CeilingNavigation",
    "version",           "1.0.0",
    "vendor",            "NAIST and FSI",
    "category",          "Navigation",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
#ifdef _OFFLINE_IMAGE
    "conf.default.NavigationMap", "./CeilingImage2_Small.BMP",
    "conf.default.OfflineImage", "./offline_image",
    "conf.default.X_POS", "367",
    "conf.default.Y_POS", "51",
    "conf.default.THETA", "-2.0",
#else
    "conf.default.NavigationMap", "./rotRTCcenterCeiling.BMP",
    "conf.default.OfflineImage", "",
    "conf.default.X_POS", "88",
    "conf.default.Y_POS", "76",
    "conf.default.THETA", "0",

#endif
    "conf.default.BlockResolution", "1",
    "conf.default.BlockSize", "96",
    "conf.default.BlockCoefficient", "0.414",//0.3
    "conf.default.SearchScope", "2", //この値は関係ないようだ
    "conf.default.BlackWhiteValue", "220",
    "conf.default.Center_X", "160",
    "conf.default.Center_Y", "120",
	"conf.default.RealMapHeight", "100.0",
	"conf.default.RealMapWidth", "100.0",
	"conf.default.VirtualMapHeight", "100.0",
	"conf.default.VirtualMapWidth", "100.0",
    ""
  };
// </rtc-template>

CeilingNavigation::CeilingNavigation(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_CameraDataIn("CameraData", m_CameraData),
    m_LocalizedPositionIn("LocalizedPosition", m_LocalizedPosition),
		m_CeilingPositionOut("CeilingPosition", m_CeilingPosition),
    // </rtc-template>
	dummy(0)
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  registerInPort("CameraData", m_CameraDataIn);
  registerInPort("LocalizedPosition", m_LocalizedPositionIn);
  
  // Set OutPort buffer
  registerOutPort("CeilingPosition", m_CeilingPositionOut);
  
}

CeilingNavigation::~CeilingNavigation()
{
}



RTC::ReturnCode_t CeilingNavigation::onInitialize()
{
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
#ifdef _OFFLINE_IMAGE
  bindParameter("NavigationMap", m_NavigationMap, "./CeilingImage2_Small.BMP");
  bindParameter("OfflineImage", m_OfflineImage, "./offline_image");
  bindParameter("X_POS", x_pos, "367");
  bindParameter("Y_POS", y_pos, "51");
  bindParameter("THETA", theta, "-2.0");
#else
  bindParameter("NavigationMap", m_NavigationMap, "./rotRTCcenterCeiling.BMP");
  bindParameter("OfflineImage", m_OfflineImage, "");
  bindParameter("X_POS", x_pos, "88");
  bindParameter("Y_POS", y_pos, "76");
  bindParameter("THETA", theta, "0");
#endif
  bindParameter("BlockResolution", m_BlockResolution, "1");
  bindParameter("BlockSize", m_BlockSize, "96");
  bindParameter("BlockCoefficient", m_BlockCoefficient, "0.414");//0.3
  bindParameter("SearchScope", m_SearchScope, "2");
  bindParameter("BlackWhiteValue", m_BlackWhiteValue, "220");
  bindParameter("Center_X", m_Center_X, "160");
  bindParameter("Center_Y", m_Center_Y, "120");
  bindParameter("RealMapHeight",m_RealMapHeight,"100.0");
  bindParameter("RealMapWidth",m_RealMapWidth,"100.0");
  bindParameter("VirtualMapHeight",m_VirtualMapHeight,"100.0");
  bindParameter("VirtualMapWidth",m_VirtualMapWidth,"100.0");


  // </rtc-template>
#ifdef _USE_SHOW_MAP
	rstat = (robot_status *)InitSharedMem(SHM_KEY, sizeof(robot_status));	//共有メモリ関連
#endif

	//各モジュール共通の値．最適な共有メカニズムは何か？
	mPerPix = 0.05;
	camX = 0.05;
	camY = -0.17;//単位 m

	fp = fopen("trajectory.dat", "w");

	return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t CeilingNavigation::onFinalize()
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t CeilingNavigation::onStartup(RTC::UniqueId ec_id)
{
	// ナビゲーション用マップ画像読み込み
	m_CeilingMap.ReadNavigationMap(m_NavigationMap);

	// 初期位置特定
#if 0	// コンフィグに移動
	#ifdef _OFFLINE_IMAGE
		m_CeilingMap.SetLocation(376,  51, -2.00);
	#else
		m_CeilingMap.SetLocation(x_pos,  y_pos, theta);
	#endif
#endif
	
	return RTC::RTC_OK;
}


RTC::ReturnCode_t CeilingNavigation::onShutdown(RTC::UniqueId ec_id)
{
	// 動的確保メモリ開放
	fclose(fp);
	return RTC::RTC_OK;
}

RTC::ReturnCode_t CeilingNavigation::onActivated(RTC::UniqueId ec_id)
{
	ImageData cameraImage;

	// カメラコンポーネントから送信されてきている画像情報を読み込む
	if(m_OfflineImage.length() == 0){
		while(!m_CameraDataIn.isEmpty())
			m_CameraDataIn.read();
		// バイナリ配列を適切な状態に変更する
		cameraImage.SetData(m_CameraData);
	}

#ifdef _OFFLINE_IMAGE
	else{
		char fname[256];
		// オフラインデータを読み込んで、m_CameraDataに格納する
		memset(fname, 0, sizeof(fname));
		sprintf( fname, "%s/cap_%05d.png", m_OfflineImage.c_str(), count );
		cameraImage.LoadImage( fname, CV_LOAD_IMAGE_GRAYSCALE);
	}
#endif

	//ブロックマッチングの準備
	m_BlockMat.CreateBlockMap ( m_Center_X, m_Center_Y, cameraImage.GetWidth(),
		cameraImage.GetHeight(), m_BlockSize, m_BlockCoefficient, m_BlockResolution, m_BlackWhiteValue);

	return RTC::RTC_OK;

}

/*
RTC::ReturnCode_t CeilingNavigation::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t CeilingNavigation::onExecute(RTC::UniqueId ec_id)
{
	ImageData cameraImage;
	// カメラコンポーネントから送信されてきている画像情報を読み込む
	if(m_OfflineImage.length() == 0){
		m_CameraDataIn.read();
		// 形式変換
		// バイナリ配列を適切な状態に変更する

		cameraImage.SetData(m_CameraData);
	}
#ifdef _OFFLINE_IMAGE
	else{
		char fname[256];

		// オフラインデータを読み込んで、m_CameraDataに格納する
		memset(fname, 0, sizeof(fname));
		if(count > 1061) count = 130;
		sprintf( fname, "%s/cap_%05d.png", m_OfflineImage.c_str(), count++ );
		cameraImage.LoadImage( fname, CV_LOAD_IMAGE_GRAYSCALE);
	}
#endif

	//統合後の位置推定値が得られるときはそれによってx_pos,y_pos,thetaを更新し
	//次のマッチング範囲の中心点とする．
	if(m_LocalizedPositionIn.isNew())
	{
	while(!m_LocalizedPositionIn.isEmpty())
		m_LocalizedPositionIn.read();

		//下のm_CeilingPosition の計算の逆計算により実世界上の位置を画像上の位置に変換
		//コンポーネント間で共通の定数があるので何とかすべし．

		theta = m_LocalizedPosition.data.heading + M_PI/2.0;
		if (theta > M_PI)		// 180度を越えている場合、マイナスに変更する
			theta = -M_PI * 2 + theta;
		else if (theta < -M_PI)	// -180を下回る場合、プラスに変更する
			theta = M_PI * 2 + theta;
		
		x_pos = round( m_LocalizedPosition.data.position.x / mPerPix ) + 69 - m_BlockSize/2;
		y_pos = - (unsigned long)round( m_LocalizedPosition.data.position.y / mPerPix ) + (333.0 - 63.0) - m_BlockSize/2;

	}

	m_CeilingMap.SetLocation(x_pos, y_pos, theta);

	//ブロックマッチングによる位置推定
	m_BlockMat.MapTracking(m_CeilingMap, cameraImage);
	
	theta = m_CeilingMap.GetTheta();
	x_pos = m_CeilingMap.GetPosX();
	y_pos = m_CeilingMap.GetPosY();

/*
	  std::cout <<"[output Data]: "<< m_CeilingOdometryd.data[0]<< ","
		    << m_CeilingOdometryd.data[1]<< ","
		    <<m_CeilingOdometryd.data[2] << std::endl;*/


  if(delaycount == 0){ 		//8ループに一回，出力ポートにデータ送信

//		m_CeilingOdometry.theta = theta;
//		m_CeilingOdometry.x = x_pos;
//		m_CeilingOdometry.y = y_pos;

	  /*std::cout <<"[output Data]: "<< m_CeilingOdometry.x<< ","
		    << m_CeilingOdometry.y<< ","
		    <<m_CeilingOdometry.theta << std::endl;*/

		//画像上位置を実世界位置に変換
		//(1ピクセルあたりの実世界長[m])×{(画像上位置[pixel])-(画像上原点位置[m])}
    //今はRTC再利用センターの天井地図に合わせてある．
    //原点は入り口の丸い電灯の下．入り口に室内を向いて立ったとき右がx，前がy．
    //方位角はx＋が0，反時計回りに＋

		//m_CeilingPosition.x = m_RealMapWidth/m_VirtualMapWidth*(double)x_pos+1.7;
		m_CeilingPosition.data.position.x = mPerPix * ((double)(x_pos + m_BlockSize/2)  - 69.0 );
		m_CeilingPosition.data.position.y = mPerPix * ((double)(y_pos + m_BlockSize/2) - (333.0 - 63.0)) * (-1); //画像と地図ではy軸の向きが逆

		m_CeilingPosition.data.heading = theta - M_PI/2.0;
		if (m_CeilingPosition.data.heading > M_PI)   //180度を越えている場合マイナスに変更する
			m_CeilingPosition.data.heading = -M_PI * 2 + m_CeilingPosition.data.heading;
		else if (m_CeilingPosition.data.heading < -M_PI)	   //-180を下回る場合プラスに変更する
			m_CeilingPosition.data.heading = M_PI * 2 + m_CeilingPosition.data.heading;

		//ここまでで計算した値はカメラの画像中央の値．これをロボットの原点での値に直す．
		m_CeilingPosition.data.position.x = m_CeilingPosition.data.position.x - camX * cos(m_CeilingPosition.data.heading) + camY * sin(m_CeilingPosition.data.heading);
		m_CeilingPosition.data.position.y = m_CeilingPosition.data.position.y - camX * sin(m_CeilingPosition.data.heading) - camY * cos(m_CeilingPosition.data.heading);

		std::cout <<"[output Data]: "<< m_CeilingPosition.data.position.x << ","
				  << m_CeilingPosition.data.position.y << ","
				  <<m_CeilingPosition.data.heading << std::endl;
	
//		m_CeilingOdometryOut.write();
		m_CeilingPositionOut.write();

		//GnuPlot表示用(オフライン)に推定値データを出力
		fprintf(fp,"%f %f %f\n",m_CeilingPosition.data.position.x,m_CeilingPosition.data.position.y,m_CeilingPosition.data.heading);
		
		delaycount = 8; //天井カメラによる位置修正を適度な間隔で行うための待ち設定．値はテキトー
  }
  delaycount--;

	usleep(10000);

  return RTC::RTC_OK;
}



extern "C"
{
 
  void CeilingNavigationInit(RTC::Manager* manager)
  {
    RTC::Properties profile(ceilingnavigation_spec);
    manager->registerFactory(profile,
                             RTC::Create<CeilingNavigation>,
                             RTC::Delete<CeilingNavigation>);
  }
  
};

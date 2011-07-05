// -*- C++ -*-
/*!
 * @file  CeilingNavigation.h
 * @brief CeilingNavigation
 * @date  $Date$
 *
 * $Id$
 */

#ifndef CEILINGNAVIGATION_H
#define CEILINGNAVIGATION_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>

#include "std_hdr.h"

#include "intellirobotStub.h"

#include "BlockMatching.h"
#include "ImageData.h"
#include "CeilingMap.h"

using namespace RTC;

/**
 *@brief 天井ナビゲーションRTCメインクラス
 */
class CeilingNavigation
  : public RTC::DataFlowComponentBase
{
 public:
  CeilingNavigation(RTC::Manager* manager);
  ~CeilingNavigation();
 
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

 protected:
	std::string m_NavigationMap;		/* 天井画像ファイル名						*/
	unsigned short m_BlockResolution;	/* 画像回転分解能							*/
	unsigned short m_BlockSize;			/* 画像縮小サイズ							*/
	double m_BlockCoefficient;			/* 画像縮小補正値							*/
	unsigned short m_SearchScope;		/* マッチング探索範囲						*/
	unsigned short m_BlackWhiteValue;	/* 2値化閾値								*/
	long m_Center_X;					/* 画像回転中心座標(X)						*/
	long m_Center_Y;					/* 画像回転中心座標(Y)						*/
	std::string m_OfflineImage;			/* オフラインイメージファイル格納フォルダ名 */
	double m_RealMapHeight;
	double m_RealMapWidth;
	double m_VirtualMapHeight;
	double m_VirtualMapWidth;

	unsigned long x_pos;
	unsigned long y_pos;
	double theta;

	double mPerPix;//画像1ピクセルあたりの実世界の長さ[m]
	//ロボット座標系は，前方をx,左をyとする．
	double camX; //ロボット座標系におけるカメラのx座標
	double camY; //ロボット座標系におけるカメラのy座標
	//角度については，画像の上方向がロボット座標x軸正方向に一致するように配置するということで固定する．

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedOctetSeq m_CameraData;
  InPort<TimedOctetSeq> m_CameraDataIn;
 
	IIS::TimedPose2D m_LocalizedPosition;
	InPort<IIS::TimedPose2D> m_LocalizedPositionIn; 

  IIS::TimedPose2D m_CeilingPosition;
  OutPort<IIS::TimedPose2D> m_CeilingPositionOut;
  
 private:
  int dummy;
  private:
	  BlockMatching	m_BlockMat;
	  CeilingMap	m_CeilingMap;

};


extern "C"
{
  void CeilingNavigationInit(RTC::Manager* manager);
};

#endif // CEILINGNAVIGATION_H

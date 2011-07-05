// -*- C++ -*-
/*!
 * @file  DispPosition.h
 * @brief ${rtcParam.description}
 * @date  $Date$
 *
 * $Id$
 */

#ifndef DISPPOSITION_H
#define DISPPOSITION_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>

#include "intellirobotStub.h"

#include "cv.h"
#include "highgui.h"

#ifndef ORG_MAP // added 20110629 k-okada
#define ORG_MAP "RTCcenterUnifiedMap.jpg"
#endif

using namespace RTC;
using namespace IIS;

class DispPosition
  : public RTC::DataFlowComponentBase
{
 public:
  DispPosition(RTC::Manager* manager);
  ~DispPosition();
  virtual RTC::ReturnCode_t onInitialize();
	virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

 protected:
  IIS::TimedPose2D m_pos;
  InPort<IIS::TimedPose2D> m_posIn;
 private:
	//画像データ
  IplImage *imOrgmap; //元の地図画像
  IplImage *imTrjmap; //元の地図画像にロボットの軌跡を追加したもの
  IplImage *imRobmap; //さらにロボットの現在位置姿勢を追加したもの

	//キー入力
	int key;

	//ロボットの位置姿勢
	int rx, ry;
	double rth;

	//1ピクセルあたりの実世界の長さ
	double mPerPix;
	//int m_BlockSize;

	//描画用 ロボットの輪郭の点
	CvPoint robopt[5];

int cnt;

};


extern "C"
{
  DLL_EXPORT void DispPositionInit(RTC::Manager* manager);
};

#endif // DISPPOSITION_H

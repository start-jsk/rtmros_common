// -*- C++ -*-
/*!
 * @file  LocalizeCenter.h
 * @brief ${rtcParam.description}
 * @date  $Date$
 *
 * $Id$
 */

#ifndef LOCALIZECENTER_H
#define LOCALIZECENTER_H

#include "std_hdr.h"
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

#include "intellirobotStub.h"

using namespace RTC;
using namespace IIS;

class LocalizeCenter
  : public RTC::DataFlowComponentBase
{
 public:
  LocalizeCenter(RTC::Manager* manager);
  ~LocalizeCenter();

  virtual RTC::ReturnCode_t onInitialize();
	virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);


 protected:
  IIS::TimedPose2D m_odm;
  InPort<IIS::TimedPose2D> m_odmIn;
	double m_odmWgt;
  IIS::TimedPose2D m_ceil;
  InPort<IIS::TimedPose2D> m_ceilIn;
	double m_ceilWgt;

	double m_totalWgt;
  
  IIS::TimedPose2D m_dp_out0;
  OutPort<IIS::TimedPose2D> m_dp_out0Out;
	TimedPosition m_avPos;
  

 private:

	double m_start_x;
	double m_start_y;
	double m_start_theta;
	int m_cycle;

	double start_x_prev;
	double start_y_prev;
	double start_theta_prev;

	int pos_init_cnt, counter;


};


extern "C"
{
  DLL_EXPORT void LocalizeCenterInit(RTC::Manager* manager);
};

#endif // LOCALIZECENTER_H

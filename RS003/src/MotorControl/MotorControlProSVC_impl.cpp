// -*-C++-*-
/*!
 * @file  MotorControlProSVC_impl.cpp
 * @brief Service implementation code of MotorControlPro.idl
 *
 * @author Yusuke Nakajima (y.nakajima@aist.go.jp)
 *
 * Copyright (c) 2008, National Institute of Advanced Industrial Science and Tec
 * hnology (AIST). All rights reserved. This program is made available under the te
 * rms of the Eclipse Public License v1.0 which accompanies this distribution, and 
 * is available at http://www.eclipse.org/legal/epl-v10.html
 *
 */

#include "MotorControlProSVC_impl.h"
//--- add --------------------------
#include <rtm/CORBA_SeqUtil.h>
#include <iostream>
//----------------------------------

/*
 * Example implementational code for IDL interface Motor
 */
MotorSVC_impl::MotorSVC_impl()
:rtclog("MotorSVC_impl")
{
   	m_GainFlag = m_StartFlag = m_FinishFlag = m_ClearFlag = m_StopFlag= false;  //service port Flag (if data were inputed)
	m_PGainL = m_PGainR = m_DGainL = m_DGainR = 0.0;   // service port data values  
}


MotorSVC_impl::~MotorSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
void MotorSVC_impl::setPDGain(CORBA::Double PGainL, CORBA::Double PGainR, CORBA::Double DGainL, CORBA::Double DGainR)
{
	RTC_INFO(("[GainControlSVC_impl::ChangeGain] called"));
	//std::cout << "[GainControlSVC_impl::ChangeGain] called "  << std::endl;
 
	//set ServicePort's Data for using in Controll Side 
	m_PGainL = PGainL;
	m_PGainR = PGainR;
	m_DGainL = DGainL;
	m_DGainR = DGainR;
	//TODO:check the datas whether balid or invalid. if OK -> set FLAG
	m_GainFlag = true; // set FLAG after inserting data from servicePort

	return;
}

void MotorSVC_impl::Start()
{
	RTC_INFO(("[MotorSVC_impl::Start] called "));
	//std::cout << "[MotorSVC_impl::Start] called "  << std::endl;

	// set FLAG 
	m_StartFlag = true;

	return;
}

void MotorSVC_impl::Finish()
{
	RTC_INFO(("[MotorSVC_impl::Finish] called "));
	//std::cout << "[MotorSVC_impl::Finish] called "  << std::endl;

	// set FLAG 
	m_FinishFlag = true;

	return;
}

void MotorSVC_impl::Clear()
{
	RTC_INFO(("[MotorSVC_impl::Clear] called "));
	//std::cout << "[MotorSVC_impl::Clear] called "  << std::endl;

	// set FLAG 
	m_ClearFlag = true;

	return;
}

void MotorSVC_impl::Stop()
{
	RTC_INFO(("[MotorSVC_impl::Stop] called "));
	//std::cout << "[MotorSVC_impl::Stop] called "  << std::endl;

	// set FLAG 
	m_StopFlag = true;

	return;
}



// End of example implementational code




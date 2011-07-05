// -*-C++-*-
/*!
 * @file  MotorControlProSVC_impl.h
 * @brief Service implementation header of MotorControlPro.idl
 *
 * @author Yusuke Nakajima (y.nakajima@aist.go.jp)
 *
 * Copyright (c) 2008, National Institute of Advanced Industrial Science and Tec
 * hnology (AIST). All rights reserved. This program is made available under the te
 * rms of the Eclipse Public License v1.0 which accompanies this distribution, and 
 * is available at http://www.eclipse.org/legal/epl-v10.html
 *
 */

#include "MotorControlProSkel.h"
#include <rtm/SystemLogger.h>

#ifndef MOTORCONTROLPROSVC_IMPL_H
#define MOTORCONTROLPROSVC_IMPL_H
 
/*!
 * @class MotorSVC_impl
 * Example class implementing IDL interface Motor
 */
class MotorSVC_impl
 : public virtual POA_Motor,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~MotorSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   MotorSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~MotorSVC_impl();

   // attributes and operations
	/*!
		@brief set gain of PD control 
		@param PGainL P gain (LeftWheel) of PD control
		@param PGainR P gain (RightWheel) of PD control
		@param DGainL D gain (LeftWheel) of PD control
		@param DGainR D gain (RightWheel) of PD control
		@return void				
	*/
   void setPDGain(CORBA::Double PGainL, CORBA::Double PGainR, CORBA::Double DGainL, CORBA::Double DGainR);
	//!	@brief start Component
   void Start();
	//!	@brief finish Component 
   void Finish();
	//!	@brief clear Component 
   void Clear();
 //!	@brief detect bump 
   void Stop();

	public:
		CORBA::Boolean m_GainFlag;   //!<  Flag (if GAIN are changed) 
		CORBA::Boolean m_StartFlag;   //!< Flag (if component are started) 
		CORBA::Boolean m_FinishFlag;  //!< Flag (if component are finished) 
		CORBA::Boolean m_ClearFlag;  //!<  Flag (if component are cleared) 
		CORBA::Boolean m_StopFlag;  //!<  Flag (if bump are detected) 

		double m_PGainL;  //!< P gain (LeftWheel) of PD control 
		double m_PGainR;  //!< P gain (RightWheel) of PD control 
		double m_DGainL;  //!< D gain (LeftWheel) of PD control 
		double m_DGainR;  //!< D gain (RightWheel) of PD control 

		RTC::Logger rtclog;
};



#endif // MOTORCONTROLPROSVC_IMPL_H



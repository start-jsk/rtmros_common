// -*- C++ -*-
/*!
 * @file  HrpsysROSBridgeUtil.h * @brief hrpsys - ros bridge util files * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef HRPSYSROSBRIDGEUTIL_H
#define HRPSYSROSBRIDGEUTIL_H

#include <string>
// http://stackoverflow.com/questions/2941491/compare-versions-as-strings
bool LessThanVersion(const std::string& a,const std::string& b);

//
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/Manager.h>
#include <rtm/CorbaNaming.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <string>
#include <iostream>

std::string GetHrpsysVersion(RTC::CorbaPort& m_ServicePort);

#endif // HRPSYSROSBRIDGEUTIL_H


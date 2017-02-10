// -*- C++ -*-
/*!
 * @file  HrpsysROSBridgeUtil.cpp * @brief hrpsys - ros bridge util files * @date  $Date$ 
 *
 * $Id$ 
 */

#include "HrpsysROSBridgeUtil.h"
#include <ros/console.h>

//
void Parse(int result[3], const std::string& input)
{
  std::istringstream parser(input);
  parser >> result[0];
  for(int idx = 1; idx < 3; idx++)
  {
    parser.get(); //Skip period
    parser >> result[idx];
  }
}

bool LessThanVersion(const std::string& a,const std::string& b)
{
  int parsedA[3], parsedB[3];
  try {
    Parse(parsedA, a);
    Parse(parsedB, b);
    return std::lexicographical_compare(parsedA, parsedA + 4, parsedB, parsedB + 4);
  } catch (...) {
    ROS_ERROR_STREAM("LessThanVersion failed " << a << " < " << b <<", force return false");
    return false;
  }
}

//
std::string GetHrpsysVersion(RTC::CorbaPort& m_ServicePort) {
  std::string hrpsys_version;
  try {
    // copied from HrpsysSeqStateROSBridgeImpl.cpp
    RTC::ConnectorProfileList* connector_profile_list = m_ServicePort.get_connector_profiles();
    for (int i = 0; i < connector_profile_list->length(); i++ ) {
      RTC::ConnectorProfile& connector_profile = (*connector_profile_list)[i];
      RTC::PortServiceList ports_list = connector_profile.ports;
      for (int j = 0; j < ports_list.length(); j++ ) {
        RTC::PortService_var port = ports_list[j];
        RTC::PortProfile* profile = port->get_port_profile();
        if ( profile ) {
          RTC::RTObject_var owner = profile->owner;
          if ( owner ) {
            RTC::ComponentProfile* component_profile = owner->get_component_profile();
            if ( component_profile && std::string(component_profile->type_name) == std::string("SequencePlayer") ) {
              ROS_WARN_STREAM(__PRETTY_FUNCTION__ << " Connected to " << component_profile->type_name  << " (" << component_profile->instance_name << ") version " << component_profile->version);
              hrpsys_version = std::string(component_profile->version);
            }
          }
        }
      }
    }
    if ( hrpsys_version == "" ) {
      ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " Could not get valid hrpsys_version");
    }
  } catch (...) {
    ROS_ERROR_STREAM( __PRETTY_FUNCTION__ << " Could not get hrpsys_version");
  }
  return hrpsys_version;
}




// -*- C++ -*-
/*!
 * @file AbsTransformToPosRpyComp.cpp
 * @brief Standalone component
 * @date $Date$ 
 *
 * $Id$ 
 */
#include <rtm/Manager.h>
#include <iostream>
#include <string>
#include "AbsTransformToPosRpy.h"


void MyModuleInit(RTC::Manager* manager)
{
  AbsTransformToPosRpyInit(manager);
  RTC::RtcBase* comp;

  // Create a component
  comp = manager->createComponent("AbsTransformToPosRpy");


  // Example
  // The following procedure is examples how handle RT-Components.
  // These should not be in this function.

  // Get the component's object reference
//  RTC::RTObject_var rtobj;
//  rtobj = RTC::RTObject::_narrow(manager->getPOA()->servant_to_reference(comp));

  // Get the port list of the component
//  PortList* portlist;
//  portlist = rtobj->get_ports();

  // getting port profiles
//  std::cout << "Number of Ports: ";
//  std::cout << portlist->length() << std::endl << std::endl; 
//  for (CORBA::ULong i(0), n(portlist->length()); i < n; ++i)
//  {
//    Port_ptr port;
//    port = (*portlist)[i];
//    std::cout << "Port" << i << " (name): ";
//    std::cout << port->get_port_profile()->name << std::endl;
//    
//    RTC::PortInterfaceProfileList iflist;
//    iflist = port->get_port_profile()->interfaces;
//    std::cout << "---interfaces---" << std::endl;
//    for (CORBA::ULong i(0), n(iflist.length()); i < n; ++i)
//    {
//      std::cout << "I/F name: ";
//      std::cout << iflist[i].instance_name << std::endl;
//      std::cout << "I/F type: ";
//      std::cout << iflist[i].type_name << std::endl;
//      const char* pol;
//      pol = iflist[i].polarity == 0 ? "PROVIDED" : "REQUIRED";
//      std::cout << "Polarity: " << pol << std::endl;
//    }
//    std::cout << "---properties---" << std::endl;
//    NVUtil::dump(port->get_port_profile()->properties);
//    std::cout << "----------------" << std::endl << std::endl;
//  }

  return;
}

int main (int argc, char** argv)
{
  { // wait for ClockGenerator
    CORBA::ORB_var orb;
    orb = CORBA::ORB_init(argc, argv);
    CosNaming::NamingContext_var cxt;
    CORBA::Object_var	nS = orb->resolve_initial_references("NameService");
    cxt = CosNaming::NamingContext::_narrow(nS);
    CosNaming::Name ncName;
    ncName.length(1);
    ncName[0].id = CORBA::string_dup("ClockGenerator");
    ncName[0].kind = CORBA::string_dup("");
    CORBA::Object_var obj = NULL;
    do {
      try {
	obj = cxt->resolve(ncName);
      } catch(...) {
	std::cerr << "[" << argv[0] << "] wait ClockGenerator" << std::endl;
	sleep(3);
      }
    } while ( obj == NULL );
    std::cerr << "[" << argv[0] << "] found ClockGenerator" << std::endl;
  }

  RTC::Manager* manager;
  manager = RTC::Manager::init(argc, argv);

  // Initialize manager
  manager->init(argc, argv);

  // Set module initialization proceduer
  // This procedure will be invoked in activateManager() function.
  manager->setModuleInitProc(MyModuleInit);

  // Activate manager and register to naming service
  manager->activateManager();

  // run the manager in blocking mode
  // runManager(false) is the default.
  manager->runManager();

  // If you want to run the manager in non-blocking mode, do like this
  // manager->runManager(true);

  return 0;
}


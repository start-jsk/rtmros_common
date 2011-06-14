// -*- C++ -*-
/*!
 * @file HrpsysStatePublisherComp.cpp
 * @brief Standalone component
 * @date $Date$ 
 *
 * $Id$ 
 */
#include <tvmet/Matrix.h>
#include <rtm/CorbaNaming.h>
#include <rtm/Manager.h>
#include <iostream>
#include <string>
#include "HrpsysStatePublisher.h"

PortService_var getPort(RTC::RTObject_var rtobj, const char *port_name) {
  // find name comp::get_ports :key #'(lambda (x) (send x :name))

  RTC::PortServiceList* portlist;
  portlist = rtobj->get_ports();

  for (CORBA::ULong i(0), n(portlist->length()); i < n; ++i)
    {
      PortService_var port;
      port = (*portlist)[i];
      if ( strcmp(strrchr(port->get_port_profile()->name,'.')+1, port_name) == 0
	   ) {
	std::cout << "Port" << i << " (name): ";
	std::cout << port->get_port_profile()->name << std::endl;
        return port;
      }
    }

  // std::cerr
  std::cerr << "not found error" << std::endl;
  return NULL;
}

void ConnectPorts(RTC::PortService_var pout, RTC::PortService_var pin)
{
  // connect ports: copy from simpleIO/ConnectorComp.cpp
  std::string subs_type("flush");
  ConnectorProfile prof;
  std::string period;
  std::string push_policy;
  std::string skip_count;
  std::string endian;
  std::string connect_origin("in");
  std::map<std::string, std::string> buffer_prop;

  prof.connector_id = "";
  prof.name = CORBA::string_dup("connector0");
  prof.ports.length(2);
  prof.ports[0] = pin;
  prof.ports[1] = pout;
  CORBA_SeqUtil::push_back(prof.properties,
                           NVUtil::newNV("dataport.interface_type",
                                         "corba_cdr"));
  CORBA_SeqUtil::push_back(prof.properties,
                           NVUtil::newNV("dataport.dataflow_type",
                                         "push"));
  if (subs_type != "")
    CORBA_SeqUtil::push_back(prof.properties,
			     NVUtil::newNV("dataport.subscription_type",
					   subs_type.c_str()));
  else
    CORBA_SeqUtil::push_back(prof.properties,
			     NVUtil::newNV("dataport.subscription_type",
					   "flush"));
  if (subs_type == "periodic" && period != "")
    CORBA_SeqUtil::push_back(prof.properties,
			     NVUtil::newNV("dataport.publisher.push_rate",
					   period.c_str()));
  if (push_policy != "")
    CORBA_SeqUtil::push_back(prof.properties,
			     NVUtil::newNV("dataport.publisher.push_policy",
					   push_policy.c_str()));
  if (push_policy == "skip" && skip_count != "")
    CORBA_SeqUtil::push_back(prof.properties,
			     NVUtil::newNV("dataport.publisher.skip_count",
					   skip_count.c_str()));
  if (endian != "")
    CORBA_SeqUtil::push_back(prof.properties,
			     NVUtil::newNV("dataport.serializer.cdr.endian",
					   endian.c_str()));

  std::map<std::string, std::string>::iterator it=buffer_prop.begin();
  while (it != buffer_prop.end()) {
    std::string key("dataport.");
    key += it->first;
    CORBA_SeqUtil::push_back(prof.properties,
                             NVUtil::newNV(key.c_str(),
                                           it->second.c_str()));
    ++it;
  }

  ReturnCode_t ret;
  if (connect_origin == "in")
    ret = pin->connect(prof);
  else
    ret = pout->connect(prof);

  assert(ret == RTC::RTC_OK);

  std::cerr << "Connector " << pout->get_port_profile()->name << " -> " << pin->get_port_profile()->name << " ID: " << prof.connector_id << std::endl;
  //NVUtil::dump(prof.properties);

  // done connect
  return;
}

void MyModuleInit(RTC::Manager* manager)
{
  HrpsysStatePublisherInit(manager);
  RTC::RtcBase* comp;

  // Create a component
  comp = manager->createComponent("HrpsysStatePublisher");

  //
  RTC::ComponentProfile_var prof;
  prof = comp->get_component_profile();
  std::cout << "=================================================" << std::endl;
  std::cout << " Component Profile" << std::endl;
  std::cout << "-------------------------------------------------" << std::endl;
  std::cout << "InstanceID:     " << prof->instance_name << std::endl;
  std::cout << "Implementation: " << prof->type_name << std::endl;
  std::cout << "Description:    " << prof->description << std::endl;
  std::cout << "Version:        " << prof->version << std::endl;
  std::cout << "Maker:          " << prof->vendor << std::endl;
  std::cout << "Category:       " << prof->category << std::endl;
  std::cout << "  Other properties   " << std::endl;
  NVUtil::dump(prof->properties);
  std::cout << "=================================================" << std::endl;

  PortServiceList* portlist;
  portlist = comp->get_ports();

  for (CORBA::ULong i(0), n(portlist->length()); i < n; ++i)
    {
      PortService_ptr port;
      port = (*portlist)[i];
      std::cout << "================================================="
                << std::endl;
      std::cout << "Port" << i << " (name): ";
      std::cout << port->get_port_profile()->name << std::endl;
      std::cout << "-------------------------------------------------"
                << std::endl;
      RTC::PortInterfaceProfileList iflist;
      iflist = port->get_port_profile()->interfaces;

      for (CORBA::ULong i(0), n(iflist.length()); i < n; ++i)
        {
	  std::cout << "I/F name: ";
	  std::cout << iflist[i].instance_name << std::endl;
	  std::cout << "I/F type: ";
	  std::cout << iflist[i].type_name << std::endl;
          const char* pol;
          pol = iflist[i].polarity == 0 ? "PROVIDED" : "REQUIRED";
	  std::cout << "Polarity: " << pol << std::endl;
        }
      std::cout << "- properties -" << std::endl;
      NVUtil::dump(port->get_port_profile()->properties);
      std::cout << "-------------------------------------------------" << std::endl;
    }
#if 0
  // Example
  // The following procedure is examples how handle RT-Components.
  // These should not be in this function.
  // Get the component's object reference
  RTC::RTObject_var rtobj;
  rtobj = RTC::RTObject::_narrow(manager->getPOA()->servant_to_reference(comp));

  //
  std::string port_no("2809");
  CORBA::ORB_ptr orb = manager->getORB();
  //std::string name_server("localhost:");
  std::string name_server("192.168.127.132:");
  name_server.append(port_no);
  RTC::CorbaNaming naming(orb, name_server.c_str());

  RTC::RTObject_var rhobj, shobj;
  //
  CorbaConsumer<RTC::RTObject> rh;
  try{
    rh.setObject(naming.resolve("HRP2JSKController(Robot)0.rtc"));
    //rh.setObject(naming.resolve("RobotHardware0.rtc"));
  } catch (...) {
    std::cerr << "err : controller holder" << std::endl;
    return;
  }
  rhobj = rh._ptr();
  //
  CorbaConsumer<RTC::RTObject> sh;
  try {
    sh.setObject(naming.resolve("StateHolder0.rtc"));
  } catch (...) {
    std::cerr << "err : state holder" << std::endl;
    return;
  }
  shobj = sh._ptr();
  ConnectPorts(getPort(shobj, "qOut"), getPort(rtobj, "in_rsangle"));

  // execute
  ExecutionContext_ptr ec = rtobj->get_context(0);
  ec->start();
  ec->activate_component(rtobj);
#endif
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
  RTC::Manager* manager;
  manager = RTC::Manager::init(argc, argv);
  ros::init(argc, argv, "hrpsys_state_publisher", ros::init_options::NoSigintHandler);

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


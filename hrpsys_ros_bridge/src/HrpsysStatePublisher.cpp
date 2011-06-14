// -*- C++ -*-
/*!
 * @file  HrpsysStatePublisher.cpp * @brief HrpsysState component * $Date$ 
 *
 * $Id$ 
 */
#include "HrpsysStatePublisher.h"

// Module specification
// <rtc-template block="module_spec">
static const char* hrpsysstatepublisher_spec[] =
  {
    "implementation_id", "HrpsysStatePublisher",
    "type_name",         "HrpsysStatePublisher",
    "description",       "HrpsysState component",
    "version",           "0.1",
    "vendor",            "JSK",
    "category",          "Generic",
    "activity_type",     "SPORADIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

HrpsysStatePublisher::HrpsysStatePublisher(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_in_rsangleIn("in_rsangle", m_in_rsangle),
    m_in_mcangleIn("in_mcangle", m_in_mcangle),
    m_in_rsrfsensorIn("in_rsrfsensor", m_in_rsrfsensor),
    m_in_rslfsensorIn("in_rslfsensor", m_in_rslfsensor),
    m_in_rsrhsensorIn("in_rsrhsensor", m_in_rsrhsensor),
    m_in_rslhsensorIn("in_rslhsensor", m_in_rslhsensor),
    m_in_gsensorIn("in_gsensor", m_in_gsensor),
    m_in_gyrometerIn("in_gyrometer", m_in_gyrometer)
    // </rtc-template>
{
}

HrpsysStatePublisher::~HrpsysStatePublisher()
{
}


RTC::ReturnCode_t HrpsysStatePublisher::onInitialize()
{
  std::cerr << "@Initilize name : " << getInstanceName() << std::endl;
  joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("in_rsangle", m_in_rsangleIn);
  addInPort("in_mcangle", m_in_mcangleIn);
  addInPort("in_rsrfsensor", m_in_rsrfsensorIn);
  addInPort("in_rslfsensor", m_in_rslfsensorIn);
  addInPort("in_rsrhsensor", m_in_rsrhsensorIn);
  addInPort("in_rslhsensor", m_in_rslhsensorIn);
  addInPort("in_gsensor", m_in_gsensorIn);
  addInPort("in_gyrometer", m_in_gyrometerIn);

  // Set OutPort buffer

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>

  RTC::Properties& prop = getProperties();
  body = new hrp::Body();
  std::string nameServer = m_pManager->getConfig ()["corba.nameservers"];
  int comPos = nameServer.find (",");
  if (comPos < 0)
    {
      comPos = nameServer.length();
    }
  nameServer = nameServer.substr(0, comPos);
  std::cerr << "nameserver " << nameServer.c_str() << std::endl;
  RTC::CorbaNaming naming(m_pManager->getORB(), nameServer.c_str());
  CosNaming::NamingContext::_duplicate(naming.getRootContext());
  //std::string modelfile = prop["model"].c_str();
  std::string modelfile = "/opt/grx/HRP2JSK/model/HRP2JSKmain.wrl";
  //std::string modelfile = "/opt/grx/HIRONX/model/main.wrl",
  try  {
    std::cerr << "Loading " << modelfile << std::endl;
    loadBodyFromModelLoader (body,
			     modelfile.c_str(),
			     CosNaming::NamingContext::_duplicate(naming.getRootContext()));
  } catch ( CORBA::SystemException& ex ) {
    std::cerr << "CORBA::SystemException " << ex._name() << std::endl;
    return RTC::RTC_ERROR;
  } catch ( ... ) {
      std::cerr << "failed to load model[" << prop["model"] << "]" << std::endl;
      return RTC::RTC_ERROR;
  }
  std::cerr << "Loaded " << body->name() << " from " << modelfile <<  std::endl;
  body->calcForwardKinematics();
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t HrpsysStatePublisher::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysStatePublisher::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysStatePublisher::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysStatePublisher::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysStatePublisher::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t HrpsysStatePublisher::onExecute(RTC::UniqueId ec_id)
{
  std::cerr << "@Execute name : " << getInstanceName() << std::endl;
  // m_in_rsangleIn
  if ( m_in_rsangleIn.isNew () ) {
    try {
      m_in_rsangleIn.read();
    }
    catch(const std::runtime_error &e)
      {
	std::cerr << e.what() << std::endl;
      }
    //
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    m_in_rsangleIn.read();

    body->calcForwardKinematics();
    std::cerr << body->name() << std::endl;
    if ( m_in_rsangle.data.length() != body->joints().size() ) {
      std::cerr << "rsangle.data.length(" << m_in_rsangle.data.length() << ") is not equal to body->joints().size(" << body->joints().size() << ")" << std::endl;
      return RTC::RTC_OK;
    }
    body->calcForwardKinematics();

    for ( unsigned int i = 0; i < m_in_rsangle.data.length() ; i++ ){
      body->joint(i)->q = m_in_rsangle.data[i];
      std::cerr << m_in_rsangle.data[i] << " ";
    }
    std::cerr << std::endl;
    std::vector<hrp::Link*>::const_iterator it = body->joints().begin();
    while ( it != body->joints().end() ) {
      hrp::Link* j = ((hrp::Link*)*it);
      std::cout << j->name << " - " << j->q << std::endl;
      joint_state.name.push_back(j->name);
      joint_state.position.push_back(j->q);
      //joint_state.velocity
      //joint_state.effort
      ++it;
    }
    joint_state_pub.publish(joint_state);
    ros::spinOnce();
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t HrpsysStatePublisher::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysStatePublisher::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysStatePublisher::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysStatePublisher::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysStatePublisher::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void HrpsysStatePublisherInit(RTC::Manager* manager)
  {
    coil::Properties profile(hrpsysstatepublisher_spec);
    manager->registerFactory(profile,
                             RTC::Create<HrpsysStatePublisher>,
                             RTC::Delete<HrpsysStatePublisher>);
  }
  
};




// -*- C++ -*-
/*!
 * @file  HrpsysSeqStateROSBridgeImpl.cpp * @brief hrpsys seq state - ros bridge * $Date$ 
 *
 * $Id$ 
 */
#include "HrpsysSeqStateROSBridgeImpl.h"

// Module specification
// <rtc-template block="module_spec">
static const char* hrpsysseqstaterosbridgeimpl_spec[] =
  {
    "implementation_id", "HrpsysSeqStateROSBridgeImpl",
    "type_name",         "HrpsysSeqStateROSBridgeImpl",
    "description",       "hrpsys seq state - ros bridge",
    "version",           "1.0",
    "vendor",            "Kei Okada",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

HrpsysSeqStateROSBridgeImpl::HrpsysSeqStateROSBridgeImpl(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_rsangleIn("rsangle", m_rsangle),
    m_mcangleIn("mcangle", m_mcangle),
    m_baseTformIn("baseTform", m_baseTform),
    m_baseRpyIn("baseRpy", m_baseRpy),
    m_rsvelIn("rsvel", m_rsvel),
    m_rstorqueIn("rstorque", m_rstorque),
    m_servoStateIn("servoState", m_servoState),
    m_rszmpIn("rszmp", m_rszmp),
    m_rsrefCPIn("rsrefCapturePoint", m_rsrefCP),
    m_rsactCPIn("rsactCapturePoint", m_rsactCP),
    m_rsCOPInfoIn("rsCOPInfo", m_rsCOPInfo),
    m_emergencyModeIn("emergencyMode", m_emergencyMode),
    m_refContactStatesIn("refContactStates", m_refContactStates),
    m_actContactStatesIn("actContactStates", m_actContactStates),
    m_controlSwingSupportTimeIn("controlSwingSupportTime", m_controlSwingSupportTime),
    m_mctorqueOut("mctorque", m_mctorque),
    m_SequencePlayerServicePort("SequencePlayerService")

    // </rtc-template>
{
}

HrpsysSeqStateROSBridgeImpl::~HrpsysSeqStateROSBridgeImpl()
{
}


RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("rsangle", m_rsangleIn);
  addInPort("mcangle", m_mcangleIn);
  addInPort("baseTform", m_baseTformIn);
  addInPort("baseRpy", m_baseRpyIn);
  addInPort("rsvel", m_rsvelIn);
  addInPort("rstorque", m_rstorqueIn);
  addInPort("rszmp", m_rszmpIn);
  addInPort("rsrefCapturePoint", m_rsrefCPIn);
  addInPort("rsactCapturePoint", m_rsactCPIn);
  addInPort("servoState", m_servoStateIn);
  addInPort("rsCOPInfo", m_rsCOPInfoIn);
  addInPort("emergencyMode", m_emergencyModeIn);
  addInPort("refContactStates", m_refContactStatesIn);
  addInPort("actContactStates", m_actContactStatesIn);
  addInPort("controlSwingSupportTime", m_controlSwingSupportTimeIn);

  // Set OutPort buffer
  addOutPort("mctorque", m_mctorqueOut);

  // Set service provider to Ports

  // Set service consumers to Ports
  m_SequencePlayerServicePort.registerConsumer("service0", "SequencePlayerService", m_service0);

  // Set CORBA Service Ports
  addPort(m_SequencePlayerServicePort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  RTC::Properties& prop = getProperties();

  dt = 0;
  coil::stringTo(dt, prop["dt"].c_str());

  body = hrp::BodyPtr(new hrp::Body());

  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
    comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  std::string modelfile =  m_pManager->getConfig ()["model"];

  if (!loadBodyFromModelLoader(body, modelfile.c_str(), 
			       CosNaming::NamingContext::_duplicate(naming.getRootContext())
			       )){
    std::cerr << "[HrpsysSeqStateROSBridge] failed to load model[" << prop["model"] << "]" 
	      << std::endl;
  }
  bool ret = false;
  while ( ! ret ) {
    try  {
      bodyinfo = hrp::loadBodyInfo(modelfile.c_str(), CosNaming::NamingContext::_duplicate(naming.getRootContext()));
      ret = loadBodyFromBodyInfo(body, bodyinfo);
    } catch ( CORBA::SystemException& ex ) {
      std::cerr << "[HrpsysSeqStateROSBridge] CORBA::SystemException " << ex._name() << std::endl;
      sleep(1);
    } catch ( ... ) {
      std::cerr << "[HrpsysSeqStateROSBridge] failed to load model[" << modelfile << "]" << std::endl;;
      sleep(1);
    }
  }

  // Force Sensor Settings
  coil::vstring virtual_force_sensor = coil::split(prop["virtual_force_sensor"], ",");
  int npforce = body->numSensors(hrp::Sensor::FORCE);
  int nvforce = virtual_force_sensor.size()/10;
  int nforce  = npforce + nvforce;
  m_rsforce.resize(nforce);
  m_rsforceIn.resize(nforce);
  m_offforce.resize(nforce);
  m_offforceIn.resize(nforce);
  m_mcforce.resize(nforce);
  m_mcforceIn.resize(nforce);
  for (unsigned int i=0; i<npforce; i++){
    hrp::Sensor *s = body->sensor(hrp::Sensor::FORCE, i);
    // force and moment
    m_rsforceIn[i] = new InPort<TimedDoubleSeq>(s->name.c_str(), m_rsforce[i]);
    m_rsforce[i].data.length(6);
    registerInPort(s->name.c_str(), *m_rsforceIn[i]);
    m_rsforceName.push_back(s->name);
    // off force and moment
    m_offforceIn[i] = new InPort<TimedDoubleSeq>(std::string("off_" + s->name).c_str(), m_offforce[i]);
    m_offforce[i].data.length(6);
    registerInPort(s->name.c_str(), *m_offforceIn[i]);
    m_offforceName.push_back(std::string("off_" + s->name));
    // ref force and moment
    m_mcforceIn[i] = new InPort<TimedDoubleSeq>(std::string("ref_" + s->name).c_str(), m_mcforce[i]);
    m_mcforce[i].data.length(6);
    registerInPort(std::string("ref_" + s->name).c_str(), *m_mcforceIn[i]);
    m_mcforceName.push_back(std::string("ref_" + s->name));
    std::cerr << i << " physical force sensor : " << s->name << std::endl;
  }

  // Sensor Settings
  for (int j = 0 ; j < body->numSensorTypes(); j++) {
    for (int i = 0 ; i < body->numSensors(j); i++) {
      hrp::Sensor* sensor = body->sensor(j, i);
      if (! sensor ) {
        std::cerr << "ERROR : Unknown sensor (type : " << j << ", id : " << i << ")" << std::endl;
        std::cerr << "ERROR : Please make sure that each sensor type start from 0 sensorId" << std::endl;
        std::cerr << "ERROR : THIS WILL CAUSE SEVERE PROBLEM, PLEASE FIX YOUR MODEL FILE " << std::endl;
        continue;
      }
      SensorInfo si;
      si.transform.setOrigin( tf::Vector3(sensor->localPos(0), sensor->localPos(1), sensor->localPos(2)) );
      hrp::Vector3 rpy;
      if ( hrp::Sensor::VISION == sensor->type )
        // Rotate sensor->localR 180[deg] because OpenHRP3 camera -Z axis equals to ROS camera Z axis
        // http://www.openrtp.jp/openhrp3/jp/create_model.html
        rpy = hrp::rpyFromRot(sensor->localR * hrp::rodrigues(hrp::Vector3(1,0,0), M_PI));
      else if ( hrp::Sensor::RANGE == sensor->type )
        {
          // OpenHRP3 range sensor, front direction is -Z axis, and detected plane is XZ plane
          // ROS LaserScan, front direction is X axis, and detected plane is XY plane
          // http://www.openrtp.jp/openhrp3/jp/create_model.html
          hrp::Matrix33 m;
          m << 0, -1, 0, 0, 0, 1, -1, 0, 0;
          rpy = hrp::rpyFromRot(sensor->localR * m);
        }
      else
      {
        // localR is parent. https://github.com/start-jsk/rtmros_common/pull/925
        rpy = hrp::rpyFromRot(sensor->link->Rs.inverse() * sensor->localR);
      }
      si.transform.setRotation( tf::createQuaternionFromRPY(rpy(0), rpy(1), rpy(2)) );
      OpenHRP::LinkInfoSequence_var links = bodyinfo->links();
      for ( int k = 0; k < links->length(); k++ ) {
        OpenHRP::SensorInfoSequence& sensors = links[k].sensors;
        for ( int l = 0; l < sensors.length(); l++ ) {
          if ( std::string(sensors[l].name) == std::string(sensor->name) ) {
            si.link_name = links[k].segments[0].name;
            si.type_name = sensors[l].type;
            sensor_info[sensor->name] = si;
          }
        }
      }
    }
  }

  // Virtual Force Sensor Settings
  for(unsigned int j = 0, i = npforce; j < nvforce; j++, i++ ){
    std::string name = virtual_force_sensor[j*10+0];
    std::string base = virtual_force_sensor[j*10+1];
    std::string target = virtual_force_sensor[j*10+2];
    hrp::dvector tr(7);
    for (int k = 0; k < 7; k++ ) {
        coil::stringTo(tr[k], virtual_force_sensor[j*10+3+k].c_str());
    }
    // virtual force and moment
    m_rsforceIn[i] = new InPort<TimedDoubleSeq>(name.c_str(), m_rsforce[i]);
    m_rsforce[i].data.length(6);
    registerInPort(name.c_str(), *m_rsforceIn[i]);
    m_rsforceName.push_back(name);
    // off force and moment
    m_offforceIn[i] = new InPort<TimedDoubleSeq>(std::string("off_" + name).c_str(), m_offforce[i]);
    m_offforce[i].data.length(6);
    registerInPort(name.c_str(), *m_offforceIn[i]);
    m_offforceName.push_back(std::string("off_" + name));
    // reference virtual force and moment
    m_mcforceIn[i] = new InPort<TimedDoubleSeq>(std::string("ref_"+name).c_str(), m_mcforce[i]);
    m_mcforce[i].data.length(6);
    registerInPort(std::string("ref_"+name).c_str(), *m_mcforceIn[i]);
    m_mcforceName.push_back(std::string("ref_"+name).c_str());

	if ( ! body->link(base) ) {
	  std::cerr << "ERROR : unknown link : " << base << std::endl;
	}
	if ( ! body->link(target) ) {
	  std::cerr << "ERROR : unknown link : " << target << std::endl;
	}

    SensorInfo si;
    si.transform.setOrigin( tf::Vector3(tr[0], tr[1], tr[2]) );
    Eigen::Quaternion<double> qtn(Eigen::AngleAxis<double>(tr[6], hrp::Vector3(tr[3],tr[4],tr[5])));
    si.transform.setRotation( tf::Quaternion(qtn.x(), qtn.y(), qtn.z(), qtn.w()) );
    OpenHRP::LinkInfoSequence_var links = bodyinfo->links();
    for ( int k = 0; k < links->length(); k++ ) {
      if ( std::string(links[k].name) == target ) {
        si.link_name = links[k].segments[0].name;
        si.type_name = "Force";
      }
    }
    sensor_info[name] = si;

    std::cerr << i << " virtual force sensor : " << name << ": "  << base << "," << target << std::endl;
  }

  int nacc = body->numSensors(hrp::Sensor::ACCELERATION);
  m_gsensor.resize(nacc);
  m_gsensorIn.resize(nacc);
  m_gsensorName.resize(nacc);
  for (unsigned int i=0; i<nacc; i++){
    hrp::Sensor *s = body->sensor(hrp::Sensor::ACCELERATION, i);
    m_gsensorIn[i] = new InPort<TimedAcceleration3D>(s->name.c_str(), m_gsensor[i]);
    m_gsensorName[i] = s->name.c_str();
    registerInPort(s->name.c_str(), *m_gsensorIn[i]);
    std::cerr << i << " acceleration sensor : " << s->name.c_str() << std::endl;
  }

  int ngyro = body->numSensors(hrp::Sensor::RATE_GYRO);
  m_gyrometer.resize(ngyro);
  m_gyrometerIn.resize(ngyro);
  m_gyrometerName.resize(ngyro);
  for (unsigned int i=0; i<ngyro; i++){
    hrp::Sensor *s = body->sensor(hrp::Sensor::RATE_GYRO, i);
    m_gyrometerIn[i] = new InPort<TimedAngularVelocity3D>(s->name.c_str(), m_gyrometer[i]);
    m_gyrometerName[i] = s->name.c_str();
    registerInPort(s->name.c_str(), *m_gyrometerIn[i]);
    std::cerr << i << " rate sensor : " << s->name.c_str() << std::endl;
  }

  // initialize basePos, baseRpy
  {
    OpenHRP::LinkInfoSequence_var links = bodyinfo->links();
    const OpenHRP::LinkInfo& li = links[0];
    hrp::Vector3 axis;
    axis << li.rotation[0], li.rotation[1], li.rotation[2];
    hrp::Matrix33 R = hrp::rodrigues(axis, li.rotation[3]);
    hrp::Vector3 rpy = hrp::rpyFromRot(R);

    m_baseRpy.data.r = rpy[0];
    m_baseRpy.data.p = rpy[1];
    m_baseRpy.data.y = rpy[2];
  }

  // End effector setting from conf file
  // rleg,TARGET_LINK,BASE_LINK,x,y,z,rx,ry,rz,rth #<=pos + rot (axis+angle)
  coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
  if (end_effectors_str.size() > 0) {
    size_t prop_num = 10;
    size_t num = end_effectors_str.size()/prop_num;
    for (size_t i = 0; i < num; i++) {
      // Parse end-effector information from conf
      std::string ee_name, ee_target, ee_base;
      coil::stringTo(ee_name, end_effectors_str[i*prop_num].c_str());
      coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
      coil::stringTo(ee_base, end_effectors_str[i*prop_num+2].c_str());
      hrp::Vector3 eep;
      for (size_t j = 0; j < 3; j++) {
        coil::stringTo(eep(j), end_effectors_str[i*prop_num+3+j].c_str());
      }
      std::cerr << "[" << m_profile.instance_name << "] End Effector [" << ee_name << "]" << ee_target << " " << ee_base << std::endl;
      // Find pair between end-effector information and force sensor name
      bool is_sensor_exists = false;
      std::string sensor_name;
      for (size_t ii = 0; ii < m_mcforceName.size(); ii++) {
        std::string tmpname = m_mcforceName[ii];
        tmpname.erase(0,4);
        hrp::ForceSensor* sensor = body->sensor<hrp::ForceSensor>(tmpname);
        std::string sensor_link_name;
        if ( sensor ) {
          // real force sensor
          sensor_link_name = sensor->link->name;
        } else if (sensor_info.find(tmpname) !=  sensor_info.end()) {
          // virtual force sensor
          sensor_link_name = sensor_info[tmpname].link_name;
          sensor_link_name = sensor_link_name.substr(0, sensor_link_name.size()-5); // such that LLEG_JOINT0_LINK -> LLEG_JOINT0
        } else {
          std::cerr << "[" << m_profile.instance_name << "]   unknown force param" << std::endl;
          continue;
        }
        hrp::Link* alink = body->link(ee_target);
        while (alink != NULL && alink->name != ee_base && !is_sensor_exists) {
          if ( alink->name == sensor_link_name ) {
            is_sensor_exists = true;
            sensor_name = tmpname;
          }
          alink = alink->parent;
        }
      }
      if (!is_sensor_exists) {
        std::cerr << "[" << m_profile.instance_name << "]   No force sensors found [" << ee_target << "]" << std::endl;
        continue;
      }
      // Set cop_link_info
      COPLinkInfo ci;
      ci.link_name = sensor_info[sensor_name].link_name; // Link name for tf frame
      ci.cop_offset_z = eep(2);
      cop_link_info.insert(std::pair<std::string, COPLinkInfo>(sensor_name, ci));
      std::cerr << "[" << m_profile.instance_name << "]   target = " << ci.link_name << ", sensor_name = " << sensor_name << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   localPos = " << eep.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
    }
  }

  // </rtc-template>
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t HrpsysSeqStateROSBridgeImpl::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void HrpsysSeqStateROSBridgeImplInit(RTC::Manager* manager)
  {
    coil::Properties profile(hrpsysseqstaterosbridgeimpl_spec);
    manager->registerFactory(profile,
                             RTC::Create<HrpsysSeqStateROSBridgeImpl>,
                             RTC::Delete<HrpsysSeqStateROSBridgeImpl>);
  }
  
};




// -*- C++ -*-
/*!
 * @file  HrpsysSeqStateROSBridge.cpp * @brief hrpsys seq state - ros bridge * $Date$ 
 *
 * $Id$ 
 */
#include "HrpsysSeqStateROSBridge.h"
#include "rtm/idl/RTC.hh"
#include "hrpsys/idl/ExecutionProfileService.hh"
#include "hrpsys/idl/RobotHardwareService.hh"
#include <boost/format.hpp>

// Module specification
// <rtc-template block="module_spec">
static const char* hrpsysseqstaterosbridge_spec[] =
  {
    "implementation_id", "HrpsysSeqStateROSBridge",
    "type_name",         "HrpsysSeqStateROSBridge",
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

HrpsysSeqStateROSBridge::HrpsysSeqStateROSBridge(RTC::Manager* manager) :
  server(nh, "fullbody_controller/joint_trajectory_action", false),
  HrpsysSeqStateROSBridgeImpl(manager)
{
  // ros
  server.registerGoalCallback(boost::bind(&HrpsysSeqStateROSBridge::onJointTrajectoryActionGoal, this));
  server.registerPreemptCallback(boost::bind(&HrpsysSeqStateROSBridge::onJointTrajectoryActionPreempt, this));
  sendmsg_srv = nh.advertiseService(std::string("sendmsg"), &HrpsysSeqStateROSBridge::sendMsg, this);
  joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  lfsensor_pub = nh.advertise<geometry_msgs::WrenchStamped>("lfsensor", 10);
  rfsensor_pub = nh.advertise<geometry_msgs::WrenchStamped>("rfsensor", 10);
  joint_controller_state_pub = nh.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("/fullbody_controller/state", 1);
  mot_states_pub = nh.advertise<hrpsys_ros_bridge::MotorStates>("/motor_states", 1);
  diagnostics_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

  server.start();
}

HrpsysSeqStateROSBridge::~HrpsysSeqStateROSBridge() {};

RTC::ReturnCode_t HrpsysSeqStateROSBridge::onFinalize() {
  ROS_ERROR_STREAM("[HrpsysSeqStateROSBridge] @onFinalize : " << getInstanceName());
  server.setPreempted();
  return RTC_OK;
}

RTC::ReturnCode_t HrpsysSeqStateROSBridge::onInitialize() {
  // impl
  HrpsysSeqStateROSBridgeImpl::onInitialize();

  nameserver = m_pManager->getConfig ()["corba.nameservers"];
  // initialize
  ROS_INFO_STREAM("[HrpsysSeqStateROSBridge] @Initilize name : " << getInstanceName());

  body = new hrp::Body();

  std::string nameServer = m_pManager->getConfig ()["corba.nameservers"];
  int comPos = nameServer.find (",");
  if (comPos < 0)
    {
      comPos = nameServer.length();
    }
  nameServer = nameServer.substr(0, comPos);
  ROS_INFO_STREAM("[HrpsysSeqStateROSBridge] nameserver " << nameServer.c_str());
  RTC::CorbaNaming naming(m_pManager->getORB(), nameServer.c_str());
  std::string modelfile =  m_pManager->getConfig ()["model"];

  bool ret = false;
  while ( ! ret ) {
    try  {
      bodyinfo = hrp::loadBodyInfo(modelfile.c_str(), CosNaming::NamingContext::_duplicate(naming.getRootContext()));
      ret = loadBodyFromBodyInfo(body, bodyinfo);
    } catch ( CORBA::SystemException& ex ) {
      ROS_ERROR_STREAM("[HrpsysSeqStateROSBridge] CORBA::SystemException " << ex._name());
      sleep(1);
    } catch ( ... ) {
      ROS_ERROR_STREAM("[HrpsysSeqStateROSBridge] failed to load model[" << modelfile << "]");
      sleep(1);
    }
  }
  if ( body == NULL ) {
    ROS_FATAL_STREAM("[HrpsysSeqStateROSBridge] Error on loading " << modelfile);
    return RTC::RTC_ERROR;
  }

  ROS_INFO_STREAM("[HrpsysSeqStateROSBridge] Loaded " << body->name() << " from " << modelfile);
  body->calcForwardKinematics();

  tm.tick();

  rootlink_name = std::string((*(bodyinfo->links()))[0].segments[0].name);
  interpolationp = false;

  ROS_INFO_STREAM("[HrpsysSeqStateROSBridge] @Initilize name : " << getInstanceName() << " done");

  return RTC::RTC_OK;
}


void HrpsysSeqStateROSBridge::onJointTrajectoryActionGoal() {
  pr2_controllers_msgs::JointTrajectoryGoalConstPtr goal = server.acceptNewGoal();
  m_mutex.lock();

  ROS_INFO_STREAM("[" << getInstanceName() << "] @onJointTrajectoryAction ");
  //std::cerr << goal->trajectory.joint_names.size() << std::endl;

  OpenHRP::dSequenceSequence angles;
  OpenHRP::dSequence duration;

  angles.length(goal->trajectory.points.size()) ;
  duration.length(goal->trajectory.points.size()) ;

  std::vector<std::string> joint_names = goal->trajectory.joint_names;

  for (unsigned int i=0; i < goal->trajectory.points.size(); i++) {
    angles[i].length(body->joints().size());

    trajectory_msgs::JointTrajectoryPoint point = goal->trajectory.points[i];
    for (unsigned int j=0; j < goal->trajectory.joint_names.size(); j++ ) {
      body->link(joint_names[j])->q = point.positions[j];
    }

    body->calcForwardKinematics();

    int j = 0;
    std::vector<hrp::Link*>::const_iterator it = body->joints().begin();
    while ( it != body->joints().end() ) {
      hrp::Link* l = ((hrp::Link*)*it);
      angles[i][j] = l->q;
      ++it;++j;
    }

    ROS_INFO_STREAM("[" << getInstanceName() << "] @onJointTrajectoryAction : time_from_start " << goal->trajectory.points[i].time_from_start.toSec());
    if ( i > 0 ) {
      duration[i] =  goal->trajectory.points[i].time_from_start.toSec() - goal->trajectory.points[i-1].time_from_start.toSec();
    } else { // if i == 0
      if ( goal->trajectory.points.size()== 1 ) {
	duration[i] = goal->trajectory.points[i].time_from_start.toSec();
      } else { // 0.2 is magic number writtein in roseus/euslisp/robot-interface.l
	duration[i] = goal->trajectory.points[i].time_from_start.toSec() - 0.2;
      }
    }
  }

  m_mutex.unlock();

  if ( duration.length() == 1 ) {
    m_service0->setJointAngles(angles[0], duration[0]);
  } else {
    OpenHRP::dSequenceSequence rpy, zmp;
    m_service0->playPattern(angles, rpy, zmp, duration);
  }

  interpolationp = true;
}

void HrpsysSeqStateROSBridge::onJointTrajectoryActionPreempt() {
  server.setPreempted();
}

bool HrpsysSeqStateROSBridge::sendMsg (dynamic_reconfigure::Reconfigure::Request &req,
				       dynamic_reconfigure::Reconfigure::Response &res)
{
  if ( req.config.strs.size() == 2 ) {
    ROS_INFO_STREAM("[" << getInstanceName() << "] @sendMsg [" << req.config.strs[0].value << "]");
    if (req.config.strs[0].value == "setInterpolationMode") {
      ROS_INFO_STREAM("[" << getInstanceName() << "] @sendMsg [" << req.config.strs[1].value  << "]");
      if ( req.config.strs[1].value == ":linear" ) m_service0->setInterpolationMode(OpenHRP::SequencePlayerService::LINEAR);
      else m_service0->setInterpolationMode(OpenHRP::SequencePlayerService::HOFFARBIB);
    } else if (req.config.strs[0].value == "setJointAngles") {
      std::istringstream iss(req.config.strs[1].value);
      OpenHRP::dSequence js;
      js.length(body->joints().size());
      double duration;
      for (size_t ii = 0; ii < body->joints().size(); ii++) iss >> js[ii];
      iss >> duration;
      m_service0->setJointAngles(js, duration);
    } else if (req.config.strs[0].value == "waitInterpolation") {
      m_service0->waitInterpolation();
    }
  } else {
    ROS_ERROR_STREAM("[" << getInstanceName() << "] @sendMsg [Invalid argument string length]");
  }
  return true;
}

RTC::ReturnCode_t HrpsysSeqStateROSBridge::onExecute(RTC::UniqueId ec_id)
{
  pr2_controllers_msgs::JointTrajectoryControllerState joint_controller_state;
  joint_controller_state.header.stamp = ros::Time::now();

  hrpsys_ros_bridge::MotorStates mot_states;
  mot_states.header.stamp = ros::Time::now();

  // servoStateIn
  if ( m_servoStateIn.isNew () ) {
    try {
      m_servoStateIn.read();
      mot_states.header.stamp = ros::Time(m_servoState.tm.sec, m_servoState.tm.nsec);
      //for ( unsigned int i = 0; i < m_servoState.data.length() ; i++ ) std::cerr << m_servoState.data[i] << " "; std::cerr << std::endl;
      assert(m_servoState.data.length() == body->joints().size());
      int joint_size = body->joints().size();
      mot_states.name.resize(joint_size);
      mot_states.calib_state.resize(joint_size);
      mot_states.servo_state.resize(joint_size);
      mot_states.power_state.resize(joint_size);
      mot_states.servo_alarm.resize(joint_size);
      mot_states.driver_temp.resize(joint_size);
      for ( unsigned int i = 0; i < joint_size ; i++ ){
	mot_states.name[i] = body->joint(i)->name;
	mot_states.calib_state[i] = (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::CALIB_STATE_MASK) >> OpenHRP::RobotHardwareService::CALIB_STATE_SHIFT;
	mot_states.servo_state[i] = (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::SERVO_STATE_MASK) >> OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT;
	mot_states.power_state[i] = (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::POWER_STATE_MASK) >> OpenHRP::RobotHardwareService::POWER_STATE_SHIFT;
	mot_states.servo_alarm[i] = (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::SERVO_ALARM_MASK) >> OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT;
	mot_states.driver_temp[i] = (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::DRIVER_TEMP_MASK) >> OpenHRP::RobotHardwareService::DRIVER_TEMP_SHIFT;
      }
      mot_states_pub.publish(mot_states);
    }
    catch(const std::runtime_error &e)
      {
	ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
  }

  // rstorqueIn
  if ( m_rstorqueIn.isNew () ) {
    try {
      m_rstorqueIn.read();
      //for ( unsigned int i = 0; i < m_rstorque.data.length() ; i++ ) std::cerr << m_rstorque.data[i] << " "; std::cerr << std::endl;

    }
    catch(const std::runtime_error &e)
      {
	ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
  }

  // m_in_rsangleIn
  if ( m_rsangleIn.isNew () ) {
    sensor_msgs::JointState joint_state;
    // convert openrtm time to ros time
    joint_state.header.stamp = ros::Time(m_rsangle.tm.sec, m_rsangle.tm.nsec);

    ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecute ec_id : " << ec_id << ", rs:" << m_rsangleIn.isNew () << ", baseTform:" << m_baseTformIn.isNew() << ", lfsensor:" << m_rslfsensorIn.isNew() << ", rfsensor:" << m_rsrfsensorIn.isNew());
    try {
      m_rsangleIn.read();
      //for ( unsigned int i = 0; i < m_rsangle.data.length() ; i++ ) std::cerr << m_rsangle.data[i] << " "; std::cerr << std::endl;
    }
    catch(const std::runtime_error &e)
      {
	ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
    //

    m_mutex.lock();
    body->calcForwardKinematics();
    if ( m_rsangle.data.length() < body->joints().size() ) {
      ROS_ERROR_STREAM("rsangle.data.length(" << m_rsangle.data.length() << ") is not equal to body->joints().size(" << body->joints().size() << ")");
      m_mutex.unlock();
      return RTC::RTC_OK;
    } else if ( m_rsangle.data.length() != body->joints().size() ) {
      ROS_INFO_STREAM("rsangle.data.length(" << m_rsangle.data.length() << ") is not equal to body->joints().size(" << body->joints().size() << ")");
    }
    for ( unsigned int i = 0; i < body->joints().size() ; i++ ){
      body->joint(i)->q = m_rsangle.data[i];
      ROS_DEBUG_STREAM(m_rsangle.data[i] << " ");
    }
    ROS_DEBUG_STREAM(std::endl);
    body->calcForwardKinematics();

    // joint state publish
    std::vector<hrp::Link*>::const_iterator it = body->joints().begin();
    while ( it != body->joints().end() ) {
      hrp::Link* j = ((hrp::Link*)*it);
      ROS_DEBUG_STREAM(j->name << " - " << j->q);
      joint_state.name.push_back(j->name);
      joint_state.position.push_back(j->q);
      joint_controller_state.joint_names.push_back(j->name);
      joint_controller_state.actual.positions.push_back(j->q);
      //joint_state.velocity
      //joint_state.effort
      ++it;
    }
    joint_state.velocity.resize(joint_state.name.size());
    // set effort if m_rstorque is available
    if (m_rstorque.data.length() == body->joints().size()) {
      for ( unsigned int i = 0; i < body->joints().size() ; i++ ){
	joint_state.effort.push_back(m_rstorque.data[i]);
      }
    } else {
      joint_state.effort.resize(joint_state.name.size());
    }
    joint_state_pub.publish(joint_state);
    // sensors publish
    tf::Transform transform;
    for (int j = 0 ; j < body->numSensorTypes(); j++) {
      for (int i = 0 ; i < body->numSensors(j); i++) {
	hrp::Sensor* sensor = body->sensor(j, i);
	transform.setOrigin( tf::Vector3(sensor->localPos(0), sensor->localPos(1), sensor->localPos(2)) );
	hrp::Vector3 rpy = hrp::rpyFromRot(sensor->localR);
	transform.setRotation( tf::createQuaternionFromRPY(rpy(0), rpy(1), rpy(2)) );
	br.sendTransform(tf::StampedTransform(transform, joint_state.header.stamp, sensor->link->name, sensor->name));
      }
    }

    m_mutex.unlock();

    if ( server.isActive() &&
	 interpolationp == true &&  m_service0->isEmpty() == true ) {
      pr2_controllers_msgs::JointTrajectoryResult result;
      server.setSucceeded(result);
      interpolationp = false;
    }

    ros::spinOnce();

    //
    static int count = 0;
    tm.tack();
    if ( tm.interval() > 1 ) {
      ROS_INFO_STREAM("[" << getInstanceName() << "] @onExecutece " << ec_id << " is working at " << count << "[Hz]");
      tm.tick();
      count = 0;
    }
    count ++;
  } else { //m_in_rsangleIn
    double interval = 5;
    tm.tack();
    if ( tm.interval() > interval ) {
      ROS_WARN_STREAM("[" << getInstanceName() << "] @onExecutece " << ec_id << " is not executed last " << interval << "[sec]");
      tm.tick();
    }
  }

  if ( m_mcangleIn.isNew () ) {
    //ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecute ec_id : " << ec_id << ", mc:" << m_mcangleIn.isNew () << ", baseTform:" << m_baseTformIn.isNew() << ", lfsensor:" << m_mclfsensorIn.isNew() << ", rfsensor:" << m_mcrfsensorIn.isNew());
    try {
      m_mcangleIn.read();
    }
    catch(const std::runtime_error &e)
      {
	ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
    // joint controller state publish <- only if both rsangle and mcangle are prepared.
    if ( !joint_controller_state.joint_names.empty() && !joint_controller_state.actual.positions.empty() ) {
      for ( unsigned int i = 0; i < body->joints().size() ; i++ ){
        ROS_DEBUG_STREAM(body->joint(i)->name << " - " << m_mcangle.data[i]);
        joint_controller_state.desired.positions.push_back(m_mcangle.data[i]);
        joint_controller_state.error.positions.push_back(joint_controller_state.actual.positions[i]-joint_controller_state.desired.positions[i]);
      }
      /* set zero for all velocities and accelerations */
      joint_controller_state.desired.velocities.resize(joint_controller_state.joint_names.size());
      joint_controller_state.desired.accelerations.resize(joint_controller_state.joint_names.size());
      joint_controller_state.actual.velocities.resize(joint_controller_state.joint_names.size());
      joint_controller_state.actual.accelerations.resize(joint_controller_state.joint_names.size());
      joint_controller_state.error.velocities.resize(joint_controller_state.joint_names.size());
      joint_controller_state.error.accelerations.resize(joint_controller_state.joint_names.size());

      joint_controller_state_pub.publish(joint_controller_state);
    }
  }

  if ( m_rsJointTemperatureIn.isNew () ) {
    try {
      m_rsJointTemperatureIn.read();
      //for ( unsigned int i = 0; i < m_rsJointTemperature.data.length() ; i++ ) std::cerr << m_rsJointTemperature.data[i] << " "; std::cerr << std::endl;
    }
    catch(const std::runtime_error &e)
      {
	ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
    //
    for ( unsigned int i = 0; i < body->joints().size() ; i++ ){
      mot_states.name.push_back(body->joint(i)->name);
      mot_states.temperature.push_back(m_rsJointTemperature.data[i]);
    }
    mot_states_pub.publish(mot_states);
  }

  if ( m_baseTformIn.isNew () ) {
    m_baseTformIn.read();
    tf::Transform base;
    double *a = m_baseTform.data.get_buffer();
    base.setOrigin( tf::Vector3(a[0], a[1], a[2]) );
    hrp::Matrix33 R;
    hrp::getMatrix33FromRowMajorArray(R, a, 3);
    hrp::Vector3 rpy = hrp::rpyFromRot(R);
    base.setRotation( tf::createQuaternionFromRPY(rpy(0), rpy(1), rpy(2)) );

    // odom publish
    br.sendTransform(tf::StampedTransform(base, ros::Time(m_baseTform.tm.sec,m_baseTform.tm.nsec), "odom", rootlink_name));
  }

  //
  if ( m_rslfsensorIn.isNew () ) {
    try {
      m_rslfsensorIn.read();
      ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecute lfsensor size = " << m_rslfsensor.data.length() );
      if ( m_rslfsensor.data.length() >= 6 ) {
	geometry_msgs::WrenchStamped lfsensor;
	lfsensor.header.stamp = ros::Time(m_rslfsensor.tm.sec, m_rslfsensor.tm.nsec);
	lfsensor.header.frame_id = "lfsensor";
	lfsensor.wrench.force.x = m_rslfsensor.data[0];
	lfsensor.wrench.force.y = m_rslfsensor.data[1];
	lfsensor.wrench.force.z = m_rslfsensor.data[2];
	lfsensor.wrench.torque.x = m_rslfsensor.data[3];
	lfsensor.wrench.torque.y = m_rslfsensor.data[4];
	lfsensor.wrench.torque.z = m_rslfsensor.data[5];
	lfsensor_pub.publish(lfsensor);
      }
    }
    catch(const std::runtime_error &e)
      {
	ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
  }
  if ( m_rsrfsensorIn.isNew () ) {
    try {
      m_rsrfsensorIn.read();
      ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecute rfsensor size = " << m_rsrfsensor.data.length() );
      if ( m_rsrfsensor.data.length() >= 6 ) {
	geometry_msgs::WrenchStamped rfsensor;
	rfsensor.header.stamp = ros::Time(m_rsrfsensor.tm.sec, m_rsrfsensor.tm.nsec);
	rfsensor.header.frame_id = "rfsensor";
	rfsensor.wrench.force.x = m_rsrfsensor.data[0];
	rfsensor.wrench.force.y = m_rsrfsensor.data[1];
	rfsensor.wrench.force.z = m_rsrfsensor.data[2];
	rfsensor.wrench.torque.x = m_rsrfsensor.data[3];
	rfsensor.wrench.torque.y = m_rsrfsensor.data[4];
	rfsensor.wrench.torque.z = m_rsrfsensor.data[5];
	rfsensor_pub.publish(rfsensor);
      }
    }
    catch(const std::runtime_error &e)
      {
	ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
  }

  static int loop = 0;
  static ros::Time last_updated = ros::Time::now();
  if ( loop++ % 1000 == 0 ) {
      diagnostic_msgs::DiagnosticArray diagnostic;
      diagnostic.header.stamp = ros::Time::now();

      diagnostic_msgs::DiagnosticStatus status;
      diagnostic_msgs::KeyValue value;

      status.name = nameserver + " hrpEC profile";
      status.message = "OK";
      status.hardware_id = nameserver;

      value.key = "Time Since Update";
      value.value = str(boost::format("%f")%(ros::Time::now() - last_updated).toSec());
      status.values.push_back(value);

      last_updated = ros::Time::now();

      value.key = "Name Server";
      value.value = nameserver;
      status.values.push_back(value);

      std::vector<RTObject_impl *> comps = m_pManager->getComponents();
      for (std::vector<RTObject_impl *>::iterator it = comps.begin(); it != comps.end(); it++) {
	  diagnostic_msgs::KeyValue value;
	  value.key = "Component";
	  value.value = (*it)->getInstanceName();
	  status.values.push_back(value);
      }
      ExecutionContext_ptr ec = this->get_context(ec_id);
      value.key = "State";
      value.value = (ec->is_running()?"Running":"Stopped");
      status.values.push_back(value);
      value.key = "Rate";
      value.value = str(boost::format("%f")%ec->get_rate());
      status.values.push_back(value);
      value.key = "Kind";
      switch (ec->get_kind()) {
      case RTC::PERIODIC:
	  value.value = "PERIODIC"; break;
      case RTC::EVENT_DRIVEN:
	  value.value = "EVENT_DRIVEN"; break;
      default:
	  value.value = "OTHER";
      }
      status.values.push_back(value);

      try {
	  OpenHRP::ExecutionProfileService_var ep;
	  ep = OpenHRP::ExecutionProfileService::_narrow(ec);
	  OpenHRP::ExecutionProfileService::Profile* profile = ep->getProfile();
	  value.key = "Max Period";
	  value.value = str(boost::format("%f")%profile->max_period);
	  status.values.push_back(value);
	  value.key = "Min Period";
	  value.value = str(boost::format("%f")%profile->min_period);
	  status.values.push_back(value);
	  value.key = "Average Period";
	  value.value = str(boost::format("%f")%profile->avg_period);
	  status.values.push_back(value);
	  value.key = "Max Total Process";
	  value.value = str(boost::format("%f")%profile->max_total_process);
	  status.values.push_back(value);
	  value.key = "Count";
	  value.value = str(boost::format("%d")%profile->count);
	  status.values.push_back(value);
	  value.key = "Time Over";
	  value.value = str(boost::format("%d")%profile->timeover);
	  status.values.push_back(value);
      } catch(CORBA::SystemException& e) {
	  status.message = e._name();
	  status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      } catch (...) {
	  status.message = "ERROR";
	  status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      }

      diagnostic.status.push_back(status);
      diagnostics_pub.publish(diagnostic);
  }
  //
  return RTC::RTC_OK;
}

extern "C"
{
  void HrpsysSeqStateROSBridgeInit(RTC::Manager* manager)
  {
    coil::Properties profile(hrpsysseqstaterosbridge_spec);
    manager->registerFactory(profile,
                             RTC::Create<HrpsysSeqStateROSBridge>,
                             RTC::Delete<HrpsysSeqStateROSBridge>);
  }
};

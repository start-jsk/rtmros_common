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
  joint_controller_state_pub = nh.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("/fullbody_controller/state", 1);
  mot_states_pub = nh.advertise<hrpsys_ros_bridge::MotorStates>("/motor_states", 1);

  server.start();
}

HrpsysSeqStateROSBridge::~HrpsysSeqStateROSBridge() {};

RTC::ReturnCode_t HrpsysSeqStateROSBridge::onFinalize() {
  ROS_ERROR_STREAM("[HrpsysSeqStateROSBridge] @onFinalize : " << getInstanceName());
  server.setPreempted();
  return RTC_OK;
}

RTC::ReturnCode_t HrpsysSeqStateROSBridge::onInitialize() {
  // initialize
  ROS_INFO_STREAM("[HrpsysSeqStateROSBridge] @Initilize name : " << getInstanceName());

  // impl
  HrpsysSeqStateROSBridgeImpl::onInitialize();

  if ( body == NULL ) {
    return RTC::RTC_ERROR;
  }
  ROS_INFO_STREAM("[HrpsysSeqStateROSBridge] Loaded " << body->name());
  body->calcForwardKinematics();

  tm.tick();

  rootlink_name = std::string((*(bodyinfo->links()))[0].segments[0].name);
  interpolationp = false;

  ROS_INFO_STREAM("[HrpsysSeqStateROSBridge] @Initilize name : " << getInstanceName() << " done");

  fsensor_pub.resize(m_rsforceIn.size());
  for (unsigned int i=0; i<m_rsforceIn.size(); i++){
    hrp::Sensor *s = body->sensor(hrp::Sensor::FORCE, i);
    fsensor_pub[i] = nh.advertise<geometry_msgs::WrenchStamped>(s->name, 10);
  }

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

      int extra_size = 0;
      if ( joint_size > 0 ) extra_size = m_servoState.data[0].length() - 1;
      mot_states.extra_data.layout.dim.resize(2);
      mot_states.extra_data.layout.dim[0].label = "joint";
      mot_states.extra_data.layout.dim[0].size = joint_size;
      mot_states.extra_data.layout.dim[0].stride = joint_size*extra_size;
      mot_states.extra_data.layout.dim[1].label = "extra_data";
      mot_states.extra_data.layout.dim[1].size = extra_size;
      mot_states.extra_data.layout.dim[1].stride = extra_size;
      mot_states.extra_data.data.resize(joint_size*extra_size);

      for ( unsigned int i = 0; i < joint_size ; i++ ){
	mot_states.name[i] = body->joint(i)->name;
	mot_states.calib_state[i] = (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::CALIB_STATE_MASK) >> OpenHRP::RobotHardwareService::CALIB_STATE_SHIFT;
	mot_states.servo_state[i] = (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::SERVO_STATE_MASK) >> OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT;
	mot_states.power_state[i] = (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::POWER_STATE_MASK) >> OpenHRP::RobotHardwareService::POWER_STATE_SHIFT;
	mot_states.servo_alarm[i] = (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::SERVO_ALARM_MASK) >> OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT;
	mot_states.driver_temp[i] = (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::DRIVER_TEMP_MASK) >> OpenHRP::RobotHardwareService::DRIVER_TEMP_SHIFT;
        for ( unsigned int j = 0; j < extra_size; j++ ) {
            mot_states.extra_data.data[i*extra_size+j] = ((float *)&(m_servoState.data[i][1]))[j];
        }
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

    ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecute ec_id : " << ec_id << ", rs:" << m_rsangleIn.isNew () << ", baseTform:" << m_baseTformIn.isNew());
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
	OpenHRP::LinkInfoSequence_var links = bodyinfo->links();
	for ( int k = 0; k < links->length(); k++ ) {
	  OpenHRP::SensorInfoSequence& sensors = links[k].sensors;
	  for ( int l = 0; l < sensors.length(); l++ ) {
	      if ( std::string(sensors[l].name) == std::string(sensor->name) ) {
		br.sendTransform(tf::StampedTransform(transform, joint_state.header.stamp, std::string(links[k].segments[0].name), sensor->name));
	      }
	    }
	  }
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

  // publish forces sonsors
  for (unsigned int i=0; i<m_rsforceIn.size(); i++){
    hrp::Sensor *s = body->sensor(hrp::Sensor::FORCE, i);
    if ( m_rsforceIn[i]->isNew() ) {
      try {
	m_rsforceIn[i]->read();
	ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecute lfsensor size = " << m_rsforce[i].data.length() );
	if ( m_rsforce[i].data.length() >= 6 ) {
	  geometry_msgs::WrenchStamped fsensor;
	  fsensor.header.stamp = ros::Time(m_rsforce[i].tm.sec, m_rsforce[i].tm.nsec);
	  fsensor.header.frame_id = s->name;
	  fsensor.wrench.force.x = m_rsforce[i].data[0];
	  fsensor.wrench.force.y = m_rsforce[i].data[1];
	  fsensor.wrench.force.z = m_rsforce[i].data[2];
	  fsensor.wrench.torque.x = m_rsforce[i].data[3];
	  fsensor.wrench.torque.y = m_rsforce[i].data[4];
	  fsensor.wrench.torque.z = m_rsforce[i].data[5];
	  fsensor_pub[i].publish(fsensor);
	}
      }
      catch(const std::runtime_error &e)
	{
	  ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
	}
    }
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

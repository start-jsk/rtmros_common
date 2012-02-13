// -*- C++ -*-
/*!
 * @file  HrpsysSeqStateROSBridge.cpp * @brief hrpsys seq state - ros bridge * $Date$ 
 *
 * $Id$ 
 */
#include "HrpsysSeqStateROSBridge.h"

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
  HrpsysSeqStateROSBridgeImpl(manager),
  server(nh, "fullbody_controller/joint_trajectory_action", false)
{
  // ros
  server.registerGoalCallback(boost::bind(&HrpsysSeqStateROSBridge::onJointTrajectoryActionGoal, this));
  server.registerPreemptCallback(boost::bind(&HrpsysSeqStateROSBridge::onJointTrajectoryActionPreempt, this));
  sendmsg_srv = nh.advertiseService(std::string("sendmsg"), &HrpsysSeqStateROSBridge::sendMsg, this);
  joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  lfsensor_pub = nh.advertise<geometry_msgs::WrenchStamped>("lfsensor", 10);
  rfsensor_pub = nh.advertise<geometry_msgs::WrenchStamped>("rfsensor", 10);

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
      ret = loadBodyFromModelLoader (body,
				     modelfile.c_str(),
				     CosNaming::NamingContext::_duplicate(naming.getRootContext()));
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
    m_mutex.unlock();
    ROS_INFO_STREAM("[" << getInstanceName() << "] @onJointTrajectoryAction : " << goal->trajectory.points[i].time_from_start.toSec());
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
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();

  // m_in_rsangleIn
  if ( m_rsangleIn.isNew () ) {
    ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecute ec_id : " << ec_id << ", rs:" << m_rsangleIn.isNew () << ", pose:" << m_poseIn.isNew() << ", lfsensor:" << m_rslfsensorIn.isNew() << ", rfsensor:" << m_rsrfsensorIn.isNew());
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
      //joint_state.velocity
      //joint_state.effort
      ++it;
    }
    joint_state.velocity.resize(joint_state.name.size());
    joint_state.effort.resize(joint_state.name.size());
    joint_state_pub.publish(joint_state);
    // sensors publish
    tf::Transform transform;
    for (int j = 0 ; j < body->numSensorTypes(); j++) {
      for (int i = 0 ; i < body->numSensors(j); i++) {
	hrp::Sensor* sensor = body->sensor(j, i);
	transform.setOrigin( tf::Vector3(sensor->localPos(0), sensor->localPos(1), sensor->localPos(2)) );
	hrp::Vector3 rpy = hrp::rpyFromRot(sensor->localR);
	transform.setRotation( tf::createQuaternionFromRPY(rpy(0), rpy(1), rpy(2)) );
	br.sendTransform(tf::StampedTransform(transform, joint_state.header.stamp, sensor->link->link_name, sensor->name));
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

  if ( m_poseIn.isNew () ) {
    m_poseIn.read();
    tf::Transform base;
    base.setOrigin( tf::Vector3(m_pose.data.position.x, m_pose.data.position.y, m_pose.data.position.z) );
    base.setRotation( tf::createQuaternionFromRPY(m_pose.data.orientation.r, m_pose.data.orientation.p, m_pose.data.orientation.y) );

    // odom publish
    br.sendTransform(tf::StampedTransform(base, joint_state.header.stamp, "odom", body->rootLink()->link_name));
  }

  //
  if ( m_rslfsensorIn.isNew () ) {
    try {
      m_rslfsensorIn.read();
      ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecute lfsensor size = " << m_rslfsensor.data.length() );
      if ( m_rslfsensor.data.length() >= 6 ) {
	geometry_msgs::WrenchStamped lfsensor;
	lfsensor.header.stamp = joint_state.header.stamp;
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
	rfsensor.header.stamp = joint_state.header.stamp;
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

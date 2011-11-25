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
  server(nh, "fullbody_controller/joint_trajectory_action", boost::bind(&HrpsysSeqStateROSBridge::onJointTrajectoryAction, this, _1), true),
  HrpsysSeqStateROSBridgeImpl(manager)
{
  sendmsg_srv = nh.advertiseService(std::string("sendmsg"), &HrpsysSeqStateROSBridge::sendMsg, this);
}
HrpsysSeqStateROSBridge::~HrpsysSeqStateROSBridge() {};

RTC::ReturnCode_t HrpsysSeqStateROSBridge::onInitialize() {
  // ros
  joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  // impl
  HrpsysSeqStateROSBridgeImpl::onInitialize();

  // initialize
  std::cerr << "[HrpsysSeqStateROSBridge] @Initilize name : " << getInstanceName() << std::endl;


  RTC::Properties& prop = getProperties();
  body = new hrp::Body();
  std::string nameServer = m_pManager->getConfig ()["corba.nameservers"];
  int comPos = nameServer.find (",");
  if (comPos < 0)
    {
      comPos = nameServer.length();
    }
  nameServer = nameServer.substr(0, comPos);
  std::cerr << "[HrpsysSeqStateROSBridge] nameserver " << nameServer.c_str() << std::endl;
  RTC::CorbaNaming naming(m_pManager->getORB(), nameServer.c_str());
  CosNaming::NamingContext::_duplicate(naming.getRootContext());
  std::string modelfile =  m_pManager->getConfig ()["model"];
  try  {
    std::cerr << "[HrpsysSeqStateROSBridge] Loading " << modelfile << std::endl;
    loadBodyFromModelLoader (body,
			     modelfile.c_str(),
			     CosNaming::NamingContext::_duplicate(naming.getRootContext()));
  } catch ( CORBA::SystemException& ex ) {
    std::cerr << "[HrpsysSeqStateROSBridge] CORBA::SystemException " << ex._name() << std::endl;
    return RTC::RTC_ERROR;
  } catch ( ... ) {
    std::cerr << "[HrpsysSeqStateROSBridge] failed to load model[" << prop["model"] << "]" << std::endl;
    return RTC::RTC_ERROR;
  }
  std::cerr << "[HrpsysSeqStateROSBridge] Loaded " << body->name() << " from " << modelfile <<  std::endl;
  body->calcForwardKinematics();

  tm.tick();
  return RTC::RTC_OK;
}


void HrpsysSeqStateROSBridge::onJointTrajectoryAction(const pr2_controllers_msgs::JointTrajectoryGoalConstPtr& goal) {
  m_mutex.lock();
  pr2_controllers_msgs::JointTrajectoryResult result;

  std::cerr << "[" << getInstanceName() << "] @onJointTrajectoryAction ";
  //std::cerr << goal->trajectory.joint_names.size() << std::endl;
  if ( goal->trajectory.points.size() != 1) ROS_ERROR("trajectory.points must be 1");
  std::vector<std::string> joint_names = goal->trajectory.joint_names;
  for (unsigned int i=0; i < goal->trajectory.points.size(); i++) {
    trajectory_msgs::JointTrajectoryPoint point = goal->trajectory.points[i];
    for (unsigned int j=0; j < goal->trajectory.joint_names.size(); j++ ) {
      body->link(joint_names[j])->q = point.positions[j];
      //std::cerr << joint_names[j] << "," << point.positions[j] << "->" << body->link(joint_names[j])->q << std::endl;
    }
  }
  body->calcForwardKinematics();

  //std::cerr <<  body->joints().size() << std::endl;
  OpenHRP::dSequence angles;
  angles.length(body->joints().size());
  int i = 0;
  std::vector<hrp::Link*>::const_iterator it = body->joints().begin();
  while ( it != body->joints().end() ) {
    hrp::Link* j = ((hrp::Link*)*it);
    std::cerr << j->q << " ";
    angles[i] = j->q;
    ++it;++i;
  }
  m_mutex.unlock();
  std::cerr << " : " << goal->trajectory.points[0].time_from_start.toSec() << std::endl;
  m_service0->setJointAngles(angles, goal->trajectory.points[0].time_from_start.toSec());
  m_service0->waitInterpolation();
  server.setSucceeded(result);
}

bool HrpsysSeqStateROSBridge::sendMsg (dynamic_reconfigure::Reconfigure::Request &req,
				       dynamic_reconfigure::Reconfigure::Response &res)
{
  if ( req.config.strs.size() == 2 ) {
    std::cerr <<"[" << getInstanceName() << "] @sendMsg [" << req.config.strs[0].value << "]" << std::endl;
    if (req.config.strs[0].value == "setInterpolationMode") {
      std::cerr <<"[" << getInstanceName() << "] @sendMsg [" << req.config.strs[1].value  << "]" << std::endl;
      if ( req.config.strs[1].value == "linear" ) m_service0->setInterpolationMode(OpenHRP::SequencePlayerService::LINEAR);
      else m_service0->setInterpolationMode(OpenHRP::SequencePlayerService::HOFFARBIB);
    }
  } else {
    std::cerr <<"[" << getInstanceName() << "] @sendMsg [Invalid argument string length]" << std::endl;
  }
  return true;
}

RTC::ReturnCode_t HrpsysSeqStateROSBridge::onExecute(RTC::UniqueId ec_id)
{
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();

  // m_in_rsangleIn
  if ( m_rsangleIn.isNew () ) {
    std::cerr <<"[" << getInstanceName() << "] @onExecute [" << joint_state.header.stamp << "] name : " << ec_id << ", rs:" << m_rsangleIn.isNew () << ", pose:" << m_poseIn.isNew() << std::endl;

    try {
      m_rsangleIn.read();
      for ( unsigned int i = 0; i < m_rsangle.data.length() ; i++ ) std::cerr << m_rsangle.data[i] << " "; std::cerr << std::endl;
    }
    catch(const std::runtime_error &e)
      {
	std::cerr << e.what() << std::endl;
      }
    //

    m_mutex.lock();
    body->calcForwardKinematics();
    if ( m_rsangle.data.length() != body->joints().size() ) {
      std::cerr << "rsangle.data.length(" << m_rsangle.data.length() << ") is not equal to body->joints().size(" << body->joints().size() << ")" << std::endl;
      m_mutex.unlock();
      return RTC::RTC_OK;
    }
    for ( unsigned int i = 0; i < m_rsangle.data.length() ; i++ ){
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

    ros::spinOnce();
  } else { //m_in_rsangleIn
    double interval = 5;
    tm.tack();
    if ( tm.interval() > interval ) {
      std::cout << "[" << getInstanceName() << "] @onExecutece " << ec_id << " is not executed last " << interval << "[sec]" << std::endl;
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

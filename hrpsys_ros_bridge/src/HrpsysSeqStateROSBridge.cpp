// -*- C++ -*-
/*!
 * @file  HrpsysSeqStateROSBridge.cpp * @brief hrpsys seq state - ros bridge * $Date$ 
 *
 * $Id$ 
 */
#include "HrpsysSeqStateROSBridge.h"
#include "rtm/idl/RTC.hh"
#include "hrpsys_ros_bridge/idl/ExecutionProfileService.hh"
#include "hrpsys_ros_bridge/idl/RobotHardwareService.hh"
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
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
  use_sim_time(false), use_hrpsys_time(false),
  joint_trajectory_server(nh, "fullbody_controller/joint_trajectory_action", false),
  follow_joint_trajectory_server(nh, "fullbody_controller/follow_joint_trajectory_action", false),
  HrpsysSeqStateROSBridgeImpl(manager), follow_action_initialized(false), prev_odom_acquired(false),
  update_odom_init_flag(false), prev_lfoot_contact_state(false), prev_rfoot_contact_state(false),
  is_robot_on_ground(false)
{
  // ros
  ros::NodeHandle pnh("~");
  pnh.param("publish_sensor_transforms", publish_sensor_transforms, true);
  pnh.param("publish_odom_transform", publish_odom_transform, true);
  pnh.param("publish_odom_init_transform", publish_odom_init_transform, true);
  pnh.param("publish_ground_transform", publish_ground_transform, true);
  pnh.param("publish_body_on_odom_transform", publish_body_on_odom_transform, true);
  pnh.param("invert_odom_init_tf", invert_odom_init_tf, true);
  pnh.param("check_robot_is_on_ground", check_robot_is_on_ground, true);
  pnh.param("tf_rate", tf_rate, 50.0);
  periodic_update_timer = pnh.createTimer(ros::Duration(1.0 / tf_rate), boost::bind(&HrpsysSeqStateROSBridge::periodicTimerCallback, this, _1));
  
  joint_trajectory_server.registerGoalCallback(boost::bind(&HrpsysSeqStateROSBridge::onJointTrajectoryActionGoal, this));
  joint_trajectory_server.registerPreemptCallback(boost::bind(&HrpsysSeqStateROSBridge::onJointTrajectoryActionPreempt, this));
  follow_joint_trajectory_server.registerGoalCallback(boost::bind(&HrpsysSeqStateROSBridge::onFollowJointTrajectoryActionGoal, this));
  follow_joint_trajectory_server.registerPreemptCallback(boost::bind(&HrpsysSeqStateROSBridge::onFollowJointTrajectoryActionPreempt, this));
  sendmsg_srv = nh.advertiseService(std::string("sendmsg"), &HrpsysSeqStateROSBridge::sendMsg, this);
  set_sensor_transformation_srv = nh.advertiseService("set_sensor_transformation", &HrpsysSeqStateROSBridge::setSensorTransformation, this);
  joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  joint_controller_state_pub = nh.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("/fullbody_controller/state", 1);
  trajectory_command_sub = nh.subscribe("/fullbody_controller/command", 1, &HrpsysSeqStateROSBridge::onTrajectoryCommandCB, this);
  odom_init_trigger_sub = nh.subscribe("/odom_init_trigger", 1, &HrpsysSeqStateROSBridge::odomInitTriggerCB, this);
  mot_states_pub = nh.advertise<hrpsys_ros_bridge::MotorStates>("/motor_states", 1);
  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);
  imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 1);
  imu_rootlink_pub = nh.advertise<sensor_msgs::Imu>("/imu_rootlink", 1);
  odom_init_pose_stamped_pub = nh.advertise<geometry_msgs::PoseStamped>("/odom_init_pose_stamped", 1, true);
  odom_init_transform_pub = nh.advertise<geometry_msgs::TransformStamped>("/odom_init_transform", 1, true);
  odom_transform = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  odom_init_transform = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  ground_transform = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  imu_floor_transform = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  body_on_odom_transform = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  // is use_sim_time is set and no one publishes clock, publish clock time
  use_sim_time = ros::Time::isSimTime();
  clock_sub = nh.subscribe("/clock", 1, &HrpsysSeqStateROSBridge::clock_cb, this);
  { // wait ...
    ros::WallDuration wtm(0, 500000000);
    wtm.sleep();
  }
  if ( use_sim_time ) {
      int num = clock_sub.getNumPublishers();
      ROS_DEBUG("[HrpsysSeqStateROSBridge] number of clock publisher : %d", num);
      clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 5);
      ros::WallTime rnow = ros::WallTime::now();
      while (clock_sub.getNumPublishers() == num) {
        if ((ros::WallTime::now() - rnow).toSec() > 15.0) { // timeout 15 sec
          break;
        }
        ros::WallDuration wtm(0, 1000000);
        wtm.sleep();
      }
      ROS_DEBUG("wating for num of clock publishers = %d", clock_sub.getNumPublishers());
      if(clock_sub.getNumPublishers() == 1) { // if use sim_time and publisher==1, which means clock publisher is only this RosBridge
          ROS_WARN("[HrpsysSeqStateROSBridge] use_hrpsys_time");
          use_hrpsys_time = true;
      } else {
        clock_sub.shutdown();
        clock_pub.shutdown();
        ROS_WARN("[HrpsysSeqStateROSBridge] use_sim_time");
      }
  }
  joint_trajectory_server.start();
  follow_joint_trajectory_server.start();
}

HrpsysSeqStateROSBridge::~HrpsysSeqStateROSBridge() {
  joint_trajectory_server.shutdown();
  follow_joint_trajectory_server.shutdown();
};

RTC::ReturnCode_t HrpsysSeqStateROSBridge::onFinalize() {
  ROS_INFO_STREAM("[HrpsysSeqStateROSBridge] @onFinalize : " << getInstanceName());
  if ( joint_trajectory_server.isActive() ) {
      joint_trajectory_server.setPreempted();
  }

  if (   follow_joint_trajectory_server.isActive() ) {
      follow_joint_trajectory_server.setPreempted();
  }

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

  fsensor_pub.resize(m_rsforceIn.size()+m_mcforceIn.size());
  for (unsigned int i=0; i<m_rsforceIn.size(); i++){
    fsensor_pub[i] = nh.advertise<geometry_msgs::WrenchStamped>(m_rsforceName[i], 10);
  }
  for (unsigned int i=0; i<m_mcforceIn.size(); i++){
    fsensor_pub[i+m_rsforceIn.size()] = nh.advertise<geometry_msgs::WrenchStamped>(m_mcforceName[i], 10);
  }
  zmp_pub = nh.advertise<geometry_msgs::PointStamped>("/zmp", 10);
  ref_cp_pub = nh.advertise<geometry_msgs::PointStamped>("/ref_capture_point", 10);
  act_cp_pub = nh.advertise<geometry_msgs::PointStamped>("/act_capture_point", 10);
  ref_contact_states_pub = nh.advertise<hrpsys_ros_bridge::ContactStatesStamped>("/ref_contact_states", 10);
  act_contact_states_pub = nh.advertise<hrpsys_ros_bridge::ContactStatesStamped>("/act_contact_states", 10);
  cop_pub.resize(m_mcforceName.size());
  for (unsigned int i=0; i<m_mcforceName.size(); i++){
    std::string tmpname(m_mcforceName[i]); // "ref_xx"
    tmpname.erase(0,4); // Remove "ref_"
    cop_pub[i] = nh.advertise<geometry_msgs::PointStamped>(tmpname+"_cop", 10);
  }
  em_mode_pub = nh.advertise<std_msgs::Int32>("emergency_mode", 10);

  return RTC::RTC_OK;
}


void HrpsysSeqStateROSBridge::onJointTrajectory(trajectory_msgs::JointTrajectory trajectory) {
  m_mutex.lock();

  ROS_INFO_STREAM("[" << getInstanceName() << "] @onJointTrajectoryAction ");
  //std::cerr << goal->trajectory.joint_names.size() << std::endl;

  OpenHRP::dSequenceSequence angles;
  OpenHRP::dSequence duration;

  angles.length(trajectory.points.size()) ;
  duration.length(trajectory.points.size()) ;

  std::vector<std::string> joint_names = trajectory.joint_names;

  for (unsigned int i=0; i < trajectory.points.size(); i++) {
    angles[i].length(body->joints().size());

    trajectory_msgs::JointTrajectoryPoint point = trajectory.points[i];
    for (unsigned int j=0; j < trajectory.joint_names.size(); j++ ) {
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

    ROS_INFO_STREAM("[" << getInstanceName() << "] @onJointTrajectoryAction : time_from_start " << trajectory.points[i].time_from_start.toSec());
    if ( i > 0 ) {
      duration[i] =  trajectory.points[i].time_from_start.toSec() - trajectory.points[i-1].time_from_start.toSec();
    } else { // if i == 0
      if ( trajectory.points.size()== 1 ) {
	duration[i] = trajectory.points[i].time_from_start.toSec();
      } else { // 0.2 is magic number writtein in roseus/euslisp/robot-interface.l
	duration[i] = trajectory.points[i].time_from_start.toSec() - 0.2;
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

void HrpsysSeqStateROSBridge::onJointTrajectoryActionGoal() {
  pr2_controllers_msgs::JointTrajectoryGoalConstPtr goal = joint_trajectory_server.acceptNewGoal();
  onJointTrajectory(goal->trajectory);
}

void HrpsysSeqStateROSBridge::onFollowJointTrajectoryActionGoal() {
  control_msgs::FollowJointTrajectoryGoalConstPtr goal = follow_joint_trajectory_server.acceptNewGoal();
  follow_action_initialized = true;
  onJointTrajectory(goal->trajectory);
}

void HrpsysSeqStateROSBridge::onJointTrajectoryActionPreempt() {
  joint_trajectory_server.setPreempted();
}

void HrpsysSeqStateROSBridge::onFollowJointTrajectoryActionPreempt() {
  follow_joint_trajectory_server.setPreempted();
}

void HrpsysSeqStateROSBridge::onTrajectoryCommandCB(const trajectory_msgs::JointTrajectoryConstPtr& msg) {
  onJointTrajectory(*msg);
}

bool HrpsysSeqStateROSBridge::setSensorTransformation(hrpsys_ros_bridge::SetSensorTransformation::Request& req,
                                                      hrpsys_ros_bridge::SetSensorTransformation::Response& res)
{
  boost::mutex::scoped_lock lock(sensor_transformation_mutex);
  sensor_transformations[req.sensor_name] = req.transform;
  return true;
}

bool HrpsysSeqStateROSBridge::sendMsg (dynamic_reconfigure::Reconfigure::Request &req,
				       dynamic_reconfigure::Reconfigure::Response &res)
{
  if ( req.config.strs.size() == 2 ) {
    res.config.strs = req.config.strs;
    ROS_INFO_STREAM("[" << getInstanceName() << "] @sendMsg [" << req.config.strs[0].value << "]");
    if (req.config.strs[0].value == "setInterpolationMode") {
      ROS_INFO_STREAM("[" << getInstanceName() << "] @sendMsg [" << req.config.strs[1].value  << "]");
      if ( req.config.strs[1].value == ":linear" ) {
        ROS_DEBUG("set interpolation mode -> :linear");
        m_service0->setInterpolationMode(OpenHRP::SequencePlayerService::LINEAR);
      } else {
        ROS_DEBUG("set interpolation mode -> :hoffarbib");
        m_service0->setInterpolationMode(OpenHRP::SequencePlayerService::HOFFARBIB);
      }
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
    return false;
  }
  return true;
}

RTC::ReturnCode_t HrpsysSeqStateROSBridge::onExecute(RTC::UniqueId ec_id)
{
  ros::Time tm_on_execute = ros::Time::now();
  pr2_controllers_msgs::JointTrajectoryControllerState joint_controller_state;
  joint_controller_state.header.stamp = tm_on_execute;

  control_msgs::FollowJointTrajectoryFeedback follow_joint_trajectory_feedback;
  follow_joint_trajectory_feedback.header.stamp = tm_on_execute;

  hrpsys_ros_bridge::MotorStates mot_states;
  mot_states.header.stamp = tm_on_execute;

  static int count_all = 0;
  static int count_read = 0; // inc only when data is arrived
  static int count_drop = 0; // inc only when data is dropped
  count_all ++;

  // servoStateIn
  if ( m_servoStateIn.isNew () ) {
    try {
      m_servoStateIn.read();
      if ( use_hrpsys_time ) {
          mot_states.header.stamp = ros::Time(m_servoState.tm.sec, m_servoState.tm.nsec);
      } else{
          mot_states.header.stamp = tm_on_execute;
      }
      if (m_servoState.data.length() != body->joints().size()){
        std::cerr << __PRETTY_FUNCTION__ << "m_servoState.data.length() = " << m_servoState.data.length() << std::endl;
        std::cerr << __PRETTY_FUNCTION__ << "body->joints().size() = " << body->joints().size() << std::endl;
        for ( unsigned int i = 0; i < body->joints().size() ; i++ ) std::cerr << body->joint(i)->name << " "; std::cerr << std::endl;
      }
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
	ROS_ERROR_STREAM("[" << getInstanceName() << "] m_rservoStateIn failed with " << e.what());
      }
  }  // end: servoStateIn

  // rstorqueIn
  if ( m_rstorqueIn.isNew () ) {
    try {
      m_rstorqueIn.read();
      //for ( unsigned int i = 0; i < m_rstorque.data.length() ; i++ ) std::cerr << m_rstorque.data[i] << " "; std::cerr << std::endl;

    }
    catch(const std::runtime_error &e)
      {
	ROS_ERROR_STREAM("[" << getInstanceName() << "] m_rstorqueIn failed with " << e.what());
      }
  }  // end: rstorqueIn

  // rsvelIn
  if ( m_rsvelIn.isNew () ) {
    try {
      m_rsvelIn.read();
      //for ( unsigned int i = 0; i < m_rstorque.data.length() ; i++ ) std::cerr << m_rstorque.data[i] << " "; std::cerr << std::endl;
    } catch(const std::runtime_error &e) {
      ROS_ERROR_STREAM("[" << getInstanceName() << "] m_rsvelIn failed with " << e.what());
    }
  }  // end: rsvelIn

  // m_in_rsangleIn
  if ( m_rsangleIn.isNew () ) {
    sensor_msgs::JointState joint_state;
    // convert openrtm time to ros time
    if ( use_hrpsys_time ) {
        joint_state.header.stamp = ros::Time(m_rsangle.tm.sec, m_rsangle.tm.nsec);
    }else{
        joint_state.header.stamp = tm_on_execute;
    }

    ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecute ec_id : " << ec_id << ", rs:" << m_rsangleIn.isNew () << ", baseTform:" << m_baseTformIn.isNew());
    try {
      m_rsangleIn.read();
      //for ( unsigned int i = 0; i < m_rsangle.data.length() ; i++ ) std::cerr << m_rsangle.data[i] << " "; std::cerr << std::endl;
    }
    catch(const std::runtime_error &e)
      {
	ROS_ERROR_STREAM("[" << getInstanceName() << "] m_rsangleIn failed with " << e.what());
      }
    //
    if ( use_hrpsys_time ) {
        rosgraph_msgs::Clock clock_msg;
        clock_msg.clock = ros::Time(m_rsangle.tm.sec,m_rsangle.tm.nsec);
        clock_pub.publish(clock_msg);
    }

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
      if (j->parent != NULL) {
        ROS_DEBUG_STREAM(j->name << " - " << j->q);
        joint_state.name.push_back(j->name);
        joint_state.position.push_back(j->q);
        joint_controller_state.joint_names.push_back(j->name);
        joint_controller_state.actual.positions.push_back(j->q);
        //joint_state.velocity
        //joint_state.effort
        follow_joint_trajectory_feedback.joint_names.push_back(j->name);
        follow_joint_trajectory_feedback.desired.positions.push_back(j->q);
        follow_joint_trajectory_feedback.actual.positions.push_back(j->q);
        follow_joint_trajectory_feedback.error.positions.push_back(0);
      }
      ++it;
    }
    // set velocity if m_rsvel is available
    if (m_rsvel.data.length() == body->joints().size()) {
      for (unsigned int i = 0; i < body->joints().size(); i++) {
        joint_state.velocity.push_back(m_rsvel.data[i]);
      }
    } else {
      joint_state.velocity.resize(joint_state.name.size());
    }
    // set effort if m_rstorque is available
    if (m_rstorque.data.length() == body->joints().size()) {
      for ( unsigned int i = 0; i < body->joints().size() ; i++ ){
	joint_state.effort.push_back(m_rstorque.data[i]);
      }
    } else {
      joint_state.effort.resize(joint_state.name.size());
    }
    joint_state_pub.publish(joint_state);
    m_mutex.unlock();

    if ( joint_trajectory_server.isActive() &&
	 interpolationp == true &&  m_service0->isEmpty() == true ) {
      pr2_controllers_msgs::JointTrajectoryResult result;
      joint_trajectory_server.setSucceeded(result);
      interpolationp = false;
    }
    if ( follow_joint_trajectory_server.isActive() &&
	 interpolationp == true &&  m_service0->isEmpty() == true ) {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      follow_joint_trajectory_server.setSucceeded(result);
      interpolationp = false;
    }

    ros::spinOnce();

    // diagnostics
    tm.tack();
    if ( tm.interval() > 1 ) {
      ROS_INFO_STREAM("[" << getInstanceName() << "] @onExecutece " << ec_id << " is working at " << count_read << "[Hz] (exec " << count_all << " Hz, dropped " << count_drop << ")");
      tm.tick();
      count_read = 0;
      count_all = 0;
      count_drop = 0;
    }
    static ros::Time t0, t1;
    t1 = ros::Time(m_rsangle.tm.sec,m_rsangle.tm.nsec);
    if ( (t1 - t0).toSec() < dt * 0.9 || dt * 1.1 < (t1 - t0).toSec() ){
      ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecutece " << ec_id << " may drop communication. " << (t1 - t0).toSec() << "[sec] (" << dt << ")");
      count_drop ++;
    }
    t0 = t1;
    count_read ++;
  } else { //m_in_rsangleIn
    double interval = 5;
    tm.tack();
    if ( tm.interval() > interval ) {
      ROS_WARN_STREAM("[" << getInstanceName() << "] @onExecutece " << ec_id << " is not executed last " << interval << "[sec]");
      tm.tick();
    }
  } // end: m_in_rsangleIn

  // m_mcangleIn
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
    if ( !follow_joint_trajectory_feedback.joint_names.empty() &&
         !follow_joint_trajectory_feedback.actual.positions.empty() &&
         follow_action_initialized ) {
      follow_joint_trajectory_server.publishFeedback(follow_joint_trajectory_feedback);
    }
  } // end: m_mcangleIn

  // m_baseTformIn
  bool is_base_valid = false;
  tf::Transform base;
  if ( m_baseTformIn.isNew () ) {
    m_baseTformIn.read();
    is_base_valid = true;
    // calculate base transform
    double *a = m_baseTform.data.get_buffer();
    hrp::Vector3 base_origin = hrp::Vector3(a[0], a[1], a[2]);
    base.setOrigin(tf::Vector3(base_origin[0], base_origin[1], base_origin[2]));
    // calculate pose quaternion
    hrp::Matrix33 R;
    hrp::getMatrix33FromRowMajorArray(R, a, 3);
    hrp::Vector3 rpy = hrp::rpyFromRot(R);
    tf::Quaternion q = tf::createQuaternionFromRPY(rpy(0), rpy(1), rpy(2));
    base.setRotation(q);

    ros::Time odom_stamp;
    if ( use_hrpsys_time ) {
      odom_stamp = ros::Time(m_baseTform.tm.sec, m_baseTform.tm.nsec);
    } else {
      odom_stamp = tm_on_execute;
    }
    
    { // update odometry topics
      boost::mutex::scoped_lock lock(odom_mutex);
      updateOdometry(base, odom_stamp);
      updateOdomInit(odom_stamp);
      updateGroundTransform();
      updateBodyOnOdom();
    }
    
  }  // end: m_baseTformIn

  { // imu block
    boost::mutex::scoped_lock lock(imu_mutex);    
    updateImu(base, is_base_valid, tm_on_execute);
  }

  // publish forces sonsors
  for (unsigned int i=0; i<m_rsforceIn.size(); i++){
    if ( m_rsforceIn[i]->isNew() ) {
      try {
	m_rsforceIn[i]->read();
	ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecute " << m_rsforceName[i] << " size = " << m_rsforce[i].data.length() );
	if ( m_rsforce[i].data.length() >= 6 ) {
	  geometry_msgs::WrenchStamped fsensor;
	  if ( use_hrpsys_time ) {
	    fsensor.header.stamp = ros::Time(m_rsforce[i].tm.sec, m_rsforce[i].tm.nsec);
	  }else{
	    fsensor.header.stamp = tm_on_execute;
	  }
	  fsensor.header.frame_id = m_rsforceName[i].find("off_") != std::string::npos ? m_rsforceName[i].substr(std::string("off_").size()) : m_rsforceName[i];
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
  } // end: publish forces sonsors

  // publish reference forces sonsors
  for (unsigned int i=0; i<m_mcforceIn.size(); i++){
    if ( m_mcforceIn[i]->isNew() ) {
      try {
	m_mcforceIn[i]->read();
	ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecute " << m_mcforceName[i] << " size = " << m_mcforce[i].data.length() );
	if ( m_mcforce[i].data.length() >= 6 ) {
	  geometry_msgs::WrenchStamped fsensor;
	  if ( use_hrpsys_time ) {
	    fsensor.header.stamp = ros::Time(m_mcforce[i].tm.sec, m_mcforce[i].tm.nsec);
	  }else{
	    fsensor.header.stamp = tm_on_execute;
	  }
	  fsensor.header.frame_id = m_mcforceName[i].substr(std::string("ref_").size());
	  fsensor.wrench.force.x = m_mcforce[i].data[0];
	  fsensor.wrench.force.y = m_mcforce[i].data[1];
	  fsensor.wrench.force.z = m_mcforce[i].data[2];
	  fsensor.wrench.torque.x = m_mcforce[i].data[3];
	  fsensor.wrench.torque.y = m_mcforce[i].data[4];
	  fsensor.wrench.torque.z = m_mcforce[i].data[5];
	  fsensor_pub[i+m_rsforceIn.size()].publish(fsensor);
	}
      }
      catch(const std::runtime_error &e)
	{
	  ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
	}
    }
  } // end: publish reference forces sonsors

  if ( m_rszmpIn.isNew() ) {
    try {
      m_rszmpIn.read();
      //ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecute " << m_rsforceName[i] << " size = " << m_rsforce[i].data.length() );
      geometry_msgs::PointStamped zmpv;
      if ( use_hrpsys_time ) {
        zmpv.header.stamp = ros::Time(m_rszmp.tm.sec, m_rszmp.tm.nsec);
      }else{
        zmpv.header.stamp = tm_on_execute;
      }
      zmpv.header.frame_id = rootlink_name;
      zmpv.point.x = m_rszmp.data.x;
      zmpv.point.y = m_rszmp.data.y;
      zmpv.point.z = m_rszmp.data.z;
      zmp_pub.publish(zmpv);
    }
    catch(const std::runtime_error &e)
      {
        ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
  }

  if ( m_rsrefCPIn.isNew() ) {
    try {
      m_rsrefCPIn.read();
      geometry_msgs::PointStamped refCPv;
      if ( use_hrpsys_time ) {
        refCPv.header.stamp = ros::Time(m_rsrefCP.tm.sec, m_rsrefCP.tm.nsec);
      }else{
        refCPv.header.stamp = tm_on_execute;
      }
      refCPv.header.frame_id = rootlink_name;
      refCPv.point.x = m_rsrefCP.data.x;
      refCPv.point.y = m_rsrefCP.data.y;
      refCPv.point.z = m_rsrefCP.data.z;
      ref_cp_pub.publish(refCPv);
    }
    catch(const std::runtime_error &e)
      {
        ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
  }

  if ( m_rsactCPIn.isNew() ) {
    try {
      m_rsactCPIn.read();
      geometry_msgs::PointStamped actCPv;
      if ( use_hrpsys_time ) {
        actCPv.header.stamp = ros::Time(m_rsactCP.tm.sec, m_rsactCP.tm.nsec);
      }else{
        actCPv.header.stamp = tm_on_execute;
      }
      actCPv.header.frame_id = rootlink_name;
      actCPv.point.x = m_rsactCP.data.x;
      actCPv.point.y = m_rsactCP.data.y;
      actCPv.point.z = m_rsactCP.data.z;
      act_cp_pub.publish(actCPv);
    }
    catch(const std::runtime_error &e)
      {
        ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
  }

  if ( m_refContactStatesIn.isNew() && m_controlSwingSupportTimeIn.isNew() ) {
    try {
      m_refContactStatesIn.read();
      m_controlSwingSupportTimeIn.read();
      hrpsys_ros_bridge::ContactStatesStamped refCSs;
      if ( use_hrpsys_time ) {
        refCSs.header.stamp = ros::Time(m_refContactStates.tm.sec, m_refContactStates.tm.nsec);
      }else{
        refCSs.header.stamp = tm_on_execute;
      }
      int limb_size = m_refContactStates.data.length();
      refCSs.states.resize(limb_size);
      for ( unsigned int i = 0; i < limb_size ; i++ ){
        hrpsys_ros_bridge::ContactState s;
        if (m_refContactStates.data[i]) {
          s.state = s.ON;
        } else {
          s.state = s.OFF;
        }
        s.remaining_time = m_controlSwingSupportTime.data[i];
        refCSs.states[i].header.stamp = refCSs.header.stamp;
        refCSs.states[i].header.frame_id = m_rsforceName[i*2];
        refCSs.states[i].state = s;
      }
      ref_contact_states_pub.publish(refCSs);
    }
    catch(const std::runtime_error &e)
      {
        ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
  }

  if ( m_actContactStatesIn.isNew() ) {
    try {
      m_actContactStatesIn.read();
      hrpsys_ros_bridge::ContactStatesStamped actCSs;
      if ( use_hrpsys_time ) {
        actCSs.header.stamp = ros::Time(m_actContactStates.tm.sec, m_actContactStates.tm.nsec);
      }else{
        actCSs.header.stamp = tm_on_execute;
      }
      int limb_size = m_actContactStates.data.length();
      bool lfoot_cs = prev_lfoot_contact_state;
      bool rfoot_cs = prev_rfoot_contact_state;
      actCSs.states.resize(limb_size);
      for ( unsigned int i = 0; i < limb_size ; i++ ){
        hrpsys_ros_bridge::ContactState s;
        if (m_actContactStates.data[i]) {
          s.state = s.ON;
        } else {
          s.state = s.OFF;
        }
        actCSs.states[i].header.stamp = actCSs.header.stamp;
        actCSs.states[i].header.frame_id = m_rsforceName[i*2];
        actCSs.states[i].state = s;
        // check contact states for robot leg sensors (TODO: do not hard-code sensor name)
        if (actCSs.states[i].header.frame_id == "lfsensor") {
          lfoot_cs = m_actContactStates.data[i];
        } else if (actCSs.states[i].header.frame_id == "rfsensor") {
          rfoot_cs = m_actContactStates.data[i];
        }
      }
      checkFootContactState(lfoot_cs, rfoot_cs); // check does robot stand on the ground and set update_odom_init_flag
      act_contact_states_pub.publish(actCSs);
    }
    catch(const std::runtime_error &e)
      {
        ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
  }

  if ( m_rsCOPInfoIn.isNew() ) {
    try {
      m_rsCOPInfoIn.read();
      //ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecute " << m_rsforceName[i] << " size = " << m_rsforce[i].data.length() );
      for (size_t i = 0; i < m_mcforceIn.size(); i++) {
        std::string tmpname = m_mcforceName[i]; // "ref_xx"
        tmpname.erase(0,4); // Remove "ref_"
        if (cop_link_info.find(tmpname) == cop_link_info.end()) continue;
        double fz = m_rsCOPInfo.data[i*3+2];
        if (fz < 1e-3) continue; // If fz is small, do not publish COP.
        geometry_msgs::PointStamped copv;
        if ( use_hrpsys_time ) {
          copv.header.stamp = ros::Time(m_rsCOPInfo.tm.sec, m_rsCOPInfo.tm.nsec);
        }else{
          copv.header.stamp = tm_on_execute;
        }
        copv.header.frame_id = cop_link_info[tmpname].link_name;
        copv.point.x = m_rsCOPInfo.data[i*3+1]/fz; // copx = my / fz
        copv.point.y = m_rsCOPInfo.data[i*3]/fz; // copy = mx / fz
        copv.point.z = cop_link_info[tmpname].cop_offset_z; // cop z position is static.
        cop_pub[i].publish(copv);
      }
    }
    catch(const std::runtime_error &e)
      {
        ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
  }

  if ( m_emergencyModeIn.isNew() ) {
    try {
      m_emergencyModeIn.read();
      //ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecute" << "  emergencyMode: " << m_emergencyMode.data);
      std_msgs::Int32 em_mode;
      em_mode.data = m_emergencyMode.data;
      em_mode_pub.publish(em_mode);
    }
    catch(const std::runtime_error &e)
      {
        ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
  }

  //
  return RTC::RTC_OK;
}

void HrpsysSeqStateROSBridge::periodicTimerCallback(const ros::TimerEvent& event)
{
  ros::Time stamp;
  if ( use_hrpsys_time ) {
    stamp = ros::Time(m_baseTform.tm.sec, m_baseTform.tm.nsec);
  } else {
    stamp = ros::Time::now();;
  }
  
  { // publish odom transforms
    boost::mutex::scoped_lock lock(odom_mutex);
    publishOdometryTransforms(stamp);
  }
  
  { // publish imu transforms
    boost::mutex::scoped_lock lock(imu_mutex);
    publishImuTransform(stamp);
  }

  { // publish sensor transforms
    boost::mutex::scoped_lock lock(sensor_transformation_mutex);
    publishSensorTransform(stamp);
  }
  
}

void HrpsysSeqStateROSBridge::updateOdometry(const tf::Transform &base, const ros::Time &stamp)
{ 
  tf::Vector3 trans = base.getOrigin();
  tf::Quaternion q = base.getRotation();
  hrp::Matrix33 R;
  tf::matrixTFToEigen(base.getBasis(), R); // TODO: It is not good to assume hrp::Matrix33 is typedef of Eigen::Matrix3d implicitly (cf. EigenTypes.h)
  hrp::Vector3 rpy = hrp::rpyFromRot(R); 
  nav_msgs::Odometry odom;
  
  odom.header.frame_id = "odom";
  odom.header.stamp = stamp;
  odom.child_frame_id = rootlink_name;
  odom.pose.pose.position.x = trans[0];
  odom.pose.pose.position.y = trans[1];
  odom.pose.pose.position.z = trans[2];
  odom.pose.pose.orientation.x = q.getX();
  odom.pose.pose.orientation.y = q.getY();
  odom.pose.pose.orientation.z = q.getZ();
  odom.pose.pose.orientation.w = q.getW();
    
  if (prev_odom_acquired) {
    // calc velocity
    double dt = (odom.header.stamp - prev_odom.header.stamp).toSec();
    if (dt > 0) {
      hrp::Matrix33 prev_R = hrp::rotFromRpy(prev_rpy[0], prev_rpy[1], prev_rpy[2]);
      // R = exp(omega_w*dt) * prev_R
      // omega_w is described in global coordinates in relationships of twist transformation.
      // omega in twist.angular is transformed into rootlink coords because twist should be described in child_frame_id.
      hrp::Vector3 omega = R.transpose() * hrp::omegaFromRot(R * prev_R.transpose()) / dt;  // omegaFromRot returns matrix_log
      odom.twist.twist.angular.x = omega[0];
      odom.twist.twist.angular.y = omega[1];
      odom.twist.twist.angular.z = omega[2];
      // calculate velocity (not strict linear twist from odom)
      hrp::Vector3 velocity, local_velocity;
      velocity[0] = (odom.pose.pose.position.x - prev_odom.pose.pose.position.x) / dt;
      velocity[1] = (odom.pose.pose.position.y - prev_odom.pose.pose.position.y) / dt;
      velocity[2] = (odom.pose.pose.position.z - prev_odom.pose.pose.position.z) / dt;
      local_velocity = R.transpose() * velocity; // global -> local
      odom.twist.twist.linear.x = local_velocity[0];
      odom.twist.twist.linear.y = local_velocity[1];
      odom.twist.twist.linear.z = local_velocity[2];
        
      // calculate covariance
      // assume dx, dy >> dz, dgamma >> dalpha, dbeta and use 2d odometry update equation
      Eigen::VectorXd sigma(6);
      sigma << 1.0, 1.0, 0.001, 0.001, 0.001, 0.1; // velocitis are assumed to have constant standard deviations and they are described in base_link local coordinates
      if (std::abs(local_velocity[0]) < 0.01) {
        sigma[0] = 0.001; // trust "stop" state in x
      }
      if (std::abs(local_velocity[1]) < 0.01) {
        sigma[1] = 0.001; // trust "stop" state in y
      }
      if (std::abs(omega[2]) < 0.01) {
        sigma[5] = 0.001; // trust "stop" state in theta
      }
      Eigen::Matrix<double,6,6> prev_pose_cov;
      for(int i = 0; i < 6; i++) { // index in col
        for (int j = 0; j < 6; j++) { // index in raw
          prev_pose_cov(i, j) = prev_odom.pose.covariance[6 * i + j];
        }
      }
      // each variance are assumed to be independent        
      Eigen::VectorXd sigma2(6);
      for (int i = 0; i < 6; i++) {
        sigma2[i] = sigma[i] * sigma[i];
      }
      Eigen::Matrix<double,6,6> twist_transformation = Eigen::Matrix<double,6,6>::Zero(); // matrix [[R, 0], [0, R]] to convert twist from local to global
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          twist_transformation(i, j) = R(i, j);
          twist_transformation(i + 3, j + 3) = R(i, j);
        }
      }
      Eigen::Matrix<double,6,6> twist_cov = sigma2.asDiagonal(); // twist is described as base_link local coordinates
      // update covariance according to the relationships from definition of variance: V(x) = E[(x-u) * (x-u)^T]
      // jacovian is described in world coordinates: x(t+dt) = f(x(t), v(t)), x is in world coordinates
      Eigen::Matrix<double,6,6> jacobi_velocity = Eigen::Matrix<double,6,6>::Identity() * dt;
      Eigen::Matrix<double,6,6> pose_cov = prev_pose_cov + jacobi_velocity * (twist_transformation.transpose() * twist_cov * twist_transformation) * jacobi_velocity.transpose();
      // insert covariance
      for(int i = 0; i < 6; i++) { // index in col
        for (int j = 0; j < 6; j++) { // index in raw
          odom.pose.covariance[6 * i + j] = pose_cov(i, j);
          odom.twist.covariance[6 * i + j] = twist_cov(i, j);
        }
      }
      // publish odometry
      odom_pub.publish(odom);

      // preserve old values
      prev_odom = odom;
      prev_rpy = rpy;

      // make transform from pose message in odom
      odom_transform = tf::Transform(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w),
                                     tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
    }
  } else {
    // previous values are needed to calculate velocity
    prev_odom = odom;
    prev_rpy = rpy;
    prev_odom_acquired = true;
  }
}


bool HrpsysSeqStateROSBridge::checkFootContactState(bool lfoot_contact_state, bool rfoot_contact_state)
{
  if (check_robot_is_on_ground) { // this function does nothing when check_robot_is_on_ground is false
    if ((!prev_lfoot_contact_state && lfoot_contact_state)
        or (!prev_rfoot_contact_state && rfoot_contact_state)) {
      if (!is_robot_on_ground) {
        is_robot_on_ground = true;
        update_odom_init_flag = true; // update odom_init when robot stands on the ground
      }
    } else if (!lfoot_contact_state && !rfoot_contact_state && is_robot_on_ground) {
      is_robot_on_ground = false;
    }
    prev_lfoot_contact_state = lfoot_contact_state;
    prev_rfoot_contact_state = rfoot_contact_state;
  }
}

void HrpsysSeqStateROSBridge::updateOdomInit(const ros::Time &stamp)
{
  if (update_odom_init_flag) {
    // publish odom_init topics
    // whether invert_odom_init is true or not odom_init_pose_stamped and odom_init_transform is described in odom coordinates.
    geometry_msgs::TransformStamped ros_odom_init_coords;
    geometry_msgs::PoseStamped ros_odom_init_pose_stamped;
    // ignore z height and roll/pitch angle transform from odom assuming odom_init is on the flat ground
    tf::Matrix3x3 odom_pose_matrix(odom_transform.getRotation());
    double odom_roll, odom_pitch, odom_yaw;
    odom_pose_matrix.getRPY(odom_roll, odom_pitch, odom_yaw);
    // double odom_init_yaw = atan2(odom_pose_matrix[1][0], odom_pose_matrix[0][0]); // ref: pcl::getEulerAngles
    odom_init_transform.setOrigin(tf::Vector3(odom_transform.getOrigin()[0], odom_transform.getOrigin()[1], 0.0));
    odom_init_transform.setRotation(tf::Quaternion(tf::Vector3(0, 0, 1), odom_yaw));
    // transform (not be affected by invert_odom_init_tf)
    ros_odom_init_coords.header.stamp = stamp;
    ros_odom_init_coords.header.frame_id = "/odom";
    ros_odom_init_coords.child_frame_id = "/odom_init";
    tf::transformTFToMsg(odom_init_transform, ros_odom_init_coords.transform);
    odom_init_transform_pub.publish(ros_odom_init_coords);
    // pose stamped
    ros_odom_init_pose_stamped.header = ros_odom_init_coords.header;
    ros_odom_init_pose_stamped.pose.position.x = ros_odom_init_coords.transform.translation.x;
    ros_odom_init_pose_stamped.pose.position.y = ros_odom_init_coords.transform.translation.y;
    ros_odom_init_pose_stamped.pose.position.z = ros_odom_init_coords.transform.translation.z;
    ros_odom_init_pose_stamped.pose.orientation = ros_odom_init_coords.transform.rotation;
    odom_init_pose_stamped_pub.publish(ros_odom_init_pose_stamped);
    update_odom_init_flag = false;
  }
}

bool HrpsysSeqStateROSBridge::isLeggedRobot()
{
  return (ee_info.find("rleg") != ee_info.end() && ee_info.find("lleg") != ee_info.end()); // TODO: do not hard-code leg link name
}

void HrpsysSeqStateROSBridge::updateGroundTransform()
{
  // TODO: It is not good to assume hrp::Vector3 is typedef of Eigen::Vector3 and hrp::Matrix33 is typedef of Eigen::Matrix3d implicitly (cf. EigenTypes.h)
  std::vector<tf::Transform> base_to_foot;
  std::vector<std::string> leg_names;
  if (isLeggedRobot()) {
    // joint angles are assumed to be already updated
    tf::vectorTFToEigen(odom_transform.getOrigin(), body->rootLink()->p);
    tf::matrixTFToEigen(odom_transform.getBasis(), body->rootLink()->R);
    body->calcForwardKinematics();
    // TODO: do not hard-code leg link name
    leg_names.push_back("lleg"); 
    leg_names.push_back("rleg");
    // calculate current foot coords
    for (size_t i = 0; i < leg_names.size(); i++) {
      hrp::Link* target_link = body->link(ee_info[leg_names[i]].target);
      Eigen::Affine3d ee_target_matrix = (Eigen::Translation3d(target_link->p) * target_link->R); // target_link->p, target_link->R is odom relative
      tf::Transform ee_target_transform;
      tf::transformEigenToTF(ee_target_matrix, ee_target_transform);
      base_to_foot.push_back(ee_target_transform * ee_info[leg_names[i]].transform);
    }
    tf::Vector3 midcoords_pos = base_to_foot[0].getOrigin().lerp(base_to_foot[1].getOrigin(), 0.5);
    tf::Quaternion midcoords_rot = base_to_foot[0].getRotation().slerp(base_to_foot[1].getRotation(), 0.5);
    tf::Transform odom_relative_ground_transform(midcoords_rot, midcoords_pos);
    ground_transform = odom_transform.inverse() * odom_relative_ground_transform;
  }
}

void HrpsysSeqStateROSBridge::updateBodyOnOdom()
{
  double odom_height = odom_transform.getOrigin().getZ();
  double odom_roll, odom_pitch, odom_yaw;
  tf::Matrix3x3(odom_transform.getRotation()).getRPY(odom_roll, odom_pitch, odom_yaw);
  Eigen::Affine3d body_on_odom_pose = (Eigen::Translation3d(0,
                                                            0,
                                                            odom_height) * 
                                       Eigen::AngleAxisd(odom_roll, Eigen::Vector3d::UnitX()) *
                                       Eigen::AngleAxisd(odom_pitch, Eigen::Vector3d::UnitY()));
  tf::transformEigenToTF(body_on_odom_pose, body_on_odom_transform);
}

void HrpsysSeqStateROSBridge::publishOdometryTransforms(const ros::Time &stamp)
{
  std::vector<geometry_msgs::TransformStamped> tf_transforms;
  geometry_msgs::TransformStamped ros_odom_to_body_coords, ros_odom_init_coords, ros_ground_coords, ros_body_on_odom_coords;
  // odom
  if(publish_odom_transform) {
    ros_odom_to_body_coords.header.stamp = stamp;
    ros_odom_to_body_coords.header.frame_id = "/odom";
    ros_odom_to_body_coords.child_frame_id = rootlink_name;
    tf::transformTFToMsg(odom_transform, ros_odom_to_body_coords.transform);
    tf_transforms.push_back(ros_odom_to_body_coords);
  }
  // odom_init
  if (publish_odom_init_transform) {
    ros_odom_init_coords.header.stamp = stamp;
    if (invert_odom_init_tf) {
      ros_odom_init_coords.header.frame_id ="/odom_init";
      ros_odom_init_coords.child_frame_id =  "/odom";
      tf::transformTFToMsg(odom_init_transform.inverse(), ros_odom_init_coords.transform);
    } else {
      ros_odom_init_coords.header.frame_id = "/odom";
      ros_odom_init_coords.child_frame_id = "/odom_init";
      tf::transformTFToMsg(odom_init_transform, ros_odom_init_coords.transform);
    }
    tf_transforms.push_back(ros_odom_init_coords);
  }
  // ground
  if (isLeggedRobot() && publish_ground_transform) {
    ros_ground_coords.header.stamp = stamp;
    ros_ground_coords.header.frame_id = rootlink_name;
    ros_ground_coords.child_frame_id = "/ground";
    tf::transformTFToMsg(ground_transform, ros_ground_coords.transform);
    tf_transforms.push_back(ros_ground_coords);
  }
  // body_on_odom
  if (publish_body_on_odom_transform) {
    ros_body_on_odom_coords.header.stamp = stamp;
    ros_body_on_odom_coords.header.frame_id = rootlink_name;
    ros_body_on_odom_coords.child_frame_id = "/body_on_odom";
    tf::transformTFToMsg(body_on_odom_transform, ros_body_on_odom_coords.transform);
    tf_transforms.push_back(ros_body_on_odom_coords);
  }
  
  br.sendTransform(tf_transforms);
}

void HrpsysSeqStateROSBridge::odomInitTriggerCB(const std_msgs::Empty &trigger)
{
  boost::mutex::scoped_lock lock(odom_mutex);
  update_odom_init_flag = true; // forcely update odom_init
}

void HrpsysSeqStateROSBridge::updateImu(tf::Transform &base, bool is_base_valid, const ros::Time &stamp)
{
  if (m_baseRpyIn.isNew()){
    m_baseRpyIn.read();
  } 
  if (m_baseRpyCurrentIn.isNew()) {
    m_baseRpyCurrentIn.read();
  } 
  for (unsigned int i = 0; i < m_gyrometerIn.size(); i++) {
    if (m_gyrometerIn[i]->isNew()) {
      m_gyrometerIn[i]->read();
    }
  }
  for (unsigned int i = 0; i < m_gsensorIn.size(); i++) {
    if (m_gsensorIn[i]->isNew()) {
      m_gsensorIn[i]->read();
    }
  }
  
  sensor_msgs::Imu imu;
  if (m_gyrometerName.size() > 0) {
    imu.header.frame_id = m_gyrometerName[0];
  } else {
    imu.header.frame_id = rootlink_name;
  }
    
  if ( use_hrpsys_time ) {
    imu.header.stamp = ros::Time(m_baseRpy.tm.sec, m_baseRpy.tm.nsec);
  } else {
    imu.header.stamp = stamp;
  }

  // original imu values
  tf::Quaternion q = tf::createQuaternionFromRPY(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
  imu.orientation.x = q.getX();
  imu.orientation.y = q.getY();
  imu.orientation.z = q.getZ();
  imu.orientation.w = q.getW();
  if (m_gyrometer.size() > 0) {
    imu.angular_velocity.x = m_gyrometer[0].data.avx;
    imu.angular_velocity.y = m_gyrometer[0].data.avy;
    imu.angular_velocity.z = m_gyrometer[0].data.avz;
  }
  if (m_gsensor.size() > 0) {
    imu.linear_acceleration.x = m_gsensor[0].data.ax;
    imu.linear_acceleration.y = m_gsensor[0].data.ay;
    imu.linear_acceleration.z = m_gsensor[0].data.az;
  }
  imu.orientation_covariance[0] = 2.89e-08;
  imu.orientation_covariance[4] = 2.89e-08;
  imu.orientation_covariance[8] = 2.89e-08;
  imu.angular_velocity_covariance[0] = 0.000144;
  imu.angular_velocity_covariance[4] = 0.000144;
  imu.angular_velocity_covariance[8] = 0.000144;
  imu.linear_acceleration_covariance[0] = 0.0096;
  imu.linear_acceleration_covariance[4] = 0.0096;
  imu.linear_acceleration_covariance[8] = 0.0096;
  imu_pub.publish(imu);

  // transformed imu values
  sensor_msgs::Imu imu_rootlink;
  imu_rootlink.header.stamp = imu.header.stamp;
  imu_rootlink.header.frame_id = rootlink_name;
  if (is_base_valid) {
    tf::vectorTFToEigen(base.getOrigin(), body->rootLink()->p);
    tf::matrixTFToEigen(base.getBasis(), body->rootLink()->R);
    body->calcForwardKinematics();
  }
  // orientation
  tf::Quaternion q_rootlink = tf::createQuaternionFromRPY(m_baseRpyCurrent.data.r, m_baseRpyCurrent.data.p, m_baseRpyCurrent.data.y);
  tf::quaternionTFToMsg(q_rootlink, imu_rootlink.orientation);
  imu_rootlink.orientation_covariance = imu.orientation_covariance;
  if (m_gyrometer.size() > 0) { // angular velocity 
    // joint angles are assumed to be already updated
    // TODO: It is not good to assume hrp::Vector3 is typedef of Eigen::Vector3 and hrp::Matrix33 is typedef of Eigen::Matrix3d implicitly (cf. EigenTypes.h)
    // calculate current gyrometer coords
    hrp::Sensor* gyrometer = body->sensor(hrp::Sensor::RATE_GYRO, 0);
    Eigen::Affine3d gyrometer_parent_matrix = Eigen::Translation3d(gyrometer->link->p) * gyrometer->link->R; // odom->gyrometer_link
    Eigen::Affine3d gyrometer_matrix = gyrometer_parent_matrix * (Eigen::Translation3d(gyrometer->localPos) * gyrometer->localR); // odom->gyrometer_link->gyrometer
    tf::Transform gyrometer_transform;
    tf::transformEigenToTF(gyrometer_matrix, gyrometer_transform);
    tf::Transform gyrometer_to_root_transform = gyrometer_transform.inverse() * base; // gyrometer->odom->base
    tf::Vector3 root_relative_imu_angular_velocity = gyrometer_to_root_transform.getBasis() * tf::Vector3(m_gyrometer[0].data.avx, m_gyrometer[0].data.avy, m_gyrometer[0].data.avz);
    tf::vector3TFToMsg(root_relative_imu_angular_velocity, imu_rootlink.angular_velocity);
    imu_rootlink.angular_velocity_covariance = imu.angular_velocity_covariance;
  }
  if (m_gsensor.size() > 0) {  // acceleration
    hrp::Sensor* acceleration = body->sensor(hrp::Sensor::ACCELERATION, 0);
    Eigen::Affine3d acceleration_parent_matrix = Eigen::Translation3d(acceleration->link->p) * acceleration->link->R; // odom->acceleration_link
    Eigen::Affine3d acceleration_matrix = acceleration_parent_matrix * (Eigen::Translation3d(acceleration->localPos) * acceleration->localR); // odom->acceleration_link->acceleration
    tf::Transform acceleration_transform;
    tf::transformEigenToTF(acceleration_matrix, acceleration_transform);
    tf::Transform acceleration_to_root_transform = acceleration_transform.inverse() * base; // acceleration->odom->base
    tf::Vector3 root_relative_imu_acceleration = acceleration_to_root_transform.getBasis() * tf::Vector3(m_gsensor[0].data.ax, m_gsensor[0].data.ay, m_gsensor[0].data.az);
    tf::vector3TFToMsg(root_relative_imu_acceleration, imu_rootlink.linear_acceleration);
    imu_rootlink.linear_acceleration_covariance = imu.linear_acceleration_covariance;
  }
  imu_rootlink_pub.publish(imu_rootlink);

  // Overwrite base pose by imu and pudate imu_floor frame in tf
  if (is_base_valid) {
    base.setRotation(tf::createQuaternionFromRPY(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y));
    imu_floor_transform = base.inverse();
    imu_floor_transform.setOrigin(tf::Vector3(0, 0, 0));
  }
}

void HrpsysSeqStateROSBridge::publishImuTransform(const ros::Time &stamp)
{
#if ROS_VERSION_MINIMUM(1,8,0)
  tf::Matrix3x3 m;
#else
  btMatrix3x3 m; // for electric
#endif
  m = imu_floor_transform.getBasis();
  bool not_nan = true;
  for (int i = 0; i < 3; ++i) {
    if (isnan(m[i].x()) || isnan(m[i].y()) || isnan(m[i].z()))
      not_nan = false;
  }

  bool found_imu_sensor_info = false;
  std::map<std::string, SensorInfo>::const_iterator its = sensor_info.begin();
  std::string imu_sensor_info_name;
  while (its != sensor_info.end()) {
    if ((*its).second.type_name == "RateGyro") {
      imu_sensor_info_name = (*its).first;
      found_imu_sensor_info = true;
      break;
    }
    ++its;
  }
  
  if (not_nan && found_imu_sensor_info) {
    br.sendTransform(tf::StampedTransform(imu_floor_transform, stamp, imu_sensor_info_name, "imu_floor"));
  } else {
    ROS_ERROR_STREAM("[" << getInstanceName() << "] " << "nan value detected in imu_floor! (input: r,p,y="
                     << m_baseRpy.data.r << ","
                     << m_baseRpy.data.p << ","
                     << m_baseRpy.data.y << ")");
  }
}

void HrpsysSeqStateROSBridge::publishSensorTransform(const ros::Time &stamp)
{
  // sensors publish
  if (publish_sensor_transforms) {
    std::map<std::string, SensorInfo>::const_iterator its = sensor_info.begin();
    while ( its != sensor_info.end() ) {
      if (sensor_transformations.find((*its).first) == sensor_transformations.end()) {
        br.sendTransform(tf::StampedTransform((*its).second.transform, stamp, std::string((*its).second.link_name), (*its).first));
      }
      else {
        geometry_msgs::Transform transform = sensor_transformations[(*its).first];
        tf::Transform tf_transform(tf::Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
                                   tf::Vector3(transform.translation.x, transform.translation.y, transform.translation.z));
        br.sendTransform(tf::StampedTransform(tf_transform, stamp, std::string((*its).second.link_name), (*its).first));
      }
      ++its;
    }
  }
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

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
  HrpsysSeqStateROSBridgeImpl(manager), follow_action_initialized(false)
{
  // ros
  joint_trajectory_server.registerGoalCallback(boost::bind(&HrpsysSeqStateROSBridge::onJointTrajectoryActionGoal, this));
  joint_trajectory_server.registerPreemptCallback(boost::bind(&HrpsysSeqStateROSBridge::onJointTrajectoryActionPreempt, this));
  follow_joint_trajectory_server.registerGoalCallback(boost::bind(&HrpsysSeqStateROSBridge::onFollowJointTrajectoryActionGoal, this));
  follow_joint_trajectory_server.registerPreemptCallback(boost::bind(&HrpsysSeqStateROSBridge::onFollowJointTrajectoryActionPreempt, this));
  sendmsg_srv = nh.advertiseService(std::string("sendmsg"), &HrpsysSeqStateROSBridge::sendMsg, this);
  joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  joint_controller_state_pub = nh.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("/fullbody_controller/state", 1);
  trajectory_command_sub = nh.subscribe("/fullbody_controller/command", 1, &HrpsysSeqStateROSBridge::onTrajectoryCommandCB, this);
  mot_states_pub = nh.advertise<hrpsys_ros_bridge::MotorStates>("/motor_states", 1);
  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);
  imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 1);

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

  fsensor_pub.resize(m_rsforceIn.size());
  for (unsigned int i=0; i<m_rsforceIn.size(); i++){
    fsensor_pub[i] = nh.advertise<geometry_msgs::WrenchStamped>(m_rsforceName[i], 10);
  }
  zmp_pub = nh.advertise<geometry_msgs::PointStamped>("/zmp", 10);

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
    std::map<std::string, SensorInfo>::const_iterator its = sensor_info.begin();
    while ( its != sensor_info.end() ) {
      br.sendTransform(tf::StampedTransform((*its).second.transform, joint_state.header.stamp, std::string((*its).second.link_name), (*its).first));
      ++its;
    }

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
  if ( m_baseTformIn.isNew () ) {
    m_baseTformIn.read();
    // tf::Transform base;
    double *a = m_baseTform.data.get_buffer();
    
    // base.setOrigin( tf::Vector3(a[0], a[1], a[2]) );
    hrp::Matrix33 R;
    hrp::getMatrix33FromRowMajorArray(R, a, 3);
    
    hrp::Vector3 rpy = hrp::rpyFromRot(R);
    tf::Quaternion q = tf::createQuaternionFromRPY(rpy(0), rpy(1), rpy(2));
    nav_msgs::Odometry odom;
    //odom.header.frame_id = rootlink_name;
    odom.header.frame_id = "odom";
    odom.header.stamp = ros::Time(m_baseTform.tm.sec, m_baseTform.tm.nsec);
    //odom.child_frame_id = "/odom";
    odom.pose.pose.position.x = a[0];
    odom.pose.pose.position.y = a[1];
    odom.pose.pose.position.z = a[2];
    odom.pose.pose.orientation.x = q.getX();
    odom.pose.pose.orientation.y = q.getY();
    odom.pose.pose.orientation.z = q.getZ();
    odom.pose.pose.orientation.w = q.getW();
    
    odom.pose.covariance[0] = 0.002 * 0.002;
    odom.pose.covariance[7] = 0.002 * 0.002;
    odom.pose.covariance[14] = 0.002 * 0.002;
    odom.pose.covariance[21] = 0.002 * 0.002;
    odom.pose.covariance[28] = 0.002 * 0.002;
    odom.pose.covariance[35] = 0.002 * 0.002;
    if (prev_odom_acquired) {
      // calc velocity
      double dt = (odom.header.stamp - prev_odom.header.stamp).toSec();
      if (dt > 0) {
        odom.twist.twist.linear.x = (odom.pose.pose.position.x - prev_odom.pose.pose.position.x) / dt;
        odom.twist.twist.linear.y = (odom.pose.pose.position.y - prev_odom.pose.pose.position.y) / dt;
        odom.twist.twist.linear.z = (odom.pose.pose.position.z - prev_odom.pose.pose.position.z) / dt;
        odom.twist.twist.angular.x = (rpy(0) - prev_rpy(0)) / dt;
        odom.twist.twist.angular.x = (rpy(1) - prev_rpy(1)) / dt;
        odom.twist.twist.angular.x = (rpy(2) - prev_rpy(2)) / dt;
        odom.twist.covariance = odom.pose.covariance;
        odom_pub.publish(odom);
      }
      prev_odom = odom;
      prev_rpy = rpy;
      
    }
    else {
      prev_odom = odom;
      prev_rpy = rpy;
      prev_odom_acquired = true;
    }
  }  // end: m_baseTformIn

  bool updateImu = false;
  // m_baseRpyIn
  if (m_baseRpyIn.isNew()){
    m_baseRpyIn.read();
    updateImu = true;
  } // end: m_baseRpyIn

  for (unsigned int i = 0; i < m_gyrometerIn.size(); i++) {
    if (m_gyrometerIn[i]->isNew()) {
      m_gyrometerIn[i]->read();
      if (i == 0) {
        updateImu = true;
      }
    }
  }

  for (unsigned int i = 0; i < m_gsensorIn.size(); i++) {
    if (m_gsensorIn[i]->isNew()) {
      m_gsensorIn[i]->read();
      if (i == 0) {
        updateImu = true;
      }
    }
  }
  
  if (updateImu){
    sensor_msgs::Imu imu;
    if (m_gyrometerName.size() > 0) {
      imu.header.frame_id = m_gyrometerName[0];
    }
    else {
      imu.header.frame_id = rootlink_name;
    }
    
    imu.header.stamp = ros::Time(m_baseRpy.tm.sec, m_baseRpy.tm.nsec);
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
	  fsensor.header.frame_id = m_rsforceName[i];
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

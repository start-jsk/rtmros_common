// -*- C++ -*-
/*!
 * @file  HrpsysSeqStateROSBridge.h * @brief hrpsys seq state - ros bridge * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef HRPSYSSEQSTATEROSBRIDGE_H
#define HRPSYSSEQSTATEROSBRIDGE_H

#include "HrpsysSeqStateROSBridgeImpl.h"

// rtm
#include <rtm/CorbaNaming.h>

// ros
#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/WrenchStamped.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "pr2_controllers_msgs/JointTrajectoryAction.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"
#include "dynamic_reconfigure/Reconfigure.h"
#include "hrpsys_ros_bridge/MotorStates.h"
#include "hrpsys_ros_bridge/ContactStatesStamped.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "sensor_msgs/Imu.h"
#include "hrpsys_ros_bridge/SetSensorTransformation.h"

extern const char* hrpsysseqstaterosbridgeimpl_spec[];

class HrpsysSeqStateROSBridge  : public HrpsysSeqStateROSBridgeImpl
{
 public:
  HrpsysSeqStateROSBridge(RTC::Manager* manager) ;
  ~HrpsysSeqStateROSBridge();

  RTC::ReturnCode_t onInitialize();
  RTC::ReturnCode_t onFinalize();
  RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void onJointTrajectory(trajectory_msgs::JointTrajectory trajectory);
  void onJointTrajectoryActionGoal();
  void onJointTrajectoryActionPreempt();
  void onFollowJointTrajectoryActionGoal();
  void onFollowJointTrajectoryActionPreempt();
  void onTrajectoryCommandCB(const trajectory_msgs::JointTrajectoryConstPtr& msg);
  bool sendMsg (dynamic_reconfigure::Reconfigure::Request &req,
                dynamic_reconfigure::Reconfigure::Response &res);
  bool setSensorTransformation(hrpsys_ros_bridge::SetSensorTransformation::Request& req,
                               hrpsys_ros_bridge::SetSensorTransformation::Response& res);
 private:
  ros::NodeHandle nh;
  ros::Publisher joint_state_pub, joint_controller_state_pub, mot_states_pub, diagnostics_pub, clock_pub, zmp_pub, ref_cp_pub, act_cp_pub, odom_pub, imu_pub,
    em_mode_pub, ref_contact_states_pub, act_contact_states_pub, odom_init_pose_stamped_pub, odom_init_transform_pub, imu_rootlink_pub;
  ros::Subscriber trajectory_command_sub, odom_init_trigger_sub;
  std::vector<ros::Publisher> fsensor_pub, cop_pub;
  actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> joint_trajectory_server;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> follow_joint_trajectory_server;
  ros::ServiceServer sendmsg_srv;
  ros::ServiceServer set_sensor_transformation_srv;
  bool interpolationp, use_sim_time, use_hrpsys_time;
  bool publish_sensor_transforms;
  tf::TransformBroadcaster br;

  coil::Mutex m_mutex;
  coil::TimeMeasure tm;

  std::string nameserver;
  std::string rootlink_name;

  ros::Subscriber clock_sub;

  std::map<std::string, geometry_msgs::Transform> sensor_transformations;
  boost::mutex sensor_transformation_mutex;

  nav_msgs::Odometry prev_odom;
  bool prev_odom_acquired;
  hrp::Vector3 prev_rpy;
  void clock_cb(const rosgraph_msgs::ClockPtr& str) {};

  bool follow_action_initialized;

  // odometry relatives
  void updateOdometry(const tf::Transform &base, const ros::Time &stamp);
  bool checkFootContactState(bool lfoot_contact_state, bool rfoot_contact_state);
  void updateOdomInit(const tf::Transform &odom_pose_transform, const ros::Time &stamp);
  void publishOdometryTransforms(const tf::Transform &odom_pose_transform, const ros::Time &stamp);
  void odomInitTriggerCB(const std_msgs::Empty &trigger);
  bool isLeggedRobot();
  void updateGroundTransform(const tf::Transform &odom_pose_transform);
  void updateImu(tf::Transform &base, bool is_base_valid, const ros::Time &stamp);
  boost::mutex odom_init_mutex;
  tf::Transform odom_init_transform, ground_transform;
  bool update_odom_init_flag;
  bool prev_lfoot_contact_state;
  bool prev_rfoot_contact_state;
  bool is_robot_on_ground;
  bool publish_odom_init_transform, invert_odom_init_tf, check_robot_is_on_ground, publish_ground_transform;
};


extern "C"
{
  DLL_EXPORT void HrpsysSeqStateROSBridgeInit(RTC::Manager* manager);
};

#endif // HRPSYSSEQSTATEROSBRIDGE_H


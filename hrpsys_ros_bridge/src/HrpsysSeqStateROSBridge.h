// -*- C++ -*-
/*!
 * @file  HrpsysSeqStateROSBridge.h * @brief hrpsys seq state - ros bridge * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef HRPSYSSEQSTATEROSBRIDGE_H
#define HRPSYSSEQSTATEROSBRIDGE_H

#include "HrpsysSeqStateROSBridgeImpl.h"
#include "HrpsysJointTrajectoryBridge.h"

// rtm
#include <rtm/CorbaNaming.h>

// ros
#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include "std_msgs/Int32.h"
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

class HrpsysJointTrajectoryAction;
class HrpsysSeqStateROSBridge  : public HrpsysSeqStateROSBridgeImpl
{
 public:
  HrpsysSeqStateROSBridge(RTC::Manager* manager) ;
  ~HrpsysSeqStateROSBridge();

  RTC::ReturnCode_t onInitialize();
  RTC::ReturnCode_t onFinalize();
  RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  bool sendMsg (dynamic_reconfigure::Reconfigure::Request &req,
                dynamic_reconfigure::Reconfigure::Response &res);
  bool setSensorTransformation(hrpsys_ros_bridge::SetSensorTransformation::Request& req,
                               hrpsys_ros_bridge::SetSensorTransformation::Response& res);
 private:
  ros::NodeHandle nh;
  ros::Publisher joint_state_pub, mot_states_pub, diagnostics_pub, clock_pub, zmp_pub, ref_cp_pub, act_cp_pub, odom_pub, imu_pub, em_mode_pub, ref_contact_states_pub, act_contact_states_pub;
  ros::Subscriber trajectory_command_sub;
  std::vector<ros::Publisher> fsensor_pub, cop_pub;
  ros::ServiceServer sendmsg_srv;
  ros::ServiceServer set_sensor_transformation_srv;
  bool use_sim_time, use_hrpsys_time;
  bool publish_sensor_transforms;
  tf::TransformBroadcaster br;

  // joint trajectry actions for limb
  std::vector<boost::shared_ptr<HrpsysJointTrajectoryAction> > trajectory_actions;

  coil::Mutex m_mutex;
  coil::TimeMeasure tm;

  std::string nameserver;
  std::string rootlink_name;

  ros::Subscriber clock_sub;

  nav_msgs::Odometry prev_odom;
  bool prev_odom_acquired;
  hrp::Vector3 prev_rpy;
  void clock_cb(const rosgraph_msgs::ClockPtr& str) {};

  boost::mutex tf_mutex;
  double tf_rate;
  ros::Timer periodic_update_timer;
  std::vector<geometry_msgs::TransformStamped> tf_transforms;
  void periodicTimerCallback(const ros::TimerEvent& event);
  
  // odometry relatives
  void updateOdometry(const hrp::Vector3 &trans, const hrp::Matrix33 &R, const ros::Time &stamp);

  // imu relatives
  void updateImu(tf::Transform &base, bool is_base_valid, const ros::Time &stamp);
  
  // sensor relatives
  void updateSensorTransform(const ros::Time &stamp);
  std::map<std::string, geometry_msgs::Transform> sensor_transformations;
  boost::mutex sensor_transformation_mutex;

  friend class HrpsysJointTrajectoryAction;
};


extern "C"
{
  DLL_EXPORT void HrpsysSeqStateROSBridgeInit(RTC::Manager* manager);
};

#endif // HRPSYSSEQSTATEROSBRIDGE_H


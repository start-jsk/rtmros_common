#include <string>
#include <vector>

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>

#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/Sensor.hh>


#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include "hrpsys_gazebo_msgs/JointCommand.h"
#include "hrpsys_gazebo_msgs/RobotState.h"

#include "PubQueue.h"

namespace gazebo
{
  typedef boost::shared_ptr< sensors::ImuSensor > ImuSensorPtr;
  typedef hrpsys_gazebo_msgs::JointCommand JointCommand;
  typedef hrpsys_gazebo_msgs::RobotState RobotState;

  class IOBPlugin : public ModelPlugin
  {
  public:
    IOBPlugin();
    virtual ~IOBPlugin();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  private:
    void UpdateStates();
    void RosQueueThread();
    void DeferredLoad();
    void GetAndPublishRobotStates(const common::Time &_curTime);
    void SetJointCommand(const JointCommand::ConstPtr &_msg);
    void LoadPIDGainsFromParameter();
    void ZeroJointCommand();
    void UpdatePIDControl(double _dt);

    void GetIMUState(const common::Time &_curTime);
    void GetForceTorqueSensorState(const common::Time &_curTime);

    struct force_sensor_info {
      physics::JointPtr joint;
      std::string joint_name;
      std::string frame_id;
    };

    struct imu_sensor_info {
      physics::LinkPtr link;
      ImuSensorPtr sensor;
      std::string sensor_name;
      std::string joint_name;
      std::string frame_id;
    };
    ros::NodeHandle* rosNode;
    ros::CallbackQueue rosQueue;

    physics::WorldPtr world;
    physics::ModelPtr model;
    sdf::ElementPtr sdf;

    event::ConnectionPtr updateConnection;

    boost::thread callbackQueeuThread;
    boost::thread deferredLoadThread;

    common::Time lastControllerUpdateTime;

    RobotState robotState;
    ros::Publisher pubRobotState;
    PubQueue<RobotState>::Ptr pubRobotStateQueue;

    JointCommand jointCommand;
    ros::Subscriber subIOBCommand;

    std::vector<std::string> jointNames;
    physics::Joint_V joints;

    std::map< std::string, struct force_sensor_info > sensorJoints;
    std::map< std::string, struct imu_sensor_info > imuSensors;

    std::vector<double> effortLimit;

    class ErrorTerms {
      /// error term contributions to final control output
      double q_p;
      double d_q_p_dt;
      double k_i_q_i;  // integral term weighted by k_i
      double qd_p;
      friend class IOBPlugin;
    };
    std::vector<ErrorTerms> errorTerms;

    //
    PubMultiQueue pmq;
    boost::mutex mutex;
    //
    std::string robot_name;
    std::string controller_name;

#if 0
    physics::JointControllerPtr jointController;
    transport::NodePtr node;
    transport::PublisherPtr jointCmdPub;
#endif

  };
}

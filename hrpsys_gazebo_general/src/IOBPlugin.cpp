#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <gazebo/transport/Node.hh>
#include <gazebo/common/Assert.hh>

#include "IOBPlugin.h"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(IOBPlugin);

IOBPlugin::IOBPlugin() {
}

IOBPlugin::~IOBPlugin() {
}

void IOBPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

  this->robot_name = "default";
  if (_sdf->HasElement("robotname")) {
    this->robot_name = _sdf->GetValueString("robotname");
  }
  this->controller_name = "default_controller";
  if (_sdf->HasElement("controller")) {
    this->controller_name = _sdf->GetValueString("controller");
  }

  // initialize ros
  if (!ros::isInitialized()) {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  // ros node
  this->rosNode = new ros::NodeHandle("");

  this->model = _parent;
  this->world = this->model->GetWorld();

  // save sdf
  this->sdf = _sdf;

  // initialize update time
  this->lastControllerUpdateTime = this->world->GetSimTime();

  // creating joints from ros param
  if (this->rosNode->hasParam(this->controller_name)) {
    XmlRpc::XmlRpcValue param_val;
    this->rosNode->getParam(this->controller_name, param_val);
    if (param_val.getType() ==  XmlRpc::XmlRpcValue::TypeStruct) {
      std::string rname = param_val["robotname"];
      XmlRpc::XmlRpcValue joint_lst = param_val["joints"];
      XmlRpc::XmlRpcValue fsensors = param_val["force_torque_sensors"];
      XmlRpc::XmlRpcValue imusensors = param_val["imu_sensors"];
      if (rname != this->robot_name) {
        ROS_ERROR("mismatch robotnames: %s (ros parameter) != %s (gazebo element)",
                  rname.c_str(), this->robot_name.c_str());
      } else {
        ROS_INFO("robotname: %s", rname.c_str());
      }
      // joint name
      if (joint_lst.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for(int s = 0; s < joint_lst.size(); s++) {
          std::string n = joint_lst[s];
          ROS_INFO("add joint: %s", n.c_str());
          this->jointNames.push_back(n);
        }
      } else {
        ROS_WARN("Controlled Joints: no setting exists");
      }
      if (fsensors.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        for(XmlRpc::XmlRpcValue::iterator f = fsensors.begin(); f != fsensors.end(); f++) {
          std::string sensor_name = f->first;
          if (f->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            std::string jn = f->second["joint_name"];
            std::string fi = f->second["frame_id"];
            ROS_INFO("force: %s, %s %s", sensor_name.c_str(), jn.c_str(), fi.c_str());

            struct force_sensor_info fsi;
            fsi.joint = this->model->GetJoint(jn);
            if(!fsi.joint) {
              gzerr << "force torque joint (" << jn << ") not found\n";
            } else {
              fsi.joint_name = jn;
              fsi.frame_id = fi;
              sensorJoints[sensor_name] = fsi;
            }
          } else {
            ROS_ERROR("Force-Torque sensor: %s has invalid configuration", sensor_name.c_str());
          }
        }
      } else {
        ROS_WARN("Force-Torque sensor: no setting exists");
      }
      if (imusensors.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        for(XmlRpc::XmlRpcValue::iterator im = imusensors.begin(); im != imusensors.end(); im++) {
          std::string sensor_name = im->first;
          if (im->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            std::string sn = im->second["name"];
            std::string ln = im->second["link_name"];
            std::string fi = im->second["frame_id"];
            ROS_INFO("imu: %s, %s, %s, %s", sensor_name.c_str(), sn.c_str(),
                     ln.c_str(), fi.c_str());

            struct imu_sensor_info msi;
            msi.sensor_name = sn;
            msi.frame_id = fi;
            msi.link = this->model->GetLink(ln);

            if (!msi.link)  {
              gzerr << ln << " not found\n";
            } else {
              // Get imu sensors
              msi.sensor = boost::shared_dynamic_cast<sensors::ImuSensor>
                (sensors::SensorManager::Instance()->GetSensor
                 (this->world->GetName() + "::" + msi.link->GetScopedName() + "::" + msi.sensor_name));

              if (!msi.sensor)  {
                gzerr << sensor_name << "("                             \
                      << (this->world->GetName() + "::" + msi.link->GetScopedName() + "::" + msi.sensor_name) \
                      <<" not found\n" << "\n";
              }
              imuSensors[sensor_name] = msi;
            }
          } else {
            ROS_ERROR("IMU sensor: %s has invalid configuration", sensor_name.c_str());
          }
        }
      } else {
        ROS_WARN("IMU sensor: no setting exists");
      }
    } else {
      ROS_WARN_STREAM("param: " << this->controller_name << ", configuration is not an array.");
    }
  } else {
    ROS_ERROR_STREAM("controller: " << this->controller_name << " has no parameter.");
  }

  // get pointers to joints from gazebo
  this->joints.resize(this->jointNames.size());
  ROS_INFO("joints size = %ld", this->joints.size());
  for (unsigned int i = 0; i < this->joints.size(); ++i) {
    this->joints[i] = this->model->GetJoint(this->jointNames[i]);
    if (!this->joints[i])  {
      ROS_ERROR("%s robot expected joint[%s] not present, plugin not loaded",
                this->robot_name.c_str(), this->jointNames[i].c_str());
      return;
    }
  }

  // get effort limits from gazebo
  this->effortLimit.resize(this->jointNames.size());
  for (unsigned i = 0; i < this->effortLimit.size(); ++i) {
    this->effortLimit[i] = this->joints[i]->GetEffortLimit(0);
    ROS_DEBUG("effort_limit: %s %f", this->joints[i]->GetName().c_str(), this->joints[i]->GetEffortLimit(0));
  }

  {
    // initialize PID states: error terms
    this->errorTerms.resize(this->joints.size());
    for (unsigned i = 0; i < this->errorTerms.size(); ++i) {
      this->errorTerms[i].q_p = 0;
      this->errorTerms[i].d_q_p_dt = 0;
      this->errorTerms[i].k_i_q_i = 0;
      this->errorTerms[i].qd_p = 0;
    }
  }

  {
    // We are not sending names due to the fact that there is an enum
    // joint indices in ...
    this->robotState.position.resize(this->joints.size());
    this->robotState.velocity.resize(this->joints.size());
    this->robotState.effort.resize(this->joints.size());
    this->robotState.kp_position.resize(this->joints.size());
    this->robotState.ki_position.resize(this->joints.size());
    this->robotState.kd_position.resize(this->joints.size());
    this->robotState.kp_velocity.resize(this->joints.size());
    this->robotState.i_effort_min.resize(this->joints.size());
    this->robotState.i_effort_max.resize(this->joints.size());
  }

  {
    this->jointCommand.position.resize(this->joints.size());
    this->jointCommand.velocity.resize(this->joints.size());
    this->jointCommand.effort.resize(this->joints.size());
    this->jointCommand.kp_position.resize(this->joints.size());
    this->jointCommand.ki_position.resize(this->joints.size());
    this->jointCommand.kd_position.resize(this->joints.size());
    this->jointCommand.kp_velocity.resize(this->joints.size());
    this->jointCommand.i_effort_min.resize(this->joints.size());
    this->jointCommand.i_effort_max.resize(this->joints.size());

    this->ZeroJointCommand();
  }

  // ros callback queue for processing subscription
  this->deferredLoadThread = boost::thread(boost::bind(&IOBPlugin::DeferredLoad, this));
}

void IOBPlugin::ZeroJointCommand() {
  for (unsigned i = 0; i < this->jointNames.size(); ++i) {
    this->jointCommand.position[i] = 0;
    this->jointCommand.velocity[i] = 0;
    this->jointCommand.effort[i] = 0;
    // store these directly on altasState, more efficient for pub later
    this->robotState.kp_position[i] = 0;
    this->robotState.ki_position[i] = 0;
    this->robotState.kd_position[i] = 0;
    this->robotState.kp_velocity[i] = 0;
    this->robotState.i_effort_min[i] = 0;
    this->robotState.i_effort_max[i] = 0;
  }
  this->jointCommand.desired_controller_period_ms = 0;
}

void IOBPlugin::LoadPIDGainsFromParameter() {
  // pull down controller parameters
  std::string namestr(this->controller_name);

  for (unsigned int i = 0; i < this->joints.size(); ++i) {
    std::string joint_ns(namestr);
    joint_ns += ("/gains/" + this->joints[i]->GetName() + "/");

    // this is so ugly
    double p_val = 0, i_val = 0, d_val = 0, i_clamp_val = 0;
    std::string p_str = std::string(joint_ns)+"p";
    std::string i_str = std::string(joint_ns)+"i";
    std::string d_str = std::string(joint_ns)+"d";
    std::string i_clamp_str = std::string(joint_ns)+"i_clamp";
    if (!this->rosNode->getParam(p_str, p_val) ||
        !this->rosNode->getParam(i_str, i_val) ||
        !this->rosNode->getParam(d_str, d_val) ||
        !this->rosNode->getParam(i_clamp_str, i_clamp_val))
      {
        ROS_ERROR("couldn't find a param for %s", joint_ns.c_str());
        continue;
      }
    // store these directly on altasState, more efficient for pub later
    this->robotState.kp_position[i]  =  p_val;
    this->robotState.ki_position[i]  =  i_val;
    this->robotState.kd_position[i]  =  d_val;
    this->robotState.i_effort_min[i] = -i_clamp_val;
    this->robotState.i_effort_max[i] =  i_clamp_val;
  }
}

void IOBPlugin::DeferredLoad() {
  // publish multi queue
  this->pmq.startServiceThread();

  // pull down controller parameters
  this->LoadPIDGainsFromParameter();

  // ROS Controller API
  this->pubRobotStateQueue = this->pmq.addPub<RobotState>();
  this->pubRobotState = this->rosNode->advertise<RobotState>(this->robot_name + "/robot_state", 100, true);

  // ros topic subscribtions
  ros::SubscribeOptions IOBCommandSo =
    ros::SubscribeOptions::create<JointCommand>(this->robot_name + "/joint_command", 100,
                                                boost::bind(&IOBPlugin::SetJointCommand, this, _1),
                                                ros::VoidPtr(), &this->rosQueue);
  // Enable TCP_NODELAY because TCP causes bursty communication with high jitter,
  IOBCommandSo.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true);
  this->subIOBCommand = this->rosNode->subscribe(IOBCommandSo);

  // ros callback queue for processing subscription
  this->callbackQueeuThread = boost::thread(boost::bind(&IOBPlugin::RosQueueThread, this));

  //
  this->updateConnection =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&IOBPlugin::UpdateStates, this));
}

void IOBPlugin::SetJointCommand(const JointCommand::ConstPtr &_msg) {
  // Update Joint Command
  boost::mutex::scoped_lock lock(this->mutex);

  this->jointCommand.header.stamp = _msg->header.stamp;

  // for jointCommand, only position, velocity and efforts are used.
  if (_msg->position.size() == this->jointCommand.position.size())
    std::copy(_msg->position.begin(), _msg->position.end(), this->jointCommand.position.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
      " elements position[%ld] than expected[%ld]",
      _msg->position.size(), this->jointCommand.position.size());

  if (_msg->velocity.size() == this->jointCommand.velocity.size())
    std::copy(_msg->velocity.begin(), _msg->velocity.end(), this->jointCommand.velocity.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
      " elements velocity[%ld] than expected[%ld]",
      _msg->velocity.size(), this->jointCommand.velocity.size());

  if (_msg->effort.size() == this->jointCommand.effort.size())
    std::copy(_msg->effort.begin(), _msg->effort.end(), this->jointCommand.effort.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
      " elements effort[%ld] than expected[%ld]",
      _msg->effort.size(), this->jointCommand.effort.size());

  // the rest are stored in robotState for publication
  if (_msg->kp_position.size() == this->robotState.kp_position.size())
    std::copy(_msg->kp_position.begin(), _msg->kp_position.end(), this->robotState.kp_position.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
      " elements kp_position[%ld] than expected[%ld]",
      _msg->kp_position.size(), this->robotState.kp_position.size());

  if (_msg->ki_position.size() == this->robotState.ki_position.size())
    std::copy(_msg->ki_position.begin(), _msg->ki_position.end(), this->robotState.ki_position.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
      " elements ki_position[%ld] than expected[%ld]",
      _msg->ki_position.size(), this->robotState.ki_position.size());

  if (_msg->kd_position.size() == this->robotState.kd_position.size())
    std::copy(_msg->kd_position.begin(), _msg->kd_position.end(), this->robotState.kd_position.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
      " elements kd_position[%ld] than expected[%ld]",
      _msg->kd_position.size(), this->robotState.kd_position.size());

  if (_msg->kp_velocity.size() == this->robotState.kp_velocity.size())
    std::copy(_msg->kp_velocity.begin(), _msg->kp_velocity.end(), this->robotState.kp_velocity.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
      " elements kp_velocity[%ld] than expected[%ld]",
      _msg->kp_velocity.size(), this->robotState.kp_velocity.size());

  if (_msg->i_effort_min.size() == this->robotState.i_effort_min.size())
    std::copy(_msg->i_effort_min.begin(), _msg->i_effort_min.end(), this->robotState.i_effort_min.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
      " elements i_effort_min[%ld] than expected[%ld]",
      _msg->i_effort_min.size(), this->robotState.i_effort_min.size());

  if (_msg->i_effort_max.size() == this->robotState.i_effort_max.size())
    std::copy(_msg->i_effort_max.begin(), _msg->i_effort_max.end(), this->robotState.i_effort_max.begin());
  else
    ROS_DEBUG("JointCommand message contains different number of"
      " elements i_effort_max[%ld] than expected[%ld]",
      _msg->i_effort_max.size(), this->robotState.i_effort_max.size());

  this->jointCommand.desired_controller_period_ms =
    _msg->desired_controller_period_ms;
}

void IOBPlugin::UpdateStates() {
  common::Time curTime = this->world->GetSimTime();
  if (curTime > this->lastControllerUpdateTime) {
    // update

    // gather robot state data and publish them
    this->GetAndPublishRobotStates(curTime);

    {
      boost::mutex::scoped_lock lock(this->mutex);
      this->UpdatePIDControl((curTime - this->lastControllerUpdateTime).Double());
    }

    this->lastControllerUpdateTime = curTime;
  }
}

void IOBPlugin::GetAndPublishRobotStates(const common::Time &_curTime){

  // populate robotState from robot
  this->robotState.header.stamp = ros::Time(_curTime.sec, _curTime.nsec);

  //
  for (unsigned int i = 0; i < this->joints.size(); ++i) {
    this->robotState.position[i] = this->joints[i]->GetAngle(0).Radian();
    this->robotState.velocity[i] = this->joints[i]->GetVelocity(0);
    //this->robotState.effort[i]   =
  }

  // force sensors
  //

  // imu sensors
  //

  // publish robot states
  this->pubRobotStateQueue->push(this->robotState, this->pubRobotState);
}

void IOBPlugin::UpdatePIDControl(double _dt) {

  /// update pid with feedforward force
  for (unsigned int i = 0; i < this->joints.size(); ++i) {
    // truncate joint position within range of motion
    double positionTarget = math::clamp(this->jointCommand.position[i],
                                        this->joints[i]->GetLowStop(0).Radian(),
                                        this->joints[i]->GetHighStop(0).Radian());

    double q_p = positionTarget - this->robotState.position[i];

    if (!math::equal(_dt, 0.0))
      this->errorTerms[i].d_q_p_dt = (q_p - this->errorTerms[i].q_p) / _dt;

    this->errorTerms[i].q_p = q_p;

    this->errorTerms[i].qd_p =
      this->jointCommand.velocity[i] - this->robotState.velocity[i];

    this->errorTerms[i].k_i_q_i = math::clamp(this->errorTerms[i].k_i_q_i +
                                              _dt * this->robotState.ki_position[i] * this->errorTerms[i].q_p,
                                              static_cast<double>(this->robotState.i_effort_min[i]),
                                              static_cast<double>(this->robotState.i_effort_max[i]));

    // use gain params to compute force cmd
    double forceUnclamped =
      this->robotState.kp_position[i] * this->errorTerms[i].q_p +
      this->errorTerms[i].k_i_q_i +
      this->robotState.kd_position[i] * this->errorTerms[i].d_q_p_dt +
      this->robotState.kp_velocity[i] * this->errorTerms[i].qd_p +
      this->jointCommand.effort[i];
    //ROS_INFO("0 force:%d -> %f", i, forceUnclamped);

    // keep unclamped force for integral tie-back calculation
    double forceClamped = math::clamp(forceUnclamped, -this->effortLimit[i], this->effortLimit[i]);
    //ROS_INFO("1 force:%d -> %f (%f %f)", i, forceClamped, -this->effortLimit[i], this->effortLimit[i]);

    // integral tie-back during control saturation if using integral gain
    if (!math::equal(forceClamped,forceUnclamped) &&
        !math::equal((double)this->robotState.ki_position[i],0.0) ) {
      // lock integral term to provide continuous control as system moves
      // out of staturation
      this->errorTerms[i].k_i_q_i = math::clamp(this->errorTerms[i].k_i_q_i + (forceClamped - forceUnclamped),
                                                static_cast<double>(this->robotState.i_effort_min[i]),
                                                static_cast<double>(this->robotState.i_effort_max[i]));
    }
    //ROS_INFO("2 force:%d -> %f", i, forceClamped);

    // clamp force after integral tie-back
    forceClamped = math::clamp(forceUnclamped, -this->effortLimit[i], this->effortLimit[i]);


    // apply force to joint
    this->joints[i]->SetForce(0, forceClamped);
    //ROS_INFO("3 force:%d -> %f", i, forceClamped);

    // fill in jointState efforts
    this->robotState.effort[i] = forceClamped;
  }
}

void IOBPlugin::RosQueueThread() {
  static const double timeout = 0.01;

  while (this->rosNode->ok()) {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
}

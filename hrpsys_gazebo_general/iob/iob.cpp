#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "hrpsys/io/iob.h"

#include <ros/ros.h>
#include <boost/algorithm/string.hpp>

#include <hrpsys_gazebo_msgs/JointCommand.h>
#include <hrpsys_gazebo_msgs/RobotState.h>

#include <hrpUtil/Eigen3d.h>

typedef hrpsys_gazebo_msgs::JointCommand JointCommand;
typedef hrpsys_gazebo_msgs::RobotState RobotState;

static ros::NodeHandle* rosnode;
static ros::Publisher pub_joint_command;
static ros::Subscriber sub_robot_state;
static JointCommand jointcommand;
static JointCommand initial_jointcommand;

static RobotState js;
static int init_sub_flag = FALSE;

static std::vector<double> command;
static std::vector<double> prev_command;
static std::vector<std::vector<double> > forces;
static std::vector<std::vector<double> > gyros;
static std::vector<std::vector<double> > accelerometers;
static std::vector<std::vector<double> > attitude_sensors;
static std::vector<std::vector<double> > force_offset;
static std::vector<std::vector<double> > gyro_offset;
static std::vector<std::vector<double> > accel_offset;
static std::vector<int> power;
static std::vector<int> servo;
static bool isLocked = false;
static int frame = 0;
static timespec g_ts;
static long g_period_ns=1000000;
static ros::Time rg_ts;

#define CHECK_JOINT_ID(id) if ((id) < 0 || (id) >= number_of_joints()) return E_ID
#define CHECK_FORCE_SENSOR_ID(id) if ((id) < 0 || (id) >= number_of_force_sensors()) return E_ID
#define CHECK_ACCELEROMETER_ID(id) if ((id) < 0 || (id) >= number_of_accelerometers()) return E_ID
#define CHECK_GYRO_SENSOR_ID(id) if ((id) < 0 || (id) >= number_of_gyro_sensors()) return E_ID
#define CHECK_ATTITUDE_SENSOR_ID(id) if ((id) < 0 || (id) >= number_of_attitude_sensors()) return E_ID

#define JOINT_ID_REAL2MODEL(id) joint_real2model_vec[id]
#define JOINT_ID_MODEL2REAL(id) joint_id_model2real(id)
#define NUM_OF_REAL_JOINT (joint_real2model_vec.size())

static std::map<int, int> joint_model2real_map;
static std::vector<int>   joint_real2model_vec;

static inline int joint_id_model2real(int id)
{
  std::map< int, int>::iterator it = joint_model2real_map.find (id);

  if (it == joint_model2real_map.end()) {
    return -1;
  } else {
    return it->second;
  }
}

#ifdef __APPLE__
typedef int clockid_t;
#define CLOCK_MONOTONIC 0
#include <mach/mach_time.h>
int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    if (clk_id != CLOCK_MONOTONIC) return -1;

    uint64_t clk;
    clk = mach_absolute_time();

    static mach_timebase_info_data_t info = {0,0};
    if (info.denom == 0) mach_timebase_info(&info);

    uint64_t elapsednano = clk * (info.numer / info.denom);

    tp->tv_sec = elapsednano * 1e-9;
    tp->tv_nsec = elapsednano - (tp->tv_sec * 1e9);
    return 0;
}

#define TIMER_ABSTIME 0
int clock_nanosleep(clockid_t clk_id, int flags, struct timespec *tp,
    struct timespec *remain)
{
    if (clk_id != CLOCK_MONOTONIC || flags != TIMER_ABSTIME) return -1;

    static mach_timebase_info_data_t info = {0,0};
    if (info.denom == 0) mach_timebase_info(&info);

    uint64_t clk = (tp->tv_sec*1e9 + tp->tv_nsec)/(info.numer/info.denom);

    mach_wait_until(clk);
    return 0;
}
#endif


int number_of_joints()
{
    return (int)command.size();
}

int number_of_force_sensors()
{
    return (int)forces.size();
}

int number_of_gyro_sensors()
{
    return (int)gyros.size();
}

int number_of_accelerometers()
{
    return (int)accelerometers.size();
}

int number_of_attitude_sensors()
{
    return (int)attitude_sensors.size();
}

int set_number_of_joints(int num)
{
    std::cerr << ";; IOB / set num of joint = " << num << std::endl;
    command.resize(num);
    prev_command.resize(num);
    power.resize(num);
    servo.resize(num);
    for (int i=0; i<num; i++){
        command[i] = power[i] = servo[i] = prev_command[i] = 0;
    }
    return TRUE;
}

int set_number_of_force_sensors(int num)
{
    forces.resize(num);
    force_offset.resize(num);
    for (unsigned int i=0; i<forces.size();i++){
        forces[i].resize(6);
        force_offset[i].resize(6);
        for (int j=0; j<6; j++){
            forces[i][j] = 0;
            force_offset[i][j] = 0;
        }
    }
    return TRUE;
}

int set_number_of_gyro_sensors(int num)
{
    gyros.resize(num);
    gyro_offset.resize(num);
    for (unsigned int i=0; i<gyros.size();i++){
        gyros[i].resize(3);
        gyro_offset[i].resize(3);
        for (int j=0; j<3; j++){
            gyros[i][j] = 0.0;
            gyro_offset[i][j] = 0.0;
        }
    }
    return TRUE;
}

int set_number_of_accelerometers(int num)
{
    accelerometers.resize(num);
    accel_offset.resize(num);
    for (unsigned int i=0; i<accelerometers.size();i++){
        accelerometers[i].resize(3);
        accel_offset[i].resize(3);
        for (int j=0; j<3; j++){
            accelerometers[i][j] = j == 2 ? 9.81 : 0.0;
            accel_offset[i][j] = 0;
        }
    }
    return TRUE;
}

int set_number_of_attitude_sensors(int num)
{
    attitude_sensors.resize(num);
    for (unsigned int i=0; i<attitude_sensors.size();i++){
        attitude_sensors[i].resize(3);
        for (int j=0; j<3; j++){
            attitude_sensors[i][j] = 0.0;
        }
    }
    return TRUE;
}

int read_power_state(int id, int *s)
{
    CHECK_JOINT_ID(id);
    *s = power[id];
    return TRUE;
}

int write_power_command(int id, int com)
{
    CHECK_JOINT_ID(id);
    power[id] = com;
    return TRUE;
}

int read_power_command(int id, int *com)
{
    CHECK_JOINT_ID(id);
    *com = power[id];
    return TRUE;
}

int read_servo_state(int id, int *s)
{
    CHECK_JOINT_ID(id);
    *s = servo[id];
    return TRUE;
}

int read_servo_alarm(int id, int *a)
{
    CHECK_JOINT_ID(id);
    *a = 0;
    return TRUE;
}

int read_control_mode(int id, joint_control_mode *s)
{
    CHECK_JOINT_ID(id);
    *s = JCM_POSITION;
    return TRUE;
}

int write_control_mode(int id, joint_control_mode s)
{
    CHECK_JOINT_ID(id);
    return TRUE;
}

int read_actual_angle(int id, double *angle)
{
  CHECK_JOINT_ID(id);

  if(init_sub_flag){
    if (JOINT_ID_MODEL2REAL(id) < 0) {
      *angle = command[id];
    }else{
      *angle = js.position[JOINT_ID_MODEL2REAL(id)];
    }
  }else{
    *angle = command[id];
  }
  return TRUE;
}

int read_actual_angles(double *angles)
{
    for (int i=0; i<number_of_joints(); i++){
        read_actual_angle(i, &angles[i]);
    }
    return TRUE;
}

int read_actual_torques(double *torques)
{
  if(init_sub_flag) {
    for(int i=0; i<number_of_joints(); i++){
      if(JOINT_ID_MODEL2REAL(i) < 0) {
        *(torques+i) = -1;
      }else{
        *(torques+i) = js.effort[JOINT_ID_MODEL2REAL(i)];
      }
    }
  }

  return TRUE;
}

int read_command_torque(int id, double *torque)
{
    return FALSE;
}

int write_command_torque(int id, double torque)
{
    return FALSE;
}

int read_command_torques(double *torques)
{
    return FALSE;
}

int read_command_angle(int id, double *angle)
{
    CHECK_JOINT_ID(id);
    *angle = command[id];
    return TRUE;
}

int write_command_angle(int id, double angle)
{
    CHECK_JOINT_ID(id);
    command[id] = angle;
    return TRUE;
}

int read_command_angles(double *angles)
{
    for (int i=0; i<number_of_joints(); i++){
        angles[i] = command[i];
    }
    return TRUE;
}

int write_command_angles(const double *angles)
{
    for (int i=0; i<number_of_joints(); i++){
      prev_command[i] = command[i];
      command[i] = angles[i];
    }

    jointcommand.header.stamp = ros::Time::now();

    for (int i=0; i<NUM_OF_REAL_JOINT; i++) {
      jointcommand.position[i] = command[JOINT_ID_REAL2MODEL(i)];
      jointcommand.velocity[i] = (command[JOINT_ID_REAL2MODEL(i)] - prev_command[JOINT_ID_REAL2MODEL(i)]) / (g_period_ns * 1e-9);
      // jointcommand.kp_velocity[i] = 100;
    }

    pub_joint_command.publish(jointcommand);

    ros::spinOnce();

    return TRUE;
}

int read_pgain(int id, double *gain)
{
  if(JOINT_ID_MODEL2REAL(id) < 0) {
    std::cerr << ";;; read pgain: " << id << " failed." << std::endl;
  }else{
    int iid = JOINT_ID_MODEL2REAL(id);
    *(gain) = jointcommand.kp_position[iid] / initial_jointcommand.kp_position[iid];
    //std::cerr << ";;; read gain: " << id << " = " << *gain << std::endl;
  }
  return TRUE;
}

int write_pgain(int id, double gain)
{

  if(JOINT_ID_MODEL2REAL(id) < 0) {
    std::cerr << ";;; write pgain: " << id << " failed." << std::endl;
  }else{
    int iid = JOINT_ID_MODEL2REAL(id);
    jointcommand.kp_position[iid] =
      gain * initial_jointcommand.kp_position[iid];
    //std::cerr << ";;; write pgain: " << id << " = " << gain << std::endl;
  }
  return TRUE;
}

int read_dgain(int id, double *gain)
{
  if(JOINT_ID_MODEL2REAL(id) < 0) {
    //
  }else{
    int iid = JOINT_ID_MODEL2REAL(id);
    *(gain) = jointcommand.kd_position[iid] / initial_jointcommand.kd_position[iid];
    //std::cerr << ";;; read dgain: " << id << " = " << *gain << std::endl;
  }
  return TRUE;
}

int write_dgain(int id, double gain)
{
  if(JOINT_ID_MODEL2REAL(id) < 0) {
    //
  }else{
    int iid = JOINT_ID_MODEL2REAL(id);
    jointcommand.kd_position[iid] =
      gain * initial_jointcommand.kd_position[iid];
    //std::cerr << ";;; write dgain: " << id << " = " << gain << std::endl;
  }
  return TRUE;
}

int read_force_sensor(int id, double *forces)
{
  CHECK_FORCE_SENSOR_ID(id);
  // for (int i=0; i<6; i++){
  //     forces[i] = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*2
  //         + 2 + force_offset[id][i]; // 2 = initial offset
  // }
#if 0
  std::vector<geometry_msgs::Wrench*> fsensors;
  fsensors.push_back(&(js.l_hand));
  fsensors.push_back(&(js.r_hand));
  fsensors.push_back(&(js.l_foot));
  fsensors.push_back(&(js.r_foot));
  if(init_sub_flag){
    forces[0] = fsensors[id]->force.x + force_offset[id][0];
    forces[1] = fsensors[id]->force.y + force_offset[id][1];
    forces[2] = fsensors[id]->force.z + force_offset[id][2];
    forces[3] = fsensors[id]->torque.x + force_offset[id][3];
    forces[4] = fsensors[id]->torque.y + force_offset[id][4];
    forces[5] = fsensors[id]->torque.z + force_offset[id][5];
  }
#endif
  return TRUE;
}

int read_gyro_sensor(int id, double *rates)
{
  CHECK_GYRO_SENSOR_ID(id);
  // for (int i=0; i<3; i++){
  //     rates[i] = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*0.01
  //         + 0.01 + gyro_offset[id][i]; // 0.01 = initial offset
  // }
#if 0
  if(init_sub_flag){
    // use atlas orientation
    Eigen::Quaternion<double> q(js.orientation.w,
                                js.orientation.x,
                                js.orientation.y,
                                js.orientation.z);
    hrp::Vector3 rpy = hrp::rpyFromRot(q.toRotationMatrix());
    rates[0] = rpy[0];
    rates[1] = rpy[1];
    rates[2] = rpy[2];

    // rates[0] = js.angular_velocity.x
    //   + 0.01 + gyro_offset[id][0]; // 0.01 = initial offset
    // rates[1] = js.angular_velocity.y
    //   + 0.01 + gyro_offset[id][1]; // 0.01 = initial offset
    // rates[2] = js.angular_velocity.z
    //   + 0.01 + gyro_offset[id][2]; // 0.01 = initial offset
  } else {
    // tempolary values when sensor is not ready.
    rates[0] = rates[1] = rates[2] = 0.0;
  }
#endif
  return TRUE;
}

int read_accelerometer(int id, double *accels)
{
  CHECK_ACCELEROMETER_ID(id);
#if 0
  // for (int i=0; i<3; i++){
  //     double randv = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*0.01;
  //     accels[i] = (i == 2 ? (9.8+randv) : randv)
  //         + 0.01 + accel_offset[id][i]; // 0.01 = initial offset
  // }
  if(init_sub_flag){
    accels[0] = js.linear_acceleration.x
      + 0.01 + accel_offset[id][0]; // 0.01 = initial offset
    accels[1] = js.linear_acceleration.y
      + 0.01 + accel_offset[id][1]; // 0.01 = initial offset
    accels[2] = js.linear_acceleration.z
      + 0.01 + accel_offset[id][2]; // 0.01 = initial offset
  } else {
    // tempolary values when sensor is not ready.
    accels[0] = accels[1] = 0.0;
    accels[2] = 9.8;
  }
#endif
  return TRUE;
}

int read_touch_sensors(unsigned short *onoff)
{
    return FALSE;
}

int read_attitude_sensor(int id, double *att)
{
  return FALSE;
}

int read_current(int id, double *mcurrent)
{
    return FALSE;
}

int read_current_limit(int id, double *v)
{
    return FALSE;
}

int read_currents(double *currents)
{
    return FALSE;
}

int read_gauges(double *gauges)
{
    return FALSE;
}

int read_command_velocity(int id, double *vel)
{
    return FALSE;
}

int write_command_velocity(int id, double vel)
{
    return FALSE;
}

int read_command_velocities(double *vels)
{
    return FALSE;
}

int write_command_velocities(double *vels)
{
    return FALSE;
}

int read_temperature(int id, double *v)
{
    return FALSE;
}

int write_servo(int id, int com)
{
    servo[id] = com;
    return TRUE;
}

int write_dio(unsigned short buf)
{
    return FALSE;
}

// callback
static void setJointStates(const RobotState::ConstPtr &_js) {
  ROS_DEBUG(";; subscribe JointState");
  js = *_js;
  init_sub_flag = TRUE;
}

int open_iob(void)
{
    static bool isInitialized = false;
    if ( isInitialized ) return TRUE;

    std::cerr << ";; Open IOB / start " << std::endl;

    std::map<std::string, std::string> arg;
    ros::init(arg, "hrpsys_gazebo_iob", ros::init_options::NoSigintHandler);

    rosnode = new ros::NodeHandle();

    joint_real2model_vec.resize(0);
    std::string controller_name;
    if (rosnode->hasParam("hrpsys_gazebo_iob")) {
      if (!rosnode->getParam("hrpsys_gazebo_iob", controller_name)) {
        controller_name = "hrpsys_gazebo_configuration";
      }
    }

    if (rosnode->hasParam(controller_name + "/joint_id_list")) {
      XmlRpc::XmlRpcValue param_val;
      rosnode->getParam(controller_name + "/joint_id_list", param_val);
      if (param_val.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for(int i = 0; i < param_val.size(); i++) {
          int num = param_val[i];
          joint_real2model_vec.push_back(num);
        }
      } else {
        ROS_WARN("%s/joint_id_list is not list of integer", controller_name.c_str());
      }
    } else {
      ROS_DEBUG("%s/joint_id_list is nothing", controller_name.c_str());
    }

    XmlRpc::XmlRpcValue param_val;
    std::vector<std::string> joint_lst;
    rosnode->getParam(controller_name + "/joints", param_val);
    if (param_val.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for(unsigned int s = 0; s < param_val.size(); s++) {
          std::string nstr = param_val[s];
          ROS_DEBUG("add joint: %s", nstr.c_str());
          joint_lst.push_back(nstr);
        }
    } else {
      ROS_ERROR("%s/joints is not list of joint name", controller_name.c_str());
    }

    if (joint_real2model_vec.size() == 0) {
      for(unsigned int i = 0; i < joint_lst.size(); i++) {
        joint_real2model_vec.push_back(i);
      }
    } else if (joint_real2model_vec.size() != joint_lst.size()) {
      ROS_ERROR("size differece on joint_id_list and joints (%d,  %d)",
                joint_real2model_vec.size(), joint_lst.size());
    }

    for(unsigned int i = 0; i < joint_real2model_vec.size(); i++) {
      joint_model2real_map[joint_real2model_vec[i]] = i;
    }

    unsigned int n = NUM_OF_REAL_JOINT;

    initial_jointcommand.position.resize(n);
    initial_jointcommand.velocity.resize(n);
    //initial_jointcommand.effort.resize(n);
    initial_jointcommand.effort.resize(0);
    initial_jointcommand.kp_position.resize(n);
    initial_jointcommand.ki_position.resize(n);
    initial_jointcommand.kd_position.resize(n);
    //initial_jointcommand.kp_velocity.resize(n);
    initial_jointcommand.kp_velocity.resize(0);
    initial_jointcommand.i_effort_min.resize(n);
    initial_jointcommand.i_effort_max.resize(n);

    for (unsigned int i = 0; i < joint_lst.size(); ++i) {
      std::string joint_ns(controller_name);
      joint_ns += ("/gains/" + joint_lst[i] + "/");

      double p_val = 0, i_val = 0, d_val = 0, i_clamp_val = 0;
      std::string p_str = std::string(joint_ns)+"p";
      std::string i_str = std::string(joint_ns)+"i";
      std::string d_str = std::string(joint_ns)+"d";
      std::string i_clamp_str = std::string(joint_ns)+"i_clamp";
      if (!rosnode->getParam(p_str, p_val) ||
          !rosnode->getParam(i_str, i_val) ||
          !rosnode->getParam(d_str, d_val) ||
          !rosnode->getParam(i_clamp_str, i_clamp_val)) {
          ROS_ERROR("couldn't find a param for %s", joint_ns.c_str());
          continue;
      }
      // store these directly on altasState, more efficient for pub later
      initial_jointcommand.kp_position[i] = p_val;
      initial_jointcommand.ki_position[i] = i_val;
      initial_jointcommand.kd_position[i] = d_val;
      initial_jointcommand.i_effort_min[i] = -i_clamp_val;
      initial_jointcommand.i_effort_max[i] = i_clamp_val;
      initial_jointcommand.velocity[i]     = 0;
      //initial_jointcommand.effort[i]       = 0;
      //initial_jointcommand.kp_velocity[i]  = 0;
    }

    std::string robotname;
    std::string rname_str = std::string(controller_name) + "/robotname";
    rosnode->getParam(rname_str, robotname);

    initial_jointcommand.desired_controller_period_ms = static_cast<unsigned int>(g_period_ns * 1e-6);

    pub_joint_command = rosnode->advertise <JointCommand> (robotname + "/joint_command", 1, true);

    // ros topic subscribtions
    ros::SubscribeOptions jointStatesSo =
      ros::SubscribeOptions::create<RobotState>(robotname + "/robot_state", 1, setJointStates,
                                                ros::VoidPtr(), rosnode->getCallbackQueue());
    // Because TCP causes bursty communication with high jitter,
    // declare a preference on UDP connections for receiving
    // joint states, which we want to get at a high rate.
    // Note that we'll still accept TCP connections for this topic
    // (e.g., from rospy nodes, which don't support UDP);
    // we just prefer UDP.
    jointStatesSo.transport_hints =
      ros::TransportHints().unreliable().reliable().tcpNoDelay(true);
    sub_robot_state = rosnode->subscribe(jointStatesSo);

    std::cerr << "JointState IOB is opened" << std::endl;
    for (int i=0; i < number_of_joints(); i++){
        command[i] = 0.0;
        power[i] = OFF;
        servo[i] = OFF;
    }
    clock_gettime(CLOCK_MONOTONIC, &g_ts);
    rg_ts = ros::Time::now();

    jointcommand = initial_jointcommand;
    isInitialized = true;
    std::cerr << ";; " << number_of_joints() << " / " << initial_jointcommand.position.size() << " / " << NUM_OF_REAL_JOINT << std::endl;
    std::cerr << ";; Open IOB / finish " << std::endl;
    return TRUE;
}

int close_iob(void)
{
    std::cerr << ";; IOB is closed" << std::endl;
    return TRUE;
}

int reset_body(void)
{
    for (int i=0; i<number_of_joints(); i++){
        power[i] = servo[i] = OFF;
    }
    return TRUE;
}

int joint_calibration(int id, double angle)
{
    return FALSE;
}

int read_gyro_sensor_offset(int id, double *offset)
{
    for (int i=0; i<3; i++){
        offset[i] = gyro_offset[id][i];
    }
    return TRUE;
}

int write_gyro_sensor_offset(int id, double *offset)
{
    for (int i=0; i<3; i++){
        gyro_offset[id][i] = offset[i];
    }
    return TRUE;
}

int read_accelerometer_offset(int id, double *offset)
{
    for (int i=0; i<3; i++){
        offset[i] = accel_offset[id][i];
    }
    return TRUE;
}

int write_accelerometer_offset(int id, double *offset)
{
    for (int i=0; i<3; i++){
        accel_offset[id][i] = offset[i];
    }
    return TRUE;
}

int read_force_offset(int id, double *offsets)
{
    for (int i=0; i<6; i++){
        offsets[i] = force_offset[id][i];
    }
    return TRUE;
}

int write_force_offset(int id, double *offsets)
{
    for (int i=0; i<6; i++){
        force_offset[id][i] = offsets[i];
    }
    return TRUE;
}

int write_attitude_sensor_offset(int id, double *offset)
{
    return FALSE;
}

int read_calib_state(int id, int *s)
{
    CHECK_JOINT_ID(id);
    int v = id/2;
    *s = v%2==0 ? ON : OFF;
    return TRUE;
}

int lock_iob()
{
    if (isLocked) return FALSE;

    //isLocked = true;
    return TRUE;
}
int unlock_iob()
{
    isLocked = false;
    return TRUE;
}

int read_lock_owner(pid_t *pid)
{
  return FALSE;
}

int read_limit_angle(int id, double *angle)
{
  return FALSE;
}

int read_angle_offset(int id, double *angle)
{
  return FALSE;
}

int write_angle_offset(int id, double angle)
{
  return FALSE;
}

int read_ulimit_angle(int id, double *angle)
{
  return FALSE;
}
int read_llimit_angle(int id, double *angle)
{
  return FALSE;
}
int read_encoder_pulse(int id, double *ec)
{
  return FALSE;
}
int read_gear_ratio(int id, double *gr)
{
  return FALSE;
}
int read_torque_const(int id, double *tc)
{
  return FALSE;
}
int read_torque_limit(int id, double *limit)
{
  return FALSE;
}

unsigned long long read_iob_frame()
{
    ++frame;
    return frame;
}

int number_of_substeps()
{
    return 1;
}

int read_power(double *voltage, double *current)
{
    *voltage = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*1+48;
    *current = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*0.5+1;
    return TRUE;
}

int read_driver_temperature(int id, unsigned char *v)
{
    *v = id * 2;
    return TRUE;
}

void timespec_add_ns(timespec *ts, long ns)
{
    ts->tv_nsec += ns;
    while (ts->tv_nsec > 1e9){
        ts->tv_sec += 1;
        ts->tv_nsec -= 1e9;
    }
}

double timespec_compare(timespec *ts1, timespec *ts2)
{
    double dts = ts1->tv_sec - ts2->tv_sec;
    double dtn = ts1->tv_nsec - ts2->tv_nsec;
    return dts*1e9+dtn;
}

int wait_for_iob_signal()
{
    ros::Time rnow;
    ros::Duration tm = ros::Duration(0, g_period_ns);
    ros::WallDuration wtm = ros::WallDuration(0, 100000); // 0.1 ms
    while ((rnow = ros::Time::now()) < rg_ts) {
      wtm.sleep();
    }

    rg_ts += tm;
    if ((rg_ts - rnow).toSec() <= 0) {
      fprintf(stderr, "iob::overrun (%f[ms]), w:%f -> %f\n",
              (rnow - rg_ts).toSec()*1000, rnow.toSec(), rg_ts.toSec());
      do {
        rg_ts += tm;
      } while ((rg_ts - rnow).toSec() <= 0);
    }

    return 0;
}

size_t length_of_extra_servo_state(int id)
{
    return 0;
}

int read_extra_servo_state(int id, int *state)
{
    return TRUE;
}

int set_signal_period(long period_ns)
{
    g_period_ns = period_ns;
    return TRUE;
}

long get_signal_period()
{
    return g_period_ns;
}

int initializeJointAngle(const char *name, const char *option)
{
    sleep(3);
    return TRUE;
}

int read_digital_input(char *dinput)
{
  return 0;
}

int write_digital_output(const char *doutput)
{
  return 0;
}

int length_digital_input(void)
{
  return 0;
}

int length_digital_output(void)
{
  return 0;
}

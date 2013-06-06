#include <iostream>

#include <ros/ros.h>
#include <pr2_controller_manager/controller_manager.h>

#include <osrf_msgs/JointCommands.h>
#include <sensor_msgs/JointState.h>
#include <roseus/StringString.h>

class HandController {
public:
  HandController() : nh_(), auto_start_(false) {
    hw_ = new pr2_hardware_interface::HardwareInterface();
    hw_->current_time_ = ros::Time::now();

    ros::NodeHandle pnh("~");
    pnh.getParam("auto_start", auto_start_);

    TiXmlElement *root;
    TiXmlElement *root_element;
    // Load robot description
    TiXmlDocument xml_doc;

    std::string robot_desc;
    if (pnh.getParam("robot_description", robot_desc)) {
      xml_doc.Parse(robot_desc.c_str());
    }  else {
      ROS_FATAL("Could not load the xml from parameter server");
      throw "end";
    }

    root_element = xml_doc.RootElement();
    root = xml_doc.FirstChildElement("robot");

    if (!root || !root_element) {
      ROS_FATAL("Could not parse the xml / %s", robot_desc.c_str());
      throw "end";
    }

    for (TiXmlElement *j = root_element->FirstChildElement("transmission"); j;
         j = j->NextSiblingElement("transmission")) {
      TiXmlElement *ael = j->FirstChildElement("actuator");
      const char *anm = ael ? ael->Attribute("name") : NULL;
#if 0
      const char *tnm = j ? j->Attribute("name") : NULL;
      TiXmlElement *jel = j->FirstChildElement("joint");
      const char *jnm = jel ? jel->Attribute("name") : NULL;
      if(!!tnm) ROS_INFO("tnm: %s", tnm);
      if(!!jnm) ROS_INFO("jnm: %s", jnm);
      if(!!anm) ROS_INFO("anm: %s", anm);
#endif
      if (!!anm) {
        hw_->addActuator(new pr2_hardware_interface::Actuator(anm));
      }
    }

    // Create controller manager
    cm_ = boost::shared_ptr<pr2_controller_manager::ControllerManager>
      (new pr2_controller_manager::ControllerManager(hw_, nh_));

    // Initialize the controller manager from robot description
    if (!cm_->initXml(root)) {
      ROS_FATAL("Could not initialize the controller manager");
      throw "end";
    } else {
      ROS_INFO("success to initialize the controller manager");
    }

    for (std::vector<pr2_mechanism_model::Transmission*>::iterator it = cm_->model_.transmissions_.begin();
         it != cm_->model_.transmissions_.end();
         ++it) { // *** js and ac must be consistent
      ROS_INFO_STREAM("JointState: " << (*it)->joint_names_[0] << " / Actuator: " << (*it)->actuator_names_[0]);
      pr2_mechanism_model::JointState *js = cm_->state_->getJointState((*it)->joint_names_[0]);
      pr2_hardware_interface::Actuator *ac = hw_->getActuator((*it)->actuator_names_[0]);
      if(!!ac) { ac->state_.is_enabled_ = true; }
      if(!!js) { js->calibrated_ = true; }
      if (!!ac && !!js) {
        jointactuator ja;
        ja.js = (void *)js;
        ja.ac = (void *)ac;
        jointactuator_map_.insert
          (std::map< std::string, jointactuator >::value_type ((*it)->joint_names_[0], ja));
      }
    }

    pub_ = nh_.advertise< osrf_msgs::JointCommands > ("commands", 1);
    srv_ = pnh.advertiseService ("query", &HandController::service_cb, this);
    if (auto_start_) {
      startSubscribe();
    }

    spinner_thread_ =
      boost::thread (boost::bind (&HandController::ManagerThread, this));
  }

  ~HandController() {

  }

  void startSubscribe() {
    sub_ = nh_.subscribe("in_joint_states", 100, &HandController::jointCallback, this);
  }

  bool service_cb (roseus::StringString::Request &req,
                   roseus::StringString::Response &res) {
    if (req.str == "stop") {
      if (sub_) {
        sub_.shutdown();
        return true;
      }
    } else if (req.str == "start") {
      if (!sub_) {
        startSubscribe();
        hw_->current_time_ = ros::Time::now();
        cm_->update();
        return true;
      }
    } else if (req.str == "query") {
      // return start or stop
    }
    res.str = "unknown_command";
    return false;
  }

  void jointCallback(const sensor_msgs::JointStateConstPtr& simjs) {
    ROS_DEBUG("callback");
    hw_->current_time_ = simjs->header.stamp;

    for(size_t i = 0; i < simjs->name.size(); i++) {
      jointactuator ja = jointactuator_map_[simjs->name[i]];
      //pr2_mechanism_model::JointState *js = (pr2_mechanism_model::JointState *)ja.js;
      pr2_hardware_interface::Actuator *ac = (pr2_hardware_interface::Actuator *)ja.ac;

      ac->state_.position_ = simjs->position[i];
      ac->state_.velocity_ = simjs->velocity[i];
    }
#if 0
    pr2_mechanism_model::JointState *js = cm_->state_->getJointState("left_f0_j0");
    pr2_hardware_interface::Actuator *ac = hw_->getActuator("left_f0_j0_motor");
    ROS_INFO("%f A: %s, js pos: %f, vel %f, mes: %f, com %f / ac pos: %f, vel: %f, com: %f",
             simjs->header.stamp.toSec(),
             simjs->name[0].c_str(),
             js->position_,
             js->velocity_,
             js->measured_effort_,
             js->commanded_effort_,
             ac->state_.position_,
             ac->state_.velocity_,
             ac->command_.effort_);
#endif
    //
    cm_->update();

#if 0
    ROS_INFO("%f B: %s, js pos: %f, vel %f, mes: %f, com %f / ac pos: %f, vel: %f, com: %f",
             simjs->header.stamp.toSec(),
             simjs->name[0].c_str(),
             js->position_,
             js->velocity_,
             js->measured_effort_,
             js->commanded_effort_,
             ac->state_.position_,
             ac->state_.velocity_,
             ac->command_.effort_);
#endif

    osrf_msgs::JointCommands joint_com;
    joint_com.name = simjs->name; //??
    joint_com.position.resize(joint_com.name.size());
#if 0
    joint_com.velocity.resize(joint_com.name.size()); // 0
    joint_com.effort.resize(joint_com.name.size());   // 0
    joint_com.kp_position.resize(joint_com.name.size());
    joint_com.kd_position.resize(joint_com.name.size());
    joint_com.ki_position.resize(joint_com.name.size()); // 0
    joint_com.kp_velocity.resize(joint_com.name.size()); // 0
    joint_com.i_effort_min.resize(joint_com.name.size());// 0
    joint_com.i_effort_max.resize(joint_com.name.size());// 0
#else
    joint_com.velocity.resize(0);
    joint_com.effort.resize(0);
    joint_com.kp_position.resize(0);
    joint_com.kd_position.resize(0);
    joint_com.ki_position.resize(0);
    joint_com.kp_velocity.resize(0);
    joint_com.i_effort_min.resize(0);
    joint_com.i_effort_max.resize(0);
#endif
    for(size_t i = 0; i < joint_com.name.size(); i++) {
      jointactuator ja = jointactuator_map_[joint_com.name[i]];
      pr2_mechanism_model::JointState *js = (pr2_mechanism_model::JointState *)ja.js;
      pr2_hardware_interface::Actuator *ac = (pr2_hardware_interface::Actuator *)ja.ac;

      joint_com.position[i] = js->position_ + ac->command_.effort_; // joint Desired
#if 0
      joint_com.velocity[i] = 0.0;   // 0
      joint_com.effort[i]   = 0.0;   // 0
      joint_com.kp_position[i] = 21.0;  // joint P gain
      joint_com.kd_position[i] = 0.004; // joint D gain
      joint_com.ki_position[i] = 0.0; // 0
      joint_com.kp_velocity[i] = 0.0; // 0
      joint_com.i_effort_min[i] = 0.0;// 0
      joint_com.i_effort_max[i] = 0.0;// 0
#endif
    }
    pub_.publish(joint_com);
  }

  void ManagerThread() {
    ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());
    while (ros::ok()) {
      usleep(1000);
      ros::spinOnce();
    }
  }

protected:
  typedef struct jointactuator {
    void *js;
    void *ac;
  } jointactuator;

  ros::NodeHandle nh_;
  boost::thread spinner_thread_;
  pr2_hardware_interface::HardwareInterface* hw_;
  boost::shared_ptr<pr2_controller_manager::ControllerManager> cm_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::ServiceServer srv_;

  // osrf_msgs::JointCommands joint_com_;
  std::map <std::string, jointactuator> jointactuator_map_; // name -> js,ac pointer

  bool auto_start_;
private:
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sandia_hand_controller");
  HandController hcontroller;

  ros::spin();

  return 0;
}

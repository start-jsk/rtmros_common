
#include "MasterSlaveROSBridge.h"

// Module specification
// <rtc-template block="module_spec">
static const char* imagesensorrosbridge_spec[] =
  {
    "implementation_id", "MasterSlaveROSBridge",
    "type_name",         "MasterSlaveROSBridge",
    "description",       "rtm range data - ros bridge",
    "version",           "1.0",
    "vendor",            "JSK",
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

MasterSlaveROSBridge::MasterSlaveROSBridge(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_exDataOut("exData", m_exData),
    m_exDataIndexOut("exDataIndex", m_exDataIndex)
{}

MasterSlaveROSBridge::~MasterSlaveROSBridge(){}

RTC::ReturnCode_t MasterSlaveROSBridge::onInitialize(){
    addOutPort("exData", m_exDataOut);
    addOutPort("exDataIndex", m_exDataIndexOut);

    ee_names.push_back("lleg");
    ee_names.push_back("rleg");
    ee_names.push_back("larm");
    ee_names.push_back("rarm");

    tgt_names = ee_names;
    tgt_names.push_back("com");
    tgt_names.push_back("head");

    for ( int i=0; i<ee_names.size(); i++) {
        std::string n = "slave_"+ee_names[i]+"_wrench_in";
        m_slaveEEWrenchesIn[ee_names[i]] = ITDS_Ptr(new RTC::InPort<RTC::TimedDoubleSeq>(n.c_str(), m_slaveEEWrenches[ee_names[i]]));
        registerInPort(n.c_str(), *m_slaveEEWrenchesIn[ee_names[i]]);
        RTCOUT("register RTC InPort " << n );
    }
    for ( int i=0; i<ee_names.size(); i++) {
        std::string n = "slave_"+ee_names[i]+"_wrench_out";
        m_slaveEEWrenchesOut[ee_names[i]] = OTDS_Ptr(new RTC::OutPort<RTC::TimedDoubleSeq>(n.c_str(), m_slaveEEWrenches[ee_names[i]]));
        registerOutPort(n.c_str(), *m_slaveEEWrenchesOut[ee_names[i]]);
        RTCOUT("register RTC OutPort " << n );
    }

    for ( int i=0; i<tgt_names.size(); i++) {
        std::string n = "master_"+tgt_names[i]+"_pose_in";
        m_masterTgtPosesIn[tgt_names[i]] = ITP3_Ptr(new RTC::InPort<RTC::TimedPose3D>(n.c_str(), m_masterTgtPoses[tgt_names[i]]));
        registerInPort(n.c_str(), *m_masterTgtPosesIn[tgt_names[i]]);
        RTCOUT("register RTC InPort " << n );
    }
    for ( int i=0; i<tgt_names.size(); i++) {
        std::string n = "master_"+tgt_names[i]+"_pose_out";
        m_masterTgtPosesOut[tgt_names[i]] = OTP3_Ptr(new RTC::OutPort<RTC::TimedPose3D>(n.c_str(), m_masterTgtPoses[tgt_names[i]]));
        registerOutPort(n.c_str(), *m_masterTgtPosesOut[tgt_names[i]]);
        RTCOUT("register RTC OutPort " << n );
    }


    ros::param::param<std::string>("~mode", mode, "MASTER");

    for ( int i=0; i<ee_names.size(); i++) {
        std::string n = "slave_"+ee_names[i]+"_wrench_in";
        slaveEEWrenches_topic2key[n] = ee_names[i];
        slaveEEWrenches_sub[ee_names[i]] = nh.subscribe(n, 1, &MasterSlaveROSBridge::onSlaveEEWrenchCB, this);
        RTCOUT("register ROS Subscriber " << n );
    }
    for ( int i=0; i<ee_names.size(); i++) {
        std::string n = "slave_"+ee_names[i]+"_wrench_out";
        slaveEEWrenches_pub[ee_names[i]] = nh.advertise<geometry_msgs::WrenchStamped>(n, 1);
        RTCOUT("register ROS Publisher " << n );
    }

    for ( int i=0; i<tgt_names.size(); i++) {
        std::string n = "master_"+tgt_names[i]+"_pose_in";
        masterTgtPoses_topic2key[n] = tgt_names[i];
        masterTgtPoses_sub[tgt_names[i]] = nh.subscribe(n, 1, &MasterSlaveROSBridge::onMasterTgtPoseCB, this);
        RTCOUT("register ROS Subscriber " << n );
    }
    for ( int i=0; i<tgt_names.size(); i++) {
        std::string n = "master_"+tgt_names[i]+"_pose_out";
        masterTgtPoses_pub[tgt_names[i]] = nh.advertise<geometry_msgs::PoseStamped>(n, 1);
        RTCOUT("register ROS Publisher " << n );
    }

    // initialize
    ROS_INFO_STREAM("[MasterSlaveROSBridge] @Initilize name : " << getInstanceName());

    tm.tick();
    return RTC::RTC_OK;
}

void MasterSlaveROSBridge::onSlaveEEWrenchCB(const ros::MessageEvent<geometry_msgs::WrenchStamped const>& e){
    const ros::M_string& header = e.getConnectionHeader();
    std::string topic = header.at("topic");
    const geometry_msgs::WrenchStampedConstPtr& msg = e.getMessage();
    const std::string key = slaveEEWrenches_topic2key[topic];
    m_slaveEEWrenches[key].data.length(6);
    m_slaveEEWrenches[key].data[0] = msg->wrench.force.x;
    m_slaveEEWrenches[key].data[1] = msg->wrench.force.y;
    m_slaveEEWrenches[key].data[2] = msg->wrench.force.z;
    m_slaveEEWrenches[key].data[3] = msg->wrench.torque.x;
    m_slaveEEWrenches[key].data[4] = msg->wrench.torque.y;
    m_slaveEEWrenches[key].data[5] = msg->wrench.torque.z;
    m_slaveEEWrenchesOut[key]->write();
}

void MasterSlaveROSBridge::onMasterTgtPoseCB(const ros::MessageEvent<geometry_msgs::PoseStamped const>& e){
    const ros::M_string& header = e.getConnectionHeader();
    std::string topic = header.at("topic");
    const geometry_msgs::PoseStampedConstPtr& msg = e.getMessage();
    const std::string key = masterTgtPoses_topic2key[topic];
    tf::Quaternion quat(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
    if(quat.length() != 0.0){
        tf::Matrix3x3(quat.normalized()).getRPY(m_masterTgtPoses[key].data.orientation.r, m_masterTgtPoses[key].data.orientation.p, m_masterTgtPoses[key].data.orientation.y);
    }
    m_masterTgtPoses[key].data.position.x = msg->pose.position.x;
    m_masterTgtPoses[key].data.position.y = msg->pose.position.y;
    m_masterTgtPoses[key].data.position.z = msg->pose.position.z;
    m_masterTgtPosesOut[key]->write();
}


RTC::ReturnCode_t MasterSlaveROSBridge::onExecute(RTC::UniqueId ec_id){
    ros::Time tm_on_execute = ros::Time::now();

    for(int i=0; i<ee_names.size(); i++){
        if(m_slaveEEWrenchesIn[ee_names[i]]->isNew()){
            m_slaveEEWrenchesIn[ee_names[i]]->read();
            geometry_msgs::WrenchStamped tmp;
            tmp.wrench.force.x  = m_slaveEEWrenches[ee_names[i]].data[0];
            tmp.wrench.force.y  = m_slaveEEWrenches[ee_names[i]].data[1];
            tmp.wrench.force.z  = m_slaveEEWrenches[ee_names[i]].data[2];
            tmp.wrench.torque.x = m_slaveEEWrenches[ee_names[i]].data[3];
            tmp.wrench.torque.y = m_slaveEEWrenches[ee_names[i]].data[4];
            tmp.wrench.torque.z = m_slaveEEWrenches[ee_names[i]].data[5];
            tmp.header.stamp = tm_on_execute;
            slaveEEWrenches_pub[ee_names[i]].publish(tmp);
        }
    }
    for(int i=0; i<tgt_names.size(); i++){
        if(m_masterTgtPosesIn[tgt_names[i]]->isNew()){
            m_masterTgtPosesIn[tgt_names[i]]->read();
            geometry_msgs::PoseStamped tmp;
            tmp.pose.position.x = m_masterTgtPoses[tgt_names[i]].data.position.x;
            tmp.pose.position.y = m_masterTgtPoses[tgt_names[i]].data.position.y;
            tmp.pose.position.z = m_masterTgtPoses[tgt_names[i]].data.position.z;
            tf::Quaternion quat = tf::createQuaternionFromRPY(
                m_masterTgtPoses[tgt_names[i]].data.orientation.r,
                m_masterTgtPoses[tgt_names[i]].data.orientation.p,
                m_masterTgtPoses[tgt_names[i]].data.orientation.y);
            tmp.pose.orientation.x = quat.getX();
            tmp.pose.orientation.y = quat.getY();
            tmp.pose.orientation.z = quat.getZ();
            tmp.pose.orientation.w = quat.getW();
            tmp.header.stamp = tm_on_execute;
            masterTgtPoses_pub[tgt_names[i]].publish(tmp);
        }
    }


    static int count = 0;
    tm.tack();
    if ( tm.interval() > 1 ) {
        ROS_INFO_STREAM("[" << getInstanceName() << "] @onExecutece " << ec_id << " is working at " << count << "[Hz]");
        tm.tick();
        count = 0;
    }
    count ++;
//    } else {  // m_range
//    double interval = 5;
//    tm.tack();
//    if ( tm.interval() > interval ) {
//    ROS_WARN_STREAM("[" << getInstanceName() << "] @onExecutece " << ec_id << " is not executed last " << interval << "[sec]");
//    tm.tick();
//    }
//    }


    ros::spinOnce();
  return RTC::RTC_OK;
}

extern "C"{
  void MasterSlaveROSBridgeInit(RTC::Manager* manager)  {
    coil::Properties profile(imagesensorrosbridge_spec);
    manager->registerFactory(profile, RTC::Create<MasterSlaveROSBridge>, RTC::Delete<MasterSlaveROSBridge>);
  }
};

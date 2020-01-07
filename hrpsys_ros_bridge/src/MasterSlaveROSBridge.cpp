#include "MasterSlaveROSBridge.h"

static const char* masterslaverosbridge_spec[] =
  {
    "implementation_id", "MasterSlaveROSBridge",
    "type_name",         "MasterSlaveROSBridge",
    "description",       "rtm data - ros bridge",
    "version",           "1.0",
    "vendor",            "JSK",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };

MasterSlaveROSBridge::MasterSlaveROSBridge(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
    m_delayCheckPacketOutboundIn("delay_check_packet_outbound", m_delayCheckPacketOutbound),
    m_delayCheckPacketInboundOut("delay_check_packet_inbound", m_delayCheckPacketInbound),
    m_exDataOut("exData", m_exData),
    m_exDataIndexOut("exDataIndex", m_exDataIndex)
{}

MasterSlaveROSBridge::~MasterSlaveROSBridge(){}

RTC::ReturnCode_t MasterSlaveROSBridge::onInitialize(){
    addInPort("delay_check_packet_outbound", m_delayCheckPacketOutboundIn);
    addOutPort("delay_check_packet_inbound", m_delayCheckPacketInboundOut);
    addOutPort("exData", m_exDataOut);
    addOutPort("exDataIndex", m_exDataIndexOut);

    ee_names.push_back("lleg");
    ee_names.push_back("rleg");
    ee_names.push_back("larm");
    ee_names.push_back("rarm");

    tgt_names = ee_names;
    tgt_names.push_back("com");
    tgt_names.push_back("head");
    tgt_names.push_back("rhand");
    tgt_names.push_back("lhand");
    tgt_names.push_back("rfloor");
    tgt_names.push_back("lfloor");

    ros::param::param<bool>("~is_master_side", is_master_side);
    ros::param::get("~is_master_side", is_master_side);

    if(is_master_side){
        RTC_INFO_STREAM("Set up ports for MASTER side connection (is_master_side = "<<is_master_side<<")");
        for ( int i=0; i<ee_names.size(); i++) { // read ROS data from Network
            std::string n = "slave_"+ee_names[i]+"_wrench";
            slaveEEWrenches_sub[ee_names[i]] = nh.subscribe<geometry_msgs::WrenchStamped>(n, 1,
                boost::bind(&MasterSlaveROSBridge::onSlaveEEWrenchCB, this, _1, ee_names[i]), ros::VoidConstPtr(),
                ros::TransportHints().unreliable().reliable().tcpNoDelay());
            RTC_INFO_STREAM("register ROS Subscriber " << n );
        }
        for ( int i=0; i<ee_names.size(); i++) { // write RTM data to HC
            std::string n = "slave_"+ee_names[i]+"_wrench";
            m_slaveEEWrenchesOut[ee_names[i]] = OTDS_Ptr(new RTC::OutPort<RTC::TimedDoubleSeq>(n.c_str(), m_slaveEEWrenches[ee_names[i]]));
            registerOutPort(n.c_str(), *m_slaveEEWrenchesOut[ee_names[i]]);
            RTC_INFO_STREAM("register RTC OutPort " << n );
        }
        for ( int i=0; i<tgt_names.size(); i++) { // read RTM data from HC
            std::string n = "master_"+tgt_names[i]+"_pose";
            m_masterTgtPosesIn[tgt_names[i]] = ITP3_Ptr(new RTC::InPort<RTC::TimedPose3D>(n.c_str(), m_masterTgtPoses[tgt_names[i]]));
            registerInPort(n.c_str(), *m_masterTgtPosesIn[tgt_names[i]]);
            RTC_INFO_STREAM("register RTC InPort " << n );
        }
        for ( int i=0; i<tgt_names.size(); i++) { // write ROS data to Network
            std::string n = "master_"+tgt_names[i]+"_pose";
            masterTgtPoses_pub[tgt_names[i]] = nh.advertise<geometry_msgs::PoseStamped>(n, 1);
            RTC_INFO_STREAM("register ROS Publisher " << n );
        }
        //////////////////
        for ( int i=0; i<tgt_names.size(); i++) { // read ROS data from Network
            std::string n = "slave_"+tgt_names[i]+"_pose";
            slaveTgtPoses_sub[tgt_names[i]] = nh.subscribe<geometry_msgs::PoseStamped>(n, 1,
                boost::bind(&MasterSlaveROSBridge::onSlaveTgtPoseCB, this, _1, tgt_names[i]), ros::VoidConstPtr(),
                ros::TransportHints().unreliable().reliable().tcpNoDelay());
            RTC_INFO_STREAM("register ROS Subscriber " << n );
        }
        for ( int i=0; i<tgt_names.size(); i++) { // write RTM data to HC
            std::string n = "slave_"+tgt_names[i]+"_pose";
            m_slaveTgtPosesOut[tgt_names[i]] = OTP3_Ptr(new RTC::OutPort<RTC::TimedPose3D>(n.c_str(), m_slaveTgtPoses[tgt_names[i]]));
            registerOutPort(n.c_str(), *m_slaveTgtPosesOut[tgt_names[i]]);
            RTC_INFO_STREAM("register RTC OutPort " << n );
        }
        for ( int i=0; i<ee_names.size(); i++) { // read RTM data from HC
            std::string n = "master_"+ee_names[i]+"_wrench";
            m_masterEEWrenchesIn[ee_names[i]] = ITDS_Ptr(new RTC::InPort<RTC::TimedDoubleSeq>(n.c_str(), m_masterEEWrenches[ee_names[i]]));
            registerInPort(n.c_str(), *m_masterEEWrenchesIn[ee_names[i]]);
            RTC_INFO_STREAM("register RTC InPort " << n );
        }
        for ( int i=0; i<ee_names.size(); i++) { // write ROS data to Network
            std::string n = "master_"+ee_names[i]+"_wrench";
            masterEEWrenches_pub[ee_names[i]] = nh.advertise<geometry_msgs::WrenchStamped>(n, 1);
            RTC_INFO_STREAM("register ROS Publisher " << n );
        }
        ///////////////////
        {// teleop Odom TF
            std::string n = "teleopOdom";
            m_teleopOdomIn = ITP3_Ptr(new RTC::InPort<RTC::TimedPose3D>(n.c_str(), m_teleopOdom));
            registerInPort(n.c_str(), *m_teleopOdomIn);
            RTC_INFO_STREAM("register RTC InPort  " << n );
        }

    }else{
        RTC_INFO_STREAM("Set up ports for SLAVE side connection (is_master_side = "<<is_master_side<<")");
        for ( int i=0; i<tgt_names.size(); i++) { // to read ROS data from Network
            std::string n = "master_"+tgt_names[i]+"_pose";
            masterTgtPoses_sub[tgt_names[i]] = nh.subscribe<geometry_msgs::PoseStamped>(n, 1,
                boost::bind(&MasterSlaveROSBridge::onMasterTgtPoseCB, this, _1, tgt_names[i]), ros::VoidConstPtr(),
                ros::TransportHints().unreliable().reliable().tcpNoDelay());
            RTC_INFO_STREAM("register ROS Subscriber " << n );
        }
        for ( int i=0; i<tgt_names.size(); i++) { // to write RTM data to WBMS
            std::string n = "master_"+tgt_names[i]+"_pose";
            m_masterTgtPosesOut[tgt_names[i]] = OTP3_Ptr(new RTC::OutPort<RTC::TimedPose3D>(n.c_str(), m_masterTgtPoses[tgt_names[i]]));
            registerOutPort(n.c_str(), *m_masterTgtPosesOut[tgt_names[i]]);
            RTC_INFO_STREAM("register RTC OutPort " << n );
        }
        for ( int i=0; i<ee_names.size(); i++) { // to read RTM data from WBMS
            std::string n = "slave_"+ee_names[i]+"_wrench";
            m_slaveEEWrenchesIn[ee_names[i]] = ITDS_Ptr(new RTC::InPort<RTC::TimedDoubleSeq>(n.c_str(), m_slaveEEWrenches[ee_names[i]]));
            registerInPort(n.c_str(), *m_slaveEEWrenchesIn[ee_names[i]]);
            RTC_INFO_STREAM("register RTC InPort " << n );
        }
        for ( int i=0; i<ee_names.size(); i++) { // to write ROS data to Network
            std::string n = "slave_"+ee_names[i]+"_wrench";
            slaveEEWrenches_pub[ee_names[i]] = nh.advertise<geometry_msgs::WrenchStamped>(n, 1);
            RTC_INFO_STREAM("register ROS Publisher " << n );
        }
        /////////
        for ( int i=0; i<ee_names.size(); i++) { // to read ROS data from Network
            std::string n = "master_"+ee_names[i]+"_wrench";
            masterEEWrenches_sub[ee_names[i]] = nh.subscribe<geometry_msgs::WrenchStamped>(n, 1,
                boost::bind(&MasterSlaveROSBridge::onMasterEEWrenchCB, this, _1, ee_names[i]), ros::VoidConstPtr(),
                ros::TransportHints().unreliable().reliable().tcpNoDelay());
            RTC_INFO_STREAM("register ROS Subscriber " << n );
        }
        for ( int i=0; i<ee_names.size(); i++) { // to write RTM data to WBMS
            std::string n = "master_"+ee_names[i]+"_wrench";
            m_masterEEWrenchesOut[ee_names[i]] = OTDS_Ptr(new RTC::OutPort<RTC::TimedDoubleSeq>(n.c_str(), m_masterEEWrenches[ee_names[i]]));
            registerOutPort(n.c_str(), *m_masterEEWrenchesOut[ee_names[i]]);
            RTC_INFO_STREAM("register RTC OutPort " << n );
        }
        for ( int i=0; i<tgt_names.size(); i++) { // to read RTM data from WBMS
            std::string n = "slave_"+tgt_names[i]+"_pose";
            m_slaveTgtPosesIn[tgt_names[i]] = ITP3_Ptr(new RTC::InPort<RTC::TimedPose3D>(n.c_str(), m_slaveTgtPoses[tgt_names[i]]));
            registerInPort(n.c_str(), *m_slaveTgtPosesIn[tgt_names[i]]);
            RTC_INFO_STREAM("register RTC InPort " << n );
        }
        for ( int i=0; i<tgt_names.size(); i++) { // to write ROS data to Network
            std::string n = "slave_"+tgt_names[i]+"_pose";
            slaveTgtPoses_pub[tgt_names[i]] = nh.advertise<geometry_msgs::PoseStamped>(n, 1);
            RTC_INFO_STREAM("register ROS Publisher " << n );
        }
    }

    // both
    delay_check_packet_sub = nh.subscribe("delay_check_packet_inbound", 1, &MasterSlaveROSBridge::ondelayCheckPacketCB, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
    delay_check_packet_pub = nh.advertise<std_msgs::Time>("delay_check_packet_outbound", 1);

    ROS_INFO_STREAM("[MasterSlaveROSBridge] @Initilize name : " << getInstanceName());

    loop = 0;
    tm.tick();
    return RTC::RTC_OK;
}

void MasterSlaveROSBridge::ondelayCheckPacketCB(const std_msgs::Time::ConstPtr& msg){
    m_delayCheckPacketInbound.sec = msg->data.sec;
    m_delayCheckPacketInbound.nsec = msg->data.nsec;
    m_delayCheckPacketInboundOut.write();
}

void MasterSlaveROSBridge::onMasterEEWrenchCB(const geometry_msgs::WrenchStamped::ConstPtr& msg, std::string& key){
    m_masterEEWrenches[key].data.length(6);
    m_masterEEWrenches[key].data[0] = msg->wrench.force.x;
    m_masterEEWrenches[key].data[1] = msg->wrench.force.y;
    m_masterEEWrenches[key].data[2] = msg->wrench.force.z;
    m_masterEEWrenches[key].data[3] = msg->wrench.torque.x;
    m_masterEEWrenches[key].data[4] = msg->wrench.torque.y;
    m_masterEEWrenches[key].data[5] = msg->wrench.torque.z;
    m_masterEEWrenchesOut[key]->write();
}

void MasterSlaveROSBridge::onSlaveEEWrenchCB(const geometry_msgs::WrenchStamped::ConstPtr& msg, std::string& key){
    m_slaveEEWrenches[key].data.length(6);
    m_slaveEEWrenches[key].data[0] = msg->wrench.force.x;
    m_slaveEEWrenches[key].data[1] = msg->wrench.force.y;
    m_slaveEEWrenches[key].data[2] = msg->wrench.force.z;
    m_slaveEEWrenches[key].data[3] = msg->wrench.torque.x;
    m_slaveEEWrenches[key].data[4] = msg->wrench.torque.y;
    m_slaveEEWrenches[key].data[5] = msg->wrench.torque.z;
    m_slaveEEWrenchesOut[key]->write();
}

void MasterSlaveROSBridge::onMasterTgtPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg, std::string& key){
    tf::Quaternion quat(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
    if(quat.length() != 0.0){
        tf::Matrix3x3(quat.normalized()).getRPY(m_masterTgtPoses[key].data.orientation.r, m_masterTgtPoses[key].data.orientation.p, m_masterTgtPoses[key].data.orientation.y);
    }else{
        m_masterTgtPoses[key].data.orientation = (RTC::Orientation3D){0,0,0};
    }
    m_masterTgtPoses[key].data.position = (RTC::Point3D){msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    m_masterTgtPosesOut[key]->write();
}

void MasterSlaveROSBridge::onSlaveTgtPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg, std::string& key){
    tf::Quaternion quat(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
    if(quat.length() != 0.0){
        tf::Matrix3x3(quat.normalized()).getRPY(m_slaveTgtPoses[key].data.orientation.r, m_slaveTgtPoses[key].data.orientation.p, m_slaveTgtPoses[key].data.orientation.y);
    }else{
        m_slaveTgtPoses[key].data.orientation = (RTC::Orientation3D){0,0,0};
    }
    m_slaveTgtPoses[key].data.position = (RTC::Point3D){msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    m_slaveTgtPosesOut[key]->write();
}


RTC::ReturnCode_t MasterSlaveROSBridge::onExecute(RTC::UniqueId ec_id){
    ros::Time tm_on_execute = ros::Time::now();


    if(is_master_side){ //master
        for(int i=0; i<tgt_names.size(); i++){
            if(m_masterTgtPosesIn[tgt_names[i]]->isNew()){
                m_masterTgtPosesIn[tgt_names[i]]->read();
                geometry_msgs::PoseStamped tmp;
                tmp.header.stamp = tm_on_execute;
                tmp.header.frame_id = "teleop_odom";
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
                masterTgtPoses_pub[tgt_names[i]].publish(tmp);
            }
        }
        for(int i=0; i<ee_names.size(); i++){
            if(m_masterEEWrenchesIn[ee_names[i]]->isNew()){
                m_masterEEWrenchesIn[ee_names[i]]->read();
                geometry_msgs::WrenchStamped tmp;
                tmp.header.stamp = tm_on_execute;
                tmp.header.frame_id = "master_"+ee_names[i]+"_aligned_to_world";
                tmp.wrench.force.x  = m_masterEEWrenches[ee_names[i]].data[0];
                tmp.wrench.force.y  = m_masterEEWrenches[ee_names[i]].data[1];
                tmp.wrench.force.z  = m_masterEEWrenches[ee_names[i]].data[2];
                tmp.wrench.torque.x = m_masterEEWrenches[ee_names[i]].data[3];
                tmp.wrench.torque.y = m_masterEEWrenches[ee_names[i]].data[4];
                tmp.wrench.torque.z = m_masterEEWrenches[ee_names[i]].data[5];
                masterEEWrenches_pub[ee_names[i]].publish(tmp);
            }
        }
        if (m_teleopOdomIn->isNew()) {
            m_teleopOdomIn->read();
            geometry_msgs::TransformStamped tf_tmp;
            tf_tmp.header.stamp = tm_on_execute;
            tf_tmp.header.frame_id = "teleop_odom";
            tf_tmp.child_frame_id = "BASE_LINK";
            tf::Quaternion quat = tf::createQuaternionFromRPY(m_teleopOdom.data.orientation.r, m_teleopOdom.data.orientation.p, m_teleopOdom.data.orientation.y);
            tf_tmp.transform.translation.x = m_teleopOdom.data.position.x;
            tf_tmp.transform.translation.y = m_teleopOdom.data.position.y;
            tf_tmp.transform.translation.z = m_teleopOdom.data.position.z;
            tf_tmp.transform.rotation.x = quat.getX();
            tf_tmp.transform.rotation.y = quat.getY();
            tf_tmp.transform.rotation.z = quat.getZ();
            tf_tmp.transform.rotation.w = quat.getW();
            if(loop%10==0){ // will be 100Hz of 1000Hz
                std::vector<geometry_msgs::TransformStamped> tf_transforms;
                tf_transforms.push_back(tf_tmp);
                br.sendTransform(tf_transforms);
            }
        }

    }else{ // slave
        for(int i=0; i<ee_names.size(); i++){
            if(m_slaveEEWrenchesIn[ee_names[i]]->isNew()){
                m_slaveEEWrenchesIn[ee_names[i]]->read();
                geometry_msgs::WrenchStamped tmp;
                tmp.header.stamp = tm_on_execute;
                tmp.header.frame_id = "slave_"+ee_names[i]+"_aligned_to_world";
                tmp.wrench.force.x  = m_slaveEEWrenches[ee_names[i]].data[0];
                tmp.wrench.force.y  = m_slaveEEWrenches[ee_names[i]].data[1];
                tmp.wrench.force.z  = m_slaveEEWrenches[ee_names[i]].data[2];
                tmp.wrench.torque.x = m_slaveEEWrenches[ee_names[i]].data[3];
                tmp.wrench.torque.y = m_slaveEEWrenches[ee_names[i]].data[4];
                tmp.wrench.torque.z = m_slaveEEWrenches[ee_names[i]].data[5];
                slaveEEWrenches_pub[ee_names[i]].publish(tmp);
            }
        }
        for(int i=0; i<tgt_names.size(); i++){
            if(m_slaveTgtPosesIn[tgt_names[i]]->isNew()){
                m_slaveTgtPosesIn[tgt_names[i]]->read();
                geometry_msgs::PoseStamped tmp;
                tmp.header.stamp = tm_on_execute;
                tmp.header.frame_id = "slave_teleop_odom";
                tmp.pose.position.x = m_slaveTgtPoses[tgt_names[i]].data.position.x;
                tmp.pose.position.y = m_slaveTgtPoses[tgt_names[i]].data.position.y;
                tmp.pose.position.z = m_slaveTgtPoses[tgt_names[i]].data.position.z;
                tf::Quaternion quat = tf::createQuaternionFromRPY(
                    m_slaveTgtPoses[tgt_names[i]].data.orientation.r,
                    m_slaveTgtPoses[tgt_names[i]].data.orientation.p,
                    m_slaveTgtPoses[tgt_names[i]].data.orientation.y);
                tmp.pose.orientation.x = quat.getX();
                tmp.pose.orientation.y = quat.getY();
                tmp.pose.orientation.z = quat.getZ();
                tmp.pose.orientation.w = quat.getW();
                slaveTgtPoses_pub[tgt_names[i]].publish(tmp);
            }
        }
    }


    // both
    if(m_delayCheckPacketOutboundIn.isNew()){
        m_delayCheckPacketOutboundIn.read();
        std_msgs::Time tmp;
        tmp.data.sec = m_delayCheckPacketOutbound.sec;
        tmp.data.nsec = m_delayCheckPacketOutbound.nsec;
        delay_check_packet_pub.publish(tmp);
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

    loop++;
    ros::spinOnce();
    return RTC::RTC_OK;
}

extern "C"{
    void MasterSlaveROSBridgeInit(RTC::Manager* manager)  {
        coil::Properties profile(masterslaverosbridge_spec);
        manager->registerFactory(profile, RTC::Create<MasterSlaveROSBridge>, RTC::Delete<MasterSlaveROSBridge>);
    }
};

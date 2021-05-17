#ifndef MASTERSLAVEROSBRIDGE_H
#define MASTERSLAVEROSBRIDGE_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/InterfaceDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
// ros
#include "ros/ros.h"
#include "std_msgs/Time.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
//#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace RTC;

class MasterSlaveROSBridge  : public RTC::DataFlowComponentBase
{
    public:
        MasterSlaveROSBridge(RTC::Manager* manager);
        ~MasterSlaveROSBridge();
        virtual RTC::ReturnCode_t onInitialize();
        virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
        void updateOdometryTF(const ros::Time &stamp);
        void onMasterTgtPoseCB  (const geometry_msgs::PoseStamped::ConstPtr& msg,   std::string& key);
        void onSlaveTgtPoseCB   (const geometry_msgs::PoseStamped::ConstPtr& msg,   std::string& key);
        void onMasterEEWrenchCB (const geometry_msgs::WrenchStamped::ConstPtr& msg, std::string& key);
        void onSlaveEEWrenchCB  (const geometry_msgs::WrenchStamped::ConstPtr& msg, std::string& key);
        void ondelayCheckPacketCB(const std_msgs::Time::ConstPtr& msg);

    protected:
        // used in both case
        bool is_master_side;
        ros::NodeHandle nh;
        coil::TimeMeasure tm;
        unsigned long loop;
        std::vector<std::string> ee_names, tgt_names;
        typedef boost::shared_ptr<RTC::InPort   <RTC::TimedPose3D>      > ITP3_Ptr;
        typedef boost::shared_ptr<RTC::InPort   <RTC::TimedDoubleSeq>   > ITDS_Ptr;
        typedef boost::shared_ptr<RTC::OutPort  <RTC::TimedPose3D>      > OTP3_Ptr;
        typedef boost::shared_ptr<RTC::OutPort  <RTC::TimedDoubleSeq>   > OTDS_Ptr;
        std::map<std::string, RTC::TimedPose3D>     m_masterTgtPoses;
        std::map<std::string, RTC::TimedPose3D>     m_slaveTgtPoses;
        std::map<std::string, RTC::TimedDoubleSeq>  m_masterEEWrenches;
        std::map<std::string, RTC::TimedDoubleSeq>  m_slaveEEWrenches;

        // master side
        std::map<std::string, ITP3_Ptr>         m_masterTgtPosesIn;
        std::map<std::string, OTDS_Ptr>         m_slaveEEWrenchesOut;
        std::map<std::string, ros::Publisher>   masterTgtPoses_pub;
        std::map<std::string, ros::Subscriber>  slaveEEWrenches_sub;
        // sub usage
        std::map<std::string, ITDS_Ptr>         m_masterEEWrenchesIn;
        std::map<std::string, OTP3_Ptr>         m_slaveTgtPosesOut;
        std::map<std::string, ros::Publisher>   masterEEWrenches_pub;
        std::map<std::string, ros::Subscriber>  slaveTgtPoses_sub;
        RTC::TimedPose3D m_teleopOdom;
        ITP3_Ptr m_teleopOdomIn;
        tf::TransformBroadcaster br;

        // slave side
        std::map<std::string, ITDS_Ptr>         m_slaveEEWrenchesIn;
        std::map<std::string, OTP3_Ptr>         m_masterTgtPosesOut;
        std::map<std::string, ros::Publisher>   slaveEEWrenches_pub;
        std::map<std::string, ros::Subscriber>  masterTgtPoses_sub;
        // sub usage
        std::map<std::string, ITP3_Ptr>         m_slaveTgtPosesIn;
        std::map<std::string, OTDS_Ptr>         m_masterEEWrenchesOut;
        std::map<std::string, ros::Publisher>   slaveTgtPoses_pub;
        std::map<std::string, ros::Subscriber>  masterEEWrenches_sub;

        // delay calc
        ros::Subscriber delay_check_packet_sub;
        ros::Publisher delay_check_packet_pub;
        RTC::Time m_delayCheckPacketInbound, m_delayCheckPacketOutbound;
        RTC::OutPort<RTC::Time> m_delayCheckPacketInboundOut;
        RTC::InPort<RTC::Time> m_delayCheckPacketOutboundIn;

        RTC::TimedDoubleSeq m_exData;
        RTC::TimedStringSeq m_exDataIndex;
        RTC::OutPort<RTC::TimedDoubleSeq> m_exDataOut;
        RTC::OutPort<RTC::TimedStringSeq> m_exDataIndexOut;
};


////// copy
#define dbg(var) std::cout<<#var"= "<<(var)<<std::endl
#define dbgn(var) std::cout<<#var"= "<<std::endl<<(var)<<std::endl
#define dbgv(var) std::cout<<#var"= "<<(var.transpose())<<std::endl
#define RTC_INFO_STREAM(var) std::cout << "[" << m_profile.instance_name << "] "<< var << std::endl;
#define RTC_WARN_STREAM(var) std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << var << "\x1b[39m" << std::endl;

#define eps_eq(a, b, c)             (fabs((a)-(b)) <= c)
#define LIMIT_NORM(x,max)           (x= ( x<(-max) ? -max : (x>max ? max : x)))
#define LIMIT_MIN(x,min)            (x= ( x<min ? min : x ))
#define LIMIT_MAX(x,max)            (x= ( x>max ? max : x ))
#define LIMIT_MINMAX(x,min,max)     (x= ( x<min ? min : ( x>max ? max : x )))
#define LIMIT_NORM_V(v,max)         if(v.norm()>max){v=v.normalized()*max;}
#define LIMIT_MIN_V(v,minv)         (v= v.cwiseMax(minv))
#define LIMIT_MAX_V(v,maxv)         (v= v.cwiseMin(maxv))
#define LIMIT_MINMAX_V(v,minv,maxv) (v= v.cwiseMin(minv).cwiseMax(maxv))

extern "C"{  DLL_EXPORT void MasterSlaveROSBridgeInit(RTC::Manager* manager);};
#endif // MASTERSLAVEROSBRIDGE_H

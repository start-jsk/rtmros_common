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
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include <tf/transform_listener.h>

using namespace RTC;

class MasterSlaveROSBridge  : public RTC::DataFlowComponentBase
{
    public:
        MasterSlaveROSBridge(RTC::Manager* manager);
        ~MasterSlaveROSBridge();
        virtual RTC::ReturnCode_t onInitialize();
        virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

        void onMasterTgtPoseCB(const ros::MessageEvent<geometry_msgs::PoseStamped const>& e);
        void onSlaveEEWrenchCB(const ros::MessageEvent<geometry_msgs::WrenchStamped const>& e);

    protected:
        typedef boost::shared_ptr<RTC::InPort   <RTC::TimedPose3D>      > ITP3_Ptr;
        typedef boost::shared_ptr<RTC::OutPort  <RTC::TimedPose3D>      > OTP3_Ptr;
        typedef boost::shared_ptr<RTC::InPort   <RTC::TimedDoubleSeq>   > ITDS_Ptr;
        typedef boost::shared_ptr<RTC::OutPort  <RTC::TimedDoubleSeq>   > OTDS_Ptr;

        std::map<std::string, RTC::TimedPose3D> m_masterTgtPoses;//used both on master mode / slave mode
        RTC::TimedPose3D test;//used both on master mode / slave mode
        std::map<std::string, ITP3_Ptr> m_masterTgtPosesIn;
        std::map<std::string, OTP3_Ptr> m_masterTgtPosesOut;

        std::map<std::string, RTC::TimedDoubleSeq> m_slaveEEWrenches;//used both on master mode / slave mode
        std::map<std::string, ITDS_Ptr> m_slaveEEWrenchesIn;
        std::map<std::string, OTDS_Ptr> m_slaveEEWrenchesOut;

        RTC::TimedDoubleSeq m_exData;
        RTC::TimedStringSeq m_exDataIndex;
        RTC::OutPort<RTC::TimedDoubleSeq> m_exDataOut;
        RTC::OutPort<RTC::TimedStringSeq> m_exDataIndexOut;

        std::map<std::string, ros::Publisher> masterTgtPoses_pub;
        std::map<std::string, ros::Subscriber> masterTgtPoses_sub;

        std::map<std::string, ros::Publisher> slaveEEWrenches_pub;
        std::map<std::string, ros::Subscriber> slaveEEWrenches_sub;

        std::map<std::string, std::string> masterTgtPoses_topic2key;
        std::map<std::string, std::string> slaveEEWrenches_topic2key;

        std::vector<std::string> ee_names, tgt_names;

        std::string mode;


    private:
        ros::NodeHandle nh;
        coil::TimeMeasure tm;
};



////// copy
#define dbg(var) std::cout<<#var"= "<<(var)<<std::endl
#define dbgn(var) std::cout<<#var"= "<<std::endl<<(var)<<std::endl
#define dbgv(var) std::cout<<#var"= "<<(var.transpose())<<std::endl
#define RTCOUT(var) std::cerr << "[" << m_profile.instance_name << "] "<<var<<std::endl;
#define LIMIT_MIN(x,min) (x= ( x<min ? min:x ))
#define LIMIT_MAX(x,max) (x= ( x<max ? x:max ))
#define LIMIT_MINMAX(x,min,max) ((x= (x<min  ? min : x<max ? x : max)))
#define eps_eq(a, b, c)  (fabs((a)-(b)) <= c)
#define LIMIT_NORM(v,max) if(v.norm()>max){ v=v.normalized()*max; }
//namespace hrp{
//    class Pose3{
//        public:
//            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//            hrp::Vector3 p;
//            hrp::Matrix33 R;
//            Pose3()                                                                                                             { reset();}
//            Pose3(const double& _X, const double& _Y, const double& _Z, const double& _r, const double& _p, const double& _y)   { p << _X,_Y,_Z; R = hrp::rotFromRpy(_r,_p,_y); }
//            Pose3(const hrp::dvector6& _xyz_rpy)                                                                                { p = _xyz_rpy.head(3); R = hrp::rotFromRpy(_xyz_rpy.tail(3)); }
//            Pose3(const hrp::Vector3& _xyz, const hrp::Matrix33& _R)                                                            { p = _xyz; R = _R; }
//            void reset()                                                                                                        { p.fill(0); R.setIdentity(); }
//            hrp::Vector3 rpy() const                                                                                            { return hrp::rpyFromRot(R); }
//            hrp::dvector6 to_dvector6() const                                                                                   { return (hrp::dvector6() << p, hrp::rpyFromRot(R)).finished(); }
//            void setRpy(const double& _r, const double& _p, const double& _y)                                                   { R = rotFromRpy(_r,_p,_y); }
//            void setRpy(const hrp::Vector3& _rpy)                                                                               { R = rotFromRpy(_rpy); }
//    };
//
//    inline hrp::Vector3         to_Vector3      (const RTC::Point3D& in)        { return hrp::Vector3(in.x, in.y, in.z); }
//    inline hrp::Vector3         to_Vector3      (const RTC::Orientation3D& in)  { return hrp::Vector3(in.r, in.p, in.y); }
//    inline hrp::dvector6        to_dvector6     (const RTC::Pose3D& in)         { return (hrp::dvector6() << in.position.x, in.position.y, in.position.z, in.orientation.r, in.orientation.p, in.orientation.y).finished(); }
//    inline hrp::Pose3           to_Pose3        (const RTC::Pose3D& in)         { return hrp::Pose3(in.position.x, in.position.y, in.position.z, in.orientation.r, in.orientation.p, in.orientation.y); }
//    inline hrp::dvector6        to_dvector6     (const OpenHRP::Wrench& in)     { return (hrp::dvector6() << in.force.x, in.force.y, in.force.z, in.torque.x, in.torque.y, in.torque.z).finished(); }
//    inline hrp::dvector         to_dvector      (const RTC::TimedDoubleSeq::_data_seq& in) { return hrp::dvector::Map(in.get_buffer(), in.length()); }
//
//    inline RTC::Point3D         to_Point3D      (const hrp::Vector3& in)        { return (RTC::Point3D){in(X),in(Y),in(Z)}; }
//    inline RTC::Orientation3D   to_Orientation3D(const hrp::Vector3& in)        { return (RTC::Orientation3D){in(X),in(Y),in(Z)}; }
//    inline RTC::Pose3D          to_Pose3D       (const hrp::dvector6& in)       { return (RTC::Pose3D){in(X),in(Y),in(Z),in(r),in(p),in(y)}; }
//    inline RTC::Pose3D          to_Pose3D       (const hrp::Pose3& in)          { return (RTC::Pose3D){in.p(X),in.p(Y),in.p(Z),in.rpy()(r),in.rpy()(p),in.rpy()(y)}; }
//    inline OpenHRP::Wrench      to_Wrench       (const hrp::dvector6& in)       { return (OpenHRP::Wrench){in(X),in(Y),in(Z),in(r),in(p),in(y)}; }
//    inline RTC::TimedDoubleSeq::_data_seq   to_DoubleSeq    (const hrp::dvector& in)    { RTC::TimedDoubleSeq::_data_seq out; out.length(in.size()); hrp::dvector::Map(out.get_buffer(), in.size()) = in; return out; }
//
//    inline hrp::dvector getQAll         (const hrp::BodyPtr _robot){ hrp::dvector tmp(_robot->numJoints()); for(int i=0;i<_robot->numJoints();i++){ tmp(i) = _robot->joint(i)->q; } return tmp; }
//    inline void         setQAll         (hrp::BodyPtr       _robot, const hrp::dvector& in){ assert(in.size() <= _robot->numJoints()); for(int i=0;i<_robot->numJoints();i++){ _robot->joint(i)->q = in(i); } }
//    inline hrp::dvector getRobotStateVec(const hrp::BodyPtr _robot){ return (hrp::dvector(_robot->numJoints()+6) << getQAll(_robot), _robot->rootLink()->p, hrp::rpyFromRot(_robot->rootLink()->R)).finished(); }
//    inline void         setRobotStateVec(hrp::BodyPtr       _robot, const hrp::dvector& _q_bpos_brpy){ assert(_q_bpos_brpy.size() == _robot->numJoints()+6); for(int i=0;i<_robot->numJoints();i++){ _robot->joint(i)->q = _q_bpos_brpy(i); }; _robot->rootLink()->p = _q_bpos_brpy.tail(6).head(3); _robot->rootLink()->R = hrp::rotFromRpy(_q_bpos_brpy.tail(6).tail(3)); }
//    inline void         setRobotStateVec(hrp::BodyPtr       _robot, const hrp::dvector& _q, const hrp::Vector3& _bpos, const hrp::Vector3& _brpy){ assert(_q.size() == _robot->numJoints()); for(int i=0;i<_robot->numJoints();i++){ _robot->joint(i)->q = _q(i); }; _robot->rootLink()->p = _bpos; _robot->rootLink()->R = hrp::rotFromRpy(_brpy); }
//    inline void         setRobotStateVec(hrp::BodyPtr       _robot, const hrp::dvector& _q, const hrp::Vector3& _bpos, const hrp::Matrix33& _bR){ assert(_q.size() == _robot->numJoints()); for(int i=0;i<_robot->numJoints();i++){ _robot->joint(i)->q = _q(i); }; _robot->rootLink()->p = _bpos; _robot->rootLink()->R = _bR; }
//    inline std::vector<std::string> getJointNameAll (const hrp::BodyPtr _robot){ std::vector<std::string> ret(_robot->numJoints()); for(int i=0;i<_robot->numJoints();i++){ ret[i] = _robot->joint(i)->name; } return ret; }
//    inline hrp::dvector getUAll         (const hrp::BodyPtr _robot){ hrp::dvector tmp(_robot->numJoints()); for(int i=0;i<_robot->numJoints();i++){ tmp(i) = _robot->joint(i)->u; } return tmp; }
//
//
//    inline std::vector<std::string> to_string_vector (const RTC::TimedStringSeq::_data_seq& in) {
//        std::vector<std::string> ret(in.length()); for(int i=0; i<in.length(); i++){ ret[i] = in[i]; } return ret;
//    }
//}

extern "C"{  DLL_EXPORT void MasterSlaveROSBridgeInit(RTC::Manager* manager);};
#endif // MASTERSLAVEROSBRIDGE_H

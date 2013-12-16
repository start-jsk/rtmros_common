// -*- C++ -*-
/*!
 * @file  HrpsysSeqStateROSBridgeImpl.h * @brief hrpsys seq state - ros bridge * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef HRPSYSSEQSTATEROSBRIDGEIMPL_H
#define HRPSYSSEQSTATEROSBRIDGEIMPL_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/Manager.h>
#include <rtm/CorbaNaming.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include "hrpsys_ros_bridge/idl/HRPDataTypes.hh"

// hrp
#include <hrpCorba/ModelLoader.hh>
#include <hrpModel/Body.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/Link.h>
#include <hrpModel/ModelLoaderUtil.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

#include "hrpsys_ros_bridge/idl/SequencePlayerServiceStub.h"

// </rtc-template>

#include "tf/transform_broadcaster.h"
using namespace RTC;

class HrpsysSeqStateROSBridgeImpl  : public RTC::DataFlowComponentBase
{
 public:
  HrpsysSeqStateROSBridgeImpl(RTC::Manager* manager);
  ~HrpsysSeqStateROSBridgeImpl();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry() 
  virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  // virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  // virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  // virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  // virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_rsangle;
  InPort<TimedDoubleSeq> m_rsangleIn;
  TimedDoubleSeq m_mcangle;
  InPort<TimedDoubleSeq> m_mcangleIn;
  std::vector<TimedDoubleSeq> m_rsforce;
  std::vector<InPort<TimedDoubleSeq> *> m_rsforceIn;
  std::vector<std::string> m_rsforceName;
  std::vector<TimedAcceleration3D> m_gsensor;
  std::vector<InPort<TimedAcceleration3D> *> m_gsensorIn;
  std::vector<TimedAngularVelocity3D> m_gyrometer;
  std::vector<InPort<TimedAngularVelocity3D> *> m_gyrometerIn;
  TimedDoubleSeq m_baseTform;
  InPort<TimedDoubleSeq> m_baseTformIn;
  TimedPoint3D m_basePos;
  InPort<TimedPoint3D> m_basePosIn;
  TimedOrientation3D m_baseRpy;
  InPort<TimedOrientation3D> m_baseRpyIn;
  TimedDoubleSeq m_rstorque;
  InPort<TimedDoubleSeq> m_rstorqueIn;
  OpenHRP::TimedLongSeqSeq m_servoState;
  InPort<OpenHRP::TimedLongSeqSeq> m_servoStateIn;
  TimedPoint3D m_rszmp;
  InPort<TimedPoint3D> m_rszmpIn;

  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_mctorque;
  OutPort<TimedDoubleSeq> m_mctorqueOut;

  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_SequencePlayerServicePort;

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  RTC::CorbaConsumer<OpenHRP::SequencePlayerService> m_service0;

  // </rtc-template>

 protected:
  hrp::BodyPtr body;
  OpenHRP::BodyInfo_var bodyinfo;

  typedef struct  {
    std::string link_name, type_name;
    tf::Transform transform;
  } SensorInfo;
  std::map<std::string, SensorInfo> sensor_info;

 private:
};


extern "C"
{
  DLL_EXPORT void HrpsysSeqStateROSBridgeImplInit(RTC::Manager* manager);
};

#endif // HRPSYSSEQSTATEROSBRIDGEIMPL_H


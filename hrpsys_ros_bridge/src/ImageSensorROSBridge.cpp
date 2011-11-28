// -*- C++ -*-
/*!
 * @file  ImageSensorROSBridge.cpp * @brief openhrp image - ros bridge * $Date$ 
 *
 * $Id$ 
 */
#include "ImageSensorROSBridge.h"

// Module specification
// <rtc-template block="module_spec">
static const char* imagesensorrosbridge_spec[] =
  {
    "implementation_id", "ImageSensorROSBridge",
    "type_name",         "ImageSensorROSBridge",
    "description",       "openrhp image - ros bridge",
    "version",           "1.0",
    "vendor",            "Kei Okada",
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

ImageSensorROSBridge::ImageSensorROSBridge(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_imageIn("image", m_image),
    // </rtc-template>
    it(node)
{
}

ImageSensorROSBridge::~ImageSensorROSBridge()
{
}


RTC::ReturnCode_t ImageSensorROSBridge::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("image", m_imageIn);

  // Set OutPort buffer

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  pub = it.advertise("image_raw", 1);
  info_pub = node.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

  // initialize
  ROS_INFO_STREAM("[ImageSensorROSBridge] @Initilize name : " << getInstanceName());

  pair_id = 0;
  ros::param::param<std::string>("~frame_id", frame, "camera");

  tm.tick();
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t ImageSensorROSBridge::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ImageSensorROSBridge::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ImageSensorROSBridge::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ImageSensorROSBridge::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ImageSensorROSBridge::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t ImageSensorROSBridge::onExecute(RTC::UniqueId ec_id)
{
  capture_time = ros::Time::now();

  // m_image
  if (m_imageIn.isNew()){

    ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecute ec_id : " << ec_id << ", image:" << m_imageIn.isNew ());

    m_imageIn.read();
#if 0
#if 0
    static ::CORBA::ULong startl = 0U;
    if(startl == 0)
      startl = m_image.tm.sec;
    printf("t = %4.1f[s] : ",m_image.tm.sec - startl + m_image.tm.nsec*1e-9);
#else
    static int count = 0;
    printf("t = %4.1f[s] : ", count*0.1);
    count++;
#endif
#define isBlack(x) (x)==0xff000000
    for (int i=320*120+150; i<320*120+170; i++){
      if (isBlack(m_image.data[i])){
	printf("0");
      }else{
	printf("1");
      }
    }
    printf("\n");
#endif

    sensor_msgs::ImagePtr image(new sensor_msgs::Image);
    sensor_msgs::CameraInfoPtr info(new sensor_msgs::CameraInfo);

    // assume that m_image.data is color and 3:4 image
    image->width  = 4*sqrt(m_image.data.length()/12);
    image->height = 3*sqrt(m_image.data.length()/12);
    image->step = 3 * image->width;
    image->encoding = sensor_msgs::image_encodings::RGB8;
    image->header.stamp = capture_time;
    image->header.seq = pair_id;
    image->header.frame_id = frame;

    image->data.resize(image->step * image->height);

    for(int i=0; i<m_image.data.length();i++){
      image->data[i*3+0] = (m_image.data[i] & 0x00ff0000) >> 16;
      image->data[i*3+1] = (m_image.data[i] & 0x0000ff00) >>  8;
      image->data[i*3+2] = (m_image.data[i] & 0x000000ff) >>  0;
    }
    pub.publish(image);

    info->width  = image->width;
    info->height = image->height;
    info->distortion_model = "plumb_bob";
    boost::array<double, 9> K = {700.0, 0.0, 160.0, 0.0, 700.0, 120.0, 0.0, 0.0, 1.0}; // TODO fieldOfView       0.785398
    info->K = K;
    boost::array<double, 12> P = {700, 0, 160, 0, 0, 700, 120, 0, 0, 0, 1, 0};
    info->P = P;
    info->header.stamp = capture_time;
    info->header.frame_id = frame;
    info_pub.publish(info);

    ++pair_id;

    static int count = 0;
    tm.tack();
    if ( tm.interval() > 1 ) {
      ROS_INFO_STREAM("[" << getInstanceName() << "] @onExecutece " << ec_id << " is working at " << count << "[Hz]");
      tm.tick();
    }
    count ++;
  } else {  // m_image
    double interval = 5;
    tm.tack();
    if ( tm.interval() > interval ) {
      ROS_WARN_STREAM("[" << getInstanceName() << "] @onExecutece " << ec_id << " is not executed last " << interval << "[sec]");
      tm.tick();
    }
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ImageSensorROSBridge::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ImageSensorROSBridge::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ImageSensorROSBridge::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ImageSensorROSBridge::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ImageSensorROSBridge::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void ImageSensorROSBridgeInit(RTC::Manager* manager)
  {
    coil::Properties profile(imagesensorrosbridge_spec);
    manager->registerFactory(profile,
                             RTC::Create<ImageSensorROSBridge>,
                             RTC::Delete<ImageSensorROSBridge>);
  }
  
};




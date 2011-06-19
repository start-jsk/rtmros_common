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
  static unsigned int pair_id = 0;
  ros::Time capture_time = ros::Time::now();

  if (m_imageIn.isNew()){
    //std::cerr << "@Execute name : " << getInstanceName() << "/" << ec_id << ", image:" << m_imageIn.isNew () << std::endl;

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
    std::string frame = "camera";

    image->width = 320;
    image->height = 240;
    image->step = 3 * image->width;
    image->encoding = sensor_msgs::image_encodings::RGB8;
    image->header.stamp = capture_time;
    image->header.seq = pair_id;
    image->header.frame_id = frame;

    image->data.resize(image->step * image->height);

    for(int i=0; i<image->width*image->height;i++){
      image->data[i*3+0] = (m_image.data[i] & 0x00ff0000) >> 16;
      image->data[i*3+1] = (m_image.data[i] & 0x0000ff00) >>  8;
      image->data[i*3+2] = (m_image.data[i] & 0x000000ff) >>  0;
    }
    pub.publish(image);

    info->width = 320;
    info->height = 240;
    info->header.stamp = capture_time;
    info->header.frame_id = frame;
    info_pub.publish(info);

    ++pair_id;
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




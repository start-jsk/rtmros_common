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
    m_timageIn("timedImage", m_timage),
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
  addInPort("timedImage", m_timageIn);

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
  // camera_param K
  K[0] = 700; K[1] =   0; K[2] = 160;
  K[3] =   0; K[4] = 700; K[5] = 120;
  K[6] =   0; K[7] =   0; K[8] = 1;
  // camera_param P
  P[0] = 700; P[1] =   0; P[2] = 160; P[3] = 0;
  P[4] =   0; P[5] = 700; P[6] = 120; P[7] = 0;
  P[8] =   0; P[9] =   0; P[10] = 1;  P[11] = 0;
  overwrite_P = overwrite_K = false;
  ros::param::param<std::string>("~frame_id", frame, "camera");
  if(ros::param::has("~camera_param_K")) {
    XmlRpc::XmlRpcValue param_list;
    if (ros::param::get("~camera_param_K", param_list)) {
      overwrite_K = true;
      if(param_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < param_list.size(); i++) {
          double k;
          if(param_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            k = (int)param_list[i];
          } else if(param_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            k = (double)param_list[i];
          }
          if(i < 9) K[i] =  k;
        }
      }
    }
  }
  if(ros::param::has("~camera_param_P")) {
    XmlRpc::XmlRpcValue param_list;
    if (ros::param::get("~camera_param_P", param_list)) {
      overwrite_P = true;
      if(param_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < param_list.size(); i++){
          double p;
          if(param_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            p = (int)param_list[i];
          } else if(param_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            p = (double)param_list[i];
          }
          if(i < 12) P[i] = p;
        }
      }
    }
  }
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
    info->K = K;
    info->P = P;
    info->R[0] = info->R[4] = info->R[8] = 1;
    info->header.stamp = capture_time;
    info->header.frame_id = frame;
    info_pub.publish(info);

    ++pair_id;

    static int count = 0;
    tm.tack();
    if ( tm.interval() > 1 ) {
      ROS_INFO_STREAM("[" << getInstanceName() << "] @onExecutece " << ec_id << " is working at " << count << "[Hz]");
      tm.tick();
      count = 0;
    }
    count ++;
  } else if (m_timageIn.isNew()) { //
    ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecute ec_id : " << ec_id << ", timedImage:" << m_timageIn.isNew ());
    //
    sensor_msgs::ImagePtr image(new sensor_msgs::Image);
    sensor_msgs::CameraInfoPtr info(new sensor_msgs::CameraInfo);

    m_timageIn.read();

    image->width  = m_timage.data.image.width;
    image->height = m_timage.data.image.height;

    switch(m_timage.data.image.format) {
    case Img::CF_GRAY:
      image->encoding = sensor_msgs::image_encodings::MONO8;
      image->step = 1 * m_timage.data.image.width;
      break;
    case Img::CF_RGB:
      image->encoding = sensor_msgs::image_encodings::RGB8;
      image->step = 3 * m_timage.data.image.width;
      break;
    }

    image->header.stamp = ros::Time(m_timage.tm.sec, m_timage.tm.nsec);
    //std::cerr << m_timage.tm.sec << " " << m_timage.tm.nsec << " / ";
    //std::cerr << m_timage.data.captured_time.sec << " " << m_timage.data.captured_time.nsec << std::endl;
    image->header.seq = pair_id;
    image->header.frame_id = frame;

    image->data.resize(m_timage.data.image.raw_data.length());
    std::copy(m_timage.data.image.raw_data.get_buffer(),
              m_timage.data.image.raw_data.get_buffer() + m_timage.data.image.raw_data.length(), 
              image->data.begin());

    pub.publish(image);

    info->width  = image->width;
    info->height = image->height;
    info->distortion_model = "plumb_bob";
    if (m_timage.data.intrinsic.distortion_coefficient.length() > 0) {
      info->D.resize(m_timage.data.intrinsic.distortion_coefficient.length());
      for(int n = 0; n < m_timage.data.intrinsic.distortion_coefficient.length(); n++) {
        info->D[n] = m_timage.data.intrinsic.distortion_coefficient[n];
      }
    }
    if (overwrite_K) {
      info->K = K;
    } else {
      info->K[0] = m_timage.data.intrinsic.matrix_element[0];
      info->K[1] = m_timage.data.intrinsic.matrix_element[1];
      info->K[2] = m_timage.data.intrinsic.matrix_element[2];
      info->K[3] = m_timage.data.intrinsic.matrix_element[1];
      info->K[4] = m_timage.data.intrinsic.matrix_element[3];
      info->K[5] = m_timage.data.intrinsic.matrix_element[4];
      info->K[6] = info->K[7] = 0.0; info->K[8] = 1.0;
    }
    if (overwrite_P) {
      info->P = P;
    } else {
      // TODO using parameters
      // P = K * m_timage.data.extrinsic ???
      info->P[0] = m_timage.data.intrinsic.matrix_element[0];
      info->P[1] = m_timage.data.intrinsic.matrix_element[1];
      info->P[2] = m_timage.data.intrinsic.matrix_element[2];
      info->P[4] = m_timage.data.intrinsic.matrix_element[1];
      info->P[5] = m_timage.data.intrinsic.matrix_element[3];
      info->P[6] = m_timage.data.intrinsic.matrix_element[4];
      info->P[3] = info->P[7] = info->P[8] = info->P[9] = info->P[11] = 0.0;
      info->P[10] = 1.0;
    }
    info->R[0] = info->R[4] = info->R[8] = 1;
    info->header = image->header;
    info_pub.publish(info);

    ++pair_id;

    static int count = 0;
    tm.tack();
    if ( tm.interval() > 1 ) {
      ROS_INFO_STREAM("[" << getInstanceName() << "] @onExecutece " << ec_id << " is working at " << count << "[Hz]");
      tm.tick();
      count = 0;
    }
    count ++;
  }else {  // m_image
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




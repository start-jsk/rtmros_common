// -*- C++ -*-
/*!
 * @file  PointCloudROSBridge.cpp * @brief openhrp image - ros bridge * $Date$ 
 *
 * $Id$ 
 */
#include "PointCloudROSBridge.h"

// Module specification
// <rtc-template block="module_spec">
static const char* imagesensorrosbridge_spec[] =
  {
    "implementation_id", "PointCloudROSBridge",
    "type_name",         "PointCloudROSBridge",
    "description",       "hrpsys pointcloud data - ros bridge",
    "version",           "1.0",
    "vendor",            "JSK",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "+",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

PointCloudROSBridge::PointCloudROSBridge(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_pointsIn("points", m_points)
{
}

PointCloudROSBridge::~PointCloudROSBridge()
{
}


RTC::ReturnCode_t PointCloudROSBridge::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("points", m_pointsIn);

  // Set OutPort buffer

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  points_pub = node.advertise<sensor_msgs::PointCloud2>("points", 1);

  // initialize
  ROS_INFO_STREAM("[PointCloudROSBridge] @Initilize name : " << getInstanceName());

  pair_id = 0;
  ros::param::param<std::string>("~frame_id", _frame_id, "camera");
  ros::param::param<bool>("~publish_depth", publish_depth, false);
  // OpenHRP3 camera, front direction is -Z axis, ROS camera is Z axis
  // http://www.openrtp.jp/openhrp3/jp/create_model.html
  // transformed_camera_frame should be true, when sensor frame has rotated
  ros::param::param<bool>("~transformed_camera_frame", transformed_frame, false);

  if(publish_depth) {
    depth_image_pub = node.advertise<sensor_msgs::Image>("depth", 1);
  }

  tm.tick();
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t PointCloudROSBridge::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t PointCloudROSBridge::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t PointCloudROSBridge::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t PointCloudROSBridge::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t PointCloudROSBridge::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t PointCloudROSBridge::onExecute(RTC::UniqueId ec_id)
{
  // capture_time = ros::Time::now();

  // m_image
  if (m_pointsIn.isNew()){

    ROS_DEBUG_STREAM("[" << getInstanceName() << "] @onExecute ec_id : " << ec_id << ", image:" << m_pointsIn.isNew ());

    m_pointsIn.read();

    sensor_msgs::PointCloud2 pc;
    pc.header.frame_id = _frame_id;
    pc.header.stamp = ros::Time(m_points.tm.sec, m_points.tm.nsec);
    pc.header.seq = pair_id; pair_id++;

    pc.height       = m_points.height;
    pc.width        = m_points.width;
    pc.is_dense     = m_points.is_dense;
    pc.point_step   = m_points.point_step;
    pc.row_step     = m_points.row_step;
    pc.is_bigendian = m_points.is_bigendian;
#if 0
    pc.fields.resize(m_points.fields.length());
    for(int i=0; i < m_points.fields.length(); i++) {
      pc.fields[i].name      = m_points.fields[i].name;
      pc.fields[i].offset    = m_points.fields[i].offset;
      pc.fields[i].datatype  = m_points.fields[i].data_type + 1;
      pc.fields[i].count     = 1; // m_points.fields[i].count;
    }
#endif
    // now this is very dirty code..., expected point with rgb, and without normal
    pc.fields.resize(4);
    pc.fields[0].name      = "x";
    pc.fields[0].offset    = 0;
    pc.fields[0].datatype  = sensor_msgs::PointField::FLOAT32;
    pc.fields[0].count     = 1;
    pc.fields[1].name      = "y";
    pc.fields[1].offset    = 4;
    pc.fields[1].datatype  = sensor_msgs::PointField::FLOAT32;
    pc.fields[1].count     = 1;
    pc.fields[2].name      = "z";
    pc.fields[2].offset    = 8;
    pc.fields[2].datatype  = sensor_msgs::PointField::FLOAT32;
    pc.fields[2].count     = 1;
    pc.fields[3].name      = "rgb";
    pc.fields[3].offset    = 12;
    pc.fields[3].datatype  = sensor_msgs::PointField::FLOAT32;
    pc.fields[3].count     = 1;

    if (transformed_frame) {
      // now this is very dirty code..., expected point with rgb, and without normal
      pc.data.resize(m_points.data.length());
      float *src_ptr = (float *)m_points.data.get_buffer();
      float *dst_ptr = (float *)pc.data.data();
      for(int n = 0; n < m_points.data.length() / (4 * sizeof(float)); n++) {
        *dst_ptr++ = src_ptr[0]; //x
        *dst_ptr++ = - src_ptr[1]; //y
        *dst_ptr++ = - src_ptr[2]; //z
        char *rgb  = (char *)((float *)&src_ptr[3]);
        char *nrgb = (char *)dst_ptr; dst_ptr++;
        nrgb[0] = rgb[2];
        nrgb[1] = rgb[1];
        nrgb[2] = rgb[0];
        nrgb[3] = 0;
        src_ptr += 4;
      }
    } else {
      // now this is very dirty code..., expected point with rgb, and without normal
      pc.data.resize(m_points.data.length());
      std::copy(m_points.data.get_buffer(),
                m_points.data.get_buffer() + m_points.data.length(),
                pc.data.begin());
    }

#if 0
    pc.data.resize(pc.row_step * pc.height);
    char *src_ptr = (char *)m_points.data.get_buffer();
    float *ptr = (float *)pc.data.data();
    for(int y = 0; y < pc.height; y++) {
      for(int x = 0; x < pc.width; x++) {
        float *fsrc = (float *)(src_ptr + m_points.point_step * x + m_points.row_step * y);
        *ptr++ = *fsrc++;
        *ptr++ = *fsrc++;
        *ptr++ = *fsrc++;
        *ptr++;
      }
    }
#endif

    points_pub.publish(pc);

    if (publish_depth) {
      sensor_msgs::Image im;
      im.header.frame_id = _frame_id;
      im.header.stamp = ros::Time(m_points.tm.sec, m_points.tm.nsec);
      im.header.seq = pair_id - 1;

      im.height = m_points.height;
      im.width = m_points.width;
      im.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      im.step  = m_points.width * 4;
      int z_offset = -1;
      for(int i=0; i < m_points.fields.length(); i++) {
        if(strncmp(m_points.fields[i].name,"z",1) == 0) {
          z_offset = m_points.fields[i].offset;
          //m_points.fields[i].data_type;
          //m_points.fields[i].count;
          break;
        }
      }
      int invert = 1;
      if(transformed_frame) invert = -1;
      if(z_offset >= 0) {
        int p_step = m_points.point_step;
        int r_step = m_points.row_step;

        im.data.resize(im.step * im.height);
        float* dst_ptr = (float *)im.data.data();
        char* src_ptr = (char *)m_points.data.get_buffer();
        for(int y = 0; y < im.height; y++) {
          unsigned int pos = r_step * y;
          for (int x = 0; x < im.width; x++) {
            *dst_ptr++ = invert * *(float *)(&src_ptr[pos + (x * p_step) + z_offset]);
          }
        }
        depth_image_pub.publish(im);
      }
    }
#if 0
    std::cerr << "width: " <<  m_points.width << std::endl;
    std::cerr << "height: " << m_points.height << std::endl;
    std::cerr << "type: " << m_points.type << std::endl;
    std::cerr << "field size: " << m_points.fields.length() << std::endl;
    for(int i=0; i < m_points.fields.length(); i++) {
      std::cerr << " fname: "  << m_points.fields[i].name << std::endl;
      std::cerr << " offset: " << m_points.fields[i].offset << std::endl;
      std::cerr << " type: " << m_points.fields[i].data_type << std::endl;
      std::cerr << " count: " << m_points.fields[i].count << std::endl;
    }
    std::cerr << "big_endian: " << m_points.is_bigendian << std::endl;
    std::cerr << "point_step: " << m_points.point_step << std::endl;
    std::cerr << "row_step: " << m_points.row_step << std::endl;
    std::cerr << "is_dense: " << m_points.is_dense << std::endl;
    std::cerr << "size of data: " << m_points.data.length() << std::endl;
#endif
    //
    static int count = 0;
    tm.tack();
    if ( tm.interval() > 1 ) {
      ROS_INFO_STREAM("[" << getInstanceName() << "] @onExecutece " << ec_id << " is working at " << count << "[Hz]");
      tm.tick();
      count = 0;
    }
    count ++;
  } else {  // m_points
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
RTC::ReturnCode_t PointCloudROSBridge::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t PointCloudROSBridge::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t PointCloudROSBridge::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t PointCloudROSBridge::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t PointCloudROSBridge::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void PointCloudROSBridgeInit(RTC::Manager* manager)
  {
    coil::Properties profile(imagesensorrosbridge_spec);
    manager->registerFactory(profile,
                             RTC::Create<PointCloudROSBridge>,
                             RTC::Delete<PointCloudROSBridge>);
  }
  
};


#include <ros/ros.h>

// Services
#include "laser_assembler/AssembleScans2.h"

// Messages
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/JointState.h"

/***
 * This uses the point_cloud_assembler's build_cloud service call to grab all the scans/clouds between two tilt-laser shutters
 * params
 *  * "~target_frame_id" (string) - This is the frame that the scanned data transformed into.  The
 *                                  output clouds are also published in this frame.
 *  * "~num_skips" (int)          - If set to N>0, then the snapshotter will skip N signals before
 *                                  requesting a new snapshot. This will make the snapshots be N times
 *                                  larger. Default 0 - no skipping.
 */

namespace atlas_laser_snapshotter
{

class AtlasLaserSnapshotter
{

public:
  ros::NodeHandle n_;
  ros::NodeHandle private_ns_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  ros::Time prev_signal_;
  double prev_angle_;

  bool first_time_;

  int num_skips_;
  int num_skips_left_;

  std::string fixed_frame_;

  AtlasLaserSnapshotter() : private_ns_("~")
  {
    prev_signal_.fromNSec(0);

    pub_ = n_.advertise<sensor_msgs::PointCloud2> ("full_cloud2", 1);
    sub_ = n_.subscribe("joint_states", 40, &AtlasLaserSnapshotter::scannerSignalCallback, this);

    private_ns_.param("num_skips", num_skips_, 0);
    num_skips_left_=num_skips_;

    prev_angle_ = -1;
    first_time_ = true;
  }

  ~AtlasLaserSnapshotter()
  {

  }

  void scannerSignalCallback(const sensor_msgs::JointStateConstPtr& js)
  {

    double ang = fmod(js->position[0], 2 * M_PI);
    // ROS_DEBUG("ang = %lf, prev_angle = %lf, %lf", ang, prev_angle_, prev_signal_.toSec());

    if ( prev_angle_ < 0 ) {
      prev_angle_ = ang;
      return;
    }
    if ((ang - prev_angle_) >= - M_PI) {
      prev_angle_ = ang;
      return;
    }

    if (prev_signal_.toSec() == 0.0) {
      first_time_ = true;
    }

    ros::Time stmp = js->header.stamp;
    if (first_time_)
    {
      prev_signal_ = stmp;
      first_time_ = false;
    }
    else
    {
      if (num_skips_ > 0)
      {
        if (num_skips_left_ > 0)
        {
          num_skips_left_ -= 1;
          return;
        }
        else
        {
          num_skips_left_ = num_skips_;
        }
      }

      laser_assembler::AssembleScans2::Request req;
      laser_assembler::AssembleScans2::Response res;

      req.begin = prev_signal_;
      req.end   = stmp;

      if (!ros::service::call("assemble_scans2", req, res))
        ROS_ERROR("Failed to call service on point cloud assembler or laser scan assembler.");

      pub_.publish(res.cloud);
      ROS_INFO("Snapshotter::Published Cloud size=%u", res.cloud.width * res.cloud.height);

      prev_signal_ = stmp;
      prev_angle_ = -1;
    }
  }
};
}

using namespace atlas_laser_snapshotter;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "atlas_laser_snapshotter");
  AtlasLaserSnapshotter snapshotter ;
  ros::spin();
  return 0;
}

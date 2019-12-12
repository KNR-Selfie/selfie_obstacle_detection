#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <cstring>
#include <cmath>
#include <vector>

using sensor_msgs::LaserScan;
using sensor_msgs::LaserScanPtr;

namespace selfie_obstacle_detection{
class RemoveBackgroundNodelet
{
  float low_threshold_;
  float high_threshold_;
  float min_size_;
  float max_size_;

  ros::Subscriber sub_;
  ros::Publisher pub_;

//   virtual void onInit();
  public:
    RemoveBackgroundNodelet();
    void processScan(const LaserScanPtr& in_scan);
};  // class RemoveBackgroundNodelet
}
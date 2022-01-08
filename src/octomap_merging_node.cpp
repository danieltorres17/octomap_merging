#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "octomap_merging.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "octomap_merging");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // OctomapMerging* octomap_merging = new OctomapMerging(nh, pnh);
  // double rate = 5;
  // ros::Rate r(rate);
  // while (nh.ok()) {
  //   ros::spinOnce();
  //   r.sleep();
  // }

  OctomapMerging om(nh, pnh);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
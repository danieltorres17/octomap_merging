#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "octomap_merging.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "octomap_merging");
  ros::NodeHandle nh;

  double rate = 5;
  // ros::CallbackQueue queue;
  // nh.setCallbackQueue(&queue);
  OctomapMerging* octomap_merging = new OctomapMerging(nh);
  // OctomapMerging om(nh);

  ros::Rate r(rate);
  while (nh.ok()) {
    ros::spinOnce();
    r.sleep();
  }

  // ros::spinOnce();
  // ros::AsyncSpinner spinner(4);
  // spinner.start();
  // // ros::spin();
  // ros::waitForShutdown();

  return 0;
}
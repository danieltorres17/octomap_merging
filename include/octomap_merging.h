#ifndef OCTOMAP_MERGING_H_
#define OCTOMAP_MERGING_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap_ros/conversions.h>
#include <rough_octomap/RoughOcTree.h>
#include <rough_octomap/conversions.h>

#include <pcl/common/common.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/transformation_validation_euclidean.h>
#include <pcl_ros/transforms.h>

// pcl::SAC_SAMPLE_SIZE is protected since PCL 1.8.0
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pcl/sample_consensus/model_types.h>
#pragma GCC diagnostic pop

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <boost/thread.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "octomap_merging/OctomapArray.h"
#include "octomap_merging/OctomapNeighbors.h"

// typedefs
typedef pcl::PointXYZ pclPoint;
typedef pcl::PointCloud<pclPoint> pointCloud;
typedef octomap::RoughOcTree RoughOcTreeT;

class OctomapMerging {
public:
    OctomapMerging(ros::NodeHandle& nodehandle, ros::NodeHandle& private_nodehandle);
    ~OctomapMerging();

private:
    void myMapCallback(const octomap_msgs::OctomapConstPtr& msg);
    void myDiffsCallback(const octomap_merging::OctomapArrayConstPtr& msg);
    void neighborDiffsCallback(const octomap_merging::OctomapNeighborsConstPtr& msg);
    void tree2PointCloud(const RoughOcTreeT* tree, pointCloud& cloud);
    void updateNeighborMaps();
    void mergeNeighbors(const ros::TimerEvent& event);
    Eigen::Matrix4f alignMap(const RoughOcTreeT* tree);
    void mergeMap(RoughOcTreeT* tree);
    void transformMap(RoughOcTreeT* tree, const Eigen::Matrix4f& tf);
    Eigen::Matrix4f getGICPTransform(pointCloud &src, pointCloud& trg);
    double getSign(double x) { if (x < 0) return -1; else return 1; }
    void publishNeighborMaps(const ros::TimerEvent& event);

    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Subscriber myDiffsSub;
    ros::Subscriber neighborDiffsSub;
    ros::Publisher myMapPub;
    ros::Publisher tfTempTreePub;
    ros::Publisher mergedMapPub;
    ros::Publisher neighborMapPub;
    ros::Publisher diffMapPub;
    ros::Publisher temp_treePCPub;
    ros::Publisher m_octreePCPub;
    ros::Publisher transformedTreePCPub;
    ros::Publisher src_BBoxPCPub;
    ros::Publisher trg_BBoxPCPub;
    tf2_ros::Buffer tfBuffer;
    ros::Timer diff_timer;
    ros::Timer merge_timer;
    ros::Timer neighbor_map_pub;
    boost::mutex m_mtx;

    octomap_msgs::Octomap mapDiffs;
    octomap_merging::OctomapArray selfDiffs;
    octomap_merging::OctomapNeighbors neighborDiffs;

    RoughOcTreeT* m_octree; // self map
    RoughOcTreeT* m_merged_tree; // combination of self map and maps from other neighbors
    RoughOcTreeT* temp_tree; // temp tree - for merging purposes

    // octomap parameters
    std::string m_mapTopic;
    double m_resolution;
    bool m_pubSelfMap;
    bool m_pubMergedMap;
    std::string m_selfMapTopic;
    std::string m_selfMergedMapTopic;
    bool m_enableTraversability = false;
    bool m_enableTraversabilitySharing = false;
    bool m_diffMerged = false;
    double probHit = 0.95;
    double probMiss = 0.48;
    double thresMin = 0.12;
    double thresMax = 0.97;
    std::map<std::string, std::vector<int>> seqs;
    std::map<std::string, char> idx;

    // diff map parameters
    bool myDiffsNew;
    int diffTimer = 10;
    bool diffMerged = false;
    int diffThreshold = 1000;
    int m_mergeTimer;

    // outlier filter, downsampling and GICP parameters
    int meanK;
    double stdDevMulThresh;
    int leafScale;
    int icpIterations;
    int icpOptimizerIterations;
    int icpMaxCorrespDistScale;
    double icpTFEpsScale;
    int ransacNumIterations;
    double ransacOutlierRejecThresh;
    double euclideanFitnessEps;
    double m_fitnessScoreThreshold;
    double m_tveMaxRange;

    // Multi-agent map merging parameters
    std::string m_neighborsTopic;
    bool m_pubNeighborMaps;
    std::string m_neighborMapTopic;
    bool m_pubSelfMapPC;
    std::string m_selfMapPCTopic;
    bool m_pubTempTreePC;
    std::string m_tempTreePCTopic;
    bool m_pubTFTempTreePC;
    std::string m_tfTempTreePCTopic;
    bool m_pubTFTempTree;
    std::string m_tfTempTreeTopic;
    bool m_pubBBoxPC;
    std::string m_srcBBoxPCTopic;
    std::string m_trgBBoxPCTopic;
    int next_idx = 2;
    int m_mapMergeTimerDuration;
    int m_numDiffMap;
    std::vector<std::string> neighbors_list{"H02", "D01", "D02"};
    std::map<std::string, Eigen::Matrix4f> neighbor_tf;
    std::map<char, std::string> agent_map;
    std::map<std::string, bool> neighbor_updated;
    std::map<std::string, RoughOcTreeT*> neighbor_maps;
    std::map<std::string, ros::Publisher> neighbor_pubs;
    std::map<std::string, bool> neighbor_aligned;
    ros::Timer diff_merge_timer;
    ros::Timer pub_neighbor_maps_timer;
    bool m_neighborMapsMerged = false;
    int m_neighborMapPubTimer;
};


#endif 
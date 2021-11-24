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
#include <pcl/common/impl/angles.hpp>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/features/from_meshes.h>

#include <pcl/registration/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/correspondence.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/registration/transformation_validation_euclidean.h>
#include <pcl_ros/transforms.h>

// pcl::SAC_SAMPLE_SIZE is protected since PCL 1.8.0
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pcl/sample_consensus/model_types.h>
#pragma GCC diagnostic pop

#include <Eigen/Dense>

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

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

class OctomapMerging {
public:
    OctomapMerging(ros::NodeHandle& nodehandle);
    ~OctomapMerging();

private:
    void myMapCallback(const octomap_msgs::OctomapConstPtr& msg);
    void myDiffsCallback(const octomap_merging::OctomapArrayConstPtr& msg);
    void neighborDiffsCallback(const octomap_merging::OctomapNeighborsConstPtr& msg);
    pointCloud::Ptr statOutlierRemoval(pointCloud& inputCloud);
    void tree2PointCloud(RoughOcTreeT* tree, pointCloud& cloud);
    void mergeNeighbors(const ros::TimerEvent& event);
    void mergeMaps();
    Eigen::Matrix4f findTransform(Eigen::Matrix4f& tfEst, Eigen::Matrix4f& prev_tf);
    void transformTree(Eigen::Matrix4f& tf);
    Eigen::Matrix4f getICPTransform(pointCloud& src, pointCloud& trg, Eigen::Matrix4f& tfEst);
    Eigen::Matrix4f getGICPTransform(pointCloud &src, pointCloud& trg, Eigen::Matrix4f& tfEst);
    double getSign(double x) { if (x < 0) return -1; else return 1; }

    ros::NodeHandle nh;
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
    boost::mutex m_mtx;

    octomap_msgs::Octomap mapDiffs;
    octomap_merging::OctomapArray selfDiffs;
    octomap_merging::OctomapNeighbors neighborDiffs;

    RoughOcTreeT* m_octree;
    RoughOcTreeT* m_diff_tree;
    RoughOcTreeT* m_merged_tree; 
    RoughOcTreeT* temp_tree; // for neighbor diff map concatenation

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
    double radiusSearchScale;
    int minNumberNeighbors;
    int leafScale;
    int gicpIterations;
    int gicpOptimizerIterations;
    int gicpMaxCorrespDistScale;
    double gicpTFEpsScale;
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
    std::map<std::string, Eigen::Matrix4f> last_tf;
    std::map<char, std::string> agent_map;
    std::map<std::string, bool> neighbor_updated;
    std::map<std::string, RoughOcTreeT*> neighbor_maps;
    ros::Timer diff_merge_timer;
};


#endif 
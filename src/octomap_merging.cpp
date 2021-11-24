#include "octomap_merging.h"

OctomapMerging::OctomapMerging(ros::NodeHandle& nodehandle)
: nh(nodehandle) {
  // Initialize parameters from launch file
  nh.param("resolution", m_resolution, 0.15);
  nh.param("map_merge_timer", m_mergeTimer, 300);
  nh.param("pub_self_map", m_pubSelfMap, true);
  nh.param("self_map_topic", m_selfMapTopic, m_selfMapTopic);
  nh.param("pub_self_merged_map", m_pubMergedMap, true);
  nh.param("self_merged_map_topic", m_selfMergedMapTopic, m_selfMergedMapTopic);
  nh.param("pub_neighbor_maps", m_pubNeighborMaps, false);
  nh.param("neighbor_map_topic", m_neighborMapTopic, m_neighborMapTopic);
  nh.param("pub_tf_temp_tree", m_pubTFTempTree, false);
  nh.param("tf_temp_tree_topic", m_tfTempTreeTopic, m_tfTempTreeTopic);
  nh.param("diff_map_in", m_mapTopic, m_mapTopic);
  nh.param("neighbor_diffs_in", m_neighborsTopic, m_neighborsTopic);
  nh.param("pub_self_map_PC", m_pubSelfMapPC, false);
  nh.param("self_map_PC_topic", m_selfMapPCTopic, m_selfMapPCTopic);
  nh.param("pub_temp_tree_PC", m_pubTempTreePC, false);
  nh.param("temp_tree_PC_topic", m_tempTreePCTopic, m_tempTreePCTopic);
  nh.param("pub_tf_temp_tree_PC", m_pubTFTempTreePC, false);
  nh.param("tf_temp_tree_PC_topic", m_tfTempTreePCTopic, m_tfTempTreePCTopic);
  nh.param("pub_bbox_intersect_PCs", m_pubBBoxPC, false);
  nh.param("src_bbox_PC_topic", m_srcBBoxPCTopic, m_srcBBoxPCTopic);
  nh.param("trg_bbox_PC_topic", m_trgBBoxPCTopic, m_trgBBoxPCTopic);

  // initialize m_octree (system's octomap)
  m_octree = new RoughOcTreeT(m_resolution);
  m_octree->setRoughEnabled(m_enableTraversability);
  m_octree->setProbHit(probHit);
  m_octree->setProbMiss(probMiss);
  m_octree->setClampingThresMin(thresMin);
  m_octree->setClampingThresMax(thresMax);

  // initialize m_merged_tree
  m_merged_tree = new RoughOcTreeT(m_resolution);
  m_merged_tree->setRoughEnabled(m_enableTraversability);
  m_merged_tree->setProbHit(probHit);
  m_merged_tree->setProbMiss(probMiss);
  m_merged_tree->setClampingThresMin(thresMin);
  m_merged_tree->setClampingThresMax(thresMax);

  // initialize temp_tree for neighbor diff map merging
  temp_tree = new RoughOcTreeT(m_resolution);

  // Subscribers
  myDiffsSub = nh.subscribe(m_mapTopic, 100, &OctomapMerging::myMapCallback, this);
  neighborDiffsSub = nh.subscribe(m_neighborsTopic, 100, &OctomapMerging::neighborDiffsCallback, this);

  // Publishers
  myMapPub = nh.advertise<octomap_msgs::Octomap>(m_selfMapTopic, 5, true);
  mergedMapPub = nh.advertise<octomap_msgs::Octomap>(m_selfMergedMapTopic, 5, true);
  neighborMapPub = nh.advertise<octomap_msgs::Octomap>(m_neighborMapTopic, 5, true);
  tfTempTreePub = nh.advertise<octomap_msgs::Octomap>(m_tfTempTreeTopic, 5, true);

  m_octreePCPub = nh.advertise<sensor_msgs::PointCloud2>(m_selfMapPCTopic, 5, true);
  temp_treePCPub = nh.advertise<sensor_msgs::PointCloud2>(m_tempTreePCTopic, 5, true);
  transformedTreePCPub = nh.advertise<sensor_msgs::PointCloud2>(m_tfTempTreePCTopic, 5, true);
  src_BBoxPCPub = nh.advertise<sensor_msgs::PointCloud2>(m_srcBBoxPCTopic, 5, true);
  trg_BBoxPCPub = nh.advertise<sensor_msgs::PointCloud2>(m_trgBBoxPCTopic, 5, true);

  // Timer events
  // diff_timer = nh.createTimer(ros::Duration(diffTimer), &OctomapMerging::updateDiff, this);
  merge_timer = nh.createTimer(ros::Duration(m_mergeTimer), &OctomapMerging::mergeNeighbors, this);

  // get outlier filter, downsampling and GICP parameters from config file
  nh.getParam("meanK", meanK);
  nh.getParam("stdDevMulThresh", stdDevMulThresh);
  nh.getParam("radiusSearchScale", radiusSearchScale);
  nh.getParam("minNumberNeighbors", minNumberNeighbors);
  nh.getParam("leafSizeScaling", leafScale);
  nh.getParam("gicp_iterations", gicpIterations);
  nh.getParam("optimizer_iterations", gicpOptimizerIterations);
  nh.getParam("max_correspondence_dist_scale", gicpMaxCorrespDistScale);
  nh.getParam("transformation_eps_scale", gicpTFEpsScale);
  nh.getParam("ransacOutlierRejecThresh", ransacOutlierRejecThresh);
  nh.getParam("ransacNumberIterations", ransacNumIterations);
  nh.getParam("eucFitnessEps", euclideanFitnessEps);
  nh.getParam("fitnessScoreThreshold", m_fitnessScoreThreshold);
  nh.getParam("tfValidationEuclideanMaxRange", m_tveMaxRange);

  // initialize tf_estimate for each neighbor with identity matrix
  Eigen::Matrix4f tfEst = Eigen::Matrix4f::Identity(); 
  for (const auto n : neighbors_list){
    last_tf[n] = tfEst;
  }
}

OctomapMerging::~OctomapMerging()
{
  if (m_octree) {
    delete m_octree;
    m_octree = NULL;
  }

  if (m_diff_tree) {
    delete m_diff_tree;
    m_diff_tree = NULL;
  }

  if (m_merged_tree) {
    delete m_merged_tree;
    m_merged_tree = NULL;
  }

  if (temp_tree) {
    delete temp_tree;
    temp_tree = NULL;
  }
}

void OctomapMerging::myMapCallback(const octomap_msgs::OctomapConstPtr& msg)
{
  // boost::mutex::scoped_lock lock(m_mtx);
  // mapDiffs = *msg;
  delete m_octree; 
  m_octree = (RoughOcTreeT*)octomap_msgs::binaryMsgToMap(*msg);

  if (m_pubSelfMap) {
    boost::mutex::scoped_lock lock(m_mtx);
    m_octree->prune();
    octomap_msgs::Octomap map_msg;
    octomap_msgs::binaryMapToMsg(*m_octree, map_msg);
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = "world";
    myMapPub.publish(map_msg);
  }
  m_octree->resetChangeDetection();
}

void OctomapMerging::neighborDiffsCallback(const octomap_merging::OctomapNeighborsConstPtr& msg)
{
  neighborDiffs = *msg;
  bool invalidMessage = (neighborDiffs.num_neighbors != neighborDiffs.neighbors.size());
  if (!invalidMessage) {
    for (int i = 0; i < msg->neighbors.size(); i++) {
      if (neighborDiffs.neighbors[i].num_octomaps != neighborDiffs.neighbors[i].octomaps.size())
        invalidMessage = true;
    }
  }

  if (invalidMessage) {
    ROS_WARN("Received invalid neighbor maps");
    neighborDiffs.hardReset = false;
    neighborDiffs.clear = false;
  }

  if (neighborDiffs.hardReset) {
    ROS_INFO("Hard resetting map due to request.");
    boost::mutex::scoped_lock lock(m_mtx);
    m_octree->clear();
    m_merged_tree->clear();
    if (m_diffMerged)
      m_merged_tree->resetChangeDetection();
    else
      m_octree->resetChangeDetection();
    neighborDiffs = octomap_merging::OctomapNeighbors();
  }

  if (m_pubNeighborMaps) {
    boost::mutex::scoped_lock lock(m_mtx);
    octomap_msgs::Octomap neighbor_map;
    octomap_msgs::binaryMapToMsg(*m_merged_tree, neighbor_map);
    neighbor_map.header.stamp = ros::Time::now();
    neighbor_map.header.frame_id = "world";
    neighborMapPub.publish(neighbor_map);
  }
}

void OctomapMerging::mergeNeighbors(const ros::TimerEvent& event) 
{
  if (neighborDiffs.neighbors.empty()) {
    ROS_INFO("No neighbor maps available");
    return;
  }
  std::shared_ptr<RoughOcTreeT> ntree(nullptr);
  bool remerge;
  char agent;
  for (int i = 0; i < neighborDiffs.num_neighbors; i++) {
    std::string nid = neighborDiffs.neighbors[i].owner;
    if (nid == "H01") {
      ROS_INFO("Won't merge self map");
      continue;
    }
    ROS_INFO("Processing maps for neighbor: %s", nid.c_str());
    // If clear passed then re-merge everything
    if (neighborDiffs.clear) {
      if (i == 0) {
        boost::mutex::scoped_lock lock(m_mtx);
        // First time through clear the merged tree, and reset change detection if necessary
        m_merged_tree->clear();

        // Copy the self map to the merged map
        for (RoughOcTreeT::iterator it = m_octree->begin(), end = m_octree->end(); it != end; ++it) {
          octomap::point3d point = it.getCoordinate();
          octomap::RoughOcTreeNode* newNode = m_merged_tree->setNodeValue(point, it->getLogOdds());
          newNode->setAgent(1);
          if (m_enableTraversability)
            newNode->setRough(it->getRough());
        }
      if (m_diffMerged)
        m_merged_tree->resetChangeDetection();
      }
      // For each neighbor, clear the diffs
      seqs[nid.data()].clear();
      neighbor_maps[nid.data()] = NULL;
    }
    remerge = false;
    // Check each diff for new ones to merge
    // Array should be in sequence order to allow re-merging if a diff comes out of sequence
    // In theory should be able to just skip nodes in out-of-sequence diffs,
    // but doesn't always work as expected due to various node sizes
    ROS_INFO("%s number of diff maps: %i", nid.c_str(), neighborDiffs.neighbors[i].octomaps.size());
    for (int j = 0; j < neighborDiffs.neighbors[i].octomaps.size(); j++) {
      // ROS_INFO("%s has %i maps", nid.c_str(), neighborDiffs.neighbors[i].octomaps.size());
      uint32_t cur_seq = neighborDiffs.neighbors[i].octomaps[j].header.seq;
      bool exists = std::count(seqs[nid.data()].cbegin(), seqs[nid.data()].cend(), cur_seq);
      // Only merge if we haven't already merged this sequence, or we got an out-of-sequence diff
      if (!exists || remerge) {
        // Add to our list of sequences, and get a unique agent id for this neighbor
        if (!exists) 
          seqs[nid.data()].push_back(cur_seq);
        if (idx[nid.data()]) 
          agent = idx[nid.data()];
        else {
          idx[nid.data()] = next_idx;
          agent = next_idx;
          next_idx++;
        }
        // map agent char to agent string
        agent_map[agent] = nid.data();
        // Check if this is an out-of-sequence diff, and force remerge of later sequences we have
        remerge = cur_seq < *std::max_element(seqs[nid.data()].cbegin(), seqs[nid.data()].cend());
        // if binary tree
        if (neighborDiffs.neighbors[i].octomaps[j].binary) {
          // Backwards compatibility for regular OcTrees
          if (neighborDiffs.neighbors[i].octomaps[j].id.find("RoughOcTree-") == std::string::npos)
            neighborDiffs.neighbors[i].octomaps[j].id = "RoughOcTree-0";
          ntree = std::shared_ptr<RoughOcTreeT>(dynamic_cast<RoughOcTreeT*>
                    (octomap_msgs::binaryMsgToMap(neighborDiffs.neighbors[i].octomaps[j])));
        }
        // if full tree
        else {
          ntree = std::shared_ptr<RoughOcTreeT>(dynamic_cast<RoughOcTreeT*>
                    (octomap_msgs::fullMsgToMap(neighborDiffs.neighbors[i].octomaps[j])));
        }
        // add reconstructed diff tree into temp_tree
        ntree->expand();
        boost::mutex::scoped_lock lock(m_mtx);
        for (RoughOcTreeT::iterator it = ntree->begin(), end = ntree->end(); it != end; ++it) {
          octomap::point3d point = it.getCoordinate();
          octomap::RoughOcTreeNode* newNode = temp_tree->setNodeValue(point, it->getLogOdds());
          newNode->setAgent(agent);
        }
      }
    }
    // if diff maps from neighbor aren't bigger than diff threshold, move on
    if (temp_tree->size() < diffThreshold) {
      ROS_INFO("temp_tree is not large enough to merge. Moving to next neighbor");
      temp_tree->clear();
      continue;
    }
    // initialize tfEst to identity matrix
    Eigen::Matrix4f tfEst = Eigen::Matrix4f::Identity();
    // Apply last tf saved from previous alignment before running merge
    Eigen::Matrix4f prev_tf = last_tf[nid];
    // std::cout << "tf estimate to initialize gicp: " << tfEst << std::endl;
    Eigen::Matrix4f est_tf = findTransform(tfEst, prev_tf);
    std::cout << "estimated tf: " << est_tf << std::endl;
    if (est_tf.isIdentity()) {
      ROS_INFO("Over fitness score threshold set. Storing map, trying again next time around");
      
    }
    // save last transformation found for agent
    last_tf[nid.data()] = est_tf;
    // transform temp_tree
    transformTree(est_tf);

    // publish transformed tree
    if (m_pubTFTempTree) {
      boost::mutex::scoped_lock lock(m_mtx);
      temp_tree->prune();
      octomap_msgs::Octomap tf_temp_tree_out;
      octomap_msgs::binaryMapToMsg(*temp_tree, tf_temp_tree_out);
      tf_temp_tree_out.header.stamp = ros::Time::now();
      tf_temp_tree_out.header.frame_id = "world";
      tfTempTreePub.publish(tf_temp_tree_out);
    }
    // merge temp tree into merged_map
    mergeMaps();
    // clear temp tree
    temp_tree->clear();
  }
  // reset neighbor maps message
  neighborDiffs = octomap_merging::OctomapNeighbors();
}

void OctomapMerging::mergeMaps()
{
  // boost::mutex::scoped_lock lock(m_mtx);
  m_octree->expand();
  for (RoughOcTreeT::iterator it = m_octree->begin(), end = m_octree->end(); it != end; ++it) {
    octomap::point3d pt = it.getCoordinate();
    octomap::RoughOcTreeNode *newNode = m_merged_tree->updateNode(pt, it->getLogOdds());
  }
  // Iterate through tree and merge
  temp_tree->expand();
  for (RoughOcTreeT::iterator it = temp_tree->begin(), end = temp_tree->end(); it != end; ++it) {
    octomap::OcTreeKey nodeKey = it.getKey();
    octomap::RoughOcTreeNode* nodeM = m_merged_tree->search(nodeKey);
    octomap::RoughOcTreeNode* nodeCurr = temp_tree->search(nodeKey);
    char agent = nodeCurr->getAgent();
    if (nodeM != NULL) {
      // Ignore any nodes that are self nodes
      if (nodeM->getAgent() != 1) {
        if (nodeM->getAgent() == agent) {
          // If the diff is newer, and the merged map node came from this neighbor
          // replace the value and maintain the agent id
          m_merged_tree->setNodeValue(nodeKey, it->getLogOdds());
          if (m_enableTraversabilitySharing)
            nodeM->setRough(it->getRough());
        }
        else {
          // If there's already a node in the merged map, but it's from another neighbor,
          // or it's been previously merged, merge the value, and set agent to merged
          m_merged_tree->updateNode(nodeKey, it->getLogOdds());
          nodeM->setAgent(0);
          if (m_enableTraversabilitySharing)
            m_merged_tree->integrateNodeRough(nodeKey, it->getRough());
        }
      }
    }
    else {
      // If the node doesn't exist in the merged map, add it with the value
      octomap::RoughOcTreeNode* newNode = m_merged_tree->setNodeValue(nodeKey, it->getLogOdds());
      newNode->setAgent(agent);
      if (m_enableTraversabilitySharing)
        newNode->setRough(it->getRough());
    }
    std::string nid = agent_map[agent];
    // If neighbor maps enabled, add the node if it's new (or needs remerge)
    if (m_pubNeighborMaps) {
      neighbor_updated[nid.data()] = true;
      octomap::RoughOcTreeNode* newNode = neighbor_maps[nid.data()]->setNodeValue(nodeKey, it->getLogOdds());
      if (m_enableTraversabilitySharing)
        newNode->setRough(it->getRough());
    }
  }
  // lock.unlock();

  if (m_pubMergedMap) {
    boost::mutex::scoped_lock lock(m_mtx);
    m_merged_tree->prune();
    octomap_msgs::Octomap merged_map_msg;
    octomap_msgs::binaryMapToMsg(*m_merged_tree, merged_map_msg);
    merged_map_msg.header.stamp = ros::Time::now();
    merged_map_msg.header.frame_id = "world";
    mergedMapPub.publish(merged_map_msg);
  }
  ROS_INFO("mergeMaps completed successfully\n");
}

void OctomapMerging::tree2PointCloud(RoughOcTreeT* tree, pointCloud& pc)
{
  for (RoughOcTreeT::iterator it = tree->begin(), end = tree->end(); it != end; ++it) {
    if (tree->isNodeOccupied(*it))
      pc.push_back(pclPoint(it.getX(), it.getY(), it.getZ()));
  }
}

Eigen::Matrix4f OctomapMerging::findTransform(Eigen::Matrix4f& tfEst, Eigen::Matrix4f& prev_tf)
{
  // get temp_tree point cloud
  pointCloud temp_treePC;
  // boost::mutex::scoped_lock lock(m_mtx);
  temp_tree->prune();
  tree2PointCloud(temp_tree, temp_treePC);
  // lock.unlock();

  // get merged_map point cloud
  pointCloud m_octreePC;
  // lock.lock();
  m_octree->prune();
  for (RoughOcTreeT::iterator it = m_octree->begin(), end = m_octree->end(); it != end; ++it) {
    octomap::point3d pt = it.getCoordinate();
    octomap::RoughOcTreeNode *newNode = m_merged_tree->updateNode(pt, it->getLogOdds());
  }
  tree2PointCloud(m_merged_tree, m_octreePC);
  // lock.unlock();

  Eigen::Matrix4f transform = getGICPTransform(temp_treePC, m_octreePC, prev_tf);
  
  if (m_pubSelfMapPC) {
    // publish m_octree point cloud
    sensor_msgs::PointCloud2 m_octreePC_out;
    pcl::toROSMsg(m_octreePC, m_octreePC_out);
    m_octreePC_out.header.frame_id = "world";
    m_octreePC_out.header.stamp = ros::Time::now();
    m_octreePCPub.publish(m_octreePC_out);
  }

  if (m_pubTempTreePC) {
    // Publish temp_tree point cloud
    sensor_msgs::PointCloud2 temp_treePC_out;
    pcl::toROSMsg(temp_treePC, temp_treePC_out);
    temp_treePC_out.header.frame_id = "world";
    temp_treePC_out.header.stamp = ros::Time::now();
    temp_treePCPub.publish(temp_treePC_out);
  }

  if (m_pubTFTempTreePC) {
    // publish transformed temp_tree cloud
    pointCloud tf_points;
    pcl::transformPointCloud(temp_treePC, tf_points, transform);
    sensor_msgs::PointCloud2 tf_points_msg;
    pcl::toROSMsg(tf_points, tf_points_msg);
    tf_points_msg.header.frame_id = "world";
    tf_points_msg.header.stamp = ros::Time::now();
    transformedTreePCPub.publish(tf_points_msg);
  }

  return transform;
}

Eigen::Matrix4f OctomapMerging::getICPTransform(pointCloud& source, pointCloud& target, Eigen::Matrix4f& tfEst) 
{
  // ROS_INFO("In GICP function");
  // find min and max points of src point cloud
  pclPoint srcMin, srcMax;
  pcl::getMinMax3D(source, srcMin, srcMax);
  pointCloud::Ptr src(new pointCloud);

  // find min and max points of trg point cloud
  pclPoint trgMin, trgMax;
  pcl::getMinMax3D(target, trgMin, trgMax);
  pointCloud::Ptr trg(new pointCloud);

  // find all points in src that are within trg bounds
  for (pointCloud::iterator it = source.begin(); it != source.end(); ++it) {
    if ((trgMin.x < it->x && trgMin.y < it->y && trgMin.z < it->z) && 
        (trgMax.x > it->x && trgMax.y > it->y && trgMax.z > it->z))
          src->push_back(*it);
  }
  if (m_pubBBoxPC) {
    // publish src intersecting points
    sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(*src, cloud_out);
    cloud_out.header.frame_id = "world";
    cloud_out.header.stamp = ros::Time::now();
    src_BBoxPCPub.publish(cloud_out);
  }

  // find all points in trg that are within src bounds
  for (pointCloud::iterator it = target.begin(); it != target.end(); ++it) {
    if ((srcMin.x < it->x && srcMin.y < it->y && srcMin.z < it->z) && 
        (srcMax.x > it->x && srcMax.y > it->y && srcMax.z > it->z))
          trg->push_back(*it);
  }
  if (m_pubBBoxPC) {
    // publish src intersecting points
    sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(*trg, cloud_out);
    cloud_out.header.frame_id = "world";
    cloud_out.header.stamp = ros::Time::now();
    trg_BBoxPCPub.publish(cloud_out);
  }

  // Downsample
  pcl::VoxelGrid<pclPoint> grid;
  grid.setLeafSize(leafScale * m_resolution, leafScale * m_resolution, leafScale * m_resolution);
  // downsample src cloud
  grid.setInputCloud(src);
  grid.filter(*src);
  // downsample trg cloud
  grid.setInputCloud(trg);
  grid.filter(*trg);

  // begin registration
  pcl::IterativeClosestPointNonLinear<pclPoint, pclPoint> icp;
  icp.setMaximumIterations(gicpOptimizerIterations);
  icp.setMaxCorrespondenceDistance(gicpMaxCorrespDistScale * m_resolution);
  icp.setTransformationEpsilon(gicpTFEpsScale);
  icp.setEuclideanFitnessEpsilon(euclideanFitnessEps);
  icp.setRANSACIterations(ransacNumIterations);
  icp.setRANSACOutlierRejectionThreshold(ransacOutlierRejecThresh * m_resolution);

  // setup output tf and cloud  
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f prev;
  Eigen::Matrix4f trgToSrcTF;
  pointCloud::Ptr icp_result;

  if (src->size() < trg->size()) 
  {
    icp.setInputSource(src);
    icp.setInputTarget(trg);
    icp_result = src;
    icp.align(*icp_result);
    Ti = icp.getFinalTransformation();
  }
  else
  {
    icp.setInputSource(trg);
    icp.setInputTarget(src);
    icp_result = trg;
    icp.align(*icp_result);
    Ti = icp.getFinalTransformation();
    Ti = Ti.inverse();
  }
  // get the transformation from target to source
  trgToSrcTF = Ti;
  // get transformation fitness score
  double fs = icp.getFitnessScore();
  // get translation and angles from transformation
  Eigen::Affine3f tf_estimated(trgToSrcTF);
  float x, y, z, roll, pitch, yaw;
  pcl::getTranslationAndEulerAngles(tf_estimated, x, y, z, roll, pitch, yaw);
  Eigen::Vector3f translation(x, y, z);
  ROS_INFO("Translation norm: %f", translation.squaredNorm());
  ROS_INFO("Roll: %f, pitch: %f, yaw: %f", pcl::rad2deg(roll), pcl::rad2deg(pitch), pcl::rad2deg(yaw));

  ROS_INFO("GICP fitness score: %f", fs);
  if (fs > m_fitnessScoreThreshold)
    ROS_INFO("GICP Fitness Score above threshold. Storing map and will reattempt merge next time around.");
  // check for GICP convergence 
  if (icp.hasConverged())
    ROS_INFO("GICP converged");
  else
    ROS_INFO("GICP did not converge");

  // clear filtered PCs
  src->clear();
  trg->clear();
  
  return trgToSrcTF;
}

Eigen::Matrix4f OctomapMerging::getGICPTransform(pointCloud& source, pointCloud& target, Eigen::Matrix4f& tfEst)
{
  // ROS_INFO("In GICP function");
  // find min and max points of src point cloud
  pclPoint srcMin, srcMax;
  pcl::getMinMax3D(source, srcMin, srcMax);
  pointCloud::Ptr src(new pointCloud);

  // find min and max points of trg point cloud
  pclPoint trgMin, trgMax;
  pcl::getMinMax3D(target, trgMin, trgMax);
  pointCloud::Ptr trg(new pointCloud);

  // find all points in src that are within trg bounds
  for (pointCloud::iterator it = source.begin(); it != source.end(); ++it) {
    if ((trgMin.x < it->x && trgMin.y < it->y && trgMin.z < it->z) && 
        (trgMax.x > it->x && trgMax.y > it->y && trgMax.z > it->z))
          src->push_back(*it);
  }
  if (m_pubBBoxPC) {
    // publish src intersecting points
    sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(*src, cloud_out);
    cloud_out.header.frame_id = "world";
    cloud_out.header.stamp = ros::Time::now();
    src_BBoxPCPub.publish(cloud_out);
  }

  // find all points in trg that are within src bounds
  for (pointCloud::iterator it = target.begin(); it != target.end(); ++it) {
    if ((srcMin.x < it->x && srcMin.y < it->y && srcMin.z < it->z) && 
        (srcMax.x > it->x && srcMax.y > it->y && srcMax.z > it->z))
          trg->push_back(*it);
  }
  if (m_pubBBoxPC) {
    // publish src intersecting points
    sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(*trg, cloud_out);
    cloud_out.header.frame_id = "world";
    cloud_out.header.stamp = ros::Time::now();
    trg_BBoxPCPub.publish(cloud_out);
  }
  
  // Statistical outlier filter to remove spurious points in PCs
  // pcl::StatisticalOutlierRemoval<pclPoint> sor;
  // sor.setMeanK(meanK);
  // sor.setStddevMulThresh(stdDevMulThresh);
  // sor.setInputCloud(srcFiltered);
  // sor.filter(*srcFiltered);
  // sor.setInputCloud(trgFiltered);
  // sor.filter(*trgFiltered);

  // Radius outlier removal filter to remove spurious points in PCs
  // pcl::RadiusOutlierRemoval<pclPoint> ror;
  // ror.setRadiusSearch(radiusSearchScale * m_resolution);
  // ror.setMinNeighborsInRadius(minNumberNeighbors);
  // ror.setKeepOrganized(true);
  // ror.setInputCloud(src);
  // ror.filter(*src);
  // ror.setInputCloud(trg);
  // ror.filter(*trg);

  // Downsample
  pcl::VoxelGrid<pclPoint> grid;
  grid.setLeafSize(leafScale * m_resolution, leafScale * m_resolution, leafScale * m_resolution);
  // downsample src cloud
  grid.setInputCloud(src);
  grid.filter(*src);
  // downsample trg cloud
  grid.setInputCloud(trg);
  grid.filter(*trg);

  // ROS_INFO("src size: %i", src->size());
  // ROS_INFO("trg size: %i", trg->size());

  // pcl::Correspondences::Ptr correspondences (new pcl::Correspondences);
  // pcl::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> cest;
  // cest.setInputSource (source_features);
  // cest.setInputTarget (target_features);
  // cest.determineCorrespondences (*correspondences);
  // Correspondences::Ptr corr_filtered (new Correspondences);
  // CorrespondenceRejectorSampleConsensus<PointXYZ> rejector;
  // rejector.setInputSource (source_keypoints);
  // rejector.setInputTarget (target_keypoints);
  // rejector.setInlierThreshold (2.5);
  // rejector.setMaximumIterations (1000000);
  // rejector.setRefineModel (false);
  // rejector.setInputCorrespondences (correspondences);;
  // rejector.getCorrespondences (*corr_filtered);
  // TransformationEstimationSVD<PointXYZ, PointXYZ> trans_est;
  // trans_est.estimateRigidTransformation (*source_keypoints, *target_keypoints, *corr_filtered, transform);

  // PointCloudT::Ptr src_grid(new PointCloudT);
  // pcl::VoxelGrid<pclPoint> normal_grid;
  // normal_grid.setLeafSize(leafScale * m_resolution, leafScale * m_resolution, leafScale * m_resolution);
  // normal_grid.setInputCloud(src);
  // normal_grid.filter(*src_grid);

  // PointCloudT::Ptr src_normal(new PointCloudT);
  // PointCloudT::Ptr trg_normal(new PointCloudT);
  // pcl::NormalEstimationOMP<pclPoint, PointNT> nest;
  // nest.setRadiusSearch(leafScale * m_resolution);
  // nest.setInputCloud(trg);
  // nest.compute(*trg_normal);
  // nest.setInputCloud(src);
  // nest.compute(*src_normal);

  // FeatureCloudT::Ptr trg_features(new FeatureCloudT);
  // FeatureCloudT::Ptr src_features(new FeatureCloudT);
  // FeatureEstimationT fest;
  // fest.setRadiusSearch(leafScale * m_resolution);
  // fest.setInputCloud(trg_normal);
  // fest.setInputNormals(trg_normal);
  // fest.compute(*trg_features);
  // fest.setInputCloud(src_normal);
  // fest.setInputNormals(src_normal);
  // fest.compute(*src_features);

  // PointCloudT::Ptr src_aligned(new PointCloudT);
  // pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
  // align.setInputSource(src_normal);
  // align.setSourceFeatures(src_features);
  // align.setInputTarget(trg_normal);
  // align.setTargetFeatures(trg_features);
  // align.setMaximumIterations(100);
  // align.setNumberOfSamples(10);
  // align.setCorrespondenceRandomness(10);
  // align.setSimilarityThreshold(0.9f);
  // align.setMaxCorrespondenceDistance(gicpMaxCorrespDistScale * m_resolution);
  // align.setInlierFraction(0.5f);
  // align.align(*src_aligned);
  // Eigen::Matrix4f feature_tf_est;
  // std::cout << "feature tf estimated: " << feature_tf_est << std::endl;
  // Eigen::Affine3f feature_tf_estimated(feature_tf_est);
  // float xf, yf, zf, rollf, pitchf, yawf;
  // pcl::getTranslationAndEulerAngles(feature_tf_estimated, xf, yf, zf, rollf, pitchf, yawf);
  // Eigen::Vector3f translationf(xf, yf, zf);
  // ROS_INFO("Feature Translation norm: %f", translationf.squaredNorm());
  // ROS_INFO("Rollf: %f, pitchf: %f, yawf: %f", pcl::rad2deg(rollf), pcl::rad2deg(pitchf), pcl::rad2deg(yawf));

  // pcl::SampleConsensusInitialAlignment<PointNT, PointNT, FeatureT> ia_align;
  // ia_align.setInputSource(src_normal);
  // ia_align.setSourceFeatures(src_features);
  // ia_align.setInputTarget(trg_normal);
  // ia_align.setTargetFeatures(trg_features);
  // ia_align.setMaximumIterations(10);
  // ia_align.setRANSACIterations(10);
  // ia_align.align(*src_aligned);
  // Eigen::Matrix4f feature_tf_est;
  // std::cout << "feature tf estimated: " << feature_tf_est << std::endl;
  // Eigen::Affine3f feature_tf_estimated(feature_tf_est);
  // float xf, yf, zf, rollf, pitchf, yawf;
  // pcl::getTranslationAndEulerAngles(feature_tf_estimated, xf, yf, zf, rollf, pitchf, yawf);
  // Eigen::Vector3f translationf(xf, yf, zf);
  // ROS_INFO("Feature Translation norm: %f", translationf.squaredNorm());
  // ROS_INFO("Rollf: %f, pitchf: %f, yawf: %f", pcl::rad2deg(rollf), pcl::rad2deg(pitchf), pcl::rad2deg(yawf));

  // pcl::PolygonMesh::Ptr trg_mesh;
  // pcl::PolygonMesh::Ptr src_mesh;
  // pcl::OrganizedFastMesh<pclPoint> fast_mesh;
  // fast_mesh.setInputCloud(trg);
  // fast_mesh.reconstruct(*trg_mesh);
  // fast_mesh.setInputCloud(src);
  // fast_mesh.reconstruct(*src_mesh);

  // PointCloudT::Ptr trg_normals;
  // std::vector<Eigen::Matrix3d> trg_covs;
  // pcl::features::computeApproximateNormals(*trg, trg_mesh->polygons, trg_normals);
  // pcl::features::computeApproximateCovariances(trg, trg_normals, trg_covs);
  // PointCloudT::Ptr src_normals;
  // boost::shared_ptr<std::vector<Eigen::Matrix3d>> src_covs;
  // pcl::features::computeApproximateNormals(*src, src_mesh->polygons, *src_normals);
  // pcl::features::computeApproximateCovariances(*src, src_normals, src_covs);

  // Uniform sampling
  // pcl::UniformSampling<pclPoint> uniform_grid;
  // uniform_grid.setRadiusSearch(leafScale * m_resolution);
  // uniform_grid.setInputCloud(src);
  // uniform_grid.filter(*src);
  // uniform_grid.setInputCloud(trg);
  // uniform_grid.filter(*trg); 

  // pcl::registration::CorrespondenceRejectorMedianDistance rejector;
  // pcl::Correspondence corresp;
  // rejector.setInputSource()
  // rejector.setInputTarget(*trg);
  // rejector.setInputCorrespondences(corresp);
  // rejector.getCorrespondences(*corresp);

  // begin registration
  pcl::GeneralizedIterativeClosestPoint<pclPoint, pclPoint> gicp;
  gicp.setMaximumIterations(gicpOptimizerIterations);
  gicp.setMaximumOptimizerIterations(100);
  gicp.setMaxCorrespondenceDistance(gicpMaxCorrespDistScale * m_resolution);
  gicp.setTransformationEpsilon(gicpTFEpsScale);
  // gicp.setRotationEpsilon(3.0);
  gicp.setEuclideanFitnessEpsilon(euclideanFitnessEps);
  gicp.setRANSACOutlierRejectionThreshold(ransacOutlierRejecThresh * m_resolution);
  gicp.setRANSACIterations(ransacNumIterations);

  // setup output tf and cloud  
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f prev;
  Eigen::Matrix4f trgToSrcTF;
  pointCloud::Ptr gicp_result;

  if (src->size() < trg->size()) 
  {
    // pcl::transformPointCloud(*src, *src, tfEst);
    gicp.setInputSource(src);
    gicp.setInputTarget(trg);
    gicp_result = src;
    // gicp.setSourceCovariances(src_covs);
    // gicp.setTargetCovariances(trg_covs);
    gicp.align(*gicp_result);
    Ti = gicp.getFinalTransformation();
  }
  else
  {
    // pcl::transformPointCloud(*trg, *trg, tfEst.inverse());
    gicp.setInputSource(trg);
    gicp.setInputTarget(src);
    gicp_result = trg;
    // gicp.setSourceCovariances(trg_covs);
    // gicp.setTargetCovariances(src_covs);
    gicp.align(*gicp_result);
    Ti = gicp.getFinalTransformation();
    Ti = Ti.inverse();
  }
  // get the transformation from target to source
  trgToSrcTF = Ti;
  // get transformation fitness score
  double fs = gicp.getFitnessScore();
  // get translation and angles from transformation
  Eigen::Affine3f tf_estimated(trgToSrcTF);
  float x, y, z, roll, pitch, yaw;
  pcl::getTranslationAndEulerAngles(tf_estimated, x, y, z, roll, pitch, yaw);
  Eigen::Vector3f translation(x, y, z);
  ROS_INFO("Translation norm: %f", translation.squaredNorm());
  ROS_INFO("Roll: %f, pitch: %f, yaw: %f", pcl::rad2deg(roll), pcl::rad2deg(pitch), pcl::rad2deg(yaw));

  ROS_INFO("GICP fitness score: %f", fs);
  if (fs > m_fitnessScoreThreshold)
    ROS_INFO("GICP Fitness Score above threshold. Storing map and will reattempt merge next time around.");
  // check for GICP convergence 
  if (gicp.hasConverged())
    ROS_INFO("GICP converged");
  else
    ROS_INFO("GICP did not converge");

  // Euclidean transformation validation
  pcl::registration::TransformationValidationEuclidean<pclPoint, pclPoint> tve;
  tve.setMaxRange(m_tveMaxRange);
  tve.setThreshold(0.25);
  double val_score = tve.validateTransformation(src, trg, trgToSrcTF);
  ROS_INFO("TVE score: %f", val_score);

  // clear filtered PCs
  src->clear();
  trg->clear();
  
  return trgToSrcTF;
}

void OctomapMerging::transformTree(Eigen::Matrix4f& transform)
{
  double res = m_resolution;
  RoughOcTreeT* transformed = new RoughOcTreeT(res);
  
  // build inverse transform
  Eigen::Matrix3f rotation;
  Eigen::Matrix3f invRotation;
  Eigen::Matrix4f invTransform;
  rotation << transform(0, 0), transform(0, 1), transform(0, 2), 
              transform(1, 0), transform(1, 1), transform(1, 2),
              transform(2, 0), transform(2, 1), transform(2, 2);
  invRotation = rotation.transpose();
  invTransform <<
    invRotation(0, 0), invRotation(0, 1), invRotation(0, 2), -transform(0, 3),
    invRotation(1, 0), invRotation(1, 1), invRotation(1, 2), -transform(1, 3),
    invRotation(2, 0), invRotation(2, 1), invRotation(2, 2), -transform(2, 3),
    0, 0, 0, 1;
  
  // size in each coordinate of each axis
  double minX, maxX, minY, maxY, minZ, maxZ;
  
  // get the min and max in so we can step along each row
  boost::mutex::scoped_lock lock(m_mtx);
  temp_tree->getMetricMin(minX, minY, minZ);
  temp_tree->getMetricMax(maxX, maxY, maxZ);
  lock.unlock();

  // allocate a vector of points
  std::vector<octomap::point3d> points;

  // make 8 points to make a map bounding box, performing the tf on them
  // to get the range of values in the transformed map
  points.push_back(octomap::point3d(minX, minY, minZ));
  points.push_back(octomap::point3d(minX, minY, maxZ));
  points.push_back(octomap::point3d(minX, maxY, minZ));
  points.push_back(octomap::point3d(minX, maxY, maxZ));
  points.push_back(octomap::point3d(maxX, minY, minZ));
  points.push_back(octomap::point3d(maxX, minY, maxZ));
  points.push_back(octomap::point3d(maxX, maxY, minZ));
  points.push_back(octomap::point3d(maxX, maxY, maxZ));

  // transform points that define bounding box
  for (int i = 0; i < points.size(); i++) {
    Eigen::Vector4f point(points[i].x(), points[i].y(), points[i].z(), 1.);
    point = transform * point;
    points[i] = octomap::point3d(point(0), point(1), point(2));
  }
  // find min and max for each axis
  for (int i = 0; i < points.size(); i++) {
    Eigen::Vector3f pt(points[i].x(), points[i].y(), points[i].z());
    if (pt(0) < minX)
      minX = pt(0);
    if (pt(1) < minY)
      minY = pt(1);
    if (pt(2) < minZ)
      minZ < pt(2);
    if (pt(0) > maxX)
      maxX = pt(0);
    if (pt(1) > maxY)
      maxY = pt(1);
    if (pt(2) > maxZ)
      maxZ = pt(2);
  }
  
  // go through the possible destination voxels on a row by row basis
  // and calculate occupancy from source voxels with inverse tf
  lock.lock();
  for (double z = minZ - res / 2; z < (maxZ + res / 2); z += res) {
    for (double y = minY - res / 2; y < (maxY + res / 2); y += res) {
      for (double x = minX - res / 2; x < (maxX + res / 2); x += res) {
        octomap::OcTreeKey destVoxel = transformed->coordToKey(octomap::point3d(x,y,z));

        Eigen::Vector4f point(x,y,z,1);
        point = invTransform * point;
        octomap::point3d sourcePoint = octomap::point3d(point(0), point(1), point(2));
        octomap::OcTreeKey sourceVoxel = temp_tree->coordToKey(sourcePoint);
        octomap::point3d nn = temp_tree->keyToCoord(sourceVoxel);

        // use nearest neighbour to set new occupancy in the transformed map
        octomap::OcTreeNode *oldNode = temp_tree->search(sourceVoxel);

        // Occupancies to interpolate between
        double c000, c001, c010, c011, c100, c101, c110,
               c111, c00, c01, c10, c11, c0, c1;
        double xd,yd,zd;

        // differences in each direction between next closest voxel
        xd = (sourcePoint.x() - nn.x()) / res;
        yd = (sourcePoint.y() - nn.y()) / res;
        zd = (sourcePoint.z() - nn.z()) / res;

        if (oldNode != NULL) {
          c000 = oldNode->getOccupancy();
          octomap::OcTreeNode *node;

          // c001
          if ((node = temp_tree->search(octomap::point3d(nn.x(), nn.y(), nn.z() +
                      getSign(zd) * res))) != NULL) {
            c001 = node->getOccupancy();
          } else
            c001 = 0;

          // c010
          if ((node = temp_tree->search(octomap::point3d(nn.x(), nn.y() + 
                      getSign(yd) * res, nn.z()))) != NULL) {
            c010 =node->getOccupancy();
          } else
            c010 = 0;

          // c011
          if ((node = temp_tree->search(octomap::point3d(nn.x(),nn.y() + 
                      getSign(yd) * res, nn.z() + getSign(zd) * res))) != NULL) {
            c011 = node->getOccupancy();
          } else
            c011 = 0;

          // c100
          if ((node = temp_tree->search(octomap::point3d(nn.x() + getSign(xd) * res, nn.y(),
                      nn.z()))) != NULL) {
            c100 = node->getOccupancy();
          } else
            c100 = 0;

          // c101
          if ((node = temp_tree->search(octomap::point3d(nn.x() + getSign(xd) * res, nn.y(),
                      nn.z() + getSign(zd) * res))) != NULL) {
            c101 = node->getOccupancy();
          } else
            c101 = 0;

          // c110
          if ((node = temp_tree->search(octomap::point3d(nn.x() + getSign(xd) * res, nn.y() + 
                      getSign(yd) * res, nn.z()))) != NULL) {
            c110 = node->getOccupancy();
          } else
            c110 = 0;

          // c111
          if ((node = temp_tree->search(octomap::point3d(nn.x() + getSign(xd) * res, nn.y() + 
                      getSign(yd) * res, nn.z() + getSign(zd) * res))) != NULL) {
            c111 = node->getOccupancy();
          } else
            c111 = 0;

          // Interpolate in x
          c00 = (1-fabs(xd)) * c000 + fabs(xd) * c100;
          c10 = (1-fabs(xd)) * c010 + fabs(xd) * c110;
          c01 = (1-fabs(xd)) * c001 + fabs(xd) * c101;
          c11 = (1-fabs(xd)) * c011 + fabs(xd) * c111;

          // interpolate in y
          c0 = (1-fabs(yd)) * c00 + fabs(yd) * c10;
          c1 = (1-fabs(yd)) * c01 + fabs(yd) * c11;

          // now let’s assign the new node value
          octomap::OcTreeNode *newNode = transformed->updateNode(destVoxel, true);
          newNode->setLogOdds(octomap::logodds((1 - fabs(zd)) * c0 + fabs(zd) * c1));
        }
      }
    }
  }

  temp_tree->swapContent(*transformed);

  delete transformed;
}

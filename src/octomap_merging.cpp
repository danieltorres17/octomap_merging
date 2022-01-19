#include "octomap_merging.h"

OctomapMerging::OctomapMerging(ros::NodeHandle& nodehandle, ros::NodeHandle& private_nodehandle)
: nh(nodehandle), 
  pnh(private_nodehandle)
{
  // Initialize parameters from launch file
  nh.param("resolution", m_resolution, 0.15);
  nh.param("map_merge_timer", m_mergeTimer, 300);
  nh.param("pub_self_map", m_pubSelfMap, true);
  nh.param("self_map_topic", m_selfMapTopic, m_selfMapTopic);
  nh.param("pub_self_merged_map", m_pubMergedMap, false);
  nh.param("self_merged_map_topic", m_selfMergedMapTopic, m_selfMergedMapTopic);
  nh.param("pub_neighbor_maps", m_pubNeighborMaps, false);
  nh.param("neighbor_map_topic", m_neighborMapTopic, m_neighborMapTopic);
  nh.param("neighbor_map_pub_timer", m_neighborMapPubTimer, m_neighborMapPubTimer);
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

  // get outlier filter, downsampling and GICP parameters from config file
  nh.getParam("meanK", meanK);
  nh.getParam("stdDevMulThresh", stdDevMulThresh);
  nh.getParam("leafSizeScaling", leafScale);
  nh.getParam("icp_iterations", icpIterations);
  nh.getParam("optimizer_iterations", icpOptimizerIterations);
  nh.getParam("max_correspondence_dist_scale", icpMaxCorrespDistScale);
  nh.getParam("transformation_eps_scale", icpTFEpsScale);
  nh.getParam("ransacOutlierRejecThresh", ransacOutlierRejecThresh);
  nh.getParam("ransacNumberIterations", ransacNumIterations);
  nh.getParam("eucFitnessEps", euclideanFitnessEps);
  nh.getParam("fitnessScoreThreshold", m_fitnessScoreThreshold);
  nh.getParam("tfValidationEuclideanMaxRange", m_tveMaxRange);

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

  //initialize temp_tree for neighbor diff map merging
  temp_tree = new RoughOcTreeT(m_resolution);
  temp_tree->setRoughEnabled(m_enableTraversability);
  temp_tree->setProbHit(probHit);
  temp_tree->setProbMiss(probMiss);
  temp_tree->setClampingThresMin(thresMin);
  temp_tree->setClampingThresMax(thresMax);

  // Subscribers
  myDiffsSub = nh.subscribe(m_mapTopic, 100, &OctomapMerging::myMapCallback, this);
  neighborDiffsSub = nh.subscribe(m_neighborsTopic, 100, &OctomapMerging::neighborDiffsCallback, this);

  // Publishers
  myMapPub = nh.advertise<octomap_msgs::Octomap>(m_selfMapTopic, 5, true);
  mergedMapPub = nh.advertise<octomap_msgs::Octomap>(m_selfMergedMapTopic, 5, true);
  // neighborMapPub = pnh.advertise<octomap_msgs::Octomap>(m_neighborMapTopic, 5, true);
  // tfTempTreePub = nh.advertise<octomap_msgs::Octomap>(m_tfTempTreeTopic, 5, true);

  // m_octreePCPub = nh.advertise<sensor_msgs::PointCloud2>(m_selfMapPCTopic, 5, true);
  // temp_treePCPub = nh.advertise<sensor_msgs::PointCloud2>(m_tempTreePCTopic, 5, true);
  // transformedTreePCPub = nh.advertise<sensor_msgs::PointCloud2>(m_tfTempTreePCTopic, 5, true);
  // src_BBoxPCPub = nh.advertise<sensor_msgs::PointCloud2>(m_srcBBoxPCTopic, 5, true);
  // trg_BBoxPCPub = nh.advertise<sensor_msgs::PointCloud2>(m_trgBBoxPCTopic, 5, true);

  // Timer events
  merge_timer = nh.createTimer(ros::Duration(m_mergeTimer), &OctomapMerging::mergeNeighbors, this);
  pub_neighbor_maps_timer = nh.createTimer(ros::Duration(m_neighborMapPubTimer), &OctomapMerging::publishNeighborMaps, this);

  // initialize tf_estimate for each neighbor with identity matrix
  Eigen::Matrix4f tfEst = Eigen::Matrix4f::Identity(); 
  for (auto& n : neighbors_list) {
    neighbor_tf[n] = tfEst;
    neighbor_aligned[n] = false;
  }
  // Populate the neighbors message so mergeNeighbors will work
  neighborDiffs.num_neighbors = 1;
  neighborDiffs.neighbors.push_back(octomap_merging::OctomapArray());
  neighborDiffs.neighbors[0].num_octomaps = 1;
  neighborDiffs.neighbors[0].octomaps.push_back(octomap_msgs::Octomap());
}

OctomapMerging::~OctomapMerging()
{
  if (m_octree) {
    delete m_octree;
    m_octree = NULL;
  }

  if (m_merged_tree) {
    delete m_merged_tree;
    m_merged_tree = NULL;
  }

  if (temp_tree) {
    delete temp_tree;
    temp_tree = NULL;
  }

  nh.shutdown();
}

void OctomapMerging::myMapCallback(const octomap_msgs::OctomapConstPtr& msg)
{
  boost::mutex::scoped_lock lock(m_mtx);
  delete m_octree; 
  m_octree = (RoughOcTreeT*)octomap_msgs::binaryMsgToMap(*msg);

  if (m_pubSelfMap) {
    m_octree->prune();
    octomap_msgs::Octomap map_msg;
    octomap_msgs::binaryMapToMsg(*m_octree, map_msg);
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = "world";
    myMapPub.publish(map_msg);
  }
}

void OctomapMerging::neighborDiffsCallback(const octomap_merging::OctomapNeighborsConstPtr& msg)
{
  neighborDiffs = *msg;
  // check if received message was corrupted
  bool invalidMessage = (neighborDiffs.num_neighbors != neighborDiffs.neighbors.size());
  if (!invalidMessage) {
    for (int i = 0; i < msg->neighbors.size(); i++) {
      if (neighborDiffs.neighbors[i].num_octomaps != neighborDiffs.neighbors[i].octomaps.size())
        invalidMessage = true;
    }
  }
  // if corrupted, proceed to reset and clear neighborDiffs
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

  // Move somewhere else?
  try {
    updateNeighborMaps();
  }
  catch (const std::exception& e) {
    ROS_ERROR("Merging error! %s", e.what());
  }
}

void OctomapMerging::updateNeighborMaps()
{
  // ntree used to reconstruct diff maps into single map
  std::shared_ptr<RoughOcTreeT> ntree(nullptr);
  bool remerge;
  char agent;
  
  for (int i = 0; i < neighborDiffs.num_neighbors; i++) {
    // Profile our processing time
    ros::WallTime startTime = ros::WallTime::now();
    std::string nid = neighborDiffs.neighbors[i].owner;
    // Ignore self maps - only for OctomapMerging, shouldn't be needed for MarbleMapping
    if (nid == "H01") {
      // ROS_INFO("Won't merge self map");
      continue;
    }
    ROS_INFO("Adding %s maps \n", nid.c_str());
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
      if (m_pubNeighborMaps && neighbor_maps.count(nid.data()))
        neighbor_maps[nid.data()]->clear();
    }

    // Initialize neighbor map data if enabled
    if (m_pubNeighborMaps && !neighbor_maps.count(nid.data()) && 
        neighborDiffs.neighbors[i].num_octomaps > 0 &&
        neighborDiffs.neighbors[i].num_octomaps == neighborDiffs.neighbors[i].octomaps.size()) {
      ROS_INFO("Adding neighbor map %s", nid.data());
      neighbor_maps[nid.data()] = new RoughOcTreeT(m_resolution);
      neighbor_maps[nid.data()]->setRoughEnabled(m_enableTraversabilitySharing);
      neighbor_updated[nid.data()] = false;
      neighbor_pubs[nid.data()] = nh.advertise<octomap_msgs::Octomap>("/" + nid + "/map", 1, true);
    }

    remerge = false;
    // Check each diff for new ones to merge
    // Array should be in sequence order to allow re-merging if a diff comes out of sequence
    // In theory should be able to just skip nodes in out-of-sequence diffs,
    // but doesn't always work as expected due to various node sizes
    // ROS_INFO("%s number of diff maps: %i", nid.c_str(), int(neighborDiffs.neighbors[i].octomaps.size()));
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
        // update neighbor map with reconstructed diffs stored in ntree
        ntree->expand();
        boost::mutex::scoped_lock lock(m_mtx);
        for (RoughOcTreeT::iterator it = ntree->begin(), end = ntree->end(); it != end; ++it) {
          octomap::point3d point = it.getCoordinate();
          // if maps haven't been merged yet, update neighbor_maps directly
          if (!m_neighborMapsMerged) {
            neighbor_updated[nid.data()] = true;
            octomap::RoughOcTreeNode* newNode = neighbor_maps[nid.data()]->setNodeValue(point, it->getLogOdds());
            if (m_enableTraversabilitySharing) {
              newNode->setRough(it->getRough());
            }
          }
          else {
            octomap::RoughOcTreeNode* newNode = temp_tree->setNodeValue(point, it->getLogOdds());
            if (m_enableTraversabilitySharing) {
              newNode->setRough(it->getRough());
            }
          }
        }
        if (m_neighborMapsMerged) {
          // if maps have been merged, set to true (for publishing purposes)
          neighbor_updated[nid.data()] = true;
          // transform map using last stored tf for neighbor
          ROS_INFO("%s already aligned. Transforming maps", nid.c_str());
          transformMap(temp_tree, neighbor_tf[nid.data()]);
          // integrate into merged map
          ROS_INFO("%s already aligned. Merging maps \n", nid.c_str());
          temp_tree->expand();
          mergeMap(temp_tree);
          // add temp_tree to agent map
          for (RoughOcTreeT::iterator it = temp_tree->begin(), end = temp_tree->end(); it != end; ++it) {
            octomap::point3d point = it.getCoordinate();
            octomap::RoughOcTreeNode* newNode = neighbor_maps[nid.data()]->setNodeValue(point, it->getLogOdds());
            if (m_enableTraversabilitySharing) {
                newNode->setRough(it->getRough());
            }
          }
          // clear temp_tree for next time around
          temp_tree->clear();
        }
      }
    }
  }
}

void OctomapMerging::mergeNeighbors(const ros::TimerEvent& event) 
{ 
  // change neighbor maps aligned bool status
  if (!m_neighborMapsMerged) {
    // boost::mutex::scoped_lock lock(m_mtx);
    ROS_INFO("Running merge procedure");
    // add self map to merged map
    m_octree->expand();
    for (RoughOcTreeT::iterator it = m_octree->begin(), end=m_octree->end();
         it != end; ++it) {
      octomap::point3d pt = it.getCoordinate();
      octomap::RoughOcTreeNode* newNode = m_merged_tree->setNodeValue(pt, it->getLogOdds());
      newNode->setAgent(1);
      if (m_enableTraversabilitySharing) {
        newNode->setRough(it->getRough());
      }
    }
    // loop through stored neighbor maps, transform and merge
    for (auto& n : neighbor_maps) {
      // store neighbor name
      std::string nid = n.first.data();
      neighbor_aligned[nid.data()] = true;
      RoughOcTreeT* neighbor_map = neighbor_maps[nid.data()];

      // align map
      neighbor_map->prune();
      Eigen::Matrix4f est_tf = alignMap(neighbor_map);
      neighbor_tf[nid.data()] = est_tf; // store neighbor map tf
      ROS_INFO("Aligning maps for %s", nid.c_str());

      // transform map
      transformMap(neighbor_map, est_tf);
      ROS_INFO("Transforming maps for %s", nid.c_str());

      // merge map
      neighbor_map->expand();
      mergeMap(neighbor_map);
      ROS_INFO("Merging maps for %s \n", nid.c_str());
    }
    m_neighborMapsMerged = true;
  }
  else {
    ROS_INFO("Already aligned");
  }
}

Eigen::Matrix4f OctomapMerging::alignMap(const RoughOcTreeT* tree)
{
  // convert tree to point cloud
  pointCloud tree_pc;
  tree2PointCloud(tree, tree_pc);

  // convert merged map to point cloud
  // boost::mutex::scoped_lock lock(m_mtx);
  pointCloud merged_pc;
  m_merged_tree->prune();
  tree2PointCloud(m_merged_tree, merged_pc);
  // lock.unlock();

  // get estimated transform from GICP
  Eigen::Matrix4f initial = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f est_tf = getGICPTransform(tree_pc, merged_pc);

  if (m_pubSelfMapPC) {
    // publish m_octree point cloud
    sensor_msgs::PointCloud2 m_octreePC_out;
    pcl::toROSMsg(merged_pc, m_octreePC_out);
    m_octreePC_out.header.frame_id = "world";
    m_octreePC_out.header.stamp = ros::Time::now();
    m_octreePCPub.publish(m_octreePC_out);
  }

  if (m_pubTempTreePC) {
    // Publish temp_tree point cloud
    sensor_msgs::PointCloud2 temp_treePC_out;
    pcl::toROSMsg(tree_pc, temp_treePC_out);
    temp_treePC_out.header.frame_id = "world";
    temp_treePC_out.header.stamp = ros::Time::now();
    temp_treePCPub.publish(temp_treePC_out);
  }

  if (m_pubTFTempTreePC) {
    // publish transformed temp_tree cloud
    pointCloud tf_points;
    pcl::transformPointCloud(tree_pc, tf_points, est_tf);
    sensor_msgs::PointCloud2 tf_points_msg;
    pcl::toROSMsg(tf_points, tf_points_msg);
    tf_points_msg.header.frame_id = "world";
    tf_points_msg.header.stamp = ros::Time::now();
    transformedTreePCPub.publish(tf_points_msg);
  }

  return est_tf;
}

void OctomapMerging::mergeMap(RoughOcTreeT* tree)
{
  // iterate through tree and merge
  for (RoughOcTreeT::iterator it = tree->begin(), end = tree->end(); it != end; ++it) {
    octomap::OcTreeKey nodeKey = it.getKey();
    octomap::RoughOcTreeNode* nodeM = m_merged_tree->search(nodeKey);
    octomap::RoughOcTreeNode* nodeCurr = tree->search(nodeKey);
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
  }

  if (m_pubMergedMap) {
    m_merged_tree->prune();
    octomap_msgs::Octomap merged_map_msg;
    octomap_msgs::binaryMapToMsg(*m_merged_tree, merged_map_msg);
    merged_map_msg.header.stamp = ros::Time::now();
    merged_map_msg.header.frame_id = "world";
    mergedMapPub.publish(merged_map_msg);
  }
}

void OctomapMerging::tree2PointCloud(const RoughOcTreeT* tree, pointCloud& pc)
{
  for (RoughOcTreeT::iterator it = tree->begin(), end = tree->end(); it != end; ++it) {
    if (tree->isNodeOccupied(*it))
      pc.push_back(pclPoint(it.getX(), it.getY(), it.getZ()));
  }
}

Eigen::Matrix4f OctomapMerging::getGICPTransform(pointCloud& source, pointCloud& target)
{
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

  // publish src intersecting points
  if (m_pubBBoxPC) {
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

  // publish src intersecting points
  if (m_pubBBoxPC) {
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

  // Statistical outlier filter to remove spurious points in PCs
  pcl::StatisticalOutlierRemoval<pclPoint> sor;
  sor.setMeanK(meanK);
  sor.setStddevMulThresh(stdDevMulThresh);
  sor.setInputCloud(src);
  sor.filter(*src);
  sor.setInputCloud(trg);
  sor.filter(*trg);

  // begin registration
  pcl::GeneralizedIterativeClosestPoint<pclPoint, pclPoint> gicp;
  gicp.setMaximumIterations(icpIterations);
  gicp.setMaximumOptimizerIterations(icpOptimizerIterations);
  gicp.setMaxCorrespondenceDistance(icpMaxCorrespDistScale * m_resolution);
  gicp.setTransformationEpsilon(icpTFEpsScale);
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
    gicp.setInputSource(src);
    gicp.setInputTarget(trg);
    gicp_result = src;
    gicp.align(*gicp_result);
    Ti = gicp.getFinalTransformation();
  }
  else
  {
    gicp.setInputSource(trg);
    gicp.setInputTarget(src);
    gicp_result = trg;
    gicp.align(*gicp_result);
    Ti = gicp.getFinalTransformation();
    Ti = Ti.inverse();
  }
  // get the transformation from target to source
  trgToSrcTF = Ti;
  // get transformation fitness score
  double fs = gicp.getFitnessScore(0.05);

  ROS_INFO("GICP fitness score: %f", fs);
  if (fs > m_fitnessScoreThreshold)
    ROS_INFO("GICP Fitness Score above threshold. Storing map and will reattempt merge next time around.");
  // check for GICP convergence 
  if (gicp.hasConverged())
    ROS_INFO("GICP converged");
  else
    ROS_INFO("GICP did not converge");

  // clear filtered PCs
  src->clear();
  trg->clear();
  
  return trgToSrcTF;
}

void OctomapMerging::transformMap(RoughOcTreeT* tree, const Eigen::Matrix4f& tf)
{
  double res = m_resolution;
  RoughOcTreeT* transformed = new RoughOcTreeT(res);
  
  // build inverse transform
  Eigen::Matrix3f rotation = tf.block<3,3> (0, 0); // extracts rotation 
  Eigen::Matrix3f invRotation = rotation.inverse();
  Eigen::Matrix4f invTransform = tf.inverse();
  
  // size in each coordinate of each axis
  double minX, maxX, minY, maxY, minZ, maxZ;
  
  // get the min and max in so we can step along each row
  // boost::mutex::scoped_lock lock(m_mtx);
  tree->getMetricMin(minX, minY, minZ);
  tree->getMetricMax(maxX, maxY, maxZ);
  // lock.unlock();

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
    point = tf * point;
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
  // lock.lock();
  for (double z = minZ - res / 2; z < (maxZ + res / 2); z += res) {
    for (double y = minY - res / 2; y < (maxY + res / 2); y += res) {
      for (double x = minX - res / 2; x < (maxX + res / 2); x += res) {
        octomap::OcTreeKey destVoxel = transformed->coordToKey(octomap::point3d(x,y,z));

        Eigen::Vector4f point(x,y,z,1);
        point = invTransform * point;
        octomap::point3d sourcePoint = octomap::point3d(point(0), point(1), point(2));
        octomap::OcTreeKey sourceVoxel = tree->coordToKey(sourcePoint);
        octomap::point3d nn = tree->keyToCoord(sourceVoxel);

        // use nearest neighbour to set new occupancy in the transformed map
        octomap::OcTreeNode *oldNode = tree->search(sourceVoxel);

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
          if ((node = tree->search(octomap::point3d(nn.x(), nn.y(), nn.z() +
                      getSign(zd) * res))) != NULL) {
            c001 = node->getOccupancy();
          } else
            c001 = 0;

          // c010
          if ((node = tree->search(octomap::point3d(nn.x(), nn.y() + 
                      getSign(yd) * res, nn.z()))) != NULL) {
            c010 =node->getOccupancy();
          } else
            c010 = 0;

          // c011
          if ((node = tree->search(octomap::point3d(nn.x(),nn.y() + 
                      getSign(yd) * res, nn.z() + getSign(zd) * res))) != NULL) {
            c011 = node->getOccupancy();
          } else
            c011 = 0;

          // c100
          if ((node = tree->search(octomap::point3d(nn.x() + getSign(xd) * res, nn.y(),
                      nn.z()))) != NULL) {
            c100 = node->getOccupancy();
          } else
            c100 = 0;

          // c101
          if ((node = tree->search(octomap::point3d(nn.x() + getSign(xd) * res, nn.y(),
                      nn.z() + getSign(zd) * res))) != NULL) {
            c101 = node->getOccupancy();
          } else
            c101 = 0;

          // c110
          if ((node = tree->search(octomap::point3d(nn.x() + getSign(xd) * res, nn.y() + 
                      getSign(yd) * res, nn.z()))) != NULL) {
            c110 = node->getOccupancy();
          } else
            c110 = 0;

          // c111
          if ((node = tree->search(octomap::point3d(nn.x() + getSign(xd) * res, nn.y() + 
                      getSign(yd) * res, nn.z() + getSign(zd) * res))) != NULL) {
            c111 = node->getOccupancy();
          } else
            c111 = 0;

          // interpolate in x
          c00 = (1-fabs(xd)) * c000 + fabs(xd) * c100;
          c10 = (1-fabs(xd)) * c010 + fabs(xd) * c110;
          c01 = (1-fabs(xd)) * c001 + fabs(xd) * c101;
          c11 = (1-fabs(xd)) * c011 + fabs(xd) * c111;

          // interpolate in y
          c0 = (1-fabs(yd)) * c00 + fabs(yd) * c10;
          c1 = (1-fabs(yd)) * c01 + fabs(yd) * c11;

          // assign the new node value
          octomap::OcTreeNode *newNode = transformed->updateNode(destVoxel, true);
          newNode->setLogOdds(octomap::logodds((1 - fabs(zd)) * c0 + fabs(zd) * c1));
        }
      }
    }
  }
  tree->swapContent(*transformed);
  delete transformed;
}

void OctomapMerging::publishNeighborMaps(const ros::TimerEvent& event)
{
  boost::mutex::scoped_lock lock(m_mtx);
  for (const auto& n : neighbor_maps) {
    // Find any maps that have been updated and have subscribers
    if (n.second && neighbor_updated.at(n.first) && 
        (neighbor_pubs.at(n.first).getNumSubscribers() > 0)) {
      neighbor_updated.at(n.first) = false;
      octomap_msgs::Octomap map;
      map.header.frame_id = "world";
      map.header.stamp = ros::Time::now();
      n.second->prune();
      if (octomap_msgs::binaryMapToMsg(*n.second, map))
        neighbor_pubs.at(n.first).publish(map);
      else
        ROS_ERROR("Error serializing neighbor Octomap");
    }
  }
}
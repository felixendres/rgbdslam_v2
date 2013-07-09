/**
* partly taken from the ros octomap server.
* @author A. Hornung, K. M. Wurm University of Freiburg, Copyright (C) 2009.
* License: GPL
*/
/*
 * Copyright (c) 2010, A. Hornung, University of Freiburg
 * All rights reserved.
 */


#include "ColorOctomapServer.h"
#include "scoped_timer.h"

ColorOctomapServer::ColorOctomapServer() :  octomap_server::OctomapServer(), m_octoMap(0.05)
{ 
  reset();
}


ColorOctomapServer::~ColorOctomapServer() {}
///Clear octomap and reset values to paramters from parameter server
void ColorOctomapServer::reset()
{
  m_octoMap.octree.clear();
  ParameterServer* ps = ParameterServer::instance();
  m_octoMap.octree.setClampingThresMin(ps->get<double>("octomap_clamping_min"));
  m_octoMap.octree.setClampingThresMax(ps->get<double>("octomap_clamping_max"));
  m_octoMap.octree.setResolution(ps->get<double>("octomap_resolution"));
  m_octoMap.octree.setOccupancyThres(ps->get<double>("octomap_occupancy_threshold"));
  m_octoMap.octree.setProbHit(ps->get<double>("octomap_prob_hit"));
  m_octoMap.octree.setProbMiss(ps->get<double>("octomap_prob_miss"));
}

bool ColorOctomapServer::save(const char* filename) const
{
  ScopedTimer s(__FUNCTION__);
  std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);
  if (outfile.is_open()){
    //m_octoMap.octree.writeConst(outfile); 
    if (ParameterServer::instance()->get<bool>("concurrent_io")) {
      ROS_INFO("Waiting for rendering thread to finish");
      rendering.waitForFinished();
    }
    ROS_INFO("Writing octomap to %s", filename);
    m_octoMap.octree.write(outfile); 
    outfile.close();
    ROS_INFO("color tree written %s", filename);
    return true;
  }
  else {
    ROS_INFO("could not open  %s for writing", filename);
    return false;
  }
}

//Same as the other insertCloudCallback, but relies on the sensor position information in the cloud
void ColorOctomapServer::insertCloudCallback(const pointcloud_type::ConstPtr cloud, double max_range) {
  
  ScopedTimer s(__FUNCTION__); //Unconditional logging of time

  Eigen::Quaternionf q = cloud->sensor_orientation_;
  Eigen::Vector4f t = cloud->sensor_origin_;
  tf::Transform trans(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(t[0], t[1], t[2]));
  //eigen_transform = cloud->sensor_origin_ * cloud->sensor_orientation_ ;
  //pcl_ros::transformAsMatrix (trans, eigen_transform);
  pointcloud_type::Ptr pcl_cloud(new pointcloud_type);
  pcl_ros::transformPointCloud(*cloud, *pcl_cloud, trans);

  if (ParameterServer::instance()->get<bool>("concurrent_io")) {
    rendering.waitForFinished();
    rendering = QtConcurrent::run(this, &ColorOctomapServer::insertCloudCallbackCommon, pcl_cloud, trans, max_range);
  }
  else {
    insertCloudCallbackCommon(pcl_cloud, trans, max_range);
  }
}

void ColorOctomapServer::insertCloudCallbackCommon(const pointcloud_type::ConstPtr  pcl_cloud,
                                                   const tf::Transform& trans, double max_range) {
  if(m_octoMap.octree.getResolution() != ParameterServer::instance()->get<double>("octomap_resolution")){
    ROS_WARN("OctoMap resolution changed from %f to %f. Resetting Octomap", 
             m_octoMap.octree.getResolution(), ParameterServer::instance()->get<double>("octomap_resolution"));
    this->reset();
  }
  geometry_msgs::Point origin;
  tf::pointTFToMsg(trans.getOrigin(), origin);

  ROS_DEBUG("inserting data");
  //    m_octoMap.insertScan(pcl_cloud, origin, m_maxRange, false, true); // no pruning 
  m_octoMap.insertScan(*pcl_cloud, origin, max_range, true, true); 

  // integrate color measurements
  unsigned char* colors = new unsigned char[3];

  ROS_DEBUG("inserting color measurements");
  pointcloud_type::const_iterator it;
  for (it = pcl_cloud->begin(); it != pcl_cloud->end(); ++it) {
    // Check if the point is invalid
    if (!isnan (it->x) && !isnan (it->y) && !isnan (it->z)) {
#ifndef RGB_IS_4TH_DIM
      const int rgb = *reinterpret_cast<const int*>(&(it->rgb));
#else
      const int rgb = *reinterpret_cast<const int*>(&(it->data[3]));
#endif
      colors[0] = ((rgb >> 16) & 0xff);
      colors[1] = ((rgb >> 8) & 0xff);
      colors[2] = (rgb & 0xff);
      m_octoMap.octree.averageNodeColor(it->x, it->y, it->z, colors[0], colors[1], colors[2]);
    }
  }

  // updates inner node colors, too
  ROS_DEBUG("updating inner nodes");
  m_octoMap.octree.updateInnerOccupancy();
}

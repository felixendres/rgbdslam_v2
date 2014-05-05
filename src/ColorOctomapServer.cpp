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
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>

ColorOctomapServer::ColorOctomapServer() : m_octoMap(0.05)
{ 
  reset();
}


ColorOctomapServer::~ColorOctomapServer() {}
///Clear octomap and reset values to paramters from parameter server
void ColorOctomapServer::reset()
{
  m_octoMap.clear();
  ParameterServer* ps = ParameterServer::instance();
  m_octoMap.setClampingThresMin(ps->get<double>("octomap_clamping_min"));
  m_octoMap.setClampingThresMax(ps->get<double>("octomap_clamping_max"));
  m_octoMap.setResolution(ps->get<double>("octomap_resolution"));
  m_octoMap.setOccupancyThres(ps->get<double>("octomap_occupancy_threshold"));
  m_octoMap.setProbHit(ps->get<double>("octomap_prob_hit"));
  m_octoMap.setProbMiss(ps->get<double>("octomap_prob_miss"));
}

bool ColorOctomapServer::save(const char* filename) const
{
  ScopedTimer s(__FUNCTION__);
  std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);
  if (outfile.is_open()){
    //m_octoMap.writeConst(outfile); 
    if (ParameterServer::instance()->get<bool>("concurrent_io")) {
      ROS_INFO("Waiting for rendering thread to finish");
      rendering.waitForFinished();
    }
    ROS_INFO("Writing octomap to %s", filename);
    m_octoMap.write(outfile); 
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
  
  ScopedTimer s(__FUNCTION__);

  //Conversions
  Eigen::Quaternionf q = cloud->sensor_orientation_;
  Eigen::Vector4f t = cloud->sensor_origin_;
  tf::Transform trans(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(t[0], t[1], t[2]));
  octomap::point3d origin(t[0], t[1], t[2]);
  pointcloud_type::Ptr pcl_cloud(new pointcloud_type);

  //Work
  pcl_ros::transformPointCloud(*cloud, *pcl_cloud, trans);

  //Conversions
  boost::shared_ptr<octomap::Pointcloud> octomapCloud(new octomap::Pointcloud());

  //Work 
  octomapCloud->reserve(pcl_cloud->size());
  for (pointcloud_type::const_iterator it = pcl_cloud->begin(); it != pcl_cloud->end(); ++it){
    if (!std::isnan (it->z)) octomapCloud->push_back(it->x, it->y, it->z);
  }

  if (ParameterServer::instance()->get<bool>("concurrent_io")) {
    rendering.waitForFinished();
    rendering = QtConcurrent::run(this, &ColorOctomapServer::insertCloudCallbackCommon, octomapCloud, pcl_cloud, origin, max_range);
  }
  else {
    insertCloudCallbackCommon(octomapCloud, pcl_cloud, origin, max_range);
  }
}

void ColorOctomapServer::insertCloudCallbackCommon(boost::shared_ptr<octomap::Pointcloud> octomapCloud,
                                                   pointcloud_type::ConstPtr color_cloud,
                                                   const octomap::point3d& origin, double max_range) {
  if(m_octoMap.getResolution() != ParameterServer::instance()->get<double>("octomap_resolution")){
    ROS_WARN("OctoMap resolution changed from %f to %f. Resetting Octomap", 
             m_octoMap.getResolution(), ParameterServer::instance()->get<double>("octomap_resolution"));
    this->reset();
  }
  //geometry_msgs::Point origin;
  //tf::pointTFToMsg(trans.getOrigin(), origin);

  ROS_DEBUG("inserting data");
  m_octoMap.insertPointCloud(*octomapCloud, origin, max_range, true); 
  // integrate color measurements
  unsigned char* colors = new unsigned char[3];

  ROS_DEBUG("inserting color measurements");
  pointcloud_type::const_iterator it;
  for (it = color_cloud->begin(); it != color_cloud->end(); ++it) {
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
      m_octoMap.averageNodeColor(it->x, it->y, it->z, colors[0], colors[1], colors[2]);
    }
  }

  // updates inner node colors, too
  ROS_DEBUG("updating inner nodes");
  m_octoMap.updateInnerOccupancy();
}

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
#include <GL/gl.h>

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
    if (!std::isnan(it->z)) octomapCloud->push_back(it->x, it->y, it->z);
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
    if (!isnan(it->x) && !isnan(it->y) && !isnan(it->z)) {
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

//Filter, e.g. points in free space
void ColorOctomapServer::occupancyFilter(pointcloud_type::ConstPtr input, 
                                         pointcloud_type::Ptr output, 
                                         double occupancy_threshold){
  if(output->points.capacity() < input->size()){ //cannot happen for input == output
    output->reserve(input->size());
  }

  Eigen::Quaternionf q = input->sensor_orientation_;
  Eigen::Vector4f t = input->sensor_origin_;

  size_t size = input->size();
  size_t outidx = 0;
  for (size_t inidx = 0; inidx < size; ++inidx){
    const point_type& in_point = (*input)[inidx];
    Eigen::Vector3f in_vec = q * in_point.getVector3fMap() + t.head<3>();
    if (std::isnan(in_vec.z())) 
      continue;

    const int radius = 1;
    int x_a = m_octoMap.coordToKey(in_vec.x()) - radius;
    int x_b = m_octoMap.coordToKey(in_vec.x()) + radius;
    int y_a = m_octoMap.coordToKey(in_vec.y()) - radius;
    int y_b = m_octoMap.coordToKey(in_vec.y()) + radius;
    int z_a = m_octoMap.coordToKey(in_vec.z()) - radius;
    int z_b = m_octoMap.coordToKey(in_vec.z()) + radius;
    double sum_of_occupancy = 0, sum_of_weights = 0;
    for(;x_a <= x_b; ++x_a){
      for(;y_a <= y_b; ++y_a){
        for(;z_a <= z_b; ++z_a){
          octomap::OcTreeNode* node = m_octoMap.search(octomap::OcTreeKey(x_a, y_a, z_a));
          if(node != NULL){
            double dx = m_octoMap.keyToCoord(x_a) - in_vec.x();
            double dy = m_octoMap.keyToCoord(y_a) - in_vec.y();
            double dz = m_octoMap.keyToCoord(z_a) - in_vec.z();
            double weight = dx*dx+dy*dy+dz*dz;
            double weighted_occ = node->getOccupancy() / weight;
            sum_of_weights += weight;
            sum_of_occupancy += weighted_occ;
          }
        }
      }
    }
            

    if(sum_of_occupancy < occupancy_threshold * sum_of_weights) //Filters points in non-existent nodes (outside of map?) 
    //if(node != NULL && node->getOccupancy() >= occupancy_threshold) 
    { //Valid point
      point_type& out_point = (*output)[outidx];
      out_point = in_point;
      ++outidx;
    } 
  }
  output->resize(outidx);//downsize
}

void ColorOctomapServer::render(){
  octomap::ColorOcTree::tree_iterator it = m_octoMap.begin_tree();
  octomap::ColorOcTree::tree_iterator end = m_octoMap.end_tree();
  int counter = 0;
  double occ_thresh = ParameterServer::instance()->get<double>("occupancy_filter_threshold");
  int level = ParameterServer::instance()->get<int>("octomap_display_level");
  if(occ_thresh > 0) {
    glDisable(GL_LIGHTING);
    glEnable (GL_BLEND); 
    //glDisable(GL_CULL_FACE);
    glBegin(GL_TRIANGLES);
    double stretch_factor = 128/(1 - occ_thresh); //occupancy range in which the displayed cubes can be
    for(; it != end; ++counter, ++it){
      if(level != it.getDepth()){
        continue;
      }
      double occ = it->getOccupancy();
      if(occ < occ_thresh){
        continue;
      }
      glColor4ub(it->getColor().r, it->getColor().g, it->getColor().b, 128 /*basic visibility*/ + (occ - occ_thresh) * stretch_factor );
      float halfsize = it.getSize()/2.0;
      float x = it.getX(); 
      float y = it.getY(); 
      float z = it.getZ(); 
      //Front
      glVertex3f(x-halfsize,y-halfsize,z-halfsize);
      glVertex3f(x-halfsize,y+halfsize,z-halfsize);
      glVertex3f(x+halfsize,y+halfsize,z-halfsize);

      glVertex3f(x-halfsize,y-halfsize,z-halfsize);
      glVertex3f(x+halfsize,y+halfsize,z-halfsize);
      glVertex3f(x+halfsize,y-halfsize,z-halfsize);

      //Back
      glVertex3f(x-halfsize,y-halfsize,z+halfsize);
      glVertex3f(x+halfsize,y-halfsize,z+halfsize);
      glVertex3f(x+halfsize,y+halfsize,z+halfsize);

      glVertex3f(x-halfsize,y-halfsize,z+halfsize);
      glVertex3f(x+halfsize,y+halfsize,z+halfsize);
      glVertex3f(x-halfsize,y+halfsize,z+halfsize);

      //Left
      glVertex3f(x-halfsize,y-halfsize,z-halfsize);
      glVertex3f(x-halfsize,y-halfsize,z+halfsize);
      glVertex3f(x-halfsize,y+halfsize,z+halfsize);

      glVertex3f(x-halfsize,y-halfsize,z-halfsize);
      glVertex3f(x-halfsize,y+halfsize,z+halfsize);
      glVertex3f(x-halfsize,y+halfsize,z-halfsize);

      //Right
      glVertex3f(x+halfsize,y-halfsize,z-halfsize);
      glVertex3f(x+halfsize,y+halfsize,z-halfsize);
      glVertex3f(x+halfsize,y+halfsize,z+halfsize);

      glVertex3f(x+halfsize,y-halfsize,z-halfsize);
      glVertex3f(x+halfsize,y+halfsize,z+halfsize);
      glVertex3f(x+halfsize,y-halfsize,z+halfsize);

      //?
      glVertex3f(x-halfsize,y-halfsize,z-halfsize);
      glVertex3f(x+halfsize,y-halfsize,z-halfsize);
      glVertex3f(x+halfsize,y-halfsize,z+halfsize);

      glVertex3f(x-halfsize,y-halfsize,z-halfsize);
      glVertex3f(x+halfsize,y-halfsize,z+halfsize);
      glVertex3f(x-halfsize,y-halfsize,z+halfsize);

      //?
      glVertex3f(x-halfsize,y+halfsize,z-halfsize);
      glVertex3f(x-halfsize,y+halfsize,z+halfsize);
      glVertex3f(x+halfsize,y+halfsize,z+halfsize);

      glVertex3f(x-halfsize,y+halfsize,z-halfsize);
      glVertex3f(x+halfsize,y+halfsize,z+halfsize);
      glVertex3f(x+halfsize,y+halfsize,z-halfsize);
    }
    glEnd();
  }
}

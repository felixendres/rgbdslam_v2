#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/icp.h>
#include "parameter_server.h"
#include <pcl/filters/filter.h>
#include "scoped_timer.h"

void filterCloud(const pointcloud_type& cloud_in, pointcloud_type& cloud_out, int desired_size){
  ScopedTimer s(__FUNCTION__);
  cloud_out.clear();

  std::vector<int> non_nan_indices;
  non_nan_indices.reserve(cloud_in.size());
  for (unsigned int i=0; i<cloud_in.size(); i++ ){
    if (!isnan(cloud_in.points.at(i).z)) 
      non_nan_indices.push_back(i);
  }


  cloud_out.reserve(desired_size+1); //just against off-by-one error
  float step = non_nan_indices.size()/static_cast<float>(desired_size);
  step =  step < 1.0 ? 1.0 : step; //only skip, don't use points more than once
  for (float i=0; i<non_nan_indices.size(); i+=step ){
    unsigned int index = non_nan_indices.at(static_cast<unsigned int>(i));
    cloud_out.push_back(cloud_in.points.at(index));
  }
  cloud_out.width=cloud_out.size();
  cloud_out.height=1;
  cloud_out.is_dense=true;
  ROS_INFO("Subsampled cloud to: %zu", cloud_out.size());
  
}

Eigen::Matrix4f icpAlignment(pointcloud_type::Ptr cloud_1, pointcloud_type::Ptr cloud_2, Eigen::Matrix4f initial_guess){
  ScopedTimer s(__FUNCTION__);
  pcl::IterativeClosestPoint<point_type, point_type>* icp = NULL;
  std::string icp_method = ParameterServer::instance()->get<std::string>("icp_method") ;
  if(icp_method == "icp"){
    icp = new pcl::IterativeClosestPoint<point_type, point_type>();
  } else if (icp_method == "icp_nl"){
    icp = new pcl::IterativeClosestPointNonLinear<point_type, point_type>();
  } else{
    ROS_WARN("Unknown icp method \"%s\". Using regular icp", icp_method.c_str());
    icp = new pcl::IterativeClosestPoint<point_type, point_type>();
  }


  pointcloud_type::Ptr filtered_in(new pointcloud_type);
  pointcloud_type::Ptr filtered_out(new pointcloud_type);
  std::vector<int> indices_nonused;
  // Set the input source and target
  icp->setInputCloud(cloud_1);
  icp->setInputTarget(cloud_2);
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp->setMaxCorrespondenceDistance(0.05);
  // Set the maximum number of iterations (criterion 1)
  icp->setMaximumIterations(50);
  // Set the transformation epsilon (criterion 2)
  icp->setTransformationEpsilon(1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp->setEuclideanFitnessEpsilon(1);
  // Perform the alignment
  pointcloud_type cloud_registered;//Not used
  icp->align(cloud_registered, initial_guess);
  // Obtain the transformation that aligned cloud_source to cloud_source_registered
  Eigen::Matrix4f transformation = icp->getFinalTransformation();
  std::cout << "has converged:" << icp->hasConverged() << " score: " << icp->getFitnessScore() << std::endl;
  if(icp->hasConverged()){
    delete icp;
    return transformation;
  } else {
    delete icp;
    return initial_guess;
  }

}



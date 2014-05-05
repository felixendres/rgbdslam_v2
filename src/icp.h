#ifndef PCL_ICP_H
#define PCL_ICP_H
#include "parameter_server.h"
Eigen::Matrix4f icpAlignment(pointcloud_type::Ptr cloud_in, pointcloud_type::Ptr cloud_out, Eigen::Matrix4f initial_guess);
///Filter NaNs. Uniformly subsample the remaining points to achieve the desired size
void filterCloud(const pointcloud_type& cloud_1, pointcloud_type& cloud_2, int desired_size);
#endif


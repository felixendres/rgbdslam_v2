#ifndef RGBD_SLAM_MISC2_H_
#define RGBD_SLAM_MISC2_H_
/* This file is part of RGBDSLAM.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "parameter_server.h"

inline double depth_std_dev(double depth)
{
  // From Khoselham and Elberink?
  double depth_std_dev = ParameterServer::instance()->get<double>("sigma_depth");
  // Previously used 0.006 from information on http://www.ros.org/wiki/openni_kinect/kinect_accuracy;
  // ...using 2sigma = 95%ile
  //static const double depth_std_dev  = 0.006;
  return depth_std_dev * depth * depth;
}
//Functions without dependencies
inline double depth_covariance(double depth)
{
  double stddev = depth_std_dev(depth);
  return stddev * stddev;
}

inline Eigen::Matrix3d point_information_matrix(double distance)
{
  Eigen::Matrix3d inf_mat = Eigen::Matrix3d::Identity();
  /* Std dev of 1 pixel in xy boils down to identity for 0,0 and 1,1:
  inf_mat(0,0) = 1.0/1.0;//-> 1/pixel_dist*pixel_dist
  inf_mat(1,1) = 1.0/1.0; 
  */
  inf_mat(2,2) = 1.0/depth_covariance(distance);

  return inf_mat;
}
#endif

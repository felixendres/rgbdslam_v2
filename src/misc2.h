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
  static double depth_std_dev = ParameterServer::instance()->get<double>("sigma_depth");
  // Previously used 0.006 from information on http://www.ros.org/wiki/openni_kinect/kinect_accuracy;
  // ...using 2sigma = 95%ile
  //static const double depth_std_dev  = 0.006;
  return depth_std_dev * depth * depth;
}
//Functions without dependencies
inline double depth_covariance(double depth)
{
  static double stddev = depth_std_dev(depth);
  static double cov = stddev * stddev;
  return cov;
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

inline void backProject(const float& fxinv, const float& fyinv,
                        const float& cx, const float& cy,
                        const float& u,  const float& v, const float& z,
                        float& out_x,  float& out_y, float& out_z)
{
    //If depth is distance, not distance projected onto optical axis:
  /*
    float tmpx = (u-cx) * fxinv;
    float tmpy = (v-cy) * fyinv;
    out_z = z / std::sqrt(tmpx*tmpx + tmpy*tmpy + 1 ) ;
    out_x = tmpx * out_z;
    out_y = tmpy * out_z;
  */
  out_x = (u - cx) * z * fxinv;
  out_y = (v - cy) * z * fyinv;
  out_z = z;
}
#endif

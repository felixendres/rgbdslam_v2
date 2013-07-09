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


inline double depth_std_dev(double depth)
{
  // From Khoselham and Elberink?
  static const double depth_cov_z_factor = 1.425e-3;
  // Previously used 0.0075 from information on http://www.ros.org/wiki/openni_kinect/kinect_accuracy;
  return depth_cov_z_factor * depth * depth;
}
//Functions without dependencies
inline double depth_covariance(double depth)
{
  double stddev = depth_std_dev(depth);
  return stddev * stddev;
}

#endif

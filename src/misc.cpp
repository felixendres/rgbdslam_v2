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
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <QString>
#include <QMatrix4x4>
#include <ctime>
#include <limits>
#include <algorithm>
#include "parameter_server.h"
#include <cv.h>
#include "scoped_timer.h"
#include "header.h"

#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/vertex_se3.h"

#include <pcl_ros/transforms.h>
#include "pcl/common/io.h"
#include "pcl/common/distances.h"

#if CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION >= 4
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#endif

#include <omp.h>
#include "misc2.h"
#include "point_types.h"

//For the observability test
#include <boost/math/distributions/chi_squared.hpp>
#include <numeric>

#include "feature_adjuster.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>


static void getCameraIntrinsics(float& fx, float& fy, float& cx, float& cy, const sensor_msgs::CameraInfo& cam_info) 
{
  ParameterServer* ps = ParameterServer::instance();
  fx = ps->get<double>("depth_camera_fx") != 0 ? ps->get<double>("depth_camera_fx") : cam_info.K[0];
  fy = ps->get<double>("depth_camera_fy") != 0 ? ps->get<double>("depth_camera_fy") : cam_info.K[4];
  cx = ps->get<double>("depth_camera_cx") != 0 ? ps->get<double>("depth_camera_cx") : cam_info.K[2];
  cy = ps->get<double>("depth_camera_cy") != 0 ? ps->get<double>("depth_camera_cy") : cam_info.K[5];
}
static void getCameraIntrinsicsInverseFocalLength(float& fxinv, float& fyinv, float& cx, float& cy, const sensor_msgs::CameraInfo& cam_info) 
{
  getCameraIntrinsics(fxinv, fyinv, cx, cy, cam_info);
  fxinv = 1./ fxinv;
  fyinv = 1./ fyinv;
}


void printQMatrix4x4(const char* name, const QMatrix4x4& m){
    ROS_DEBUG("QMatrix %s:", name);
    ROS_DEBUG("%f\t%f\t%f\t%f", m(0,0), m(0,1), m(0,2), m(0,3));
    ROS_DEBUG("%f\t%f\t%f\t%f", m(1,0), m(1,1), m(1,2), m(1,3));
    ROS_DEBUG("%f\t%f\t%f\t%f", m(2,0), m(2,1), m(2,2), m(2,3));
    ROS_DEBUG("%f\t%f\t%f\t%f", m(3,0), m(3,1), m(3,2), m(3,3));
}

void printTransform(const char* name, const tf::StampedTransform t) {
    ROS_INFO_STREAM(name << ": Translation " << t.getOrigin().x() << " " << t.getOrigin().y() << " " << t.getOrigin().z());
    ROS_INFO_STREAM(name << ": Rotation " << t.getRotation().getX() << " " << t.getRotation().getY() << " " << t.getRotation().getZ() << " " << t.getRotation().getW());
    ROS_INFO_STREAM(name << ": From " << t.frame_id_ << " to " << t.child_frame_id_ << " at " << t.stamp_.sec << ":" << std::setfill('0') << std::setw(9) << t.stamp_.nsec);
}
void printTransform(const char* name, const tf::Transform t) {
    ROS_INFO_STREAM(name << ": Translation " << t.getOrigin().x() << " " << t.getOrigin().y() << " " << t.getOrigin().z());
    ROS_INFO_STREAM(name << ": Rotation " << t.getRotation().getX() << " " << t.getRotation().getY() << " " << t.getRotation().getZ() << " " << t.getRotation().getW());
}

void logTransform(QTextStream& out, const tf::Transform& t, double timestamp, const char* label) {
    if(label) out << label << ": ";
    out << timestamp << " " << t.getOrigin().x() << " " << t.getOrigin().y() << " " << t.getOrigin().z() << " " << t.getRotation().getX() << " " << t.getRotation().getY() << " " << t.getRotation().getZ() << " " << t.getRotation().getW() << "\n";
}


QMatrix4x4 g2o2QMatrix(const g2o::SE3Quat se3) {
    Eigen::Matrix<float, 4, 4> m = se3.to_homogeneous_matrix().cast<float>(); //_Matrix< 4, 4, double >
    ROS_DEBUG_STREAM("Eigen Matrix:\n" << m);
    QMatrix4x4 qmat( static_cast<float*>( m.data()), 4, 4);
    // g2o/Eigen seems to use a different row-major/column-major array layout
    printQMatrix4x4("from conversion", qmat.transposed());//thus the transposes
    return qmat.transposed();//thus the transposes
}

tf::Transform g2o2TF(const g2o::SE3Quat se3) {
    tf::Transform result;
    tf::Vector3 translation;
    translation.setX(se3.translation().x());
    translation.setY(se3.translation().y());
    translation.setZ(se3.translation().z());

    tf::Quaternion rotation;
    rotation.setX(se3.rotation().x());
    rotation.setY(se3.rotation().y());
    rotation.setZ(se3.rotation().z());
    rotation.setW(se3.rotation().w());

    result.setOrigin(translation);
    result.setRotation(rotation);
    //printTransform("from conversion", result);
    return result;
}

#ifdef HEMACLOUDS
void transformAndAppendPointCloud (const pointcloud_type &cloud_in, 
                                   pcl::PointCloud<hema::PointXYZRGBCamSL> &cloud_to_append_to,
                                   const tf::Transform transformation, float max_Depth, int idx)
{
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix(transformation, eigen_transform);
    size_t original_size = cloud_to_append_to.size();
    cloud_to_append_to.points.reserve(cloud_in.size()+original_size);

    if(cloud_to_append_to.points.size() ==0){
        cloud_to_append_to.header   = cloud_in.header;
        cloud_to_append_to.width    = 0;
        cloud_to_append_to.height   = 0;
        cloud_to_append_to.is_dense = false;
    }

    Eigen::Matrix3f rot   = eigen_transform.block<3, 3> (0, 0);
    Eigen::Vector3f trans = eigen_transform.block<3, 1> (0, 3);
    size_t i = 0;
    for (; i < cloud_in.points.size (); ++i)
    { 
      const point_type& in_pt = cloud_in[i];
      const Eigen::Map<Eigen::Vector3f> vec_in (const_cast<float*>(&in_pt.x), 3, 1);
      hema::PointXYZRGBCamSL out_pt;
      Eigen::Map<Eigen::Vector3f> vec_out (&out_pt.x, 3, 1);
      //filter out points with a range greater than the given Parameter or do nothing if negativ
      float distance = vec_in.norm();
      out_pt.distance = distance;
      out_pt.cameraIndex = idx;
      out_pt.segment = 0;
      out_pt.label = 0;
#ifndef RGB_IS_4TH_DIM
      out_pt.rgb = in_pt.rgb;
#else
      out_pt.data[3] = in_pt.data[3];
#endif
      if(max_Depth >= 0 && max_Depth > distance){//Erase coordinates high-noise points if desired
         vec_out[0]= vec_out[1]= vec_out[2]= std::numeric_limits<float>::quiet_NaN();
      }
      vec_out = rot * vec_in + trans;//Might be nan, but whatever
      cloud_to_append_to.push_back(out_pt);
    }
}


#else //Now #if HEMACLOUDS
//From: /opt/ros/unstable/stacks/perception_pcl/pcl/src/pcl/registration/transforms.hpp
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Apply an affine transform defined by an Eigen Transform
  * \param cloud_in the input point cloud
  * \param cloud_to_append_to the transformed cloud will be appended to this one
  * \param transform a tf::Transform stating the transformation of cloud_to_append_to relative to cloud_in
  * \note The density of the point cloud is lost, if parameter preserve_raster_on_save is set, as NaNs will be copied to keep raster structure
  * \note Can not(?) be used with cloud_in equal to cloud_to_append_to
  */
//template <typename PointT> void
//transformAndAppendPointCloud (const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_to_append_to,
//                              const tf::Transform transformation)
void transformAndAppendPointCloud (const pointcloud_type &cloud_in, 
                                   pointcloud_type &cloud_to_append_to,
                                   const tf::Transform transformation, float max_Depth, int )
{
    bool compact = !ParameterServer::instance()->get<bool>("preserve_raster_on_save");
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix(transformation, eigen_transform);
    unsigned int cloud_to_append_to_original_size = cloud_to_append_to.size();
    if(cloud_to_append_to.points.size() ==0){
        cloud_to_append_to.header   = cloud_in.header;
        cloud_to_append_to.width    = 0;
        cloud_to_append_to.height   = 0;
        cloud_to_append_to.is_dense = false;
    }

    ROS_DEBUG("max_Depth = %f", max_Depth);
    ROS_DEBUG("cloud_to_append_to_original_size = %i", cloud_to_append_to_original_size);

    //Append all points untransformed. This also copies other fields, e.g rgb
    cloud_to_append_to += cloud_in;

    Eigen::Matrix3f rot   = eigen_transform.block<3, 3> (0, 0);
    Eigen::Vector3f trans = eigen_transform.block<3, 1> (0, 3);
    point_type origin = point_type();
    origin.x = 0;
    origin.y = 0;
    origin.z = 0;
    int j = 0; //Output index
    for (size_t i = 0; i < cloud_in.points.size (); ++i) //i: Input index
    { 
      Eigen::Map<Eigen::Vector3f> p_in (const_cast<float*>(&cloud_in.points[i].x), 3, 1);
      Eigen::Map<Eigen::Vector3f> p_out (&cloud_to_append_to.points[j+cloud_to_append_to_original_size].x, 3, 1);
      if(compact){ cloud_to_append_to.points[j+cloud_to_append_to_original_size] = cloud_in.points[i]; }
      //filter out points with a range greater than the given Parameter or do nothing if negativ
      if(max_Depth >= 0){
        if(pcl::squaredEuclideanDistance(cloud_in.points[i], origin) > max_Depth*max_Depth){
           p_out[0]= std::numeric_limits<float>::quiet_NaN();
           p_out[1]= std::numeric_limits<float>::quiet_NaN();
           p_out[2]= std::numeric_limits<float>::quiet_NaN();
           if(!compact) j++; 
           continue;
         }
      }
      if (pcl_isnan (cloud_in.points[i].x) || pcl_isnan (cloud_in.points[i].y) || pcl_isnan (cloud_in.points[i].z)){
         if(!compact) j++; //Skip, but leave output as is
         continue;
      }
      p_out = rot * p_in + trans;
      j++;
    }
    if(compact){
      cloud_to_append_to.points.resize(j+cloud_to_append_to_original_size);
      cloud_to_append_to.width    = 1;
      cloud_to_append_to.height   = j+cloud_to_append_to_original_size;
    }
}
#endif //HEMACLOUDS

//do spurious type conversions
geometry_msgs::Point pointInWorldFrame(const Eigen::Vector4f& point3d, const g2o::VertexSE3::EstimateType& transf)
{
    Eigen::Vector3d tmp(point3d[0], point3d[1], point3d[2]);
    tmp = transf * tmp; //transform to world frame
    geometry_msgs::Point p;
    p.x = tmp.x(); 
    p.y = tmp.y(); 
    p.z = tmp.z();
    return p;
}
void mat2dist(const Eigen::Matrix4f& t, double &dist){
    dist = sqrt(t(0,3)*t(0,3)+t(1,3)*t(1,3)+t(2,3)*t(2,3));
}
///Get euler angles from affine matrix (helper for isBigTrafo)
void mat2RPY(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw) {
    roll = atan2(t(2,1),t(2,2));
    pitch = atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2)));
    yaw = atan2(t(1,0),t(0,0));
}
void mat2components(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw, double& dist){

  mat2RPY(t, roll,pitch,yaw);
  mat2dist(t, dist);

  roll = roll/M_PI*180;
  pitch = pitch/M_PI*180;
  yaw = yaw/M_PI*180;

}

void trafoSize(const Eigen::Isometry3d& t, double& angle, double& dist){
  angle = acos((t.rotation().trace() -1)/2 ) *180.0 / M_PI;
  dist = t.translation().norm();
  ROS_INFO("Rotation:% 4.2f, Distance: % 4.3fm", angle, dist);
}

bool isBigTrafo(const Eigen::Isometry3d& t){
    double angle, dist;
    trafoSize(t, angle, dist);
    return (dist > ParameterServer::instance()->get<double>("min_translation_meter") ||
    	      angle > ParameterServer::instance()->get<double>("min_rotation_degree"));
}

// true iff edge qualifies for generating a new vertex
bool isBigTrafo(const Eigen::Matrix4f& t){
    double roll, pitch, yaw, dist;

    mat2RPY(t, roll,pitch,yaw);
    mat2dist(t, dist);

    roll = roll/M_PI*180;
    pitch = pitch/M_PI*180;
    yaw = yaw/M_PI*180;

    double max_angle = std::max(roll,std::max(pitch,yaw));

    return (dist > ParameterServer::instance()->get<double>("min_translation_meter")
    		|| max_angle > ParameterServer::instance()->get<double>("min_rotation_degree"));
}


bool isSmallTrafo(const Eigen::Isometry3d& t, double seconds){
    if(seconds <= 0.0){
      ROS_WARN("Time delta invalid: %f. Skipping test for small transformation", seconds);
      return true;
    }
    
    double angle_around_axis, dist;
    trafoSize(t, angle_around_axis, dist);

    ParameterServer* ps =  ParameterServer::instance();
    return (dist / seconds < ps->get<double>("max_translation_meter") &&
            angle_around_axis / seconds < ps->get<double>("max_rotation_degree"));
}

bool isSmallTrafo(const g2o::SE3Quat& t, double seconds){
    if(seconds <= 0.0){
      ROS_WARN("Time delta invalid: %f. Skipping test for small transformation", seconds);
      return true;
    }
    float angle_around_axis = 2.0*acos(t.rotation().w()) *180.0 / M_PI;
    float dist = t.translation().norm();
    QString infostring;
    ROS_DEBUG("Rotation:% 4.2f, Distance: % 4.3fm", angle_around_axis, dist);
    infostring.sprintf("Rotation:% 4.2f, Distance: % 4.3fm", angle_around_axis, dist);
    //Q_EMIT setGUIInfo2(infostring);
    ParameterServer* ps =  ParameterServer::instance();
    //Too big fails too
    return (dist / seconds < ps->get<double>("max_translation_meter") &&
            angle_around_axis / seconds < ps->get<double>("max_rotation_degree"));
}

bool isBigTrafo(const g2o::SE3Quat& t){
    float angle_around_axis = 2.0*acos(t.rotation().w()) *180.0 / M_PI;
    float dist = t.translation().norm();
    QString infostring;
    ROS_DEBUG("Rotation:% 4.2f, Distance: % 4.3fm", angle_around_axis, dist);
    infostring.sprintf("Rotation:% 4.2f, Distance: % 4.3fm", angle_around_axis, dist);
    //Q_EMIT setGUIInfo2(infostring);
    ParameterServer* ps =  ParameterServer::instance();
    return (dist > ps->get<double>("min_translation_meter") ||
            angle_around_axis > ps->get<double>("min_rotation_degree"));
}


/*
bool overlappingViews(LoadedEdge3D edge){
    //opening angles
   double alpha = 57.0/180.0*M_PI;
   double beta = 47.0/180.0*M_PI; 
   //assumes robot coordinate system (x is front, y is left, z is up)
   Eigen::Matrix<double, 4,3> cam1;
   cam1 <<  1.0, std::tan(alpha),  std::tan(beta),//upper left
                                       1.0, -std::tan(alpha), std::tan(beta),//upper right
                                       1.0, std::tan(alpha), -std::tan(beta),//lower left
                                       1.0, -std::tan(alpha),-std::tan(beta);//lower right
   return false;
}
bool triangleRayIntersection(Eigen::Vector3d triangle1,Eigen::Vector3d triangle2, 
                             Eigen::Vector3d ray_origin, Eigen::Vector3d ray){
    Eigen::Matrix3d m;
    m.col(2) = -ray;
    m.col(0) = triangle1;
    m.col(1) = triangle2;
    Eigen::Vector3d lengths = m.inverse() * ray_origin;
    return (lengths(0) < 0 && lengths(1) > 0 && lengths(2) > 0 );
}
*/


g2o::SE3Quat tf2G2O(const tf::Transform t) 
{
  Eigen::Quaterniond eigen_quat(t.getRotation().getW(), t.getRotation().getX(), t.getRotation().getY(), t.getRotation().getZ());
  Eigen::Vector3d translation(t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z());
  g2o::SE3Quat result(eigen_quat, translation);
  return result;
}

g2o::SE3Quat eigen2G2O(const Eigen::Matrix4d& eigen_mat) 
{
  Eigen::Affine3d eigen_transform(eigen_mat);
  Eigen::Quaterniond eigen_quat(eigen_transform.rotation());
  Eigen::Vector3d translation(eigen_mat(0, 3), eigen_mat(1, 3), eigen_mat(2, 3));
  g2o::SE3Quat result(eigen_quat, translation);

  return result;
}

//Little debugging helper functions
std::string openCVCode2String(unsigned int code){
  switch(code){
    case 0 : return std::string("CV_8UC1" );
    case 8 : return std::string("CV_8UC2" );
    case 16: return std::string("CV_8UC3" );
    case 24: return std::string("CV_8UC4" );
    case 2 : return std::string("CV_16UC1");
    case 10: return std::string("CV_16UC2");
    case 18: return std::string("CV_16UC3");
    case 26: return std::string("CV_16UC4");
    case 5 : return std::string("CV_32FC1");
    case 13: return std::string("CV_32FC2");
    case 21: return std::string("CV_32FC3");
    case 29: return std::string("CV_32FC4");
  }
  return std::string("Unknown");
}

void printMatrixInfo(const cv::Mat& image, std::string name){
  ROS_INFO_STREAM("Matrix " << name << " - Type:" << openCVCode2String(image.type()) <<  " rows: " <<  image.rows  <<  " cols: " <<  image.cols);
}


void depthToCV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img){
  //Process images
  if(depth_img.type() == CV_32FC1){
    depth_img.convertTo(mono8_img, CV_8UC1, 100,0); //milimeter (scale of mono8_img does not matter)
  }
  else if(depth_img.type() == CV_16UC1){
    mono8_img = cv::Mat(depth_img.size(), CV_8UC1);
    cv::Mat float_img;
    depth_img.convertTo(mono8_img, CV_8UC1, 0.05, -25); //scale to 2cm
    depth_img.convertTo(float_img, CV_32FC1, 0.001, 0);//From mm to m(scale of depth_img matters)
    depth_img = float_img;
  }
  else {
    printMatrixInfo(depth_img, "Depth Image");
    ROS_ERROR_STREAM("Don't know how to handle depth image of type "<< openCVCode2String(depth_img.type()));
  }
}

bool asyncFrameDrop(ros::Time depth, ros::Time rgb)
{
  long rgb_timediff = abs(static_cast<long>(rgb.nsec) - static_cast<long>(depth.nsec));
  if(rgb_timediff > 33333333){
     ROS_DEBUG("Depth image time: %d - %d", depth.sec,   depth.nsec);
     ROS_DEBUG("RGB   image time: %d - %d", rgb.sec, rgb.nsec);
     ROS_INFO("Depth and RGB image off more than 1/30sec: %li (nsec)", rgb_timediff);
     if(ParameterServer::instance()->get<bool>("drop_async_frames")){
       ROS_WARN("Asynchronous frames ignored. See parameters if you want to keep async frames.");
       return true;
     }
  } else {
     ROS_DEBUG("Depth image time: %d - %d", depth.sec,   depth.nsec);
     ROS_DEBUG("RGB   image time: %d - %d", rgb.sec, rgb.nsec);
  }
  return false;
}


///\cond
/** Union for easy "conversion" of rgba data */
typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;
///\endcond

pointcloud_type* createXYZRGBPointCloud (const cv::Mat& depth_img, 
                                         const cv::Mat& rgb_img,
                                         const sensor_msgs::CameraInfoConstPtr& cam_info) 
{
  ScopedTimer s(__FUNCTION__);
  pointcloud_type* cloud (new pointcloud_type() );
  cloud->is_dense         = false; //single point of view, 2d rasterized NaN where no depth value was found

  float fxinv, fyinv, cx, cy;
  getCameraIntrinsicsInverseFocalLength(fxinv, fyinv, cx, cy, *cam_info);
  ParameterServer* ps = ParameterServer::instance();
  int data_skip_step = ps->get<int>("cloud_creation_skip_step");
  if(depth_img.rows % data_skip_step != 0 || depth_img.cols % data_skip_step != 0){
    ROS_WARN("The parameter cloud_creation_skip_step is not a divisor of the depth image dimensions. This will most likely crash the program!");
  }
  cloud->height = ceil(depth_img.rows / static_cast<float>(data_skip_step));
  cloud->width = ceil(depth_img.cols / static_cast<float>(data_skip_step));
  int pixel_data_size = 3;
  //Assume RGB
  char red_idx = 0, green_idx = 1, blue_idx = 2;
  if(rgb_img.type() == CV_8UC1) pixel_data_size = 1;
  else if(ps->get<bool>("encoding_bgr")) { red_idx = 2; blue_idx = 0; }

  unsigned int color_row_step, color_pix_step, depth_pix_step, depth_row_step;
  color_pix_step = pixel_data_size * (rgb_img.cols / cloud->width);
  color_row_step = pixel_data_size * (rgb_img.rows / cloud->height -1 ) * rgb_img.cols;
  depth_pix_step = (depth_img.cols / cloud->width);
  depth_row_step = (depth_img.rows / cloud->height -1 ) * depth_img.cols;

  cloud->points.resize (cloud->height * cloud->width);

  //const uint8_t* rgb_buffer = &rgb_msg->data[0];

  // depth_img already has the desired dimensions, but rgb_img may be higher res.
  int color_idx = 0 * color_pix_step - 0 * color_row_step, depth_idx = 0; //FIXME: Hack for hard-coded calibration of color to depth
  double depth_scaling = ps->get<double>("depth_scaling_factor");
  float max_depth = ps->get<double>("maximum_depth");
  float min_depth = ps->get<double>("minimum_depth");
  if(max_depth < 0.0) max_depth = std::numeric_limits<float>::infinity();

  //Main Looop
  pointcloud_type::iterator pt_iter = cloud->begin();
  for (int v = 0; v < (int)rgb_img.rows; v += data_skip_step, color_idx += color_row_step, depth_idx += depth_row_step)
  {
    for (int u = 0; u < (int)rgb_img.cols; u += data_skip_step, color_idx += color_pix_step, depth_idx += depth_pix_step, ++pt_iter)
    {
      if(pt_iter == cloud->end()){
        break;
      }
      point_type& pt = *pt_iter;
      if(u < 0 || v < 0 || u >= depth_img.cols || v >= depth_img.rows){
        pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
        continue;
      }

      float Z = depth_img.at<float>(depth_idx) * depth_scaling;

      // Check for invalid measurements
      if (!(Z >= min_depth)) //Should also be trigger on NaN//std::isnan (Z))
      {
        pt.x = (u - cx) * 1.0 * fxinv; //FIXME: better solution as to act as at 1meter?
        pt.y = (v - cy) * 1.0 * fyinv;
        pt.z = std::numeric_limits<float>::quiet_NaN();
      }
      else // Fill in XYZ
      {
        backProject(fxinv, fyinv, cx, cy, u, v, Z, pt.x, pt.y, pt.z);
      }
      // Fill in color
      RGBValue color;
      if(color_idx > 0 && color_idx < rgb_img.total()*color_pix_step){ //Only necessary because of the color_idx offset 
        if(pixel_data_size == 3){
          color.Red   = rgb_img.at<uint8_t>(color_idx + red_idx);
          color.Green = rgb_img.at<uint8_t>(color_idx + green_idx);
          color.Blue  = rgb_img.at<uint8_t>(color_idx + blue_idx);
        } else {
          color.Red   = color.Green = color.Blue  = rgb_img.at<uint8_t>(color_idx);
        }
        color.Alpha = 0;
#ifndef RGB_IS_4TH_DIM
        pt.rgb = color.float_value;
#else
        pt.data[3] = color.float_value;
#endif
      }
    }
  }

  return cloud;
}
pointcloud_type* createXYZRGBPointCloud (const sensor_msgs::ImageConstPtr& depth_msg, 
                                         const sensor_msgs::ImageConstPtr& rgb_msg,
                                         const sensor_msgs::CameraInfoConstPtr& cam_info) 
{
  ScopedTimer s(__FUNCTION__);
  pointcloud_type* cloud (new pointcloud_type() );
  myHeader h(0, depth_msg->header.stamp,  rgb_msg->header.frame_id);
  cloud->header = h;
  //cloud->header.stamp     = depth_msg->header.stamp;
  //cloud->header.frame_id  = rgb_msg->header.frame_id;
  cloud->is_dense         = false; //single point of view, 2d rasterized NaN where no depth value was found

  float fxinv, fyinv, cx, cy;
  getCameraIntrinsicsInverseFocalLength(fxinv, fyinv, cx, cy, *cam_info);
  ParameterServer* ps = ParameterServer::instance();
  int data_skip_step = ps->get<int>("cloud_creation_skip_step");
  cloud->height = depth_msg->height / data_skip_step;
  cloud->width = depth_msg->width / data_skip_step;
  //Reciprocal to use multiplication in loop, not slower division
  int pixel_data_size = 3;
  char red_idx = 0, green_idx = 1, blue_idx = 2;
  if(rgb_msg->encoding.compare("mono8") == 0) pixel_data_size = 1;
  if(rgb_msg->encoding.compare("bgr8") == 0) { red_idx = 2; blue_idx = 0; }


  ROS_ERROR_COND(pixel_data_size == 0, "Unknown image encoding: %s!", rgb_msg->encoding.c_str());
  unsigned int color_row_step, color_pix_step, depth_pix_step, depth_row_step;
  color_pix_step = pixel_data_size * (rgb_msg->width / cloud->width);
  color_row_step = pixel_data_size * (rgb_msg->height / cloud->height -1 ) * rgb_msg->width;
  depth_pix_step = (depth_msg->width / cloud->width);
  depth_row_step = (depth_msg->height / cloud->height -1 ) * depth_msg->width;

  cloud->points.resize (cloud->height * cloud->width);

  const uint8_t* rgb_buffer = &rgb_msg->data[0];

  // depth_msg already has the desired dimensions, but rgb_msg may be higher res.
  int color_idx = 0, depth_idx = 0;
  double depth_scaling = ps->get<double>("depth_scaling_factor");
  float max_depth = ps->get<double>("maximum_depth");
  if(max_depth < 0.0) max_depth = std::numeric_limits<float>::infinity();

  const float* depth_buffer = reinterpret_cast<const float*>(&depth_msg->data[0]);
  pointcloud_type::iterator pt_iter = cloud->begin ();
  for (int v = 0; v < (int)rgb_msg->height; v += data_skip_step, color_idx += color_row_step, depth_idx += depth_row_step)
  {
    for (int u = 0; u < (int)rgb_msg->width; u += data_skip_step, color_idx += color_pix_step, depth_idx += depth_pix_step, ++pt_iter)
    {
      point_type& pt = *pt_iter;
      float Z = depth_buffer[depth_idx] * depth_scaling;

      // Check for invalid measurements
      if (!(Z <= max_depth)) //Should also be trigger on NaN//std::isnan (Z))
      {
        pt.z = std::numeric_limits<float>::quiet_NaN();
      }
      else // Fill in XYZ
      {
        backProject(fxinv, fyinv, cx, cy, u, v, Z, pt.x, pt.y, pt.z);
      }
      // Fill in color
      RGBValue color;
      if(pixel_data_size == 3){
        color.Red   = rgb_buffer[color_idx + red_idx];
        color.Green = rgb_buffer[color_idx + green_idx];
        color.Blue  = rgb_buffer[color_idx + blue_idx];
      } else {
        color.Red   = color.Green = color.Blue  = rgb_buffer[color_idx];
      }
      color.Alpha = 0;
#ifndef RGB_IS_4TH_DIM
      pt.rgb = color.float_value;
#else
      pt.data[3] = color.float_value;
#endif
    }
  }

  return cloud;
}

double errorFunction(const Eigen::Vector4f& x1, const double x1_depth_cov, 
                     const Eigen::Vector4f& x2, const double x2_depth_cov, 
                     const Eigen::Matrix4f& tf_1_to_2)
{
  const double cam_angle_x = 58.0/180.0*M_PI;
  const double cam_angle_y = 45.0/180.0*M_PI;
  const double cam_resol_x = 640;
  const double cam_resol_y = 480;
  const double raster_stddev_x = 2*tan(cam_angle_x/cam_resol_x);  //2pix stddev in x
  const double raster_stddev_y = 2*tan(cam_angle_y/cam_resol_y);  //2pix stddev in y
  const double raster_cov_x = raster_stddev_x * raster_stddev_x;
  const double raster_cov_y = raster_stddev_y * raster_stddev_y;

  ROS_DEBUG_COND(x1(3) != 1.0, "4th element of x1 should be 1.0, is %f", x1(3));
  ROS_DEBUG_COND(x2(3) != 1.0, "4th element of x2 should be 1.0, is %f", x2(3));
  
  Eigen::Vector3d mu_1 = x1.head<3>().cast<double>();
  Eigen::Vector3d mu_2 = x2.head<3>().cast<double>();
  Eigen::Matrix3d rotation_mat = tf_1_to_2.block(0,0,3,3).cast<double>();

  //Point 1
  Eigen::Matrix3d cov1 = Eigen::Matrix3d::Identity();
  cov1(0,0) = raster_cov_x* mu_1(2); //how big is 1px std dev in meter, depends on depth
  cov1(1,1) = raster_cov_y* mu_1(2); //how big is 1px std dev in meter, depends on depth
  cov1(2,2) = x1_depth_cov;
  //Point2
  Eigen::Matrix3d cov2 = Eigen::Matrix3d::Identity();
  cov2(0,0) = raster_cov_x* mu_2(2); //how big is 1px std dev in meter, depends on depth
  cov2(1,1) = raster_cov_y* mu_2(2); //how big is 1px std dev in meter, depends on depth
  cov2(2,2) = x2_depth_cov;

  Eigen::Matrix3d cov2inv = cov2.inverse(); // Σ₂⁻¹  

  Eigen::Vector3d mu_1_in_frame_2 = (tf_1_to_2 * x1).head<3>().cast<double>(); // μ₁⁽²⁾  = T₁₂ μ₁⁽¹⁾  
  Eigen::Matrix3d cov1_in_frame_2 = rotation_mat.transpose() * cov1 * rotation_mat;//Works since the cov is diagonal => Eig-Vec-Matrix is Identity
  Eigen::Matrix3d cov1inv_in_frame_2 = cov1_in_frame_2.inverse();// Σ₁⁻¹  

  Eigen::Matrix3d cov_sum = (cov1inv_in_frame_2 + cov2inv);
  Eigen::Matrix3d inv_cov_sum = cov_sum.inverse();
  ROS_ERROR_STREAM_COND(inv_cov_sum!=inv_cov_sum,"Sum of Covariances not invertible: \n" << cov_sum);

  Eigen::Vector3d x_ml;//Max Likelhood Position of latent point, that caused the sensor msrmnt
  x_ml = inv_cov_sum * (cov1inv_in_frame_2 * mu_1_in_frame_2 + cov2inv * mu_2); // (Σ₁⁻¹ +  Σ₂⁻¹)⁻¹(Σ₁⁻¹μ₁  +  Σ₂⁻¹μ₂)
  Eigen::Vector3d delta_mu_1 = mu_1_in_frame_2 - x_ml;
  Eigen::Vector3d delta_mu_2 = mu_2 - x_ml;
  
  float sqrd_mahalanobis_distance1 = delta_mu_1.transpose() * cov1inv_in_frame_2 * delta_mu_1;// Δx_2^T Σ Δx_2 
  float sqrd_mahalanobis_distance2 = delta_mu_2.transpose() * cov2inv * delta_mu_2; // Δx_1^T Σ Δx_1
  float bad_mahalanobis_distance = sqrd_mahalanobis_distance1 + sqrd_mahalanobis_distance2; //FIXME

  if(!(bad_mahalanobis_distance >= 0.0))
  {
    ROS_ERROR_STREAM("Non-Positive Mahalanobis Distance");
    return std::numeric_limits<double>::max();
  }
  ROS_DEBUG_STREAM_NAMED("statistics", "Mahalanobis ML: " << std::setprecision(25) << bad_mahalanobis_distance);
  return bad_mahalanobis_distance;
}

double errorFunction2(const Eigen::Vector4f& x1,
                      const Eigen::Vector4f& x2,
                      const Eigen::Matrix4d& transformation)
{
  //FIXME: Take from paramter_server or cam info
  static const double cam_angle_x = 58.0/180.0*M_PI;/*{{{*/
  static const double cam_angle_y = 45.0/180.0*M_PI;
  static const double cam_resol_x = 640;
  static const double cam_resol_y = 480;
  static const double raster_stddev_x = 3*tan(cam_angle_x/cam_resol_x);  //5pix stddev in x
  static const double raster_stddev_y = 3*tan(cam_angle_y/cam_resol_y);  //5pix stddev in y
  static const double raster_cov_x = raster_stddev_x * raster_stddev_x;
  static const double raster_cov_y = raster_stddev_y * raster_stddev_y;/*}}}*/
  static const bool use_error_shortcut = true;//ParameterServer::instance()->get<bool>("use_error_shortcut");

  bool nan1 = std::isnan(x1(2));
  bool nan2 = std::isnan(x2(2));
  if(nan1||nan2){
    //TODO: Handle Features with NaN, by reporting the reprojection error
    return std::numeric_limits<double>::max();
  }
  Eigen::Vector4d x_1 = x1.cast<double>();
  Eigen::Vector4d x_2 = x2.cast<double>();

  Eigen::Matrix4d tf_12 = transformation;
  Eigen::Vector3d mu_1 = x_1.head<3>();
  Eigen::Vector3d mu_2 = x_2.head<3>();
  Eigen::Vector3d mu_1_in_frame_2 = (tf_12 * x_1).head<3>(); // μ₁⁽²⁾  = T₁₂ μ₁⁽¹⁾  
  //New Shortcut to determine clear outliers
  if(use_error_shortcut)
  {
    double delta_sq_norm = (mu_1_in_frame_2 - mu_2).squaredNorm();
    double sigma_max_1 = std::max(raster_cov_x, depth_covariance(mu_1(2)));//Assuming raster_cov_x and _y to be approx. equal
    double sigma_max_2 = std::max(raster_cov_x, depth_covariance(mu_2(2)));//Assuming raster_cov_x and _y to be approx. equal
    if(delta_sq_norm > 2.0 * (sigma_max_1+sigma_max_2)) //FIXME: Factor 3 for mahal dist should be gotten from caller
    {
      return std::numeric_limits<double>::max();
    }
  } 

  Eigen::Matrix3d rotation_mat = tf_12.block(0,0,3,3);

  //Point 1
  Eigen::Matrix3d cov1 = Eigen::Matrix3d::Zero();
  cov1(0,0) = raster_cov_x * mu_1(2); //how big is 1px std dev in meter, depends on depth
  cov1(1,1) = raster_cov_y * mu_1(2); //how big is 1px std dev in meter, depends on depth
  cov1(2,2) = depth_covariance(mu_1(2));

  //Point2
  Eigen::Matrix3d cov2 = Eigen::Matrix3d::Zero();
  cov2(0,0) = raster_cov_x* mu_2(2); //how big is 1px std dev in meter, depends on depth
  cov2(1,1) = raster_cov_y* mu_2(2); //how big is 1px std dev in meter, depends on depth
  cov2(2,2) = depth_covariance(mu_2(2));

  Eigen::Matrix3d cov1_in_frame_2 = rotation_mat.transpose() * cov1 * rotation_mat;//Works since the cov is diagonal => Eig-Vec-Matrix is Identity

  // Δμ⁽²⁾ =  μ₁⁽²⁾ - μ₂⁽²⁾
  Eigen::Vector3d delta_mu_in_frame_2 = mu_1_in_frame_2 - mu_2;
  if(std::isnan(delta_mu_in_frame_2(2))){
    ROS_ERROR("Unexpected NaN");
    return std::numeric_limits<double>::max();
  }
  // Σc = (Σ₁ + Σ₂)
  Eigen::Matrix3d cov_mat_sum_in_frame_2 = cov1_in_frame_2 + cov2;     
  //ΔμT Σc⁻¹Δμ  
  //double sqrd_mahalanobis_distance = delta_mu_in_frame_2.transpose() * cov_mat_sum_in_frame_2.inverse() * delta_mu_in_frame_2;
  double sqrd_mahalanobis_distance = delta_mu_in_frame_2.transpose() *cov_mat_sum_in_frame_2.llt().solve(delta_mu_in_frame_2);
  
  if(!(sqrd_mahalanobis_distance >= 0.0))
  {
    return std::numeric_limits<double>::max();
  }
  return sqrd_mahalanobis_distance;
}



float getMinDepthInNeighborhood(const cv::Mat& depth, cv::Point2f center, float diameter){
    // Get neighbourhood area of keypoint
    int radius = (diameter - 1)/2;
    int top   = center.y - radius; top   = top   < 0 ? 0 : top;
    int left  = center.x - radius; left  = left  < 0 ? 0 : left;
    int bot   = center.y + radius; bot   = bot   > depth.rows ? depth.rows : bot;
    int right = center.x + radius; right = right > depth.cols ? depth.cols : right;

    cv::Mat neigborhood(depth, cv::Range(top, bot), cv::Range(left,right));
    double minZ = std::numeric_limits<float>::quiet_NaN();
    cv::minMaxLoc(neigborhood, &minZ);
    if(minZ == 0.0){ //FIXME: Why are there features with depth set to zero?
      ROS_INFO_THROTTLE(1,"Caught feature with zero in depth neighbourhood");
      minZ = std::numeric_limits<float>::quiet_NaN();
    }

    return static_cast<float>(minZ);
}


//#include "parameter_server.h" //For pointcloud definitions
#include <pcl/conversions.h>
//#include <pcl_ros/transforms.h>

//#include <Eigen/Core>
#include <math.h>
#define SQRT_2_PI 2.5066283
#define SQRT_2 1.41421
#define LOG_SQRT_2_PI = 0.9189385332

inline int round(float d)
{
  return static_cast<int>(floor(d + 0.5));
}
// Returns the probability of [-inf,x] of a gaussian distribution
double cdf(double x, double mu, double sigma)
{
	return 0.5 * (1 + erf((x - mu) / (sigma * SQRT_2)));
}

void observationLikelihood(const Eigen::Matrix4f& proposed_transformation,//new to old
                             pointcloud_type::Ptr new_pc,
                             pointcloud_type::Ptr old_pc,
                             const sensor_msgs::CameraInfo& old_cam_info,
                             double& likelihood, 
                             double& confidence,
                             unsigned int& inliers,
                             unsigned int& outliers,
                             unsigned int& occluded,
                             unsigned int& all) 
{
  ScopedTimer s(__FUNCTION__);
 
  int skip_step = ParameterServer::instance()->get<int>("emm__skip_step");
  const bool mark_outliers = ParameterServer::instance()->get<bool>("emm__mark_outliers");
  double observability_threshold = ParameterServer::instance()->get<double>("observability_threshold");
  inliers = outliers = occluded = all = 0;
  if(skip_step < 0 || observability_threshold <= 0.0){
    inliers = all = 1;
    return;
  }
  if(old_pc->width <= 1 || old_pc->height <= 1){
    //We need structured point clouds
    ROS_ERROR("Point Cloud seems to be non-structured: %u/%u (w/h). Observation can not be evaluated! ", old_pc->width, old_pc->height);
    if(ParameterServer::instance()->get<double>("voxelfilter_size") > 0){
      ROS_ERROR("The parameter voxelfilter_size is set. This is incompatible with the environment measurement model (parameter 'observability_threshold')");
    }
    inliers = all = 1;
    return;
  }
  if(old_pc->width != new_pc->width){
    ROS_ERROR("Differing cloud dimensions: %d vs %d width. Skipping observationLikelihood.", old_pc->width, new_pc->width);
    return;
  }

  pointcloud_type new_pc_transformed;
  pcl::transformPointCloud(*new_pc, new_pc_transformed, proposed_transformation);

  //Camera Calibration FIXME: Get actual values from cameraInfo (need to store in node?)
  ParameterServer* ps = ParameterServer::instance();
  float fx, fy, cx, cy;
  getCameraIntrinsics(fx, fy, cx, cy, old_cam_info);
  int cloud_creation_skip_step = 1; 
  if(ps->get<std::string>("topic_points").empty()){//downsampled cloud?
    cloud_creation_skip_step = ps->get<int>("cloud_creation_skip_step");
    fx = fx / cloud_creation_skip_step;
    fy = fy / cloud_creation_skip_step;
    cx = cx / cloud_creation_skip_step;
    cy = cy / cloud_creation_skip_step;
  }

  double sumloglikelihood = 0.0, observation_count = 0.0;
  unsigned int bad_points = 0, good_points = 0, occluded_points = 0;
  uint8_t r1 = rand() % 32, g1 = 128 + rand() % 128, b1 = 128+rand() % 128; // Mark occluded point in cyan color
  uint32_t rgb1 = ((uint32_t)r1 << 16 | (uint32_t)g1 << 8 | (uint32_t)b1);
  uint8_t r2 = 128 + rand() % 128, g2 = rand() % 32,  b2 = 128+rand() % 128; // Mark bad points in magenta color
  uint32_t rgb2 = ((uint32_t)r2 << 16 | (uint32_t)g2 << 8 | (uint32_t)b2);

//#pragma omp parallel for reduction(+: good_points, bad_points, occluded_points) 
  for(int new_ry = 0; new_ry < (int)new_pc->height; new_ry+=skip_step){
    for(int new_rx = 0; new_rx < (int)new_pc->width; new_rx+=skip_step, all++){
      //Backproject transformed new 3D point to 2d raster of old image
      //TODO: Cache this?
      point_type& p = new_pc_transformed.at(new_rx, new_ry);
      if(p.z != p.z) continue; //NaN
      if(p.z < 0) continue; // Behind the camera
      int old_rx_center = round((p.x / p.z)* fx + cx);
      int old_ry_center = round((p.y / p.z)* fy + cy);
      //ROS_INFO_COND(new_ry % 32 == 0 && new_rx % 32 == 0, "Projected point from [%d, %d] to [%d, %d]", new_rx, new_ry, old_rx_center, old_ry_center);
      if(old_rx_center >= (int)old_pc->width || old_rx_center < 0 ||
         old_ry_center >= (int)old_pc->height|| old_ry_center < 0 )
      {
        ROS_DEBUG("New point not projected into old image, skipping");
        continue;
      }
      int nbhd = 2; //1 => 3x3 neighbourhood
      bool good_point = false, occluded_point = false, bad_point = false;
      int startx = std::max(0,old_rx_center - nbhd);
      int starty = std::max(0,old_ry_center - nbhd);
      int endx = std::min(static_cast<int>(old_pc->width), old_rx_center + nbhd +1);
      int endy = std::min(static_cast<int>(old_pc->height), old_ry_center + nbhd +1);
      int neighbourhood_step = 2; //Search for depth jumps in this area
      for(int old_ry = starty; old_ry < endy; old_ry+=neighbourhood_step){
        for(int old_rx = startx; old_rx < endx; old_rx+=neighbourhood_step){

          const point_type& old_p = old_pc->at(old_rx, old_ry);
          if(old_p.z != old_p.z) continue; //NaN
          
          // likelihood for old msrmnt = new msrmnt:
          double old_sigma = cloud_creation_skip_step*depth_covariance(old_p.z);
          //TODO: (Wrong) Assumption: Transformation does not change the viewing angle. 
          double new_sigma = cloud_creation_skip_step*depth_covariance(p.z);
          //Assumption: independence of sensor noise lets us sum variances
          double joint_sigma = old_sigma + new_sigma;
          ///TODO: Compute correctly transformed new sigma in old_z direction
          
          //Cumulative of position: probability of being occluded
          double p_new_in_front = cdf(old_p.z, p.z, sqrt(joint_sigma));
          //ROS_INFO("Msrmnt. dz=%g MHD=%g sigma=%g obs_p=%g, behind_p=%g", dz, mahal_dist, sqrt(joint_sigma), observation_p, p_new_in_front);
          if( p_new_in_front < 0.001)
          { // it is in behind and outside the 99.8% interval
            occluded_point = true; //Outside, but occluded
          }
          else if( p_new_in_front < 0.999)
          { // it is inside the 99.8% interval (and not behind)
            good_point = true;
            //goto end_of_neighbourhood_loop; <-- Don't break, search for better Mahal distance
          }
          else {//It would have blocked the view from the old cam position
            bad_point = true;
            //Bad point?
          }
          //sumloglikelihood += observation_p;
          //observation_count += p_new_in_front; //Discount by probability of having been occluded
        }
      }//End neighbourhood loop
      end_of_neighbourhood_loop:
      if(good_point) {
        good_points++;
      } else if(occluded_point){
        occluded_points++;
        if(mark_outliers){
#ifndef RGB_IS_4TH_DIM
          new_pc->at(new_rx, new_ry).rgb = *reinterpret_cast<float*>(&rgb1);
          old_pc->at(old_rx_center, old_ry_center).rgb = *reinterpret_cast<float*>(&rgb1);
#else
          new_pc->at(new_rx, new_ry).data[3] = *reinterpret_cast<float*>(&rgb1);
          old_pc->at(old_rx_center, old_ry_center).data[3] = *reinterpret_cast<float*>(&rgb1);
#endif
        }
      }
      else if(bad_point){
        bad_points++;
        if(mark_outliers){
          //uint8_t r1 = 255, g1 = 0, b1 = 0; // Mark bad point in red color
#ifndef RGB_IS_4TH_DIM
          new_pc->at(new_rx, new_ry).rgb = *reinterpret_cast<float*>(&rgb2);
          old_pc->at(old_rx_center, old_ry_center).rgb = *reinterpret_cast<float*>(&rgb2);
#else
          new_pc->at(new_rx, new_ry).data[3] = *reinterpret_cast<float*>(&rgb2);
          old_pc->at(old_rx_center, old_ry_center).data[3] = *reinterpret_cast<float*>(&rgb2);
#endif
          //Kill point
          //new_pc->at(new_rx, new_ry).z = std::numeric_limits<float>::quiet_NaN();
          //old_pc->at(old_rx_center, old_ry_center).z = std::numeric_limits<float>::quiet_NaN();
        }
      }
      else {} //only NaN?
    }
  }
  likelihood = sumloglikelihood/observation_count;//more readable
  confidence = observation_count;
  inliers = good_points;
  outliers = bad_points;
  occluded = occluded_points;
}

/** This function computes the p-value of the null hypothesis that the transformation is the true one.
 * It is too sensitive to outliers
 */
double rejectionSignificance(const Eigen::Matrix4f& proposed_transformation,//new to old
                           pointcloud_type::Ptr new_pc,
                           pointcloud_type::Ptr old_pc) 
{
   ScopedTimer s(__FUNCTION__);
 
  int skip_step = ParameterServer::instance()->get<int>("emm__skip_step");
  bool mark_outliers = ParameterServer::instance()->get<bool>("emm__mark_outliers");
  double observability_threshold = ParameterServer::instance()->get<double>("observability_threshold");
  if(skip_step < 0 || observability_threshold <= 0.0){
    return 1.0;
  }
  if(old_pc->width <= 1 || old_pc->height <= 1){
    //We need structured point clouds
    ROS_ERROR("Point Cloud seems to be non-structured: %u/%u (w/h). Observation can not be evaluated! ", old_pc->width, old_pc->height);
    if(ParameterServer::instance()->get<double>("voxelfilter_size") > 0){
      ROS_ERROR("The parameter voxelfilter_size is set. This is incompatible with the environment measurement model (parameter 'observability_threshold')");
    }
    return 1.0;
  }

  pointcloud_type new_pc_transformed;
  pcl::transformPointCloud(*new_pc, new_pc_transformed, proposed_transformation);

  //Camera Calibration FIXME: Get actual values
  float cx = old_pc->width /2 - 0.5;
  float cy = old_pc->height/2 - 0.5;
  float fx = 521.0f / (640.0/old_pc->width); 
  float fy = 521.0f / (480.0/old_pc->height); 

  double sum_mahalanobis_sq = 0.0;
  float observation_count = 0.0;

  unsigned int bad_points = 0, good_points = 0, occluded_points = 0;
//#pragma omp parallel for reduction(+: good_points, bad_points, occluded_points) 
  for(int new_ry = 0; new_ry < (int)new_pc->height; new_ry+=skip_step){
    for(int new_rx = 0; new_rx < (int)new_pc->width; new_rx+=skip_step){
      //Backproject transformed new 3D point to 2d raster of old image
      //TODO: Cache this?
      point_type& p = new_pc_transformed.at(new_rx, new_ry);
      if(p.z != p.z) continue; //NaN
      int old_rx_center = round((p.x / p.z)* fx + cx);
      int old_ry_center = round((p.y / p.z)* fy + cy);
      if(old_rx_center >= (int)old_pc->width || old_rx_center < 0 ||
         old_ry_center >= (int)old_pc->height|| old_ry_center < 0 )
      {
        ROS_DEBUG("New point not projected into old image, skipping");
        continue;
      }
      int nbhd = 10; //1 => 3x3 neighbourhood
      bool good_point = false, occluded_point = false, bad_point = false;
      int startx = std::max(0,old_rx_center - nbhd);
      int starty = std::max(0,old_ry_center - nbhd);
      int endx = std::min(static_cast<int>(old_pc->width), old_rx_center + nbhd +1);
      int endy = std::min(static_cast<int>(old_pc->height), old_ry_center + nbhd +1);
      int neighbourhood_step = 1; //Search for depth jumps in this area
      double min_mahalanobis_sq = std::numeric_limits<double>::infinity();
      double min_dz = std::numeric_limits<double>::infinity();
      std::vector<double> depth_values_for_neighborhood;
      for(int old_ry = starty; old_ry < endy; old_ry+=neighbourhood_step){
        for(int old_rx = startx; old_rx < endx; old_rx+=neighbourhood_step){
          
          const point_type& old_p = old_pc->at(old_rx, old_ry);
          if(old_p.z != old_p.z) continue; //NaN
          //ROS_INFO("Msrmnt. P1: (%f;%f;%f) P2: (%f;%f;%f)", p.x, p.y, p.z, old_p.x, old_p.y, old_p.z);
          
          //Sensor model
          //New point (p) is projected to old point (old_p)
          double dz = (old_p.z - p.z);//Positive: p.z smaller than (in front of) old_p.z
          depth_values_for_neighborhood.push_back(old_p.z);
          double dz_sq = dz*dz;
          // likelihood for old msrmnt = new msrmnt:
          double old_cov = depth_covariance(old_p.z);
          //TODO: (Wrong) Assumption: Transformation does not change the viewing angle. 
          double new_cov = depth_covariance(p.z);
          //Assumption: independence of sensor noise lets us sum variances
          double joint_cov = old_cov + new_cov;
          ///TODO: Compute correctly transformed new sigma in old_z direction
          //Gaussian probability of new being same as old
          double mahal_distance_sq = (dz_sq) / joint_cov;

          //Independent statistical tests.
          if(dz_sq < 9*joint_cov){ //Within 3 sigma
            good_point = true;
            //if(min_mahalanobis_sq > mahal_distance_sq){
            //  min_mahalanobis_sq = mahal_distance_sq;
            //}
            if(min_dz > dz){
              min_dz = dz;
            }
          }
          else { ///Very unlikely to be inlier as bigger than 3*std_dev
            if(dz > 0){ //New point was projected in front of old point
              bad_point = true;
              //if(min_mahalanobis_sq > mahal_distance_sq){
              //  min_mahalanobis_sq = mahal_distance_sq;
              //}
              if(min_dz > dz){
                min_dz = dz;
              }
            }
            else {
              occluded_point = true;
            }
          }
          //ROS_INFO("Msrmnt. dz=%g MHD=%g sigma=%g obs_p=%g, behind_p=%g", dz, mahal_dist, sqrt(joint_cov), observation_p, p_new_in_front);
        }
      }//End neighbourhood loop
      end_of_neighbourhood_loop:

      ///Compute covariance statistics also based on depth variance in the neighborhood
      if(good_point || (!occluded_point && bad_point)){
        double sum = std::accumulate(depth_values_for_neighborhood.begin(), depth_values_for_neighborhood.end(), 0.0);
        double depth_mean_nbhd = sum / depth_values_for_neighborhood.size();

        double sq_sum = std::inner_product(depth_values_for_neighborhood.begin(), depth_values_for_neighborhood.end(), depth_values_for_neighborhood.begin(), 0.0);
        double nbhd_cov = sq_sum / depth_values_for_neighborhood.size() - depth_mean_nbhd * depth_mean_nbhd;
        double nbhd_dz = (depth_mean_nbhd - p.z);//Positive: p.z smaller than (in front of) old_p.z
        double new_cov = depth_covariance(p.z);

        min_mahalanobis_sq = (nbhd_dz*nbhd_dz) / (new_cov + depth_covariance(depth_mean_nbhd) + nbhd_cov);
      }

      if(good_point){
        good_points++;
        //TODO: One tail of the distribution, containing the occluded points, is removed,
        //so samples in the bulk need to be discounted to keep the distribution
        sum_mahalanobis_sq += min_mahalanobis_sq;
        observation_count ++;
      }
      else if(occluded_point){
        occluded_points++;
      }
      else if(bad_point){
        bad_points++;
        sum_mahalanobis_sq += min_mahalanobis_sq;
        observation_count++;
        if(mark_outliers){
          uint8_t r1 = 255, g1 = 0, b1 = 0; // Mark bad boint in new point cloud with red color
          uint32_t rgb1 = ((uint32_t)r1 << 16 | (uint32_t)g1 << 8 | (uint32_t)b1);
          uint8_t r2 = 0, g2 = 255, b2 = 255; // Mark bad boint in older point cloud with cyan color
          uint32_t rgb2 = ((uint32_t)r2 << 16 | (uint32_t)g2 << 8 | (uint32_t)b2);
#ifndef RGB_IS_4TH_DIM
          new_pc->at(new_rx, new_ry).rgb = *reinterpret_cast<float*>(&rgb1);
          old_pc->at(old_rx_center, old_ry_center).rgb = *reinterpret_cast<float*>(&rgb2);
#else
          new_pc->at(new_rx, new_ry).data[3] = *reinterpret_cast<float*>(&rgb1);
          old_pc->at(old_rx_center, old_ry_center).data[3] = *reinterpret_cast<float*>(&rgb2);
#endif
        }
      }
      else {} //only NaN?
    }
  }
  boost::math::chi_squared chi_square(observation_count);
  double p_value = boost::math::cdf(chi_square, sum_mahalanobis_sq);
  double quantile = boost::math::quantile(chi_square, 0.75);
  ROS_INFO("Hypothesis Checking: Chi² with %f degrees of freedom. 75%% Quantile: %g, MD: %g", observation_count, quantile, sum_mahalanobis_sq);
  ROS_INFO("Hypothesis Checking: P-Value: %.20g, good_point_ratio: %d/%d: %g, occluded points: %d", p_value, good_points, good_points+bad_points, ((float)good_points)/(good_points+bad_points), occluded_points);
  return p_value;
}

bool observation_criterion_met(unsigned int inliers, unsigned int outliers, unsigned int all, double& quality)
{
  double obs_thresh = ParameterServer::instance()->get<double>("observability_threshold");
  if(obs_thresh < 0) return true;
  quality = inliers/static_cast<double>(inliers+outliers);
  double certainty = inliers/static_cast<double>(all);
  bool criterion1_met = quality > obs_thresh; //TODO: parametrice certainty (and use meaningful statistic)
  bool criterion2_met = certainty > 0.25; //TODO: parametrice certainty (and use meaningful statistic)
  bool both_criteria_met = criterion1_met && criterion2_met;
  ROS_WARN_COND(!criterion1_met, "Transformation does not meet observation likelihood criterion, because of ratio good/(good+bad): %f. Threshold: %f", inliers/static_cast<float>(inliers+outliers), obs_thresh);
  ROS_WARN_COND(!criterion2_met, "Transformation does not meet observation likelihood criterion, because of ratio good/(all): %f. Threshold: 0.25", certainty);
  return both_criteria_met;
}

void getColor(const point_type& p, unsigned char& r, unsigned char& g, unsigned char& b){
#ifndef RGB_IS_4TH_DIM
    b = *(  (unsigned char*)&(p.rgb));
    g = *(1+(unsigned char*)&(p.rgb));
    r = *(2+(unsigned char*)&(p.rgb));
#else
    b = *(  (unsigned char*)&(p.data[3]));
    g = *(1+(unsigned char*)&(p.data[3]));
    r = *(2+(unsigned char*)&(p.data[3]));
#endif
}

///Overlay the monochrom edges and depth jumps
void overlay_edges(cv::Mat visual, cv::Mat depth, cv::Mat& visual_edges, cv::Mat& depth_edges)
{
  if(visual.type() != CV_8UC1){
    visual_edges = cv::Mat( visual.rows, visual.cols, CV_8UC1); 
    cv::cvtColor(visual, visual_edges, CV_RGB2GRAY);
  }
  else 
  {
    visual_edges = visual;
  }
  cv::blur( visual_edges, visual_edges, cv::Size(3,3) );
  cv::Canny(visual_edges, visual_edges, 25, 300);
  cv::Canny(depth, depth_edges, 10, 300);
}


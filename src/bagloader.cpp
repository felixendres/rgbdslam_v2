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
//Documentation see header file
#include "pcl/ros/conversions.h"
#include <pcl/io/io.h>
//#include "pcl/common/transform.h"
#include "pcl_ros/transforms.h"
#include "bagloader.h"
#include <iostream>
#include <sstream>
#include <string>
//#include <ctime>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include "misc.h"
//#include <image_geometry/pinhole_camera_model.h>
//#include "pcl/ros/for_each_type.h"

//For rosbag reading
#include <rosbag/view.h>
#include <boost/foreach.hpp>


#include "scoped_timer.h"
//for comparison with ground truth from mocap and movable cameras on robots
#include <tf/transform_listener.h>

typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_type;      




BagLoader::BagLoader()
: pause_(ParameterServer::instance()->get<bool>("start_paused")),
  data_id_(0)
{
  ros::NodeHandle nh;
  tflistener_ = new tf::TransformListener(nh);
}

 
//! Load data from bag file
/**This function reads the sensor input from a bagfile specified in the parameter bagfile_name.
 * It is meant for offline processing of each frame */
void BagLoader::loadBag(std::string filename)
{
  ScopedTimer s(__FUNCTION__);
  ros::NodeHandle nh;
  ParameterServer* ps = ParameterServer::instance();
  std::string points_tpc = ps->get<std::string>("individual_cloud_out_topic").c_str();//ps->get<std::string>("topic_points");
  ROS_INFO_STREAM("Listening to " << points_tpc);
  std::string tf_tpc = std::string("/tf");
  tf_pub_ = nh.advertise<tf::tfMessage>(tf_tpc, 10);

  ROS_INFO("Loading Bagfile %s", filename.c_str());
  Q_EMIT setGUIStatus(QString("Loading ")+filename.c_str());
  { //bag will be destructed after this block (hopefully frees memory for the optimizer)
    rosbag::Bag bag;
    try{
      bag.open(filename, rosbag::bagmode::Read);
    } catch(rosbag::BagIOException ex) {
      ROS_FATAL("Opening Bagfile %s failed: %s Quitting!", filename.c_str(), ex.what());
      ros::shutdown();
      return;
    }
    ROS_INFO("Opened Bagfile %s", filename.c_str());
    Q_EMIT setGUIStatus(QString("Opened ")+filename.c_str());

    // Image topics to load for bagfiles
    std::vector<std::string> topics;
    topics.push_back(points_tpc);
    topics.push_back(tf_tpc);

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    // Simulate sending of the messages in the bagfile
    std::deque<sensor_msgs::PointCloud2::ConstPtr> pointclouds;
    ros::Time last_tf(0);
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      if(!ros::ok()) return;
      while(pause_) { 
        usleep(100);
        ROS_INFO_THROTTLE(5.0,"Paused - press Space to unpause");
        if(!ros::ok()) return;
      } 

      ROS_INFO_STREAM("Processing message on topic " << m.getTopic());
      if (m.getTopic() == points_tpc || ("/" + m.getTopic() == points_tpc))
      {
        sensor_msgs::PointCloud2::ConstPtr pointcloud = m.instantiate<sensor_msgs::PointCloud2>();
        //if (cam_info) cam_info_sub_->newMessage(cam_info);
        if (pointcloud) pointclouds.push_back(pointcloud);
        ROS_INFO("Found Message of %s", points_tpc.c_str());
      }
      if (m.getTopic() == tf_tpc|| ("/" + m.getTopic() == tf_tpc)){
        tf::tfMessage::ConstPtr tf_msg = m.instantiate<tf::tfMessage>();
        if (tf_msg) {
          //if(tf_msg->transforms[0].header.frame_id == "/kinect") continue;//avoid destroying tf tree if odom is used
          //prevents missing callerid warning
          boost::shared_ptr<std::map<std::string, std::string> > msg_header_map = tf_msg->__connection_header;
          (*msg_header_map)["callerid"] = "rgbdslam";
          tf_pub_.publish(tf_msg);
          ROS_INFO("Found Message of %s", tf_tpc.c_str());
          last_tf = tf_msg->transforms[0].header.stamp;
          last_tf -= ros::Duration(0.3);
        }
      }
      while(!pointclouds.empty() && pointclouds.front()->header.stamp < last_tf){
          Q_EMIT setGUIInfo(QString("Processing frame ") + QString::number(data_id_));
          callback(pointclouds.front());
          pointclouds.pop_front();
      }

    }
    bag.close();
  }
  ROS_INFO("Bagfile fully processed");
  Q_EMIT setGUIStatus(QString("Done with ")+filename.c_str());
  Q_EMIT bagFinished();
}


void BagLoader::callback(const sensor_msgs::PointCloud2ConstPtr& point_cloud)
{
    ScopedTimer s(__FUNCTION__);
    ROS_INFO("Processing cloud");
    if(++data_id_ < ParameterServer::instance()->get<int>("skip_first_n_frames") 
       || data_id_ % ParameterServer::instance()->get<int>("data_skip_step") != 0){ 
      ROS_INFO_THROTTLE(1, "Skipping Frame %i because of data_skip_step setting (this msg is only shown once a sec)", data_id_);
      return;
    };

    pointcloud_type* pc_col = new pointcloud_type();//will belong to viewer?
    pcl::fromROSMsg(*point_cloud,*pc_col);
    sendWithTransformation(pc_col);
}

BagLoader::~BagLoader(){
  delete tflistener_;
}



void BagLoader::togglePause(){
  pause_ = !pause_;
  ROS_INFO("Pause toggled to: %s", pause_? "true":"false");
  if(pause_) Q_EMIT setGUIInfo2("Processing Thread Stopped");
  else Q_EMIT setGUIInfo2("Processing Thread Running");
}

//Retrieve the transform between the lens and the base-link at capturing time
void BagLoader::sendWithTransformation(pointcloud_type* cloud)
{
  ScopedTimer s(__FUNCTION__);
  ParameterServer* ps = ParameterServer::instance();
  std::string fixed_frame  = ps->get<std::string>("fixed_frame_name");
  std::string depth_frame_id = cloud->header.frame_id;
  ros::Time depth_time = cloud->header.stamp;
  tf::StampedTransform map2points;

  try{
    tflistener_->waitForTransform(fixed_frame, depth_frame_id, depth_time, ros::Duration(0.005));
    tflistener_->lookupTransform( fixed_frame, depth_frame_id, depth_time,map2points);
    //printTransform("Current Transform", map2points);

    QMatrix4x4 transform_to_map = g2o2QMatrix(tf2G2O(map2points));
    Q_EMIT setPointCloud(cloud, transform_to_map);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s - No transformation available",ex.what());
  }
}


void BagLoader::loadBagFileAsync(std::string file)
{
    QtConcurrent::run(this, &BagLoader::loadBag, file);
}

void BagLoader::loadBagFileAsync(QString file)
{
    loadBagFileAsync(file.toStdString());
}

void BagLoader::clearPointCloud(pointcloud_type* pc) {
  ROS_DEBUG("Should clear cloud at %p", pc);
  delete pc; pc = NULL;
}

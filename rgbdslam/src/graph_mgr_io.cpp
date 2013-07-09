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

/** This file contains the methods of GraphManager concerned with transfering 
 * data via ROS or to disk. They are declared in graph_manager.h */

#include <sys/time.h>
#include "scoped_timer.h"
#include "graph_manager.h"
#include "misc.h"
#include "pcl_ros/transforms.h"
#include "pcl/io/pcd_io.h"
//#include <sensor_msgs/PointCloud2.h>
#include <opencv2/features2d/features2d.hpp>
#include <qtconcurrentrun.h>
#include <QFile>
#include <utility>
#include <fstream>
#include <boost/foreach.hpp>

#include "g2o/types/slam3d/se3quat.h"
//#include "g2o/types/slam3d/edge_se3_quat.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/core/optimizable_graph.h"

#include <pcl_ros/point_cloud.h>

// If QT Concurrent is available, run the saving in a seperate thread
void GraphManager::sendAllClouds(bool threaded){
    if (ParameterServer::instance()->get<bool>("concurrent_io") && threaded) {
        QFuture<void> f1 = QtConcurrent::run(this, &GraphManager::sendAllCloudsImpl);
        //f1.waitForFinished();
    }
    else {// Otherwise just call it without threading
        sendAllCloudsImpl();
    }
}


void GraphManager::sendAllCloudsImpl()
{
  ScopedTimer s(__FUNCTION__);

  if (batch_cloud_pub_.getNumSubscribers() == 0){
    ROS_WARN("No Subscribers: Sending of clouds cancelled");
    return;
  }

  ROS_INFO("Sending out all clouds");
  batch_processing_runs_ = true;
  ros::Rate r(ParameterServer::instance()->get<double>("send_clouds_rate")); //slow down a bit, to allow for transmitting to and processing in other nodes

  std::string fixed_frame = ParameterServer::instance()->get<std::string>("fixed_frame_name");
  std::string base_frame  = ParameterServer::instance()->get<std::string>("base_frame_name");
  if(base_frame.empty()){ //if there is no base frame defined, use frame of sensor data
    base_frame = graph_.begin()->second->pc_col->header.frame_id;
  }

  for (graph_it it = graph_.begin(); it != graph_.end(); ++it){

    Node* node = it->second;
    if(!node->valid_tf_estimate_) {
      ROS_INFO("Skipping node %i: No valid estimate", node->id_);
      continue;
    }

    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(node->vertex_id_));
    if(!v){ 
      ROS_ERROR("Nullpointer in graph at position %i!", it->first);
      continue;
    }

    tf::Transform base2points = node->getBase2PointsTransform();//get pose of base w.r.t current pc at capture time
    printTransform("base2points", base2points);
    tf::Transform computed_motion = g2o2TF(v->estimateAsSE3Quat());//get pose of point cloud w.r.t. first frame's pc
    printTransform("computed_motion", computed_motion);
    printTransform("init_base_pose_", init_base_pose_);

    tf::Transform world2base = init_base_pose_*base2points*computed_motion*base2points.inverse();
    tf::Transform gt_world2base = node->getGroundTruthTransform();//get mocap pose of base in map
    tf::Transform err = gt_world2base.inverseTimes(world2base);
    //TODO: Compute err from relative transformations betw. time steps

    //makes sure things have a corresponding timestamp
    //also avoids problems with tflistener cache size if mapping took long. Must be synchronized with tf broadcasting
    ros::Time now = ros::Time::now(); 

    ROS_DEBUG("Sending out transform %i", it->first);
    printTransform("World->Base", world2base);
    br_.sendTransform(tf::StampedTransform(world2base, now, fixed_frame, base_frame));
    br_.sendTransform(tf::StampedTransform(err, now, fixed_frame, "/where_mocap_should_be"));
    ROS_DEBUG("Sending out cloud %i", it->first);
    //graph_[i]->publish("/batch_transform", now, batch_cloud_pub_);
    publishCloud(node, now, batch_cloud_pub_);

    //co_server.insertCloudCallbackCommon(*(node->pc_col.get()), world2base*base2points);
    //tf::Transform ground_truth_tf = graph_[i]->getGroundTruthTransform();
    QString message;
    Q_EMIT setGUIInfo(message.sprintf("Sending pointcloud and map transform (%i/%i) on topics %s and /tf", it->first, (int)graph_.size(), ParameterServer::instance()->get<std::string>("individual_cloud_out_topic").c_str()) );
    r.sleep();
  }

  batch_processing_runs_ = false;
  Q_EMIT sendFinished();
}

// If QT Concurrent is available, run the saving in a seperate thread
void GraphManager::saveAllClouds(QString filename, bool threaded){
    if (ParameterServer::instance()->get<bool>("concurrent_io") && threaded) {
        QFuture<void> f1 = QtConcurrent::run(this, &GraphManager::saveAllCloudsToFile, filename);
        //f1.waitForFinished();
    }
    else {// Otherwise just call it without threading
        saveAllCloudsToFile(filename);
    }
}

void GraphManager::saveIndividualClouds(QString filename, bool threaded){
  if (ParameterServer::instance()->get<bool>("concurrent_io") && threaded) {
    QFuture<void> f1 = QtConcurrent::run(this, &GraphManager::saveIndividualCloudsToFile, filename);
    //f1.waitForFinished();
  }
  else {
    saveIndividualCloudsToFile(filename);
  }
}

void GraphManager::saveOctomap(QString filename, bool threaded){
  if (ParameterServer::instance()->get<bool>("concurrent_io") && threaded) {
    //saveOctomapImpl(filename);
    QFuture<void> f1 = QtConcurrent::run(this, &GraphManager::saveOctomapImpl, filename);
    //f1.waitForFinished();
  }
  else {
    saveOctomapImpl(filename);
  }
}

void GraphManager::saveOctomapImpl(QString filename)
{
  ScopedTimer s(__FUNCTION__);

  batch_processing_runs_ = true;
  Q_EMIT iamBusy(0, "Saving Octomap", 0);
  std::string fixed_frame_id = ParameterServer::instance()->get<std::string>("fixed_frame_name");
  std::list<Node*> nodes_for_octomapping;
  unsigned int points_to_render = 0;
  { //Get the transformations from the optimizer and store them in the node's point cloud header
    QMutexLocker locker(&optimizer_mutex);
    for (graph_it it = graph_.begin(); it != graph_.end(); ++it){
      Node* node = it->second;
      if(!node->valid_tf_estimate_) {
        ROS_INFO("Skipping node %i: No valid estimate", node->id_);
        continue;
      }
      g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(node->vertex_id_));
      if(!v){ 
        ROS_ERROR("Nullpointer in graph at position %i!", it->first);
        continue;
      }
      if(node->pc_col->size() == 0){
        ROS_INFO("Skipping Node %i, point cloud data is empty!", it->first);
        continue;
      }
      nodes_for_octomapping.push_back(node);
      // Update the sensor pose stored in the point clouds
      g2o::SE3Quat pose = v->estimateAsSE3Quat();
      node->pc_col->sensor_origin_.head<3>() = pose.translation().cast<float>();
      node->pc_col->sensor_orientation_ =  pose.rotation().cast<float>();
      node->pc_col->header.frame_id = fixed_frame_id;
      points_to_render += node->pc_col->size();
    }
  } 
  // Now (this takes long) render the clouds into the octomap
  int counter = 0;
  co_server_.reset();
  Q_EMIT iamBusy(0, "Saving Octomap", nodes_for_octomapping.size());
  unsigned int rendered_points = 0;
  BOOST_FOREACH(Node* node, nodes_for_octomapping)
  {
      QString message;
      Q_EMIT setGUIStatus(message.sprintf("Inserting Node %i/%i into octomap", ++counter, (int)nodes_for_octomapping.size()));
      co_server_.insertCloudCallback(node->pc_col, ParameterServer::instance()->get<double>("maximum_depth")); // Will be transformed according to sensor pose set previously
      Q_EMIT progress(0, "Saving Octomap", counter);
      rendered_points += node->pc_col->size();
      ROS_INFO("Rendered %u points of %u", rendered_points, points_to_render);
      if(ParameterServer::instance()->get<bool>("octomap_clear_raycasted_clouds")){
        node->clearPointCloud();
      }
      if(counter % ParameterServer::instance()->get<int>("octomap_autosave_step") == 0){
        Q_EMIT setGUIStatus(QString("Autosaving preliminary octomap to ") + filename);
        co_server_.save(qPrintable(filename));
      }
      ROS_INFO("Cleared pointcloud of Node ", node->id_);
  }


  Q_EMIT setGUIStatus(QString("Saving final octomap to ") + filename);
  co_server_.save(qPrintable(filename));
  Q_EMIT progress(0, "Finished Saving Octomap", 1e6);
  ROS_INFO ("Saved Octomap to %s", qPrintable(filename));
  if(ParameterServer::instance()->get<bool>("octomap_clear_after_save")){
    co_server_.reset();
    ROS_INFO ("Reset Octomap to free memory");
  }

  batch_processing_runs_ = false;
}

void GraphManager::saveIndividualCloudsToFile(QString file_basename)
{
  ScopedTimer s(__FUNCTION__);

  ROS_INFO("Saving all clouds to %sxxxx.pcd", qPrintable(file_basename));
  std::string gt = ParameterServer::instance()->get<std::string>("ground_truth_frame_name");
  ROS_INFO_COND(!gt.empty(), "Saving all clouds with ground truth sensor position to gt_%sxxxx.pcd", qPrintable(file_basename));

  batch_processing_runs_ = true;
  tf::Transform  world2base;
  QString message, filename;
  std::string fixed_frame_id = ParameterServer::instance()->get<std::string>("fixed_frame_name");
  for (graph_it it = graph_.begin(); it != graph_.end(); ++it){

    Node* node = it->second;
    if(!node->valid_tf_estimate_) {
      ROS_INFO("Skipping node %i: No valid estimate", node->id_);
      continue;
    }

    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(node->vertex_id_));
    if(!v){ 
      ROS_ERROR("Nullpointer in graph at position %i!", it->first);
      continue;
    }
    if(node->pc_col->size() == 0){
      ROS_INFO("Skipping Node %i, point cloud data is empty!", it->first);
      continue;
    }
    /*/TODO: is all this correct?
      tf::Transform transform = g2o2TF(v->estimateAsSE3Quat());
      tf::Transform cam2rgb;
      cam2rgb.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
      cam2rgb.setOrigin(tf::Point(0,-0.04,0));
      world2base = cam2rgb*transform;
      */
    tf::Transform pose = g2o2TF(v->estimateAsSE3Quat());
    tf::StampedTransform base2points =  node->getBase2PointsTransform();//get pose of base w.r.t current pc at capture time
    world2base = init_base_pose_*base2points*pose*base2points.inverse();

    Eigen::Vector4f sensor_origin(world2base.getOrigin().x(),world2base.getOrigin().y(),world2base.getOrigin().z(),world2base.getOrigin().w());
    Eigen::Quaternionf sensor_orientation(world2base.getRotation().w(),world2base.getRotation().x(),world2base.getRotation().y(),world2base.getRotation().z());

    node->pc_col->sensor_origin_ = sensor_origin;
    node->pc_col->sensor_orientation_ = sensor_orientation;
    node->pc_col->header.frame_id = fixed_frame_id;

    filename.sprintf("%s_%04d.pcd", qPrintable(file_basename), it->first);
    Q_EMIT setGUIStatus(message.sprintf("Saving to %s: Transformed Node %i/%i", qPrintable(filename), it->first, (int)camera_vertices.size()));
    pcl::io::savePCDFile(qPrintable(filename), *(node->pc_col), true); //Last arg: true is binary mode. ASCII mode drops color bits

    if(!gt.empty()){
      tf::StampedTransform gt_world2base = node->getGroundTruthTransform();//get mocap pose of base in map
      if( gt_world2base.frame_id_   == "/missing_ground_truth" ){ 
        ROS_WARN_STREAM("Skipping ground truth: " << gt_world2base.child_frame_id_ << " child/parent " << gt_world2base.frame_id_);
        continue;
      }
      Eigen::Vector4f sensor_origin(gt_world2base.getOrigin().x(),gt_world2base.getOrigin().y(),gt_world2base.getOrigin().z(),gt_world2base.getOrigin().w());
      Eigen::Quaternionf sensor_orientation(gt_world2base.getRotation().w(),gt_world2base.getRotation().x(),gt_world2base.getRotation().y(),gt_world2base.getRotation().z());

      node->pc_col->sensor_origin_ = sensor_origin;
      node->pc_col->sensor_orientation_ = sensor_orientation;
      node->pc_col->header.frame_id = fixed_frame_id;

      filename.sprintf("%s_%04d_gt.pcd", qPrintable(file_basename), it->first);
      Q_EMIT setGUIStatus(message.sprintf("Saving to %s: Transformed Node %i/%i", qPrintable(filename), it->first, (int)camera_vertices.size()));
      pcl::io::savePCDFile(qPrintable(filename), *(node->pc_col), true); //Last arg: true is binary mode. ASCII mode drops color bits
    }

  }
  Q_EMIT setGUIStatus("Saved all point clouds");
  ROS_INFO ("Saved all points clouds to %sxxxx.pcd", qPrintable(file_basename));
  batch_processing_runs_ = false;
}

void GraphManager::saveAllFeatures(QString filename, bool threaded)
{
    if (ParameterServer::instance()->get<bool>("concurrent_io") && threaded) {
        QFuture<void> f1 = QtConcurrent::run(this, &GraphManager::saveAllFeaturesToFile, filename);
        //f1.waitForFinished();
    }
    else {// Otherwise just call it without threading
        saveAllFeaturesToFile(filename);
    }
}
void GraphManager::saveAllFeaturesToFile(QString filename)
{
    ScopedTimer s(__FUNCTION__);

    cv::FileStorage fs(qPrintable(filename), cv::FileStorage::WRITE);
    fs << "Feature_Locations" << "[";

    ROS_INFO("Saving all features to %s transformed to a common coordinate frame.", qPrintable(filename));
    batch_processing_runs_ = true;
    tf::Transform  world2rgb, cam2rgb;
    QString message;
    cam2rgb.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
    cam2rgb.setOrigin(tf::Point(0,-0.04,0));
    int feat_count = 0;
    for (graph_it it = graph_.begin(); it != graph_.end(); ++it){

      Node* node = it->second;
      if(!node->valid_tf_estimate_) {
        ROS_INFO("Skipping node %i: No valid estimate", node->id_);
        continue;
      }

      g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(node->vertex_id_));
            tf::Transform world2cam = g2o2TF(v->estimateAsSE3Quat());
            world2rgb = cam2rgb*world2cam;
            Eigen::Matrix4f world2rgbMat;
            pcl_ros::transformAsMatrix(world2rgb, world2rgbMat);
      BOOST_FOREACH(Eigen::Vector4f loc, node->feature_locations_3d_)
            {
              loc.w() = 1.0;
              Eigen::Vector4f new_loc = world2rgbMat * loc;
              fs << "{:" << "x" << new_loc.x() << "y" << new_loc.y() << "z" << new_loc.z() << "}";
              feat_count++;
            }
      Q_EMIT setGUIStatus(message.sprintf("Saving to %s: Features of Node %i/%i", qPrintable(filename), it->first, (int)camera_vertices.size()));
    }
    fs << "]";
    //Assemble all descriptors into a big matrix
 assert(graph_.size()>0);
    int descriptor_size = graph_[0]->feature_descriptors_.cols;
    int descriptor_type = graph_[0]->feature_descriptors_.type();
    cv::Mat alldescriptors(0, descriptor_size,  descriptor_type);
    alldescriptors.reserve(feat_count);
    for (unsigned int i = 0; i < graph_.size(); ++i) {
      alldescriptors.push_back(graph_[i]->feature_descriptors_);
    }
    fs << "Feature_Descriptors" << alldescriptors;
    fs.release();

    Q_EMIT setGUIStatus(message.sprintf("Saved %d features points to %s", feat_count, qPrintable(filename)));
    ROS_INFO ("Saved %d feature points to %s", feat_count, qPrintable(filename));
    batch_processing_runs_ = false;
}




void GraphManager::saveAllCloudsToFile(QString filename){
    ScopedTimer s(__FUNCTION__);

    pointcloud_type aggregate_cloud; ///will hold all other clouds
    ROS_INFO("Saving all clouds to %s, this may take a while as they need to be transformed to a common coordinate frame.", qPrintable(filename));
    batch_processing_runs_ = true;

    std::string base_frame  = ParameterServer::instance()->get<std::string>("base_frame_name");
    if(base_frame.empty()){ //if there is no base frame defined, use frame of sensor data
      base_frame = graph_.begin()->second->pc_col->header.frame_id;
    }

    tf::Transform  world2cam;
    //fill message
    //rgbdslam::CloudTransforms msg;
    QString message;
    tf::Transform cam2rgb;
    cam2rgb.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
    cam2rgb.setOrigin(tf::Point(0,-0.04,0));
    for (graph_it it = graph_.begin(); it != graph_.end(); ++it){
      Node* node = it->second;
      if(!node->valid_tf_estimate_) {
        ROS_INFO("Skipping node %i: No valid estimate", node->id_);
        continue;
      }
      g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(node->vertex_id_));
      if(!v){ 
        ROS_ERROR("Nullpointer in graph at position %i!", it->first);
        continue;
      }
      tf::Transform transform = g2o2TF(v->estimateAsSE3Quat());
      world2cam = cam2rgb*transform;
      transformAndAppendPointCloud (*(node->pc_col), aggregate_cloud, world2cam, ParameterServer::instance()->get<double>("maximum_depth"));

      if(ParameterServer::instance()->get<bool>("batch_processing"))
        node->clearPointCloud(); //saving all is the last thing to do, so these are not required anymore
      Q_EMIT setGUIStatus(message.sprintf("Saving to %s: Transformed Node %i/%i", qPrintable(filename), it->first, (int)camera_vertices.size()));
    }
    aggregate_cloud.header.frame_id = base_frame;
    if(filename.endsWith(".ply", Qt::CaseInsensitive))
      pointCloud2MeshFile(filename, aggregate_cloud);
    if(filename.endsWith(".pcd", Qt::CaseInsensitive))
      pcl::io::savePCDFile(qPrintable(filename), aggregate_cloud, true); //Last arg is binary mode
    else {
      ROS_WARN("Filename misses correct extension (.pcd or .ply) using .pcd");
      filename.append(".pcd");
      pcl::io::savePCDFile(qPrintable(filename), aggregate_cloud, true); //Last arg is binary mode
    }
    Q_EMIT setGUIStatus(message.sprintf("Saved %d data points to %s", (int)aggregate_cloud.points.size(), qPrintable(filename)));
    ROS_INFO ("Saved %d data points to %s", (int)aggregate_cloud.points.size(), qPrintable(filename));

    if (whole_cloud_pub_.getNumSubscribers() > 0){ //if it should also be send out
      /*
      sensor_msgs::PointCloud2 cloudMessage_; //this will be send out in batch mode
      pcl::toROSMsg(aggregate_cloud,cloudMessage_);
      cloudMessage_.header.frame_id = base_frame;
      cloudMessage_.header.stamp = ros::Time::now();
      whole_cloud_pub_.publish(cloudMessage_);
      */
      whole_cloud_pub_.publish(aggregate_cloud.makeShared());
      ROS_INFO("Aggregate pointcloud sent");
    }
    batch_processing_runs_ = false;
}

void GraphManager::pointCloud2MeshFile(QString filename, pointcloud_type full_cloud)
{
  QFile file(filename);//file is closed on destruction
  if(!file.open(QIODevice::WriteOnly|QIODevice::Text)){
    ROS_ERROR("Could not open file %s", qPrintable(filename));
    return; 
  }
  QTextStream out(&file);
	out << "ply\n";
	out << "format ascii 1.0\n";
	out << "element vertex " << (int)full_cloud.points.size() << "\n"; 
	out << "property float x\n";
	out << "property float y\n";
	out << "property float z\n";
	out << "property uchar red\n";
	out << "property uchar green\n";
	out << "property uchar blue\n";
	out << "end_header\n";
  unsigned char r,g,b;
  float x, y, z ;
  for(unsigned int i = 0; i < full_cloud.points.size() ; i++){
    getColor(full_cloud.points[i], r,g,b);
    x = full_cloud.points[i].x;
    y = full_cloud.points[i].y;
    z = full_cloud.points[i].z;
    out << qSetFieldWidth(8) << x << " " << y << " " << z << " ";
    out << qSetFieldWidth(3) << r << " " << g << " " << b << "\n";
  }
}
  

void GraphManager::saveTrajectory(QString filebasename, bool with_ground_truth)
{
    ScopedTimer s(__FUNCTION__);

    if(graph_.size() == 0){
      ROS_ERROR("Graph is empty, no trajectory can be saved");
      return;
    }
    ROS_INFO("Logging Trajectory");
    QMutexLocker locker(&optimizer_mutex);

    //FIXME: DO this block only if with_ground_truth is true and !gt.empty()
    std::string gt = ParameterServer::instance()->get<std::string>("ground_truth_frame_name");

    ROS_INFO("Comparison of relative motion with ground truth");
    QString gtt_fname("_ground_truth.txt");
    QFile gtt_file(gtt_fname.prepend(filebasename));//file is closed on destruction
    if(!gtt_file.open(QIODevice::WriteOnly|QIODevice::Text)) return; //TODO: Errormessage
    QTextStream gtt_out(&gtt_file);
    tf::StampedTransform b2p = graph_[0]->getGroundTruthTransform();
    gtt_out.setRealNumberNotation(QTextStream::FixedNotation);
    gtt_out << "# TF Coordinate Frame ID: " << b2p.frame_id_.c_str() << "(data: " << b2p.child_frame_id_.c_str() << ")\n";

     
    QString et_fname("_estimate.txt");
    QFile et_file (et_fname.prepend(filebasename));//file is closed on destruction
    if(!et_file.open(QIODevice::WriteOnly|QIODevice::Text)) return; //TODO: Errormessage
    QTextStream et_out(&et_file);
    et_out.setRealNumberNotation(QTextStream::FixedNotation);
    b2p = graph_[0]->getBase2PointsTransform();
    et_out << "# TF Coordinate Frame ID: " << b2p.frame_id_.c_str() << "(data: " << b2p.child_frame_id_.c_str() << ")\n";

    for (graph_it it = graph_.begin(); it != graph_.end(); ++it){
      Node* node = it->second;
      if(!node->valid_tf_estimate_) {
        ROS_INFO("Skipping node %i: No valid estimate", node->id_);
        continue;
      }
      g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(node->vertex_id_));

      ROS_ERROR_COND(!v, "Nullpointer in graph at position %i!", it->first);

      tf::Transform pose = g2o2TF(v->estimateAsSE3Quat());

      tf::StampedTransform base2points = node->getBase2PointsTransform();//get pose of base w.r.t current pc at capture time
      tf::Transform world2base = init_base_pose_*base2points*pose*base2points.inverse();

      logTransform(et_out, world2base, node->pc_col->header.stamp.toSec());
      //Eigen::Matrix<double, 6,6> uncertainty = v->uncertainty();
      //et_out << uncertainty(0,0) << "\t" << uncertainty(1,1) << "\t" << uncertainty(2,2) << "\t" << uncertainty(3,3) << "\t" << uncertainty(4,4) << "\t" << uncertainty(5,5) <<"\n" ;
      if(with_ground_truth && !gt.empty()){
        tf::StampedTransform gt_world2base = node->getGroundTruthTransform();//get mocap pose of base in map
        if( gt_world2base.frame_id_   == "/missing_ground_truth" ){ 
          ROS_WARN_STREAM("Skipping ground truth: " << gt_world2base.child_frame_id_ << " child/parent " << gt_world2base.frame_id_);
          continue;
        }
        logTransform(gtt_out, gt_world2base, gt_world2base.stamp_.toSec()); 
        //logTransform(et_out, world2base, gt_world2base.stamp_.toSec()); 
      } 
    }
    ROS_INFO_COND(!gt.empty() && with_ground_truth, "Written logfiles ground_truth_trajectory.txt and estimated_trajectory.txt");
    ROS_INFO_COND(gt.empty(),  "Written logfile estimated_trajectory.txt");
}

/** The following function are used for visualization in RVIZ. 
 * They are only activated if their ros topics are subscribed to. 
 * Be careful, they haven't been tested in a long time 
 */
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;

void GraphManager::visualizeFeatureFlow3D(unsigned int marker_id, bool draw_outlier)
{
    ScopedTimer s(__FUNCTION__);

    if (ransac_marker_pub_.getNumSubscribers() > 0){ //don't visualize, if nobody's looking
        if(curr_best_result_.edge.id1 < 0 || curr_best_result_.edge.id2 < 0) {
          ROS_WARN("Attempted to visualize invalid pairwise result");
          return;
        }
        visualization_msgs::Marker marker_lines;

        marker_lines.header.frame_id = "/openni_rgb_optical_frame";
        marker_lines.ns = "ransac_markers";
        marker_lines.header.stamp = ros::Time::now();
        marker_lines.action = visualization_msgs::Marker::ADD;
        marker_lines.pose.orientation.w = 1.0;
        marker_lines.id = marker_id;
        marker_lines.type = visualization_msgs::Marker::LINE_LIST;
        marker_lines.scale.x = 0.002;
        
        std_msgs::ColorRGBA color_red  ;  //red outlier
        color_red.r = 1.0;
        color_red.a = 1.0;
        std_msgs::ColorRGBA color_green;  //green inlier, newer endpoint
        color_green.g = 1.0;
        color_green.a = 1.0;
        std_msgs::ColorRGBA color_yellow;  //yellow inlier, earlier endpoint
        color_yellow.r = 1.0;
        color_yellow.g = 1.0;
        color_yellow.a = 1.0;
        std_msgs::ColorRGBA color_blue  ;  //red-blue outlier
        color_blue.b = 1.0;
        color_blue.a = 1.0;

        marker_lines.color = color_green; //just to set the alpha channel to non-zero
        const g2o::VertexSE3* earlier_v; //used to get the transform
        const g2o::VertexSE3* newer_v; //used to get the transform
        // end of initialization
        ROS_DEBUG("Matches Visualization start: %zu Matches, %zu Inliers", 
                  curr_best_result_.all_matches.size(), 
                  curr_best_result_.inlier_matches.size());

        // write all inital matches to the line_list
        marker_lines.points.clear();//necessary?

        earlier_v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(nodeId2VertexId(curr_best_result_.edge.id1)));
        newer_v   = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(nodeId2VertexId(curr_best_result_.edge.id2)));
        Node* last = graph_.find(curr_best_result_.edge.id2)->second;
        Node* prev = graph_.find(curr_best_result_.edge.id1)->second;

        if (draw_outlier)
        {
          BOOST_FOREACH(cv::DMatch match, curr_best_result_.all_matches)
          {
          //for (unsigned int i=0;i<curr_best_result_.all_matches.size(); i++){
            int newer_id = match.queryIdx; //feature id in newer node
            int earlier_id = match.trainIdx; //feature id in earlier node

            //Outliers are red (newer) to blue (older)
            marker_lines.colors.push_back(color_red);
            marker_lines.colors.push_back(color_blue);

            marker_lines.points.push_back(
                    pointInWorldFrame(last->feature_locations_3d_[newer_id], newer_v->estimateAsSE3Quat()));
            marker_lines.points.push_back(
                    pointInWorldFrame(prev->feature_locations_3d_[earlier_id], earlier_v->estimateAsSE3Quat()));
          }
        }

        BOOST_FOREACH(cv::DMatch match, curr_best_result_.inlier_matches)
        {
        //for (unsigned int i=0;i<last_inlier_matches_.size(); i++){
          int newer_id = match.queryIdx; //feature id in newer node
          int earlier_id = match.trainIdx; //feature id in earlier node

          //inliers are green (newer) to blue (older)
          marker_lines.colors.push_back(color_green);
          marker_lines.colors.push_back(color_blue);

          marker_lines.points.push_back(
                  pointInWorldFrame(last->feature_locations_3d_[newer_id], newer_v->estimateAsSE3Quat()));
          marker_lines.points.push_back(
                  pointInWorldFrame(prev->feature_locations_3d_[earlier_id], earlier_v->estimateAsSE3Quat()));
        }

        ransac_marker_pub_.publish(marker_lines);
        ROS_DEBUG_STREAM("Published  " << marker_lines.points.size()/2 << " lines");
    }
}


void GraphManager::visualizeGraphEdges() const {
    ScopedTimer s(__FUNCTION__);

    if (marker_pub_.getNumSubscribers() > 0){ //no visualization for nobody
        ROS_WARN("Sending RVIZ Marker");
        visualization_msgs::Marker edges_marker;
        edges_marker.header.frame_id = "/openni_rgb_optical_frame"; //TODO: Should be a meaningfull fixed frame with known relative pose to the camera
        edges_marker.header.stamp = ros::Time::now();
        edges_marker.ns = "camera_pose_graph"; // Set the namespace and id for this marker.  This serves to create a unique ID
        edges_marker.id = 0;    // Any marker sent with the same namespace and id will overwrite the old one

        edges_marker.type = visualization_msgs::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::Marker::ADD; // Set the marker action.  Options are ADD and DELETE
        edges_marker.frame_locked = true; //rviz automatically retransforms the markers into the frame every update cycle
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        edges_marker.scale.x = 0.005; //line width
        //Global pose (used to transform all points)
        edges_marker.pose.position.x = 0;
        edges_marker.pose.position.y = 0;
        edges_marker.pose.position.z = 0;
        edges_marker.pose.orientation.x = 0.0;
        edges_marker.pose.orientation.y = 0.0;
        edges_marker.pose.orientation.z = 0.0;
        edges_marker.pose.orientation.w = 1.0;
        // Set the color -- be sure to set alpha to something non-zero!
        edges_marker.color.r = 1.0f;
        edges_marker.color.g = 1.0f;
        edges_marker.color.b = 1.0f;
        edges_marker.color.a = 0.5f;//looks smoother
        geometry_msgs::Point point; //start and endpoint for each line segment
        g2o::VertexSE3* v1,* v2; //used in loop
  EdgeSet::iterator edge_iter = cam_cam_edges.begin();
        int counter = 0;
  for(;edge_iter != cam_cam_edges.end(); edge_iter++, counter++) {
            g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
            std::vector<g2o::HyperGraph::Vertex*>& myvertices = myedge->vertices();
            v1 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(1));
            v2 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(0));

            point.x = v1->estimateAsSE3Quat().translation().x();
            point.y = v1->estimateAsSE3Quat().translation().y();
            point.z = v1->estimateAsSE3Quat().translation().z();
            edges_marker.points.push_back(point);
            
            point.x = v2->estimateAsSE3Quat().translation().x();
            point.y = v2->estimateAsSE3Quat().translation().y();
            point.z = v2->estimateAsSE3Quat().translation().z();
            edges_marker.points.push_back(point);
        }

        marker_pub_.publish (edges_marker);
        ROS_INFO("published %d graph edges", counter);
    }

}

void GraphManager::visualizeGraphNodes() const {
    ScopedTimer s(__FUNCTION__);

    if (marker_pub_.getNumSubscribers() > 0){ //don't visualize, if nobody's looking
        visualization_msgs::Marker nodes_marker;
        nodes_marker.header.frame_id = "/openni_rgb_optical_frame"; //TODO: Should be a meaningfull fixed frame with known relative pose to the camera
        nodes_marker.header.stamp = ros::Time::now();
        nodes_marker.ns = "camera_pose_graph"; // Set the namespace and id for this marker.  This serves to create a unique ID
        nodes_marker.id = 1;    // Any marker sent with the same namespace and id will overwrite the old one


        nodes_marker.type = visualization_msgs::Marker::LINE_LIST;
        nodes_marker.action = visualization_msgs::Marker::ADD; // Set the marker action.  Options are ADD and DELETE
        nodes_marker.frame_locked = true; //rviz automatically retransforms the markers into the frame every update cycle
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        nodes_marker.scale.x = 0.002;
        //Global pose (used to transform all points) //TODO: is this the default pose anyway?
        nodes_marker.pose.position.x = 0;
        nodes_marker.pose.position.y = 0;
        nodes_marker.pose.position.z = 0;
        nodes_marker.pose.orientation.x = 0.0;
        nodes_marker.pose.orientation.y = 0.0;
        nodes_marker.pose.orientation.z = 0.0;
        nodes_marker.pose.orientation.w = 1.0;
        // Set the color -- be sure to set alpha to something non-zero!
        nodes_marker.color.r = 1.0f;
        nodes_marker.color.g = 0.0f;
        nodes_marker.color.b = 0.0f;
        nodes_marker.color.a = 1.0f;


        geometry_msgs::Point tail; //same startpoint for each line segment
        geometry_msgs::Point tip;  //different endpoint for each line segment
        std_msgs::ColorRGBA arrow_color_red  ;  //red x axis
        arrow_color_red.r = 1.0;
        arrow_color_red.a = 1.0;
        std_msgs::ColorRGBA arrow_color_green;  //green y axis
        arrow_color_green.g = 1.0;
        arrow_color_green.a = 1.0;
        std_msgs::ColorRGBA arrow_color_blue ;  //blue z axis
        arrow_color_blue.b = 1.0;
        arrow_color_blue.a = 1.0;
        Eigen::Vector3d origin(0.0,0.0,0.0);
        Eigen::Vector3d x_axis(0.2,0.0,0.0); //20cm long axis for the first (almost fixed) node
        Eigen::Vector3d y_axis(0.0,0.2,0.0);
        Eigen::Vector3d z_axis(0.0,0.0,0.2);
        Eigen::Vector3d tmp; //the transformed endpoints
        int counter = 0;
        g2o::VertexSE3* v; //used in loop
  for (g2o::HyperGraph::VertexSet::iterator it = camera_vertices.begin(); it != camera_vertices.end(); ++it){

   // VertexIDMap::iterator vertex_iter = optimizer_->vertices().begin();
   // for(/*see above*/; vertex_iter != optimizer_->vertices().end(); vertex_iter++, counter++) {

   v = dynamic_cast<g2o::VertexSE3* >(*it);
            //v->estimateAsSE3Quat().rotation().x()+ v->estimateAsSE3Quat().rotation().y()+ v->estimateAsSE3Quat().rotation().z()+ v->estimateAsSE3Quat().rotation().w();
            tmp = v->estimateAsSE3Quat() * origin;
            tail.x = tmp.x();
            tail.y = tmp.y();
            tail.z = tmp.z();
            //Endpoints X-Axis
            nodes_marker.points.push_back(tail);
            nodes_marker.colors.push_back(arrow_color_red);
            tmp = v->estimateAsSE3Quat() * x_axis;
            tip.x  = tmp.x();
            tip.y  = tmp.y();
            tip.z  = tmp.z();
            nodes_marker.points.push_back(tip);
            nodes_marker.colors.push_back(arrow_color_red);
            //Endpoints Y-Axis
            nodes_marker.points.push_back(tail);
            nodes_marker.colors.push_back(arrow_color_green);
            tmp = v->estimateAsSE3Quat() * y_axis;
            tip.x  = tmp.x();
            tip.y  = tmp.y();
            tip.z  = tmp.z();
            nodes_marker.points.push_back(tip);
            nodes_marker.colors.push_back(arrow_color_green);
            //Endpoints Z-Axis
            nodes_marker.points.push_back(tail);
            nodes_marker.colors.push_back(arrow_color_blue);
            tmp = v->estimateAsSE3Quat() * z_axis;
            tip.x  = tmp.x();
            tip.y  = tmp.y();
            tip.z  = tmp.z();
            nodes_marker.points.push_back(tip);
            nodes_marker.colors.push_back(arrow_color_blue);
            //shorten all nodes after the first one
            x_axis.x() = 0.1;
            y_axis.y() = 0.1;
            z_axis.z() = 0.1;
        }

        marker_pub_.publish (nodes_marker);
        ROS_INFO("published %d graph nodes", counter);
    }

}

void GraphManager::saveG2OGraph(QString filename)
{
  ROS_INFO("Saving G2O graph to %s", qPrintable(filename));
  optimizer_->save(qPrintable(filename));
}

void GraphManager::broadcastTransform(Node* node, tf::Transform& computed_motion){
    std::string fixed_frame = ParameterServer::instance()->get<std::string>("fixed_frame_name");
    std::string base_frame  = ParameterServer::instance()->get<std::string>("base_frame_name");
    if(base_frame.empty()){ //if there is no base frame defined, use frame of sensor data
      base_frame = node->pc_col->header.frame_id;
    }
    
    /*
    if(graph_.size() == 0){
      ROS_WARN("Cannot broadcast transform while graph is empty sending identity");
      br_.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(), ros::Time::now(), fixed_frame, base_frame));
      return;
    }
    */
    const tf::StampedTransform& base2points = node->getBase2PointsTransform();//get pose of base w.r.t current pc at capture time

    //Assumption: computed_motion_ contains last pose
    tf::Transform world2base = init_base_pose_*base2points*computed_motion*base2points.inverse();
    printTransform("World->Base", world2base);

    ROS_DEBUG("Broadcasting transform");
    
    br_.sendTransform(tf::StampedTransform(world2base.inverse(), base2points.stamp_, base_frame, fixed_frame));
}

///Send node's pointcloud with given publisher and timestamp
void publishCloud(Node* node, ros::Time timestamp, ros::Publisher pub){
  ros::Time stamp = node->pc_col->header.stamp; //temp storage
  node->pc_col->header.stamp = timestamp; //to sync with tf
  pub.publish(node->pc_col);
  node->pc_col->header.stamp = stamp; //restore original stamp (to minimize unexpected side-effects of this function)
  ROS_INFO("Pointcloud with id %i sent with frame %s", node->id_, node->pc_col->header.frame_id.c_str());
}

void GraphManager::drawFeatureFlow(cv::Mat& canvas, cv::Scalar line_color,
                                   cv::Scalar circle_color)
{
    struct timespec starttime, finish; double elapsed; clock_gettime(CLOCK_MONOTONIC, &starttime);
    if(!ParameterServer::instance()->get<bool>("use_gui")){ return; }
    ROS_DEBUG("Number of features to draw: %d", (int)curr_best_result_.inlier_matches.size());

    const double pi_fourth = 3.14159265358979323846 / 4.0;
    const int line_thickness = 1;
    const int circle_radius = 6;
    const int cv_aa = 16;
    if(graph_.empty()) {
      ROS_WARN("Feature Flow for empty graph requested. Bug?");
      return;
    } else if(graph_.size() == 1 || curr_best_result_.edge.id1 == -1 ) {//feature flow is only available between at least two nodes
      Node* newernode = graph_[graph_.size()-1];
      cv::drawKeypoints(canvas, newernode->feature_locations_2d_, canvas, cv::Scalar(255), 5);
      return;
    } 

    Node* earliernode = graph_[curr_best_result_.edge.id1];//graph_.size()-2; //compare current to previous
    Node* newernode = graph_[curr_best_result_.edge.id2];
    if(earliernode == NULL){
      if(newernode == NULL ){ ROS_ERROR("Nullpointer for Node %u", (unsigned int)graph_.size()-1); }
      ROS_ERROR("Nullpointer for Node %d", curr_best_result_.edge.id1);
      curr_best_result_.edge.id1 = 0;
      return;
    } else if(newernode == NULL ){
      ROS_ERROR("Nullpointer for Node %u", (unsigned int)graph_.size()-1);
      return;
    }

    //encircle all keypoints in this image
    //for(unsigned int feat = 0; feat < newernode->feature_locations_2d_.size(); feat++) {
    //    cv::Point2f p; 
    //    p = newernode->feature_locations_2d_[feat].pt;
    //    cv::circle(canvas, p, circle_radius, circle_color, line_thickness, 8);
    //}
    cv::Mat tmpimage = cv::Mat::zeros(canvas.rows, canvas.cols, canvas.type());
    if(canvas.type() == CV_8UC1) circle_color = cv::Scalar(255);

    //Generate different keypoint sets for those with depth and those without
    std::vector<cv::KeyPoint> with_depth, without_depth;
    for(int i = 0; i < newernode->feature_locations_2d_.size(); i++){
      if(isnan(newernode->feature_locations_3d_[i](2))){
          without_depth.push_back(newernode->feature_locations_2d_[i]);
      } else {
          with_depth.push_back(newernode->feature_locations_2d_[i]);
      }
    }

    cv::drawKeypoints(canvas, with_depth, tmpimage, circle_color, 5);
    cv::drawKeypoints(canvas, without_depth, tmpimage, cv::Scalar(0,128,255,0), 5);


    canvas+=tmpimage;
    for(unsigned int mtch = 0; mtch < curr_best_result_.inlier_matches.size(); mtch++) {
        cv::Point2f p,q; //TODO: Use sub-pixel-accuracy
        unsigned int newer_idx = curr_best_result_.inlier_matches[mtch].queryIdx;
        unsigned int earlier_idx = curr_best_result_.inlier_matches[mtch].trainIdx;
        q = newernode->feature_locations_2d_[newer_idx].pt;
        p = earliernode->feature_locations_2d_[earlier_idx].pt;

        double angle;    angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
        double hypotenuse = cv::norm(p-q);
            cv::line(canvas, p, q, line_color, line_thickness, cv_aa);
        if(hypotenuse > 1.5){  //only larger motions larger than one pix get an arrow tip
            cv::line( canvas, p, q, line_color, line_thickness, cv_aa );
            /* Now draw the tips of the arrow.  */
            p.x =  (q.x + 4 * cos(angle + pi_fourth));
            p.y =  (q.y + 4 * sin(angle + pi_fourth));
            cv::line( canvas, p, q, line_color, line_thickness, cv_aa );
            p.x =  (q.x + 4 * cos(angle - pi_fourth));
            p.y =  (q.y + 4 * sin(angle - pi_fourth));
            cv::line( canvas, p, q, line_color, line_thickness, cv_aa );
        } else { //draw a smaller circle into the bigger one 
            cv::circle(canvas, p, circle_radius-2, circle_color, line_thickness, cv_aa);
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &finish); elapsed = (finish.tv_sec - starttime.tv_sec); elapsed += (finish.tv_nsec - starttime.tv_nsec) / 1000000000.0; ROS_INFO_STREAM_COND_NAMED(elapsed > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << " runtime: "<< elapsed <<" s");
}


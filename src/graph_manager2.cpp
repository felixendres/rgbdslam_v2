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
#include "graph_manager.h"
#include "misc.h"
#include "covariance_estimation.h"
#include "g2o/types/slam3d/edge_se3.h" //For setEmpiricalCovariances
#include "scoped_timer.h"

///This file contains methods of the GraphManager class without extra 
///dependencies (except for the class header).

void GraphManager::toggleMapping(bool mappingOn){
  QMutexLocker locker(&optimizer_mutex_);
  ROS_INFO_COND(mappingOn, "Switching mapping back on");
  ROS_INFO_COND(!mappingOn, "Switching mapping off: Localization continues");
  localization_only_ = !mappingOn;
  ROS_ERROR("Implementation of localization only broken");
#ifdef DO_FEATURE_OPTIMIZATION
  optimizer_->setFixed(landmark_vertices, localization_only_);
#endif
  optimizer_->setFixed(camera_vertices, localization_only_);
}

bool GraphManager::isBusy(){
  return (batch_processing_runs_ || process_node_runs_ );
}

void GraphManager::clearPointClouds() {
  ROS_WARN("Clearing PointCloud. They will not be available for save/send, EMM.");
  BOOST_REVERSE_FOREACH(GraphNodeType entry, graph_){
    entry.second->clearPointCloud();
  }
}

void GraphManager::clearPointCloud(pointcloud_type const * pc) {
  ROS_DEBUG("Should clear cloud at %p", pc);
  BOOST_REVERSE_FOREACH(GraphNodeType entry, graph_){
    if(entry.second->pc_col.get() == pc){
      entry.second->clearPointCloud();
      ROS_WARN("Cleared PointCloud after rendering to openGL list. It will not be available for save/send.");
      return;
    }
    ROS_DEBUG("Compared to node %d (cloud at %p has size %zu)", entry.first, entry.second->pc_col.get(), entry.second->pc_col->points.size());
  }
  ROS_WARN("Should Clear cloud at %p, but didn't find it in any node.", pc);
}

void GraphManager::deleteLastFrame(){
    if(graph_.size() <= 1) {
      ROS_INFO("Resetting, as the only node is to be deleted");
      reset_request_ = true;
      Q_EMIT deleteLastNode();
      return;
    }

    deleteCameraFrame(graph_.size()-1);

    Q_EMIT deleteLastNode();
    optimizeGraph();//s.t. the effect of the removed edge transforms are removed to
    ROS_INFO("Removed most recent node");
    Q_EMIT setGUIInfo("Removed most recent node");
    //Q_EMIT setGraphEdges(getGraphEdges());
    //updateTransforms needs to be last, as it triggers a redraw
    //Q_EMIT updateTransforms(getAllPosesAsMatrixList());
}

void GraphManager::reset(){
    reset_request_ = true;
}

GraphManager::~GraphManager() {
  //TODO: delete all Nodes
    //for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
    //Q_FOREACH(Node* node, graph_) { delete node; }
    BOOST_FOREACH(GraphNodeType entry, graph_) { 
      delete entry.second; 
    }
    graph_.clear();
    QMutexLocker locker(&optimizer_mutex_);
    QMutexLocker locker2(&optimization_mutex_);
    //delete (optimizer_); optimizer_=NULL; //FIXME: this leads to a double free corruption. Bug in g2o?
    ransac_marker_pub_.shutdown();
    whole_cloud_pub_.shutdown();
    marker_pub_.shutdown();
    batch_cloud_pub_.shutdown();

}

//WARNING: Dangerous
void GraphManager::deleteFeatureInformation() {
  ROS_WARN("Clearing out Feature information from nodes");
  //Q_FOREACH(Node* node, graph_) {
  BOOST_FOREACH(GraphNodeType entry, graph_){
    entry.second->clearFeatureInformation();
  }
}

void GraphManager::setEmpiricalCovariancesForEdgeSet(EdgeSet& edges){
  ScopedTimer s(__FUNCTION__);

  typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6Xd;
  typedef g2o::BaseEdge<6, Eigen::Isometry3d> BaseEdgeSE3;
  typedef g2o::EdgeSE3::InformationType InfoMat;

  Matrix6Xd measurements, errors;
  edgesToMeasurementMatrix<BaseEdgeSE3>(edges, measurements);
  edgesToErrorMatrix<BaseEdgeSE3>(edges, errors);

  //Precomputation of weights and weighted errors
  Vector6d meanMsr = measurements.rowwise().mean();
  Vector6d stdDev = (measurements.colwise() - meanMsr).cwiseAbs().rowwise().mean();

  EdgeSet::iterator it = edges.begin();
  for(int i = 0; it != edges.end(); ++it, ++i)
  {
    BaseEdgeSE3* edge = dynamic_cast<BaseEdgeSE3*>( *it );
    if(edge){
      InfoMat infoMat = computeEmpiricalInformationMatrix(measurements, errors, measurements.col(i), stdDev);
      edge->setInformation(infoMat);
    }
  }

}

void GraphManager::setEmpiricalCovariances(){
  optimizer_->initializeOptimization();
  optimizer_->computeActiveErrors();

  setEmpiricalCovariancesForEdgeSet(cam_cam_edges_);
  //setEmpiricalCovariancesForEdgeSet(odometry_edges_);
}


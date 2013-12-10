/*
 * landmark.cpp
 *
 *  Created on: Jun 1, 2012
 *      Author: Nikolas Engelhard
 */


#include "landmark.h"
#include "misc2.h"

#ifdef DO_FEATURE_OPTIMIZATION

#include "graph_manager.h"

#include "g2o/types/slam3d/parameter_camera.h"
//#include "g2o/types/slam3d/camera_parameters.h"



#include <utility>
using namespace std;


// look for features with more than one match
MatchingResult removeStrangeMatches(const MatchingResult& match_result){


 map<int,int>  good_inlier;
 for (uint i=0; i<match_result.inlier_matches.size(); ++i){
  cv::DMatch m = match_result.inlier_matches[i];
  good_inlier[m.trainIdx] = m.queryIdx;
 }

// now each training feature has no more than one query feature
 map<int,int> final_inlier;
 for (map<int,int>::iterator it = good_inlier.begin(); it != good_inlier.end(); ++it){
  final_inlier[it->second] = it->first; // no query has more than one trainIdx
 }


 MatchingResult cleaned; cleaned.inlier_matches.reserve(good_inlier.size());
 for (map<int,int>::iterator it = final_inlier.begin(); it != final_inlier.end(); ++it){
  cv::DMatch m;
  m.trainIdx = it->second; m.queryIdx=it->first;
  cleaned.inlier_matches.push_back(m);
 }

 ROS_INFO("Cleaning: removed %zu matches from %zu", match_result.inlier_matches.size()-good_inlier.size(), match_result.inlier_matches.size() );

 return cleaned;



}



void GraphManager::removeFeaturesFromGraph(){

// for (Vset_it it = landmark_vertices.begin(); it != landmark_vertices.end(); ++it){
//  optimizer_->removeVertex(*it);
// }

 for (uint i=0; i<landmarks.size(); ++i){
  if (landmarks[i].g2o_vertex){
   optimizer_->removeVertex(landmarks[i].g2o_vertex);
   landmarks[i].g2o_vertex = NULL;
  }

 }

 for (EdgeSet_it it = cam_lm_edges.begin(); it != cam_lm_edges.end(); ++it){
  optimizer_->removeEdge(*it);
 }

}


void GraphManager::printLandmarkStatistic(){

 float mean_obs_cnt = 0;
 uint lm_cnt = landmarks.size();
 if (lm_cnt == 0) return;

 for (uint i=0; i<lm_cnt; ++i){
  Landmark *lm = &landmarks[i];
  // ROS_INFO("Landmark %i has %zu obs", lm->id, lm->observations.size());
  mean_obs_cnt += (lm->observations.size()*1.0/lm_cnt);
 }

 ROS_WARN("%i Landmarks with mean of %.2f observations", lm_cnt, mean_obs_cnt);
}


// increases next_vertex_id
void GraphManager::updateLandmarkInGraph(Landmark* lm){

 // Landmark is not in the graph, create new vertex
 // use first observation as initial position
 if (!lm->g2o_vertex){
  int node_id = lm->observations.begin()->first;
  int feature_id = lm->observations.begin()->second;
  Node *n = graph_[node_id];
  g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(n->vertex_id_ ));

  assert(v1);
  assert(int(n->feature_locations_3d_.size()) > feature_id);

  Eigen::Vector4f pos_relative = n->feature_locations_3d_[feature_id];
  if(isnan(pos_relative(2))){
    pos_relative(2) = 1.0; //FIXME instead of using an arbitrary depth value, use the correct vertex type
    return;
  }
  Eigen::Vector3d pos_absolute = v1->estimate()*Eigen::Vector3d(pos_relative[0],pos_relative[1],pos_relative[2]);

  lm->g2o_vertex = new LM_vertex_type();
  lm->g2o_vertex->setId(next_vertex_id++);
  lm->g2o_vertex->setFixed(false);
  lm->g2o_vertex->setEstimate(pos_absolute);

  optimizer_->addVertex(lm->g2o_vertex);
  landmark_vertices.insert(lm->g2o_vertex);
 }


 if (lm->observations.size()>0){
  // adding all new observation edges
  for (std::map<int,int>::iterator it=lm->observations.begin(); it != lm->observations.end(); ++it){
   int node_id = it->first;
   int kpt = it->second;


   // ROS_INFO("adding kpt %i in image %i", kpt, node_id);

   if (graph_.find(node_id) == graph_.end()){
    ROS_INFO("Trying to create projection to node %i which is not in the graph!", node_id);
    continue;
   }

   Node * node = graph_[node_id];
   g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(node->vertex_id_));


   assert(node->feature_locations_3d_.size() == node->feature_locations_2d_.size());
   assert(kpt < int(node->feature_locations_2d_.size()));


   Proj_edge_type* projectionEdge = new Proj_edge_type();
   cv::KeyPoint kp = node->feature_locations_2d_[kpt];
   Eigen::Vector4f pos_relative = node->feature_locations_3d_[kpt];
   if(!isnan(pos_relative(2))){
     projectionEdge->setMeasurement(Eigen::Vector3d(kp.pt.x,kp.pt.y,pos_relative[2]));
     Eigen::Matrix3d info_mat = point_information_matrix(pos_relative[2]);
     projectionEdge->setInformation(info_mat);
   } else {//FIXME instead of using an arbitrary depth value and high uncertainty, use the correct vertex type
     continue;
     projectionEdge->setMeasurement(Eigen::Vector3d(kp.pt.x,kp.pt.y,1.0));
     Eigen::Matrix3d info_mat = point_information_matrix(1e12);//as uncertain as if it was ...meter away
     projectionEdge->setInformation(info_mat);
   }
   projectionEdge->setParameterId(0,0);
   projectionEdge->vertices()[0] = v1;
   projectionEdge->vertices()[1] = lm->g2o_vertex;


//   projectionEdge->setParameterId(0,0);
//   projectionEdge->setVertex(0,v1);
//   projectionEdge->setVertex(1,lm->vertex_xyz);
//   projectionEdge->setId(next_vertex_id++);

   //projectionEdge->setRobustKernel(true);
   optimizer_->addEdge(projectionEdge);
   lm->g2o_vertex->setFixed(false);//Vertex involved in update
   cam_lm_edges.insert(projectionEdge);
   // lm->proj_edges.insert(projectionEdge);

  }

 // lm->observations_with_edges.insert(lm->new_observations.begin(), lm->new_observations.end());
 // lm->new_observations.clear();
 }
}

void GraphManager::updateProjectionEdges(){
 //ROS_INFO("Adding all edges to the graph!");

 for (uint i=0; i<landmarks.size(); ++i){
  Landmark *lm = &landmarks[i];
  updateLandmarkInGraph(lm);
  // ROS_INFO("LM %i has %zu edges", lm->id,lm->proj_edges.size());
 }

 ROS_WARN("Updated all Landmarks!");

// optimizer_->save("before.g2o");
//
// optimizer_->initializeOptimization();
// optimizer_->setVerbose(true);
// optimizer_->computeActiveErrors();
//
// optimizer_->optimize(100);
// optimizer_->computeActiveErrors();
//
// optimizer_->save("after.g2o");
}



void GraphManager::updateLandmarks(const MatchingResult& match_result, Node* old_node, Node* new_node){

// ROS_ERROR("processing landmarks for nodes %i and %i (ids: %i %i) (%i inlier)", match_result.edge.id1, match_result.edge.id2, old_node->id_, new_node->id_,match_result.inlier_matches.size());

 assert(old_node && new_node);
 assert(int(old_node->id_) == match_result.edge.id1);
 assert(int(new_node->id_) == match_result.edge.id2);

 MatchingResult cleaned = removeStrangeMatches(match_result);

 for (uint i=0; i<cleaned.inlier_matches.size(); ++i)
 {
  cv::DMatch m = cleaned.inlier_matches[i];
  // ROS_INFO("Kpt %i in img %i matches with kpt %i in img %i", m.queryIdx,new_node->id_, m.trainIdx,  old_node->id_);
  std::map<int, int>::iterator it_old, it_new;

  int lm_id_old = -1;
  int lm_id_new = -1;

  //  ROS_INFO("Node %i has %zu obs", old_node->id_, old_node->kpt_to_landmark.size());
  //  ROS_INFO("Node %i has %zu obs", new_node->id_, new_node->kpt_to_landmark.size());

  // check if one or both keypoints already belong to a landmark
  it_old = old_node->kpt_to_landmark.find(m.trainIdx);
  if (it_old != old_node->kpt_to_landmark.end())
    lm_id_old = it_old->second;

  it_new = new_node->kpt_to_landmark.find(m.queryIdx);
  if (it_new != new_node->kpt_to_landmark.end())
    lm_id_new = it_new->second;


//   ROS_INFO("t: %i q: %i, old_id %i, new_id %i", m.trainIdx, m.queryIdx,  lm_id_old, lm_id_new);

  // no point belongs to a landmark, create new
  if (lm_id_old == -1 && lm_id_new == -1){


   // create new landmark


   Landmark lm;
   lm.id = next_landmark_id++;

//    ROS_INFO("Creating landmark %i",lm.id);



   lm.observations[old_node->id_] = m.trainIdx;
   lm.observations[new_node->id_] = m.queryIdx;

   //   ROS_INFO("old, new %i %i", it_old->second, it);

   assert(lm.observations.size() == 2);

   // ROS_INFO("node %i: kpt %i points to landmark %i", old_node->id_, m.queryIdx, lm.id);
   old_node->kpt_to_landmark[m.trainIdx] = lm.id;
   new_node->kpt_to_landmark[m.queryIdx] = lm.id;

//   ROS_INFO("old: assingning feature %i to lm %i", )

//   old_node->visible_landmarks.insert(lm.id);
//   new_node->visible_landmarks.insert(lm.id);


   landmarks.push_back(lm);
   continue;
  }

  // one point belongs to landmark, add new observation
  if (lm_id_old > -1 && lm_id_new == -1){

   Landmark* lm = &landmarks[lm_id_old];
   // add new observation
   lm->observations[new_node->id_] = m.queryIdx;
   // connect keypoint with landmark
   new_node->kpt_to_landmark[m.queryIdx] = lm->id;

   continue;
  }

  // again:
  if (lm_id_old == -1 && lm_id_new > -1){

   Landmark* lm = &landmarks[lm_id_new];
   // add new observation
   lm->observations[old_node->id_] = m.trainIdx;
   // connect keypoint with landmark
   old_node->kpt_to_landmark[m.trainIdx] = lm->id;
//   old_node->visible_landmarks.insert(lm.id);
   continue;
  }

  assert(lm_id_old > -1 && lm_id_new > -1);

  // both point to the same landmark
  // a) could be the same landmark!
  if (lm_id_new == lm_id_new){
   // Landmark *lm = &landmarks[lm_id_new];

   // both landmarks should already registered as observation
   //   assert(lm->observations.find(new_node->id_)->second = it_new->first);
   //   assert(lm->observations.find(it_old->second)->second = it_old->first);

   continue;
  }

  // last case: both landmarks belong to different landmarks -> merge landmarks


  // assert(1==0);

  Landmark *lm_1 = &landmarks[lm_id_old];
  Landmark *lm_2 = &landmarks[lm_id_new];

  // merge landmark with less observations into larger one and delete it
  if  (lm_1->observations.size() >= lm_2->observations.size()){
   mergeLandmarks(lm_1, lm_2);
   landmarks.erase(landmarks.begin()+lm_id_new);
  }else{
   mergeLandmarks(lm_2, lm_1);
   landmarks.erase(landmarks.begin()+lm_id_old);
  }

 }

}



// merge lm_2 into lm_1
void GraphManager::mergeLandmarks( Landmark *lm_1, Landmark *lm_del){

 ROS_INFO("Merging lm %i (%zu pts) into lm %i (%zu pts)", lm_1->id, lm_1->observations.size(),lm_del->id, lm_del->observations.size());

 int new_lm_id = lm_1->id;


 // remove old vertex and edges from graph:
// if (lm_del->g2o_vertex != NULL)
//  optimizer_->removeVertex(lm_del->g2o_vertex);
// for (g2o::HyperGraph::EdgeSet::iterator it = lm_del->proj_edges.begin(); it != lm_del->proj_edges.end(); ++it){
//  optimizer_->removeEdge(*it);
// }


// lm_del->observations_with_edges.insert(lm_del->new_observations.begin(), lm_del->new_observations.end());

 for (std::map<int,int>::iterator it = lm_del->observations.begin(); it != lm_del->observations.end(); ++it){
  Node *n = graph_[it->first];
  int kpt = it->second;

  // check bookkeeping
  //assert(n->kpt_to_landmark[kpt] == lm_del->id);

  // keypoint now directs to new landmark
  n->kpt_to_landmark[kpt] = new_lm_id;
//  n->visible_landmarks.erase(lm_del->id);

  // and new landmark gets new observation
  lm_1->observations[n->id_] = kpt;

 }


 // add new edges for the new observations
 // updateLandmarkInGraph(lm_1);


}
#endif


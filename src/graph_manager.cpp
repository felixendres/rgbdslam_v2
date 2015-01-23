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


#include <sys/time.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
//#include <rgbdslam/CloudTransforms.h>
#include "graph_manager.h"
#include "misc.h"
//#include <sensor_msgs/PointCloud2.h>
#include <opencv2/features2d/features2d.hpp>
#include <QThread>
#include <qtconcurrentrun.h>
#include <QtConcurrentMap> 
#include <utility>
#include <fstream>
#include <limits>
#include <boost/foreach.hpp>
#include <pcl_ros/point_cloud.h>

#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/edge_se3.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "scoped_timer.h"


//typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearCSparseSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;
typedef g2o::LinearSolverPCG<SlamBlockSolver::PoseMatrixType> SlamLinearPCGSolver;
typedef g2o::LinearSolverDense<SlamBlockSolver::PoseMatrixType> SlamLinearDenseSolver;
//typedef std::map<int, g2o::VertexSE3*> VertexIDMap;
typedef std::tr1::unordered_map<int, g2o::HyperGraph::Vertex*>     VertexIDMap;
typedef std::pair<int, g2o::HyperGraph::Vertex*> VertexIDPair;
//std::tr1::unordered_map<int, g2o::HyperGraph::Vertex* >
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;


GraphManager::GraphManager() :
    optimizer_(NULL), 
    reset_request_(false),
    marker_id_(0),
    batch_processing_runs_(false),
    process_node_runs_(false),
    localization_only_(false),
    loop_closures_edges(0), sequential_edges(0),
    next_seq_id(0), next_vertex_id(0),
    current_backend_("none"),
    last_odom_time_(0),
    calibration_vertex_(NULL)
{
  ScopedTimer s(__FUNCTION__);
#ifdef DO_FEATURE_OPTIMIZATION
 next_landmark_id = 0;
#endif

  ParameterServer* ps = ParameterServer::instance();
  createOptimizer(ps->get<std::string>("backend_solver"));
  ros::NodeHandle nh;
  batch_cloud_pub_ = nh.advertise<pointcloud_type>(ps->get<std::string>("individual_cloud_out_topic"),
                                                   ps->get<int>("publisher_queue_size"));
  online_cloud_pub_ = nh.advertise<pointcloud_type>(ps->get<std::string>("online_cloud_out_topic"),
                                                   ps->get<int>("publisher_queue_size"));
  whole_cloud_pub_ = nh.advertise<pointcloud_type>(ps->get<std::string>("aggregate_cloud_out_topic"),
                                                   ps->get<int>("publisher_queue_size"));
  ransac_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/rgbdslam/correspondence_marker", 
                                                                ps->get<int>("publisher_queue_size"));
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("/rgbdslam/pose_graph_markers",
                                                         ps->get<int>("publisher_queue_size"));
  computed_motion_ = tf::Transform::getIdentity();
  init_base_pose_  = tf::Transform::getIdentity();
  std::string fixed_frame = ParameterServer::instance()->get<std::string>("fixed_frame_name");
  std::string base_frame  = ParameterServer::instance()->get<std::string>("base_frame_name");
    
  latest_transform_cache_ = tf::StampedTransform(tf::Transform::getIdentity(), ros::Time::now(), base_frame, fixed_frame);
  timer_ = nh.createTimer(ros::Duration(0.1), &GraphManager::broadcastLatestTransform, this);
}


void GraphManager::setOptimizerVerbose(bool verbose){
    optimizer_->setVerbose(verbose);
}
void GraphManager::createOptimizer(std::string backend, g2o::SparseOptimizer* optimizer)
{
  QMutexLocker locker(&optimizer_mutex_);
  QMutexLocker locker2(&optimization_mutex_);
  // allocating the optimizer
  if(optimizer == NULL){
    if(optimizer_ != NULL){
      for(g2o::SparseOptimizer::VertexIDMap::iterator it = optimizer_->vertices().begin(); it != optimizer_->vertices().end(); it++)
      {
        it->second->edges().clear();
      }
      for(EdgeSet::iterator it = optimizer_->edges().begin(); it != optimizer_->edges().end(); it++)
      {
        //delete *it;
      }
      optimizer_->edges().clear();
      optimizer_->vertices().clear();
    }
    delete optimizer_; 
    optimizer_ = new g2o::SparseOptimizer();
    optimizer_->setVerbose(false);
  } else if (optimizer_ != optimizer){
    delete optimizer_; 
    optimizer_ = new g2o::SparseOptimizer();
    optimizer_->setVerbose(false);
  } 

  //optimizer_->setMethod(g2o::SparseOptimizer::LevenbergMarquardt);

  ParameterServer* ps = ParameterServer::instance();
  if(ps->get<bool>("optimize_landmarks")){
     g2o::BlockSolverX::LinearSolverType *linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>(); // alternative: CHOLMOD
     g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
     //optimizer_->setSolver(solver_ptr);
    g2o::OptimizationAlgorithmDogleg * algo = new g2o::OptimizationAlgorithmDogleg(solver_ptr);
    optimizer_->setAlgorithm(algo);
  }
  else
  {
    SlamBlockSolver* solver = NULL;
    if(backend == "cholmod" || backend == "auto"){
      SlamLinearCholmodSolver* linearSolver = new SlamLinearCholmodSolver();
      linearSolver->setBlockOrdering(false);
      solver = new SlamBlockSolver(linearSolver);
      current_backend_ = "cholmod";
    }
    else if(backend == "csparse"){
      SlamLinearCSparseSolver* linearSolver = new SlamLinearCSparseSolver();
      linearSolver->setBlockOrdering(false);
      solver = new SlamBlockSolver(linearSolver);
      current_backend_ = "csparse";
    }
    else if(backend == "dense"){
      SlamLinearDenseSolver* linearSolver = new SlamLinearDenseSolver();
      solver = new SlamBlockSolver(linearSolver);
      current_backend_ = "dense";
    }
    else if(backend == "pcg"){
      SlamLinearPCGSolver* linearSolver = new SlamLinearPCGSolver();
      solver = new SlamBlockSolver(linearSolver);
      current_backend_ = "pcg";
    }
    else {
      ROS_ERROR("Bad Parameter for g2o Solver backend: %s. User cholmod, csparse or pcg", backend.c_str());
      ROS_INFO("Falling Back to Cholmod Solver");
      SlamLinearCholmodSolver* linearSolver = new SlamLinearCholmodSolver();
      linearSolver->setBlockOrdering(false);
      solver = new SlamBlockSolver(linearSolver);
      current_backend_ = "cholmod";
    }
    //optimizer_->setSolver(solver);
    g2o::OptimizationAlgorithmLevenberg * algo = new g2o::OptimizationAlgorithmLevenberg(solver);
    //g2o::OptimizationAlgorithmDogleg * algo = new g2o::OptimizationAlgorithmDogleg(solver);
    optimizer_->setAlgorithm(algo);
  }



 optimizer_->setVerbose(false);
 //optimizer_->setUserLambdaInit(100.);

#ifdef DO_FEATURE_OPTIMIZATION
 float fx = ps->get<double>("depth_camera_fx") > 0 ? ps->get<double>("depth_camera_fx") : 525; 
 float fy = ps->get<double>("depth_camera_fy") > 0 ? ps->get<double>("depth_camera_fy") : 525;
 float cx = ps->get<double>("depth_camera_cx") > 0 ? ps->get<double>("depth_camera_cx") : 319.5;
 float cy = ps->get<double>("depth_camera_cy") > 0 ? ps->get<double>("depth_camera_cy") : 239.5;

 g2o::ParameterCamera* cameraParams = new g2o::ParameterCamera();
 cameraParams->setKcam(fx, fy, cx, cy);
 g2o::SE3Quat offset; // identity
 cameraParams->setOffset(offset);
 cameraParams->setId(0);
 optimizer_->addParameter(cameraParams);
#endif
}


QList<int> GraphManager::getPotentialEdgeTargetsWithDijkstra(const Node* new_node, int sequential_targets, int geodesic_targets, int sampled_targets, int predecessor_id, bool include_predecessor)
{
    QList<int> ids_to_link_to; //return value
    if(predecessor_id < 0) predecessor_id = graph_.size()-1;
    //Prepare output
    std::stringstream ss;
    ss << "Node ID's to compare with candidate for node " << graph_.size() << ". Sequential: ";

   if((int)camera_vertices.size() <= sequential_targets+geodesic_targets+sampled_targets ||
      camera_vertices.size() <= 1)
    { //if less prev nodes available than targets requestet, just use all
      sequential_targets = sequential_targets+geodesic_targets+sampled_targets;
      geodesic_targets = 0;
      sampled_targets = 0;
      predecessor_id = graph_.size()-1;
    }

    if(sequential_targets > 0){
      //all the sequential targets (will be checked last)
      for (int i=1; i < sequential_targets+1 && predecessor_id-i >= 0; i++) { 
          ids_to_link_to.push_back(predecessor_id-i); 
          ss << ids_to_link_to.back() << ", " ; 
      }
    }

    if(geodesic_targets > 0){
      g2o::HyperDijkstra hypdij(optimizer_);
      g2o::UniformCostFunction cost_function;
      g2o::VertexSE3* prev_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[predecessor_id]->vertex_id_));
      hypdij.shortestPaths(prev_vertex,&cost_function,ParameterServer::instance()->get<int>("geodesic_depth"));
      g2o::HyperGraph::VertexSet& vs = hypdij.visited();

      //Need to convert vertex_id to node id
      std::map<int, int> vertex_id_to_node_id;
      for (graph_it it = graph_.begin(); it !=graph_.end(); ++it){
            Node *node = it->second;
            vertex_id_to_node_id[node->vertex_id_] = node->id_;
            //ROS_WARN("ID Pair: %d, %d", node->vertex_id_, node->id_);
      }

      //Geodesic Neighbours except sequential
      std::map<int,int> neighbour_indices; //maps neighbour ids to their weights in sampling
      int sum_of_weights=0;
      for (g2o::HyperGraph::VertexSet::iterator vit=vs.begin(); vit!=vs.end(); vit++) { //FIXME: Mix of vertex id and graph node (with features) id
        int vid = (*vit)->id();
        //ROS_WARN("Vertex ID: %d", vid);
        int id = 0;
        try{
          id = vertex_id_to_node_id.at(vid);
        }
        catch (std::exception e){//Catch exceptions: Unexpected problems shouldn't crash the application
          ROS_ERROR("Vertex ID %d has no corresponding node", vid);
          ROS_ERROR("Map Content:");
          for(std::map<int,int>::const_iterator it = vertex_id_to_node_id.begin(); it != vertex_id_to_node_id.end(); it++){
            ROS_ERROR("Node ID %d: Vertex ID %d", it->first, it->second);
          }
          for(g2o::SparseOptimizer::VertexIDMap::iterator it = optimizer_->vertices().begin(); it != optimizer_->vertices().end(); it++)
          {
            ROS_ERROR("Vertex ID %d", it->first);
          }
        }
        if(!graph_.at(id)->matchable_) continue;
        if(id < predecessor_id-sequential_targets || (id > predecessor_id && id <= (int)graph_.size()-1)){ //Geodesic Neighbours except sequential 
            int weight = abs(predecessor_id-id);
            neighbour_indices[id] = weight; //higher probability to be drawn if far away
            sum_of_weights += weight;
        }
      }

      //Sample targets from graph-neighbours
      ss << "Dijkstra: ";
      while(ids_to_link_to.size() < sequential_targets+geodesic_targets && neighbour_indices.size() != 0){ 
        int random_pick = rand() % sum_of_weights;
        ROS_DEBUG("Pick: %d/%d", random_pick, sum_of_weights);
        int weight_so_far = 0;
        for(std::map<int,int>::iterator map_it = neighbour_indices.begin(); map_it != neighbour_indices.end(); map_it++ ){
          weight_so_far += map_it->second;
          ROS_DEBUG("Checking: %d, %d, %d", map_it->first, map_it-> second, weight_so_far);
          if(weight_so_far > random_pick){//found the selected one
            int sampled_id = map_it->first;
            ids_to_link_to.push_front(sampled_id);
            ss << ids_to_link_to.front() << ", " ; 
            sum_of_weights -= map_it->second;
            ROS_DEBUG("Taking ID: %d, decreasing sum of weights to %d", map_it->first, sum_of_weights);
            neighbour_indices.erase(map_it);
            ROS_ERROR_COND(sum_of_weights<0, "Sum of weights should never be zero");
            break;
          }
          ROS_DEBUG("Skipping ID: %d", map_it->first);
        }//for
      }
    }
    
    if(sampled_targets > 0){
      ss << "Random Sampling: ";
      std::vector<int> non_neighbour_indices;//initially holds all, then neighbours are deleted
      non_neighbour_indices.reserve(graph_.size());
      for (QList<int>::iterator it = keyframe_ids_.begin(); it != keyframe_ids_.end(); it++){
        if(ids_to_link_to.contains(*it) == 0 && graph_.at(*it)->matchable_){
          non_neighbour_indices.push_back(*it); 
        }
      }

      //Sample targets from non-neighbours (search new loops)
      while(ids_to_link_to.size() < geodesic_targets+sampled_targets+sequential_targets && non_neighbour_indices.size() != 0){ 
          int index_of_v_id = rand() % non_neighbour_indices.size();
          int sampled_id = non_neighbour_indices[index_of_v_id];
          non_neighbour_indices[index_of_v_id] = non_neighbour_indices.back(); //copy last id to position of the used id
          non_neighbour_indices.resize(non_neighbour_indices.size()-1); //drop last id
          ids_to_link_to.push_front(sampled_id);
          ss << ids_to_link_to.front() << ", " ; 
      }
    }

    if(include_predecessor){
      ids_to_link_to.push_back(predecessor_id); 
      ss << predecessor_id;
    }
    ROS_INFO("%s", ss.str().c_str());
    return ids_to_link_to; //only compare to first frame
}

void GraphManager::resetGraph(){
    #ifdef DO_FEATURE_OPTIMIZATION
     next_landmark_id = 0;
     landmarks.clear();
    #endif

     next_seq_id = next_vertex_id = 0;
     camera_vertices.clear();
     cam_cam_edges_.clear();
     odometry_edges_.clear();
    marker_id_ =0;
    ParameterServer* ps = ParameterServer::instance();
    createOptimizer(ps->get<std::string>("backend_solver"));

    //Q_FOREACH(Node* node, graph_) { delete node; }
    BOOST_FOREACH(GraphNodeType entry, graph_){ delete entry.second; entry.second = NULL; }
    //for(unsigned int i = 0; i < graph_.size(); delete graph_[i++]);//No body
    graph_.clear();
    keyframe_ids_.clear();
    Q_EMIT resetGLViewer();
    curr_best_result_ = MatchingResult();
    //current_poses_.clear();
    current_edges_.clear();
    reset_request_ = false;
    loop_closures_edges = 0; 
    sequential_edges = 0;
}
/*NEW
void GraphManager::addOutliers(Node* new_node, std::vector<cv::DMatch> inlier_matches){
    std::vector<bool> outlier_flags(new_node->feature_locations_3d_.size(), true);
    BOOST_FOREACH(cv::DMatch& m, inlier_matches){ 
      outlier_flags[m.queryIdx] = false;
}
*/
void GraphManager::firstNode(Node* new_node) 
{
    Q_EMIT renderableOctomap(&co_server_);
    //set the node id only if the node is actually added to the graph
    //needs to be done here as the graph size can change inside this function
    new_node->id_ = graph_.size();
    new_node->seq_id_ = next_seq_id++; // allways incremented, even if node is not added
    init_base_pose_ =  new_node->getGroundTruthTransform();//identity if no MoCap available
    printTransform("Ground Truth Transform for First Node", init_base_pose_);
    //new_node->buildFlannIndex(); // create index so that next nodes can use it
    g2o::VertexSE3* reference_pose = new g2o::VertexSE3;

    new_node->vertex_id_ = next_vertex_id++;
    graph_[new_node->id_] = new_node;
    reference_pose->setId(new_node->vertex_id_);

    camera_vertices.insert(reference_pose);

    ROS_INFO("Adding initial node with id %i and seq %i, v_id: %i", new_node->id_, new_node->seq_id_, new_node->vertex_id_);
    g2o::SE3Quat g2o_ref_se3 = tf2G2O(init_base_pose_);
    reference_pose->setEstimate(g2o_ref_se3);
    reference_pose->setFixed(true);//fix at origin
    optimizer_mutex_.lock();
    optimizer_->addVertex(reference_pose); 
    optimizer_mutex_.unlock();
    QString message;
    Q_EMIT setGUIInfo(message.sprintf("Added first node with %i keypoints to the graph", (int)new_node->feature_locations_2d_.size()));
    //pointcloud_type::Ptr the_pc(new_node->pc_col); //this would delete the cloud after the_pc gets out of scope
    QMatrix4x4 latest_transform = g2o2QMatrix(g2o_ref_se3);
    if(!ParameterServer::instance()->get<bool>("glwidget_without_clouds")) { 
      Q_EMIT setPointCloud(new_node->pc_col.get(), latest_transform);
      Q_EMIT setFeatures(&(new_node->feature_locations_3d_));
    }
    //current_poses_.append(latest_transform);
    this->addKeyframe(new_node->id_);
    if(ParameterServer::instance()->get<bool>("octomap_online_creation")) { 
      optimizeGraph(); //will do the following at the end:
      //updateCloudOrigin(new_node);
      //renderToOctomap(new_node);
    }

    process_node_runs_ = false;
}

/*/Translational norm
inline float sqrTransNorm(const Eigen::Matrix4f& t){
    return t(0,3)*t(0,3)+t(1,3)*t(1,3)+t(2,3)*t(2,3);
}
*/
void updateInlierFeatures(const MatchingResult& mr, Node* new_node, Node* old_node)
{
  BOOST_FOREACH(const cv::DMatch& match, mr.inlier_matches){
    assert(new_node->feature_matching_stats_.size() > match.queryIdx);
    assert(old_node->feature_matching_stats_.size() > match.trainIdx);
    unsigned char& new_flag = new_node->feature_matching_stats_[match.queryIdx];
    if(new_flag < 255) ++new_flag;
    unsigned char& old_flag = old_node->feature_matching_stats_[match.trainIdx];
    if(old_flag < 255) ++old_flag;
  }
}

bool GraphManager::nodeComparisons(Node* new_node, 
                                   QMatrix4x4& curr_motion_estimate,
                                   bool& edge_to_keyframe)///Output:contains the best-yet of the pairwise motion estimates for the current node
{
    /// \callergraph
    ScopedTimer s(__FUNCTION__);
    process_node_runs_ = true;

    ParameterServer* ps = ParameterServer::instance();
    int num_keypoints = (int)new_node->feature_locations_2d_.size();
    if (num_keypoints < ps->get<int>("min_matches") && 
        ! ps->get<bool>("keep_all_nodes"))
    {
        ROS_INFO("Found only %i features on image, node is not included", num_keypoints);
        process_node_runs_ = false;
        return false;
    }

    //setting of the node id needs to be done here as the graph size can change inside this method
    new_node->id_ = graph_.size();
    new_node->seq_id_ = next_seq_id++; // allways incremented, even if node is not added


    earliest_loop_closure_node_ = new_node->id_;
    unsigned int num_edges_before = cam_cam_edges_.size();
    edge_to_keyframe = false; //not yet found
    marker_id_ = 0; //overdraw old markers
    ROS_DEBUG("Graphsize: %d Nodes", (int) graph_.size());

    int sequentially_previous_id = graph_.rbegin()->second->id_; 
    //int best_match_candidate_id = sequentially_previous_id; 
    MatchingResult mr;
    int prev_best = mr.edge.id1;
    curr_best_result_ = mr;

    //Initial Comparison ######################################################################
    bool predecessor_matched = false;
    if(ps->get<double>("min_translation_meter") > 0.0 ||
       ps->get<double>("min_rotation_degree") > 0.0)
    {
      //First check if trafo to last frame is big
      //Node* prev_frame = graph_[graph_.size()-1];
      Node* prev_frame = graph_[graph_.size()-1];
      if(localization_only_ && curr_best_result_.edge.id1 > 0){ prev_frame =  graph_[curr_best_result_.edge.id1]; }
      ROS_INFO("Comparing new node (%i) with previous node %i", new_node->id_, prev_frame->id_);
      mr = new_node->matchNodePair(prev_frame);
      ROS_INFO("Node comparison result: %s", mr.toString());
      if(mr.edge.id1 >= 0 && mr.edge.id2 >= 0) {//Found trafo
        ros::Time time1 = prev_frame->header_.stamp;
        ros::Time time2 = new_node->header_.stamp;
        ros::Duration delta_time =  time2 - time1;
        if(!isBigTrafo(mr.edge.transform) || !isSmallTrafo(mr.edge.transform, delta_time.toSec())){ //Found trafo, but bad trafo (too small to big)
            ROS_WARN("Transformation not within bounds. Did not add as Node");
            //Send the current pose via tf nevertheless
            tf::Transform incremental = eigenTransf2TF(mr.edge.transform);
            g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[prev_frame->id_]->vertex_id_));
            tf::Transform previous = eigenTransf2TF(v->estimate());
            tf::Transform combined = previous*incremental;
            latest_transform_cache_ = stampedTransformInWorldFrame(new_node, combined);
            printTransform("Computed new transform", latest_transform_cache_);
            broadcastTransform(latest_transform_cache_);
            process_node_runs_ = false;
            curr_best_result_ = mr;
            return false;
        } else { //Good Transformation
          ROS_DEBUG_STREAM("Information Matrix for Edge (" << mr.edge.id1 << "<->" << mr.edge.id2 << ") \n" << mr.edge.informationMatrix);
          if (addEdgeToG2O(mr.edge, prev_frame, new_node,  true, true, curr_motion_estimate)) 
          {
            graph_[new_node->id_] = new_node; //Needs to be added
            if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
#ifdef DO_FEATURE_OPTIMIZATION
            updateLandmarks(mr, prev_frame,new_node);
#endif
            updateInlierFeatures(mr, new_node, prev_frame);
            graph_[mr.edge.id1]->valid_tf_estimate_ = true;
            ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
            curr_best_result_ = mr;
            //addOutliers(Node* new_node, mr.inlier_matches);

          } else {
            ROS_INFO("Edge not added");
            process_node_runs_ = false;
            return false;
          }
        }
        predecessor_matched = true;
      }
      else {
        ROS_WARN("Found no transformation to predecessor (edge ids are negative)");
      // No transformation to predecessor. No other choice than try other candidates. This is done below 
      }
    }//end: Initial Comparison ######################################################################

    //Eigen::Matrix4f ransac_trafo, final_trafo;
    QList<int> vertices_to_comp;
    int  seq_cand = localization_only_ ? 0 : ps->get<int>("predecessor_candidates") - 1; //minus one, because the first predecessor has already been checked
    int geod_cand = ps->get<int>("neighbor_candidates");
    int samp_cand = ps->get<int>("min_sampled_candidates");
    if(predecessor_matched){
      vertices_to_comp = getPotentialEdgeTargetsWithDijkstra(new_node, seq_cand, geod_cand, samp_cand, curr_best_result_.edge.id1); 
    } else {
      vertices_to_comp = getPotentialEdgeTargetsWithDijkstra(new_node, seq_cand, geod_cand, samp_cand, sequentially_previous_id, true); 
    }
    if(prev_best >= 0 && !vertices_to_comp.contains(prev_best)){
      vertices_to_comp.append(prev_best);//Test: definitely reuse best (currently: the oldest) matched node from last
    }

    QList<const Node* > nodes_to_comp;//only necessary for parallel computation

    //MAIN LOOP: Compare node pairs ######################################################################
    if (ps->get<bool>("concurrent_edge_construction")) 
    {
        std::stringstream ss;
        for (int id_of_id = (int) vertices_to_comp.size() - 1; id_of_id >= 0; id_of_id--) {
            //First compile a qlist of the nodes to be compared, then run the comparisons in parallel,
            //collecting a qlist of the results (using the blocking version of mapped).
            nodes_to_comp.push_front(graph_[vertices_to_comp[id_of_id]]); 
            ss << vertices_to_comp[id_of_id] << ", ";
        }
        ROS_INFO_STREAM("Nodes to compare: " << ss);
        QThreadPool* qtp = QThreadPool::globalInstance();
        ROS_INFO("Running node comparisons in parallel in %i (of %i) available threads", qtp->maxThreadCount() - qtp->activeThreadCount(), qtp->maxThreadCount());
        if (qtp->maxThreadCount() - qtp->activeThreadCount() == 1) {
            //Never seen this problem...
            ROS_WARN("Few Threads Remaining: Increasing maxThreadCount to %i", qtp->maxThreadCount()+1);
            qtp->setMaxThreadCount(qtp->maxThreadCount() + 1);
        }
        QList<MatchingResult> results = QtConcurrent::blockingMapped(nodes_to_comp, boost::bind(&Node::matchNodePair, new_node, _1));

        for (int i = 0; i < results.size(); i++) 
        {
            MatchingResult& mr = results[i];
            ROS_INFO("Result of comparison %d: %s", i, mr.toString());
            if (mr.edge.id1 >= 0 ) {
              //ROS_INFO("new node has id %i", new_node->id_);
              assert(graph_[mr.edge.id1]);

              ros::Duration delta_time = new_node->header_.stamp - graph_[mr.edge.id1]->header_.stamp;
              if (isSmallTrafo(mr.edge.transform, delta_time.toSec()) &&
                  addEdgeToG2O(mr.edge,graph_[mr.edge.id1],new_node, isBigTrafo(mr.edge.transform), mr.inlier_matches.size() > curr_best_result_.inlier_matches.size(), curr_motion_estimate))
                { 
                  graph_[new_node->id_] = new_node; //Needs to be added
                  if(mr.edge.id1 == mr.edge.id2-1 ) {//older == newer-1
                    predecessor_matched = true;
                  }

#ifdef DO_FEATURE_OPTIMIZATION
                  updateLandmarks(mr, graph_[mr.edge.id1],new_node);
#endif
                  updateInlierFeatures(mr, new_node, graph_[mr.edge.id1]);
                  graph_[mr.edge.id1]->valid_tf_estimate_ = true;
                  ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
                  if (mr.inlier_matches.size() > curr_best_result_.inlier_matches.size()) {
                  //if (curr_best_result_.edge.id1 == -1 || sqrTransNorm(mr.final_trafo) < sqrTransNorm(curr_best_result_.final_trafo)) {
                    curr_best_result_ = mr;
                  }
                  if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
                }
                else{
                  ROS_WARN("Rejected edge from %d to %d", mr.edge.id1, mr.edge.id2);
                }
            }
        }
    } else { //Nonconcurrent
        for (int id_of_id = (int) vertices_to_comp.size() - 1; id_of_id >= 0; id_of_id--) {
            Node* node_to_compare = graph_[vertices_to_comp[id_of_id]];
            ROS_INFO("Comparing new node (%i) with node %i / %i", new_node->id_, vertices_to_comp[id_of_id], node_to_compare->id_);
            MatchingResult mr = new_node->matchNodePair(node_to_compare);
            ROS_INFO("Result of comparison: %s", mr.toString());

            if (mr.edge.id1 >= 0) {
              ros::Duration delta_time = new_node->header_.stamp - graph_[mr.edge.id1]->header_.stamp;
              if (isSmallTrafo(mr.edge.transform, delta_time.toSec()) &&
                  addEdgeToG2O(mr.edge, node_to_compare, new_node, isBigTrafo(mr.edge.transform), mr.inlier_matches.size() > curr_best_result_.inlier_matches.size(), curr_motion_estimate))
              {
#ifdef DO_FEATURE_OPTIMIZATION
                updateLandmarks(mr, node_to_compare, new_node);
#endif
                graph_[new_node->id_] = new_node; //Needs to be added
                if(mr.edge.id1 == mr.edge.id2-1 ) {//older == newer-1
                  predecessor_matched = true;
                }
                updateInlierFeatures(mr, new_node, node_to_compare);
                graph_[mr.edge.id1]->valid_tf_estimate_ = true;
                ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
                if (mr.inlier_matches.size() > curr_best_result_.inlier_matches.size()) {
                  //if (curr_best_result_.edge.id1 == -1 || sqrTransNorm(mr.final_trafo) < sqrTransNorm(curr_best_result_.final_trafo)) {
                  curr_best_result_ = mr;
                }
                if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
              }
              else {
                ROS_INFO("Matching result rejected for being too big? Time Delta: %f", delta_time.toSec());
              }
            }
            else 
            {
              ROS_INFO("Matching result rejected for edge.id1");
            }
        }
    }

    //END OF MAIN LOOP: Compare node pairs ######################################################################
    bool found_trafo = (cam_cam_edges_.size() != num_edges_before);
    bool valid_odometry = !ps->get<std::string>("odom_frame_name").empty();// || odom_tf_old.frame_id_ == "missing_odometry" || odom_tf_new.frame_id_ == "missing_odometry"; 

    bool keep_anyway = (ps->get<bool>("keep_all_nodes") || 
                        (((int)new_node->feature_locations_3d_.size() > ps->get<int>("min_matches")) 
                         && ps->get<bool>("keep_good_nodes")));
    ros::Duration delta_time = new_node->header_.stamp - graph_[sequentially_previous_id]->header_.stamp;
    float time_delta_sec = fabs(delta_time.toSec());
    ROS_WARN_COND(time_delta_sec >= 0.1, "Time jump (time delta: %.2f)", time_delta_sec);

    //If no trafo is found, only keep if a parameter says so or odometry is available. 
    //Otherwise only add a constant position edge, if the predecessor wasn't matched and its timestamp is nearby
    if((!found_trafo && valid_odometry) || 
       ((!found_trafo && keep_anyway) || 
        (!predecessor_matched && time_delta_sec < 0.1))) //FIXME: Add parameter for constant position assumption and time_delta
    { 
      LoadedEdge3D odom_edge;

      odom_edge.id1 = sequentially_previous_id;
      odom_edge.id2 = new_node->id_;
      odom_edge.transform.setIdentity();
      curr_motion_estimate = eigenTF2QMatrix(odom_edge.transform);
      odom_edge.informationMatrix = Eigen::Matrix<double,6,6>::Zero(); 
      ROS_WARN("No valid (sequential) transformation between %d and %d: Using constant position assumption.", odom_edge.id1, odom_edge.id2);
      odom_edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity() / time_delta_sec;//e-9; 
      addEdgeToG2O(odom_edge,graph_[sequentially_previous_id],new_node, true,true, curr_motion_estimate);
      graph_[new_node->id_] = new_node; //Needs to be added
      new_node->valid_tf_estimate_ = false; //Don't use for postprocessing, rendering etc
      MatchingResult mr;
      mr.edge = odom_edge;
      curr_best_result_ = mr;
    }

    return cam_cam_edges_.size() > num_edges_before;
}

void GraphManager::localizationUpdate(Node* new_node, QMatrix4x4 motion_estimate)
{
    //First render the cloud with the best frame-to-frame estimate
    //The transform will get updated when optimizeGraph finishes
    pointcloud_type* cloud_to_visualize = new_node->pc_col.get();
    std_vector_of_eigen_vector4f * features_to_visualize = &(new_node->feature_locations_3d_);
    if(!new_node->valid_tf_estimate_){
      cloud_to_visualize = new pointcloud_type();
      features_to_visualize = new std_vector_of_eigen_vector4f();
    }

    Q_EMIT setPointCloud(cloud_to_visualize, motion_estimate);
    Q_EMIT setFeatures(features_to_visualize);

    optimizeGraph();
    g2o::VertexSE3* new_v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[new_node->id_]->vertex_id_));
    QMutexLocker locker(&optimizer_mutex_);
    QMutexLocker locker2(&optimization_mutex_);
    optimizer_->removeVertex(new_v); //Also removes the edges
}
// returns true, iff node could be added to the cloud
bool GraphManager::addNode(Node* new_node) 
{
  ScopedTimer s(__FUNCTION__, false);

  if(reset_request_) resetGraph(); 
  ParameterServer* ps = ParameterServer::instance();
  if (new_node->feature_locations_2d_.size() < ps->get<int>("min_matches")) {
      ROS_WARN("Skipping node because it has only %zu features (minimum is %d)",new_node->feature_locations_2d_.size(), ps->get<int>("min_matches"));
      return false;
  }

  //First Node, so only build its index, insert into storage and add a
  //vertex at the origin, of which the position is very certain
  if (graph_.size()==0){
      firstNode(new_node);
      return true;
  }

  //All other nodes are processed in the following
  QMatrix4x4 motion_estimate;///Output:contains the best-yet of the pairwise motion estimates for the current node
  bool edge_to_last_keyframe_found = false;
  bool found_match = nodeComparisons(new_node, motion_estimate, edge_to_last_keyframe_found);

  if (found_match) 
  { //Success
    if(localization_only_)
    {
      ROS_INFO("Localizing (only)");
      localizationUpdate(new_node, motion_estimate);
    }
    else //Mapping
    {
      //This needs to be done before rendering, so deleting the cloud always works
      graph_[new_node->id_] = new_node; //Node->id_ == Graph_ Index
      //First render the cloud with the best frame-to-frame estimate
      //The transform will get updated when optimizeGraph finishes
      pointcloud_type* cloud_to_visualize = new_node->pc_col.get();
      std_vector_of_eigen_vector4f * features_to_visualize = &(new_node->feature_locations_3d_);
      if(!new_node->valid_tf_estimate_) {
        cloud_to_visualize = new pointcloud_type();
        features_to_visualize = new std_vector_of_eigen_vector4f();
      }
      ROS_INFO("Adding node with id %i and seq id %i to the graph", new_node->id_, new_node->seq_id_);

      //Juergen: bad hack, should instead prevent the creation of the cloud, but this is faster implementation wise
      ROS_INFO_STREAM("create cloud " << new_node->id_ << " " << ps->get<int>("create_cloud_every_nth_node") << " " << new_node->id_%ps->get<int>("create_cloud_every_nth_node")) ;
      if((new_node->id_%ps->get<int>("create_cloud_every_nth_node"))!=0){
        new_node->clearPointCloud();
      }

      if(!edge_to_last_keyframe_found && earliest_loop_closure_node_ > keyframe_ids_.back()) {
        this->addKeyframe(new_node->id_-1);//use the id of the node before, because that one is still localized w.r.t. a keyframe. So keyframes are connected
      } else {
        if(ps->get<bool>("visualize_keyframes_only")){
          cloud_to_visualize = new pointcloud_type();
          features_to_visualize = new std_vector_of_eigen_vector4f();
        }
      }
      if(ps->get<bool>("glwidget_without_clouds")){
        cloud_to_visualize = new pointcloud_type();
        features_to_visualize = new std_vector_of_eigen_vector4f();
      }

      Q_EMIT setPointCloud(cloud_to_visualize, motion_estimate);
      Q_EMIT setFeatures(features_to_visualize);
      ROS_INFO("Added Node, new graphsize: %i nodes", (int) graph_.size());
      if(ps->get<int>("optimizer_skip_step") > 0 && 
          (camera_vertices.size() % ps->get<int>("optimizer_skip_step")) == 0)
      { 
        optimizeGraph();
      }

      //This is old stuff for visualization via rviz - not tested in a long time, would be safe to delete _if_ nobody uses it
      visualizeGraphEdges();
      visualizeGraphNodes();
      visualizeFeatureFlow3D(marker_id_++);

    } 
  }
  else //Unsuccesful
  { 
    if(graph_.size() == 1){//if there is only one node which has less features, replace it by the new one
      ROS_WARN("Choosing new initial node, because it has more features");
      if(new_node->feature_locations_2d_.size() > graph_[0]->feature_locations_2d_.size()){
        this->resetGraph();
        process_node_runs_ = false;
        firstNode(new_node);
        return true;
      }
    } else { //delete new_node; //is now  done by auto_ptr
      ROS_WARN("Did not add as Node");
    }
  }

 //Info output
 QString message;
 Q_EMIT setGUIInfo(message.sprintf("%s, Camera Pose Graph Size: %iN/%iE, Duration: %f, Inliers:%5i",// &chi;<sup>2</sup>: %f", 
       found_match ? "Added" : "Ignored", (int)camera_vertices.size(), (int)cam_cam_edges_.size(), s.elapsed(), (int)curr_best_result_.inlier_matches.size()));//, optimizer_->chi2()));
 process_node_runs_ = false;
 ROS_INFO("%s", qPrintable(message));
 return found_match;
}

void GraphManager::addKeyframe(int id)
{
  ScopedTimer s(__FUNCTION__);
  //Delete content of nodes that are behind the keyframe that is added
  if(ParameterServer::instance()->get<bool>("clear_non_keyframes")){
    if(keyframe_ids_.size() >= 2){
      int most_recent= keyframe_ids_.back();
      int second_most_recent= keyframe_ids_.at(keyframe_ids_.size() - 2);
      ROS_INFO("Clearing out data for nodes between keyframes %d and %d", second_most_recent, most_recent);
      for (std::map<int, Node*>::iterator it=graph_.begin(); it!=graph_.end(); ++it){
        Node* mynode = it->second;
        if(mynode->id_ > second_most_recent && mynode->id_ < most_recent){
          //mynode->getMemoryFootprint(true);//print 
          mynode->clearPointCloud();
          mynode->clearFeatureInformation();
        }
      }
    }
  }

  keyframe_ids_.push_back(id); 

  std::stringstream ss; ss << keyframe_ids_.size() << " Keyframes: ";
  BOOST_FOREACH(int i, keyframe_ids_){ ss << i << ", "; }
  ROS_INFO("%s", ss.str().c_str());
}

bool GraphManager::addEdgeToG2O(const LoadedEdge3D& edge,
                                Node* n1, Node* n2,  
                                bool largeEdge, bool set_estimate, 
                                QMatrix4x4& motion_estimate) //Pure output
{
    ScopedTimer s(__FUNCTION__);
    assert(n1);
    assert(n2);
    assert(n1->id_ == edge.id1);
    assert(n2->id_ == edge.id2);

    QMutexLocker locker(&optimizer_mutex_);
    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(n1->vertex_id_));
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(n2->vertex_id_));

    // at least one vertex has to be created, assert that the transformation
    // is large enough to avoid to many vertices on the same spot
    if (!v1 || !v2){
        if (!largeEdge) {
            ROS_INFO("Edge to new vertex is too short, vertex will not be inserted");
            return false; 
        }
    }

    if(!v1 && !v2){
      ROS_ERROR("Missing both vertices: %i, %i, cannot create edge", edge.id1, edge.id2);
      return false;
    }
    else if (!v1 && v2) {
        v1 = new g2o::VertexSE3;
        assert(v1);
        int v_id = next_vertex_id++;
        v1->setId(v_id);

        n1->vertex_id_ = v_id; // save vertex id in node so that it can find its vertex
        v1->setEstimate(v2->estimate() * edge.transform.inverse());
        camera_vertices.insert(v1);
        optimizer_->addVertex(v1); 
        motion_estimate = eigenTF2QMatrix(v1->estimate()); 
        ROS_WARN("Creating previous id. This is unexpected by the programmer");
    }
    else if (!v2 && v1) {
        v2 = new g2o::VertexSE3;
        assert(v2);
        int v_id = next_vertex_id++;
        v2->setId(v_id);
        n2->vertex_id_ = v_id;
        v2->setEstimate(v1->estimate() * edge.transform);
        camera_vertices.insert(v2);
        optimizer_->addVertex(v2); 
        motion_estimate = eigenTF2QMatrix(v2->estimate()); 
    }
    else if(set_estimate){
        v2->setEstimate(v1->estimate() * edge.transform);
        motion_estimate = eigenTF2QMatrix(v2->estimate()); 
    }
    g2o::EdgeSE3* g2o_edge = new g2o::EdgeSE3;
    g2o_edge->vertices()[0] = v1;
    g2o_edge->vertices()[1] = v2;
    Eigen::Isometry3d meancopy(edge.transform); 
    g2o_edge->setMeasurement(meancopy);
    //Change setting from which mahal distance the robust kernel is used: robust_kernel_.setDelta(1.0);
    g2o_edge->setRobustKernel(&robust_kernel_);
    // g2o_edge->setInverseMeasurement(edge.trannsform.inverse());
    g2o_edge->setInformation(edge.informationMatrix);
    optimizer_->addEdge(g2o_edge);
    //ROS_DEBUG_STREAM("Added Edge ("<< edge.id1 << "-" << edge.id2 << ") to Optimizer:\n" << edge.transform << "\nInformation Matrix:\n" << edge.informationMatrix);
    cam_cam_edges_.insert(g2o_edge);
    current_match_edges_.insert(g2o_edge); //Used if all previous vertices are fixed ("pose_relative_to" == "all")
//    new_edges_.append(qMakePair(edge.id1, edge.id2));

    if(abs(edge.id1 - edge.id2) > ParameterServer::instance()->get<int>("predecessor_candidates")){
      loop_closures_edges++;
    } else {
      sequential_edges++;
    }
    current_edges_.append( qMakePair(n1->id_, n2->id_));

    if (ParameterServer::instance()->get<std::string>("pose_relative_to") == "inaffected") {
      v1->setFixed(false);
      v2->setFixed(false);
    }
    else if(ParameterServer::instance()->get<std::string>("pose_relative_to") == "largest_loop") {
      earliest_loop_closure_node_ = std::min(earliest_loop_closure_node_, edge.id1);
      earliest_loop_closure_node_ = std::min(earliest_loop_closure_node_, edge.id2);
    }
    return true;
}

double GraphManager::optimizeGraph(double break_criterion, bool nonthreaded){
  if(ParameterServer::instance()->get<bool>("concurrent_optimization") && !nonthreaded) {
    ROS_DEBUG("Optimization done in Thread");
    QtConcurrent::run(this, &GraphManager::optimizeGraphImpl, break_criterion); 
    return -1.0;
  }
  else { //Non-concurrent
    return optimizeGraphImpl(break_criterion);//regular function call
  }
}

void fixationOfVertices(std::string strategy, 
                        g2o::SparseOptimizer* optimizer, 
                        std::map<int, Node* >& graph,
                        g2o::HyperGraph::VertexSet& camera_vertices,
                        int earliest_loop_closure_node
                        ){
    //Fixation strategies
    if (strategy == "previous" && graph.size() > 2) {
      optimizer->setFixed(camera_vertices, false);
      optimizer->vertex(graph[graph.size() - 2]->vertex_id_)->setFixed(true);
    }
    else if (strategy == "largest_loop"){
      //std::stringstream ss; ss << "Nodes in or outside loop: ";
      for (std::map<int, Node*>::iterator it=graph.begin(); it!=graph.end(); ++it){
        Node* mynode = it->second;
        //Even before oldest matched node?
        bool is_outside_largest_loop =  mynode->id_ < earliest_loop_closure_node;
        //ss << mynode->id_ << (is_outside_largest_loop ? "o, " : "i, ");
        optimizer->vertex(mynode->vertex_id_)->setFixed(is_outside_largest_loop);
      }
      //ROS_INFO("%s", ss.str().c_str());
    }
    else if (strategy == "first") {
      optimizer->setFixed(camera_vertices, false);
      optimizer->vertex(graph[0]->vertex_id_)->setFixed(true);
    }
}
double GraphManager::optimizeGraphImpl(double break_criterion)
{
  ScopedTimer s(__FUNCTION__, false, true); // not only for logging
  ParameterServer* ps = ParameterServer::instance();
  double stop_cond = break_criterion > 0.0 ? break_criterion : ps->get<double>("optimizer_iterations");
  ROS_WARN_NAMED("eval", "Loop Closures: %u, Sequential Edges: %u", loop_closures_edges, sequential_edges);
  ROS_WARN("Starting Optimization");
  double chi2 = std::numeric_limits<double>::max();
  if(!optimization_mutex_.tryLock(2/*milliseconds*/))
  {
    ROS_INFO("Attempted Graph Optimization, but it is already running. Skipping.");
    return -1.0;
  }
  else //Got the lock
  {
    optimizer_mutex_.lock();
    if(optimizer_->vertices().size() == 0){
      ROS_ERROR("Attempted Graph Optimization on an empty graph!");
      return -1.0;
    }

    fixationOfVertices(ps->get<std::string>("pose_relative_to"), 
                       optimizer_, graph_, camera_vertices, earliest_loop_closure_node_); 

#ifdef DO_FEATURE_OPTIMIZATION
   printLandmarkStatistic();
   if (ps->get<bool>("optimize_landmarks")){
     updateProjectionEdges();
     optimizer_->initializeOptimization(cam_lm_edges);
   } else /*continued as else if below*/
#endif
   if (ps->get<std::string>("pose_relative_to") == "inaffected") {
     ScopedTimer s2("Optimizer Initialization (inaffected)");
     g2o::HyperDijkstra hypdij(optimizer_);
     g2o::UniformCostFunction cost_function;
     g2o::VertexSE3* new_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[graph_.size()-1]->vertex_id_));
     hypdij.shortestPaths(new_vertex,&cost_function,4);
     g2o::HyperGraph::VertexSet& vs = hypdij.visited();
     optimizer_->initializeOptimization(vs);
   } 
   else {
      g2o::HyperGraph::EdgeSet edges;
      if (!ps->get<bool>("use_robot_odom_only")){
        edges.insert(cam_cam_edges_.begin(), cam_cam_edges_.end());
      }
      if (ps->get<bool>("use_robot_odom")){
          edges.insert(odometry_edges_.begin(), odometry_edges_.end());
      }
      optimizer_->initializeOptimization(edges);
    }

    {
      ScopedTimer s2("Optimizer Initialization");
      optimizer_->initializeOptimization(cam_cam_edges_);
    }

    ROS_WARN_NAMED("eval", "Optimization with %zu cams, %zu nodes and %zu edges in the graph", graph_.size(), optimizer_->vertices().size(), optimizer_->edges().size());
    Q_EMIT iamBusy(1, "Optimizing Graph", 0); 
    int currentIt = 0;
    //Optimize certain number of iterations
    if(stop_cond >= 1.0){ 
      do {
        currentIt += optimizer_->optimize(ceil(stop_cond / 10));//optimize in maximally 10 steps
      } while(ros::ok() && currentIt < stop_cond && currentIt > 0); //the latter avoids infinite looping if there's nothing to optimize
      optimizer_->computeActiveErrors();
      chi2 = optimizer_->chi2();
    } 
    //Optimize to convergence
    else { 
      double prev_chi2;
      do {
        prev_chi2 = chi2; //chi2 is numeric_limits::max() in first iteration
        currentIt += optimizer_->optimize(5);//optimize 5 iterations per step
        optimizer_->computeActiveErrors();
        chi2 = optimizer_->chi2();
      } while(chi2/prev_chi2 < (1.0 - stop_cond));//e.g.  999/1000 < (1.0 - 0.01) => 0.999 < 0.99
    }

    ROS_WARN_STREAM_NAMED("eval", "G2O Statistics: " << std::setprecision(15) << camera_vertices.size() 
                          << " cameras, " << optimizer_->edges().size() << " edges. " << chi2
                          << " ; chi2 "<< ", Iterations: " << currentIt);
    optimizer_mutex_.unlock();
    optimization_mutex_.unlock();
  }

  Q_EMIT progress(1, "Optimizing Graph", 1); 

  //ROS_INFO("Finished G2O optimization after %d iterations", i);
  //optimizer_->save((bagfile_name + "_g2o-optimizer-save-file-after").c_str());

  ROS_INFO("A: last cam: %i", last_added_cam_vertex_id());

  QMutexLocker locker(&optimizer_mutex_);
  if (ps->get<std::string>("pose_relative_to") == "inaffected") {
    optimizer_->setFixed(camera_vertices, true);
  }
  else {
    optimizer_->setFixed(camera_vertices, false);
  }
  g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(last_added_cam_vertex_id()));
  //ROS_INFO("Sending Transform for Vertex ID: %d", new_node
  computed_motion_ =  eigenTransf2TF(v->estimate());
  Node* newest_node = graph_[graph_.size()-1];
  latest_transform_cache_ = stampedTransformInWorldFrame(newest_node, computed_motion_);
  //printTransform("Computed final transform", latest_transform_cache_);
  broadcastTransform(latest_transform_cache_);
  if(ps->get<bool>("octomap_online_creation")) { 
    if(updateCloudOrigin(newest_node)){
      renderToOctomap(newest_node);
      //v->setFixed(true);
    }
  }

  current_match_edges_.clear();
  //ROS_WARN("GM: 1198: no graph edges in visualzation"  );
  Q_EMIT setGraphEdges(getGraphEdges());
  Q_EMIT updateTransforms(getAllPosesAsMatrixList());

  ROS_WARN_STREAM_NAMED("eval", "Optimizer Runtime; "<< s.elapsed() <<" s");
#ifdef DO_LOOP_CLOSING
  loopClosingTest();

  // createSearchTree();
  // std::vector<std::pair<int,float> > neighbours;
  // getNeighbours(graph_.begin()->first, 1, neighbours);
#endif
  Q_EMIT setGraph(optimizer_);
  return chi2; 
}

bool containsVertex(g2o::HyperGraph::Edge* myedge, g2o::HyperGraph::Vertex* v_to_del)
{
    std::vector<g2o::HyperGraph::Vertex*>& myvertices = myedge->vertices();
    g2o::VertexSE3 *v1, *v2; //used in loop as temporaries
    v1 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(1));
    v2 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(0));
    return (v1->id() == v_to_del->id() || v2->id() == v_to_del->id());
}

void GraphManager::deleteCameraFrame(int id)
{
    QMutexLocker locker(&optimizer_mutex_);
    QMutexLocker locker2(&optimization_mutex_);

    g2o::VertexSE3* v_to_del = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[id]->vertex_id_));//last vertex

#ifdef DO_FEATURE_OPTIMIZATION
    //Erase edges from cam_cam edge set
    EdgeSet::iterator edge_iter = cam_cam_edges_.begin();
    for(;edge_iter != cam_cam_edges_.end(); edge_iter++) {
        if(containsVertex(*edge_iter, v_to_del))
          cam_cam_edges_.erase(edge_iter);
    }
    //Erase edges from cam_lm edge set
    edge_iter = cam_lm_edges.begin();
    for(;edge_iter != cam_lm_edges.end(); edge_iter++) {
        if(containsVertex(*edge_iter, v_to_del))
          cam_cam_edges_.erase(edge_iter);
    }
#endif

    optimizer_->removeVertex(v_to_del); //This takes care of removing all edges too
    camera_vertices.erase(v_to_del);
    graph_.erase(id);
}

///Actually only discount them drastically. 
///Thresh corresponds to a squared error
unsigned int GraphManager::pruneEdgesWithErrorAbove(float thresh){
    QMutexLocker locker(&optimizer_mutex_);
    QMutexLocker locker2(&optimization_mutex_);

    optimizer_->computeActiveErrors();
    unsigned int counter = 0;

#ifdef DO_FEATURE_OPTIMIZATION
    //remove feature edges
    if (ParameterServer::instance()->get<bool>("optimize_landmarks")){
      EdgeSet::iterator edge_iter = cam_lm_edges.begin();
      for(;edge_iter != cam_lm_edges.end(); edge_iter++) 
      {
          g2o::EdgeSE3PointXYZDepth* myedge = dynamic_cast<g2o::EdgeSE3PointXYZDepth*>(*edge_iter);
          Eigen::Vector3d ev = myedge->error();
          if(ev.squaredNorm() > thresh){
            optimizer_->removeEdge(myedge); 
            cam_lm_edges.erase(edge_iter);
          }
      }
    }
#endif

    //This block is only required for the ROS_INFO message
    g2o::VertexSE3 *v1, *v2; //used in loop
    std::map<int, int> vertex_id_to_node_id;
    for (graph_it it = graph_.begin(); it !=graph_.end(); ++it){
          Node *node = it->second;
          vertex_id_to_node_id[node->vertex_id_] = node->id_;
    }

    //Discount cam2cam edges
    g2o::HyperGraph::EdgeSet remaining_cam_cam_edges, edges_to_remove;
    EdgeSet::iterator edge_iter = cam_cam_edges_.begin();
    for(;edge_iter != cam_cam_edges_.end(); edge_iter++) {
        g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
        g2o::EdgeSE3::ErrorVector ev = myedge->error();

        //For Output Message:
        std::vector<g2o::HyperGraph::Vertex*>& myvertices = myedge->vertices();
        v1 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(1));
        v2 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(0));
        int n_id1 = vertex_id_to_node_id[v1->id()]; 
        int n_id2 = vertex_id_to_node_id[v2->id()];

        ROS_DEBUG("Mahalanobis Distance for edge from node %d to %d is %f", n_id1, n_id2, myedge->chi2());
        if(myedge->chi2() > thresh){
          counter++;

            //Constant position estimate 
            Eigen::Quaterniond eigen_quat(1,0,0,0);
            Eigen::Vector3d translation(0,0,0);
            g2o::SE3Quat unit_tf(eigen_quat, translation);
            myedge->setMeasurement(unit_tf);

            if(abs(n_id1 - n_id2) != 1)//Non-Consecutive
            { 
              //If not disconnecting a vertex, remove it completely
              if(v1->edges().size() > 1 && v2->edges().size() > 1){
                ROS_INFO("Removing edge from node %d to %d, because error is %f", n_id1, n_id2, ev.squaredNorm());
                edges_to_remove.insert(myedge);
              }
              else {
                ROS_WARN("Setting edge from node %d to %d to Identity and Information to diag(1e-100), because error is %f", n_id1, n_id2, ev.squaredNorm());
                //Set highly uncertain, so non-erroneous edges prevail
                Eigen::Matrix<double,6,6> new_info = Eigen::Matrix<double,6,6>::Identity()* 1e-100;
                new_info(3,3) = 1e-100;
                new_info(4,4) = 1e-100;
                new_info(5,5) = 1e-100;
                myedge->setInformation(new_info);
              }
            } else {
              //consecutive: zero motion assumption
              ROS_WARN("Setting edge from node %d to %d and Information to Identity because error is %f", n_id1, n_id2, ev.squaredNorm());
              Eigen::Matrix<double,6,6> new_info = Eigen::Matrix<double,6,6>::Identity();
              myedge->setInformation(new_info);
            }
          /*
          }
          else{
            //Remove erroneous loop closures competely
            optimizer_->removeEdge(myedge); 
            ROS_INFO("Removing edge from node %d to %d because error is %f", n_id1, n_id2, ev.squaredNorm());
            continue; //do not take into remaining edge set
          }
          */
        }
        /*
        else
        { //for subsequent nodes, check distance
          int n_id1 = vertex_id_to_node_id[v1->id()]; 
          int n_id2 = vertex_id_to_node_id[v2->id()];
          if(abs(n_id1 - n_id2) == 1){ //predecessor-successor
            ros::Duration delta_time = graph_[n_id2]->header_.stamp - graph_[n_id1]->header_.stamp;
            if(!isSmallTrafo(myedge->measurement(), delta_time.toSec()))
            { 
              ROS_INFO("Setting edge from node %d to %d to Identity because translation is too large", n_id1, n_id2);
              //Constant position estimate
              Eigen::Quaterniond eigen_quat(1,0,0,0);
              Eigen::Vector3d translation(0,0,0);
              g2o::SE3Quat unit_tf(eigen_quat, translation);
              myedge->setMeasurement(unit_tf);
              //Set highly uncertain, so non-erroneous edges prevail
              Eigen::Matrix<double,6,6> new_info = Eigen::Matrix<double,6,6>::Identity()* 1e-2;
              new_info(3,3) = 1e-4;
              new_info(4,4) = 1e-4;
              new_info(5,5) = 1e-4;
              myedge->setInformation(new_info);
            }
          }
        }*/
        //remaining_cam_cam_edges.insert(*edge_iter);
    }
    EdgeSet::iterator remove_edge_iter = edges_to_remove.begin();
    for(;remove_edge_iter!= edges_to_remove.end();remove_edge_iter++) {
      cam_cam_edges_.erase(*remove_edge_iter);
    }
    //Now cut, without making vertices disconnected
    /*
    for(g2o::SparseOptimizer::VertexIDMap::iterator it = optimizer_->vertices().begin(); it != optimizer_->vertices().end(); it++)
    {
      //Go through all edges of vertex
      g2o::HyperGraph::EdgeSet es = it->second->edges();
      EdgeSet::iterator edge_iter = es.begin(); 
      unsigned int nonconsec_edges = 0;
      bool edge_to_prev = false, edge_to_next = false;
      for(;edge_iter != es.end(); ++edge_iter) {
        g2o::EdgeSE3* curr_edge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
        g2o::EdgeSE3::ErrorVector ev = curr_edge->error();

        if(curr_edge->chi2() > thresh){
          optimizer_->removeEdge(curr_edge); 
          continue;
        } 
      }
    }
    */
    //cam_cam_edges_.swap(remaining_cam_cam_edges_);
    //Q_EMIT setGraphEdges(getGraphEdges());
    return counter;
}

QList<QPair<int, int> >* GraphManager::getGraphEdges()
{
    ScopedTimer s(__FUNCTION__);
    //QList<QPair<int, int> >* edge_list = new QList<QPair<int, int> >();
    g2o::VertexSE3 *v1, *v2; //used in loop
    std::map<int, int> vertex_id_to_node_id;
    for (graph_it it = graph_.begin(); it !=graph_.end(); ++it){
          Node *node = it->second;
          vertex_id_to_node_id[node->vertex_id_] = node->id_;
    }
    QList<QPair<int, int> >* current_edges = new QList<QPair<int, int> >();
    EdgeSet odom_and_feature_edges(odometry_edges_);
    odometry_edges_.insert(cam_cam_edges_.begin(), cam_cam_edges_.end());
    EdgeSet::iterator edge_iter = odom_and_feature_edges.begin();
    for(;edge_iter != odom_and_feature_edges.end(); edge_iter++) {
        g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
        if(myedge){
          std::vector<g2o::HyperGraph::Vertex*>& myvertices = myedge->vertices();
          v1 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(1));
          v2 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(0));
        } else {
          ROS_WARN("Unexpected edge type in getGraphEdges()");
          continue;
        }

        if(vertex_id_to_node_id.find(v1->id()) == vertex_id_to_node_id.end()){
            ROS_WARN("Vertex ID %d does not match any Node ", v1->id());
        }
        if(vertex_id_to_node_id.find(v2->id()) == vertex_id_to_node_id.end()){
            ROS_WARN("Vertex ID %d does not match any Node ", v2->id());
        }
        std::map<int, int>::iterator node_id1_it = vertex_id_to_node_id.find(v1->id());
        if (node_id1_it == vertex_id_to_node_id.end()) continue;
        std::map<int, int>::iterator node_id2_it = vertex_id_to_node_id.find(v2->id());
        if (node_id2_it == vertex_id_to_node_id.end()) continue;
        current_edges->append( qMakePair(node_id1_it->second, node_id2_it->second));
    }
    return current_edges;
}

QList<QMatrix4x4>* GraphManager::getAllPosesAsMatrixList() const{
    ScopedTimer s(__FUNCTION__);
    //QList<QMatrix4x4> current_poses;
    ROS_DEBUG("Retrieving all transformations from optimizer");
    QList<QMatrix4x4>* current_poses = new QList<QMatrix4x4>();
    //current_poses.clear();
#if defined(QT_VERSION) && QT_VERSION >= 0x040700
    current_poses->reserve(camera_vertices.size()+10);//only allocates the internal pointer array. +10 for things like calibration vertices or whatever
#endif

    for (auto it = graph_.cbegin(); it !=graph_.cend(); ++it){
      const Node *node = it->second;
      g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex( node->vertex_id_));
      if(v){ 
        current_poses->push_back(eigenTF2QMatrix(v->estimate())); 
      } else {
        ROS_ERROR("Nullpointer in graph at position %i!", it->first);
      }
    }
    return current_poses;// new QList<QMatrix4x4>(current_poses); //pointer to a copy
}

void GraphManager::reducePointCloud(pointcloud_type const * pc) {
  double vfs = ParameterServer::instance()->get<double>("voxelfilter_size");
  BOOST_REVERSE_FOREACH(GraphNodeType entry, graph_){
    if(entry.second->pc_col.get() == pc){
      entry.second->reducePointCloud(vfs);
      ROS_INFO("Reduced PointCloud after rendering to openGL list.");
      return;
    }
  }
}

void GraphManager::printEdgeErrors(QString filename){
  QMutexLocker locker(&optimizer_mutex_);
  std::fstream filestr;
  filestr.open (qPrintable(filename),  std::fstream::out );

 EdgeSet::iterator edge_iter = cam_cam_edges_.begin();
 for(int i =0;edge_iter != cam_cam_edges_.end(); edge_iter++, i++) {
      g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
      g2o::EdgeSE3::ErrorVector ev = myedge->error();
      ROS_INFO_STREAM("Error Norm for edge " << i << ": " << ev.squaredNorm());
      filestr << "Error for edge " << i << ": " << ev.squaredNorm() << std::endl;
  }
  filestr.close();
}

double GraphManager::geodesicDiscount(g2o::HyperDijkstra& hypdij, const MatchingResult& mr){
    //Discount by geodesic distance to root node
    const g2o::HyperDijkstra::AdjacencyMap am = hypdij.adjacencyMap();

    g2o::VertexSE3* older_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex( nodeId2VertexId(mr.edge.id1) ));
    double discount_factor = am.at(older_vertex).distance();
    discount_factor = discount_factor > 0.0? 1.0/discount_factor : 1.0;//avoid inf
    ROS_INFO("Discount weight for connection to Node %i = %f", mr.edge.id1, discount_factor);
    return discount_factor;
}

void GraphManager::sanityCheck(float thresh){ 
  thresh *=thresh; //squaredNorm
  QMutexLocker locker(&optimizer_mutex_);
  QMutexLocker locker2(&optimization_mutex_);
  EdgeSet::iterator edge_iter = cam_cam_edges_.begin();
  for(int i =0;edge_iter != cam_cam_edges_.end(); edge_iter++, i++) {
    g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
    Eigen::Vector3d ev = myedge->measurement().translation();
    if(ev.squaredNorm() > thresh){
      //optimizer_->removeEdge(myedge); 
      myedge->setInformation(Eigen::Matrix<double,6,6>::Identity()* 0.000001);
    }
  }
}

void GraphManager::filterNodesByPosition(float x, float y, float z)
{
  ROS_WARN("filtering Nodes");
    Eigen::Vector3f center(x,y,z);
    float radius = 0.5;
    for (graph_it it = graph_.begin(); it !=graph_.end(); ++it){
          Node *node = it->second;
          Node* clone = node->copy_filtered(center, radius);
    }
}
void GraphManager::occupancyFilterClouds(){
  Q_EMIT iamBusy(0, "Filtering Clouds by Occupancy", graph_.size());
  BOOST_FOREACH(GraphNodeType entry, graph_){ 
    ROS_INFO("Filtering points of Node %d", entry.first);
    co_server_.occupancyFilter(entry.second->pc_col, entry.second->pc_col, ParameterServer::instance()->get<double>("occupancy_filter_threshold"));
    Q_EMIT progress(0, "Filtering  Clouds by Occupancy", entry.first);
  }
    Q_EMIT progress(0,  "Filtering  Clouds by Occupancy", 1e6);
  ROS_INFO("Done Filtering, display will not be updated!");
}

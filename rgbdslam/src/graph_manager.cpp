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
#include "g2o/core/optimization_algorithm_dogleg.h"
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
    current_backend_("none")
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
  whole_cloud_pub_ = nh.advertise<pointcloud_type>(ps->get<std::string>("aggregate_cloud_out_topic"),
                                                   ps->get<int>("publisher_queue_size"));
  ransac_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/rgbdslam/correspondence_marker", 
                                                                ps->get<int>("publisher_queue_size"));
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("/rgbdslam/pose_graph_markers",
                                                         ps->get<int>("publisher_queue_size"));
  computed_motion_ = tf::Transform::getIdentity();
  init_base_pose_  = tf::Transform::getIdentity();
  base2points_     = tf::Transform::getIdentity();
  //timer_ = nh.createTimer(ros::Duration(0.1), &GraphManager::broadcastTransform, this);
}



void GraphManager::createOptimizer(std::string backend, g2o::SparseOptimizer* optimizer)
{
  QMutexLocker locker(&optimizer_mutex);
  QMutexLocker locker2(&optimization_mutex);
  // allocating the optimizer
  if(optimizer == NULL){
    delete optimizer_; 
    optimizer_ = new g2o::SparseOptimizer();
    optimizer_->setVerbose(true);
  } else if (optimizer_ != optimizer){
    delete optimizer_; 
    optimizer_ = new g2o::SparseOptimizer();
    optimizer_->setVerbose(true);
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
    g2o::OptimizationAlgorithmDogleg * algo = new g2o::OptimizationAlgorithmDogleg(solver);
    optimizer_->setAlgorithm(algo);
  }



 optimizer_->setVerbose(true);
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


      //Geodesic Neighbours except sequential
      std::map<int,int> neighbour_indices; //maps neighbour ids to their weights in sampling
      int sum_of_weights=0;
      for (g2o::HyperGraph::VertexSet::iterator vit=vs.begin(); vit!=vs.end(); vit++) { //FIXME: Mix of vertex id and graph node (with features) id
        int id = (*vit)->id();
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
        if(ids_to_link_to.contains(*it) == 0){
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
     cam_cam_edges.clear();
    marker_id_ =0;
    ParameterServer* ps = ParameterServer::instance();
    createOptimizer(ps->get<std::string>("backend_solver"));

    //Q_FOREACH(Node* node, graph_) { delete node; }
    BOOST_FOREACH(GraphNodeType entry, graph_){ delete entry.second; }
    //for(unsigned int i = 0; i < graph_.size(); delete graph_[i++]);//No body
    graph_.clear();
    keyframe_ids_.clear();
    Q_EMIT resetGLViewer();
    curr_best_result_ = MatchingResult();
    current_poses_.clear();
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
    init_base_pose_ =  new_node->getGroundTruthTransform();//identity if no MoCap available
    printTransform("Ground Truth Transform for First Node", init_base_pose_);
    //new_node->buildFlannIndex(); // create index so that next nodes can use it
    g2o::VertexSE3* reference_pose = new g2o::VertexSE3;

    new_node->vertex_id_ = next_vertex_id++;
    graph_[new_node->id_] = new_node;
    reference_pose->setId(new_node->vertex_id_);

    camera_vertices.insert(reference_pose);

    ROS_WARN("ADDING NODE ZERO with id %i and seq %i, v_id: %i", new_node->id_, new_node->seq_id_, new_node->vertex_id_);
    g2o::SE3Quat g2o_ref_se3 = tf2G2O(init_base_pose_);
    reference_pose->setEstimate(g2o_ref_se3);
    reference_pose->setFixed(true);//fix at origin
    optimizer_mutex.lock();
    optimizer_->addVertex(reference_pose); 
    optimizer_mutex.unlock();
    QString message;
    Q_EMIT setGUIInfo(message.sprintf("Added first node with %i keypoints to the graph", (int)new_node->feature_locations_2d_.size()));
    //pointcloud_type::Ptr the_pc(new_node->pc_col); //this would delete the cloud after the_pc gets out of scope
    QMatrix4x4 latest_transform = g2o2QMatrix(g2o_ref_se3);
    printQMatrix4x4("Latest Transform", latest_transform);
    Q_EMIT setPointCloud(new_node->pc_col.get(), latest_transform);
    Q_EMIT setFeatures(&(new_node->feature_locations_3d_));
    current_poses_.append(latest_transform);
    ROS_DEBUG("GraphManager is thread %d, New Node is at (%p, %p)", (unsigned int)QThread::currentThreadId(), new_node, graph_[0]);
    keyframe_ids_.push_back(new_node->id_);

    process_node_runs_ = false;
}

/*/Translational norm
inline float sqrTransNorm(const Eigen::Matrix4f& t){
    return t(0,3)*t(0,3)+t(1,3)*t(1,3)+t(2,3)*t(2,3);
}
*/

// returns true, iff node could be added to the cloud
bool GraphManager::addNode(Node* new_node) 
{
    /// \callergraph
    ScopedTimer s(__FUNCTION__);
    process_node_runs_ = true;

    QMatrix4x4 curr_motion_estimate;///contains the best-yet of the pairwise motion estimates for the current node
    if(reset_request_) resetGraph(); 
    ParameterServer* ps = ParameterServer::instance();
    if ((int)new_node->feature_locations_2d_.size() < ps->get<int>("min_matches") && 
        ! ps->get<bool>("keep_all_nodes"))
    {
        ROS_DEBUG("found only %i features on image, node is not included",(int)new_node->feature_locations_2d_.size());
        process_node_runs_ = false;
        return false;
    }

    //set the node id only if the node is actually added to the graph
    //needs to be done here as the graph size can change inside this function
    new_node->id_ = graph_.size();
    new_node->seq_id_ = next_seq_id++; // allways incremented, even if node is not added

    //First Node, so only build its index, insert into storage and add a
    //vertex at the origin, of which the position is very certain
    if (graph_.size()==0){
        firstNode(new_node);
        return true;
    }

    earliest_loop_closure_node_ = new_node->id_;
    unsigned int num_edges_before = cam_cam_edges.size();
    bool edge_to_keyframe = false;

    ROS_DEBUG("Graphsize: %d Nodes", (int) graph_.size());
    marker_id_ = 0; //overdraw old markers

    //Odometry Stuff
    int sequentially_previous_id = graph_.rbegin()->second->id_; 
    tf::StampedTransform odom_tf_new = new_node->getOdomTransform();
    tf::StampedTransform odom_tf_old = graph_[sequentially_previous_id]->getOdomTransform();
    tf::Transform odom_delta_tf = odom_tf_new * odom_tf_old.inverse();
    printTransform("Odometry Delta", odom_delta_tf);
    if(odom_tf_old.frame_id_ == "missing_odometry" || odom_tf_new.frame_id_ == "missing_odometry"){
      ROS_INFO("No Valid Odometry, using identity");
      odom_delta_tf = tf::Transform::getIdentity();
    }
    //int best_match_candidate_id = sequentially_previous_id; 
    MatchingResult mr;
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
      if(mr.edge.id1 > 0 && mr.edge.id2 > 0) {//Found trafo
        ros::Time time1 = prev_frame->pc_col->header.stamp;
        ros::Time time2 = new_node->pc_col->header.stamp;
        ros::Duration delta_time =  time2 - time1;
        if(!isBigTrafo(mr.edge.mean) || !isSmallTrafo(mr.edge.mean, delta_time.toSec())){ //Found trafo, but bad trafo (too small to big)
            ROS_WARN("Transformation not within bounds. Did not add as Node");
            //Send the current pose via tf nevertheless
            tf::Transform incremental = g2o2TF(mr.edge.mean);
            g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[prev_frame->id_]->vertex_id_));
            tf::Transform previous = g2o2TF(v->estimateAsSE3Quat());
            tf::Transform combined = previous*incremental;
            broadcastTransform(new_node, combined);
            process_node_runs_ = false;
            curr_best_result_ = mr;
            return false;
        } else { //Good Transformation
          ROS_INFO_STREAM("Information Matrix for Edge (" << mr.edge.id1 << "<->" << mr.edge.id2 << ") \n" << mr.edge.informationMatrix);
          if (addEdgeToG2O(mr.edge, prev_frame, new_node,  true, true, curr_motion_estimate)) 
          {
            if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
#ifdef DO_FEATURE_OPTIMIZATION
            updateLandmarks(mr, prev_frame,new_node);
#endif
            graph_[mr.edge.id1]->valid_tf_estimate_ = true;
            ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
            curr_best_result_ = mr;
            //addOutliers(Node* new_node, mr.inlier_matches);

          } else {
            process_node_runs_ = false;
            return false;
          }
        }
        predecessor_matched = true;
      }
      else {
      // No transformation to predecessor. No other choice than try other candidates. This is done below 
      }
    }//end: Initial Comparison ######################################################################

    //Eigen::Matrix4f ransac_trafo, final_trafo;
    QList<int> vertices_to_comp;
    int  seq_cand = localization_only_ ? 0 : ps->get<int>("predecessor_candidates");
    int geod_cand = ps->get<int>("neighbor_candidates");
    int samp_cand = ps->get<int>("min_sampled_candidates");
    if(predecessor_matched)
      vertices_to_comp = getPotentialEdgeTargetsWithDijkstra(new_node, seq_cand, geod_cand, samp_cand, curr_best_result_.edge.id1); 
    else
      vertices_to_comp = getPotentialEdgeTargetsWithDijkstra(new_node, seq_cand, geod_cand, samp_cand, sequentially_previous_id, true); 

    QList<const Node* > nodes_to_comp;//only necessary for parallel computation

    //MAIN LOOP: Compare node pairs ######################################################################
    if (ps->get<bool>("concurrent_edge_construction")) 
    {
        for (int id_of_id = (int) vertices_to_comp.size() - 1; id_of_id >= 0; id_of_id--) {
            //First compile a qlist of the nodes to be compared, then run the comparisons in parallel,
            //collecting a qlist of the results (using the blocking version of mapped).
            nodes_to_comp.push_front(graph_[vertices_to_comp[id_of_id]]); 
            ROS_INFO("Nodes to compare: %i", vertices_to_comp[id_of_id]);
        }
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
            if (mr.edge.id1 >= 0 ) {
              //mr.edge.informationMatrix *= geodesicDiscount(hypdij, mr);
              //ROS_INFO_STREAM("XY Information Matrix for Edge (" << mr.edge.id1 << "<->" << mr.edge.id2 << ") \n" << mr.edge.informationMatrix);

              ROS_INFO("new node has id %i", new_node->id_);
              assert(graph_[mr.edge.id1]);

              ros::Duration delta_time = new_node->pc_col->header.stamp - graph_[mr.edge.id1]->pc_col->header.stamp;
              if (isSmallTrafo(mr.edge.mean, delta_time.toSec()) &&
                  addEdgeToG2O(mr.edge,graph_[mr.edge.id1],new_node, isBigTrafo(mr.edge.mean), mr.inlier_matches.size() > curr_best_result_.inlier_matches.size(), curr_motion_estimate))
                { 
#ifdef DO_FEATURE_OPTIMIZATION
                  updateLandmarks(mr, graph_[mr.edge.id1],new_node);
#endif
                  graph_[mr.edge.id1]->valid_tf_estimate_ = true;
                  ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
                  if (mr.inlier_matches.size() > curr_best_result_.inlier_matches.size()) {
                  //if (curr_best_result_.edge.id1 == -1 || sqrTransNorm(mr.final_trafo) < sqrTransNorm(curr_best_result_.final_trafo)) {
                    curr_best_result_ = mr;
                  }
                  if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
                }
            }
        }
    } else { //Nonconcurrent
        for (int id_of_id = (int) vertices_to_comp.size() - 1; id_of_id >= 0; id_of_id--) {
            Node* node_to_compare = graph_[vertices_to_comp[id_of_id]];
            ROS_INFO("Comparing new node (%i) with node %i / %i", new_node->id_, vertices_to_comp[id_of_id], node_to_compare->id_);
            MatchingResult mr = new_node->matchNodePair(node_to_compare);

            if (mr.edge.id1 >= 0) {
                //mr.edge.informationMatrix *= geodesicDiscount(hypdij, mr);
              //ROS_INFO_STREAM("XX Information Matrix for Edge (" << mr.edge.id1 << "<->" << mr.edge.id2 << "\n" << mr.edge.informationMatrix);

              ros::Duration delta_time = new_node->pc_col->header.stamp - graph_[mr.edge.id1]->pc_col->header.stamp;
              if (isSmallTrafo(mr.edge.mean, delta_time.toSec()) &&
                  addEdgeToG2O(mr.edge, node_to_compare, new_node, isBigTrafo(mr.edge.mean), mr.inlier_matches.size() > curr_best_result_.inlier_matches.size(), curr_motion_estimate))
              {
#ifdef DO_FEATURE_OPTIMIZATION
                updateLandmarks(mr, node_to_compare, new_node);
#endif
                graph_[mr.edge.id1]->valid_tf_estimate_ = true;
                ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
                if (mr.inlier_matches.size() > curr_best_result_.inlier_matches.size()) {
                  //if (curr_best_result_.edge.id1 == -1 || sqrTransNorm(mr.final_trafo) < sqrTransNorm(curr_best_result_.final_trafo)) {
                  curr_best_result_ = mr;
                }
                if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
              }
            }
        }
    }

    //END OF MAIN LOOP: Compare node pairs ######################################################################
    bool found_trafo = (cam_cam_edges.size() != num_edges_before);
    bool invalid_odometry = ps->get<std::string>("odom_frame_name").empty() || 
                            odom_tf_old.frame_id_ == "missing_odometry" || 
                            odom_tf_new.frame_id_ == "missing_odometry"; 

    bool keep_anyway = (ps->get<bool>("keep_all_nodes") || 
                        (((int)new_node->feature_locations_3d_.size() > ps->get<int>("min_keypoints")) 
                         && ps->get<bool>("keep_good_nodes")));
    if(!invalid_odometry)
    {
      ROS_INFO("Adding odometry motion edge for Node %i (if available, otherwise using identity)", (int)graph_.rbegin()->second->id_);
      LoadedEdge3D odom_edge;
      odom_edge.id1 = sequentially_previous_id;
      odom_edge.id2 = new_node->id_;
      odom_edge.mean = tf2G2O(odom_delta_tf);

      //Real odometry
      //FIXME get odometry information matrix and transform it to the optical frame
      odom_edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity(); //0.1m /0.1rad error
      /*
      //Assumption: Motion in the plane
      odom_edge.informationMatrix(0,0) = 100; //0.1m accuracy in the floor plane
      odom_edge.informationMatrix(1,1) = 100; //
      odom_edge.informationMatrix(2,2) = 1000000;//0.01m information on rotation w.r. to floor
      odom_edge.informationMatrix(3,3) = 400000000;//0.02rad information on rotation w.r. to floor
      odom_edge.informationMatrix(4,4) = 400000000;//0.02rad information on rotation w.r. to floor
      odom_edge.informationMatrix(5,5) = 1600; //0.4rad (~20°) on rotation about vertical
      */
      addEdgeToG2O(odom_edge,graph_[sequentially_previous_id],new_node, true,true, curr_motion_estimate);
    }
    else if(!found_trafo && keep_anyway) //Constant position assumption
    { 
      LoadedEdge3D odom_edge;

      Eigen::Quaterniond eigen_quat(1,0,0,0);
      Eigen::Vector3d translation(0,0,0);
      odom_edge.id1 = sequentially_previous_id;
      odom_edge.id2 = new_node->id_;
      odom_edge.mean = g2o::SE3Quat(eigen_quat, translation);
      curr_motion_estimate = g2o2QMatrix(odom_edge.mean);
      odom_edge.informationMatrix = Eigen::Matrix<double,6,6>::Zero(); 
      ///High information value for translation 
      ///10000 corresponds to 1cm std deviation, i.e. we expect the camera to travel about 1cm in any direction (with mean 0)
      ///FIXME this should have a dependency on time.
      ROS_INFO_STREAM("No valid transformation: Using constant position assumption.");
      odom_edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity() * 1;//e-9; 
      odom_edge.informationMatrix(3,3) = 1e-100;
      odom_edge.informationMatrix(4,4) = 1e-100;
      odom_edge.informationMatrix(5,5) = 1e-100;
      addEdgeToG2O(odom_edge,graph_[sequentially_previous_id],new_node, true,true, curr_motion_estimate);
      new_node->valid_tf_estimate_ = false; //Don't use for postprocessing, rendering etc
      //new_node->clearPointCloud();

      //Used (only?) for visualization
      MatchingResult mr;
      mr.edge = odom_edge;
      curr_best_result_ = mr;
    }
     /* 
    { 
      LoadedEdge3D const_pos_edge;
      const_pos_edge.id1 = sequentially_previous_id;
      const_pos_edge.id2 = new_node->id_;

      Eigen::Quaterniond eigen_quat(1,0,0,0);
      Eigen::Vector3d translation(0,0,0);
      g2o::SE3Quat unit_tf(eigen_quat, translation);

      const_pos_edge.mean = unit_tf;
      const_pos_edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity() * 1e-9; 
      //Rotation should be less than translation, so translation is not changed to have constant orientation, which gives bad effects
      const_pos_edge.informationMatrix(3,3) = 1e-50;
      const_pos_edge.informationMatrix(4,4) = 1e-50;
      const_pos_edge.informationMatrix(5,5) = 1e-50;
      QMatrix4x4 tmp;
      addEdgeToG2O(const_pos_edge, graph_[sequentially_previous_id], new_node, true, false, tmp);
      new_node->valid_tf_estimate_ = false; //Don't use for postprocessing, rendering etc
    }
    */


 if (cam_cam_edges.size() > num_edges_before) 
 { //Success
   //First render the cloud with the best frame-to-frame estimate
   //The transform will get updated when optimizeGraph finishes
   pointcloud_type* cloud_to_visualize = new_node->pc_col.get();
   if(!found_trafo) cloud_to_visualize = new pointcloud_type();
   Q_EMIT setPointCloud(cloud_to_visualize, curr_motion_estimate);
   Q_EMIT setFeatures(&(new_node->feature_locations_3d_));

   if(localization_only_)
   {
     optimizeGraph();
     g2o::VertexSE3* new_v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[new_node->id_]->vertex_id_));
     QMutexLocker locker(&optimizer_mutex);
     QMutexLocker locker2(&optimization_mutex);
     optimizer_->removeVertex(new_v); //Also removes the edges
   }
   else //localization_only_ == false
   {
     graph_[new_node->id_] = new_node; //Node->id_ == Graph_ Index
     ROS_INFO("Adding node with id %i and seq id %i to the graph", new_node->id_, new_node->seq_id_);
     if(!edge_to_keyframe) {
       std::stringstream ss;
       ss << "Keyframes: ";
       BOOST_FOREACH(int id, keyframe_ids_){ ss << id << ", "; }
       ROS_INFO("%s", ss.str().c_str());
       keyframe_ids_.push_back(new_node->id_-1); //use the one before, because that one is still localized w.r.t. a keyframe
     }
     ROS_INFO("Added Node, new graphsize: %i nodes", (int) graph_.size());
     if(ps->get<int>("optimizer_skip_step") > 0 && 
        (camera_vertices.size() % ps->get<int>("optimizer_skip_step")) == 0)
     { 
       optimizeGraph();
     } else {
       //QList<QPair<int, int> >* edgelist = new QList<QPair<int, int> >(current_edges_);
       //edgelist->append( qMakePair(curr_best_result_.edge.id2, curr_best_result_.edge.id1));
       //Q_EMIT setGraphEdges(&current_edges_);
     }
     //make the transform of the last node known
     //computed_motion_ is set by optimizeGraph() 
     //FIXME: it probably hasn't finished yet when concurrent, e.g. initialize with motion_estimate before
     broadcastTransform(new_node, computed_motion_);

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
       return this->addNode(new_node);
     }
   } else { //delete new_node; //is now  done by auto_ptr
     ROS_WARN("Did not add as Node");
   }
 }

 //Info output
 QString message;
 Q_EMIT setGUIInfo(message.sprintf("%s, Camera Pose Graph Size: %iN/%iE, Duration: %f, Inliers: %i, &chi;<sup>2</sup>: %f", 
       (cam_cam_edges.size() > num_edges_before) ? "Added" : "Ignored",
       (int)camera_vertices.size(), (int)cam_cam_edges.size(), s.elapsed(), (int)curr_best_result_.inlier_matches.size(), optimizer_->chi2()));
 process_node_runs_ = false;
 ROS_INFO("%s", qPrintable(message));
 return (cam_cam_edges.size() > num_edges_before);
}


bool GraphManager::addEdgeToG2O(const LoadedEdge3D& edge,Node* n1, Node* n2,  bool largeEdge, bool set_estimate, QMatrix4x4& motion_estimate) {
    ScopedTimer s(__FUNCTION__);
    assert(n1);
    assert(n2);
    assert(n1->id_ == edge.id1);
    assert(n2->id_ == edge.id2);

    QMutexLocker locker(&optimizer_mutex);
    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(n1->vertex_id_));
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(n2->vertex_id_));

    // at least one vertex has to be created, assert that the transformation
    // is large enough to avoid to many vertices on the same spot
    if (!v1 || !v2){
        if (!largeEdge) {
            ROS_INFO("Edge to new vertex is to short, vertex will not be inserted");
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
        v1->setEstimate(v2->estimateAsSE3Quat() * edge.mean.inverse());
        camera_vertices.insert(v1);
        optimizer_->addVertex(v1); 
        motion_estimate = g2o2QMatrix(v1->estimateAsSE3Quat()); 
        ROS_WARN("Creating previous id. This is unexpected by the programmer");
    }
    else if (!v2 && v1) {
        v2 = new g2o::VertexSE3;
        assert(v2);
        int v_id = next_vertex_id++;
        v2->setId(v_id);
        n2->vertex_id_ = v_id;
        v2->setEstimate(v1->estimateAsSE3Quat() * edge.mean);
        camera_vertices.insert(v2);
        optimizer_->addVertex(v2); 
        motion_estimate = g2o2QMatrix(v2->estimateAsSE3Quat()); 
    }
    else if(set_estimate){
        v2->setEstimate(v1->estimateAsSE3Quat() * edge.mean);
        motion_estimate = g2o2QMatrix(v2->estimateAsSE3Quat()); 
    }
    g2o::EdgeSE3* g2o_edge = new g2o::EdgeSE3;
    g2o_edge->vertices()[0] = v1;
    g2o_edge->vertices()[1] = v2;
    g2o::SE3Quat meancopy(edge.mean); 
    g2o_edge->setMeasurement(meancopy);
    //Change setting from which mahal distance the robust kernel is used: robust_kernel_.setDelta(1.0);
    g2o_edge->setRobustKernel(&robust_kernel_);
    // g2o_edge->setInverseMeasurement(edge.mean.inverse());
    g2o_edge->setInformation(edge.informationMatrix);
    optimizer_->addEdge(g2o_edge);
    ROS_INFO_STREAM("Added Edge ("<< edge.id1 << "-" << edge.id2 << ") to Optimizer:\n" << edge.mean.to_homogeneous_matrix() << "\nInformation Matrix" << edge.informationMatrix);
    cam_cam_edges.insert(g2o_edge);

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

double GraphManager::optimizeGraph(double break_criterion, bool nonthreaded, QString filebasename){
  if(ParameterServer::instance()->get<bool>("concurrent_optimization") && !nonthreaded) {
    ROS_DEBUG("Optimization done in Thread");
    QtConcurrent::run(this, &GraphManager::optimizeGraphImpl, break_criterion); 
    return -1.0;
  }
  else { //Non-concurrent
    return optimizeGraphImpl(break_criterion);//regular function call
  }
}

double GraphManager::optimizeGraphImpl(double break_criterion)
{
  ScopedTimer s(__FUNCTION__);
  double stop_cond = break_criterion > 0.0 ? break_criterion : ParameterServer::instance()->get<double>("optimizer_iterations");
  ROS_WARN_NAMED("eval", "Loop Closures: %u, Sequential Edges: %u", loop_closures_edges, sequential_edges);
  ROS_WARN("Starting Optimization");
  double chi2 = std::numeric_limits<double>::max();
  if(!optimization_mutex.tryLock(2/*milliseconds*/))
  {
    ROS_INFO("Attempted Graph Optimization, but it is already running. Skipping.");
    return -1.0;
  }
  else //Got the lock
  {
    optimizer_mutex.lock();

    //Fixation strategies
    if (ParameterServer::instance()->get<std::string>("pose_relative_to") == "previous" && graph_.size() > 2) {
      optimizer_->setFixed(camera_vertices, false);
      optimizer_->vertex(graph_[graph_.size() - 2]->vertex_id_)->setFixed(true);
    }
    else if (ParameterServer::instance()->get<std::string>("pose_relative_to") == "largest_loop"){
      for(int i = 0; i < (int)graph_.size(); i++){
        //Don't optimize for nodes before the earliest matched one
        bool is_outside_largest_loop =  i < earliest_loop_closure_node_ ;
        optimizer_->vertex(graph_[i]->vertex_id_)->setFixed(is_outside_largest_loop);
      }
    }
    else if (ParameterServer::instance()->get<std::string>("pose_relative_to") == "first") {
      optimizer_->vertex(graph_[0]->vertex_id_)->setFixed(true);
    }

   //std::string bagfile_name = ParameterServer::instance()->get<std::string>("bagfile_name");
   //optimizer_->save((bagfile_name + "_g2o-optimizer-save-file-before").c_str());
   //optimizer_->save("before.g2o");
#ifdef DO_FEATURE_OPTIMIZATION
   printLandmarkStatistic();
   if (ParameterServer::instance()->get<bool>("optimize_landmarks")){
     updateProjectionEdges();
     optimizer_->initializeOptimization(cam_lm_edges);
   }else
     optimizer_->initializeOptimization(cam_cam_edges);
#else
   optimizer_->initializeOptimization();
#endif
    //optimizer_mutex.unlock(); //optimizer does not require the graph data structure for optimization after initialization

    ROS_WARN("Optimization with %zu cams, %zu nodes and %zu edges in the graph", graph_.size(), optimizer_->vertices().size(), optimizer_->edges().size());
    Q_EMIT iamBusy(1, "Optimizing Graph", 0); 
    int currentIt = 0;
    //Optimize certain number of iterations
    if(stop_cond >= 1.0){ 
      do {
        currentIt += optimizer_->optimize(ceil(stop_cond / 10));//optimize in maximally 10 steps
      } while(currentIt < stop_cond);
      //optimizer_mutex.lock();
      optimizer_->computeActiveErrors();
      chi2 = optimizer_->chi2();
    } 
    //Optimize to convergence
    else { 
      double prev_chi2;
      do {
        prev_chi2 = chi2; //chi2 is numeric_limits::max() in first iteration
        currentIt += optimizer_->optimize(5);//optimize 10 iterations per step
        optimizer_->computeActiveErrors();
        chi2 = optimizer_->chi2();
      } while(chi2/prev_chi2 < (1.0 - stop_cond));//e.g.  999/1000 < (1.0 - 0.01) => 0.999 < 0.99
    }

    ROS_WARN_STREAM_NAMED("eval", "G2O Statistics: " << std::setprecision(15) << camera_vertices.size() 
                          << " cameras, " << cam_cam_edges.size() << " edges. " << chi2
                          << " ; chi2 "<< ", Iterations: " << currentIt);
    optimizer_mutex.unlock();
    optimization_mutex.unlock();
  }

  Q_EMIT progress(1, "Optimizing Graph", 1); 

  //ROS_INFO("Finished G2O optimization after %d iterations", i);
  //optimizer_->save((bagfile_name + "_g2o-optimizer-save-file-after").c_str());

  ROS_INFO("A: last cam: %i", last_added_cam_vertex_id());

  QMutexLocker locker(&optimizer_mutex);
  if (ParameterServer::instance()->get<std::string>("pose_relative_to") == "inaffected") {
    optimizer_->setFixed(camera_vertices, true);
  }
  else {
    optimizer_->setFixed(camera_vertices, false);
  }
  g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(last_added_cam_vertex_id()));

  computed_motion_ =  g2o2TF(v->estimateAsSE3Quat());
  ROS_WARN("GM: 1198: no graph edges in visualzation"  );
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
    QMutexLocker locker(&optimizer_mutex);
    QMutexLocker locker2(&optimization_mutex);

    g2o::VertexSE3* v_to_del = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[id]->vertex_id_));//last vertex

#ifdef DO_FEATURE_OPTIMIZATION
    //Erase edges from cam_cam edge set
    EdgeSet::iterator edge_iter = cam_cam_edges.begin();
    for(;edge_iter != cam_cam_edges.end(); edge_iter++) {
        if(containsVertex(*edge_iter, v_to_del))
          cam_cam_edges.erase(edge_iter);
    }
    //Erase edges from cam_lm edge set
    edge_iter = cam_lm_edges.begin();
    for(;edge_iter != cam_lm_edges.end(); edge_iter++) {
        if(containsVertex(*edge_iter, v_to_del))
          cam_cam_edges.erase(edge_iter);
    }
#endif

    optimizer_->removeVertex(v_to_del); //This takes care of removing all edges too
    camera_vertices.erase(v_to_del);
    graph_.erase(id);
}

///Actually only discount them drastically. 
///Thresh corresponds to a squared error
unsigned int GraphManager::pruneEdgesWithErrorAbove(float thresh){
    QMutexLocker locker(&optimizer_mutex);
    QMutexLocker locker2(&optimization_mutex);

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
    g2o::HyperGraph::EdgeSet remaining_cam_cam_edges;
    EdgeSet::iterator edge_iter = cam_cam_edges.begin();
    for(;edge_iter != cam_cam_edges.end(); edge_iter++) {
        g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
        g2o::EdgeSE3::ErrorVector ev = myedge->error();

        //For Output Message:
        std::vector<g2o::HyperGraph::Vertex*>& myvertices = myedge->vertices();
        v1 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(1));
        v2 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(0));
        int n_id1 = vertex_id_to_node_id[v1->id()]; 
        int n_id2 = vertex_id_to_node_id[v2->id()];

        ROS_INFO("Mahalanobis Distance for edge from node %d to %d is %f", n_id1, n_id2, myedge->chi2());
        if(myedge->chi2() > thresh){
          counter++;
          //if(abs(n_id1 - n_id2) == 1){ //predecessor-successor
            // Only for sequential camera edges
            ROS_INFO("Setting edge from node %d to %d to Identity because error is %f", n_id1, n_id2, ev.squaredNorm());
            //Constant position estimate
            Eigen::Quaterniond eigen_quat(1,0,0,0);
            Eigen::Vector3d translation(0,0,0);
            g2o::SE3Quat unit_tf(eigen_quat, translation);
            myedge->setMeasurement(unit_tf);
            //Set highly uncertain, so non-erroneous edges prevail
            Eigen::Matrix<double,6,6> new_info = Eigen::Matrix<double,6,6>::Identity()* 1e-100;
            new_info(3,3) = 1e-100;
            new_info(4,4) = 1e-100;
            new_info(5,5) = 1e-100;
            myedge->setInformation(new_info);
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
            ros::Duration delta_time = graph_[n_id2]->pc_col->header.stamp - graph_[n_id1]->pc_col->header.stamp;
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
    //cam_cam_edges.swap(remaining_cam_cam_edges);
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
    EdgeSet::iterator edge_iter = cam_cam_edges.begin();
    for(;edge_iter != cam_cam_edges.end(); edge_iter++) {
        g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
        std::vector<g2o::HyperGraph::Vertex*>& myvertices = myedge->vertices();
        v1 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(1));
        v2 = dynamic_cast<g2o::VertexSE3*>(myvertices.at(0));
        current_edges->append( qMakePair(vertex_id_to_node_id[v1->id()], vertex_id_to_node_id[v2->id()]));
    }
    return current_edges;
}

QList<QMatrix4x4>* GraphManager::getAllPosesAsMatrixList(){
    ScopedTimer s(__FUNCTION__);
    ROS_DEBUG("Retrieving all transformations from optimizer");
    //QList<QMatrix4x4>* result = new QList<QMatrix4x4>();
    current_poses_.clear();
#if defined(QT_VERSION) && QT_VERSION >= 0x040700
    current_poses_.reserve(camera_vertices.size());//only allocates the internal pointer array
#endif

    for (graph_it it = graph_.begin(); it !=graph_.end(); ++it){
      Node *node = it->second;

      g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex( node->vertex_id_));
      if(v){ 
        current_poses_.push_back(g2o2QMatrix(v->estimateAsSE3Quat())); 
      } else {
        ROS_ERROR("Nullpointer in graph at position %i!", it->first);
      }
    }
    return new QList<QMatrix4x4>(current_poses_); //pointer to a copy
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
  QMutexLocker locker(&optimizer_mutex);
  std::fstream filestr;
  filestr.open (qPrintable(filename),  std::fstream::out );

 EdgeSet::iterator edge_iter = cam_cam_edges.begin();
 for(int i =0;edge_iter != cam_cam_edges.end(); edge_iter++, i++) {
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
  QMutexLocker locker(&optimizer_mutex);
  QMutexLocker locker2(&optimization_mutex);
  EdgeSet::iterator edge_iter = cam_cam_edges.begin();
  for(int i =0;edge_iter != cam_cam_edges.end(); edge_iter++, i++) {
    g2o::EdgeSE3* myedge = dynamic_cast<g2o::EdgeSE3*>(*edge_iter);
    Eigen::Vector3d ev = myedge->measurement().translation();
    if(ev.squaredNorm() > thresh){
      //optimizer_->removeEdge(myedge); 
      myedge->setInformation(Eigen::Matrix<double,6,6>::Identity()* 0.000001);
    }
  }
}


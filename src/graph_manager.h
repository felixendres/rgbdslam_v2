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


/*
 * graph_manager.h
 *
 *  Created on: 19.01.2011
 *      Author: hess
 */

#ifndef GRAPH_MANAGER_H_
#define GRAPH_MANAGER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "node.h"
#include "ColorOctomapServer.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <map>
#include <QObject>
#include <QString>
#include <QMatrix4x4>
#include <QList>
#include <QMap>
#include <QMutex>

#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <memory> //for auto_ptr
#include <utility>
#include "parameter_server.h"
// #define DO_LOOP_CLOSING
// DO_FEATURE_OPTIMIZATION is set in CMakeLists.txt
#ifdef DO_FEATURE_OPTIMIZATION
#include "landmark.h"
#endif

//#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/sparse_optimizer.h"

//#include "g2o/types/slam3d/camera_parameters.h"
#include "g2o/types/slam3d/parameter_camera.h"
#include "g2o/types/slam2d/vertex_se2.h"

#include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/robust_kernel_impl.h"

namespace tf {
class TransformListener;
}

//typedef g2o::HyperGraph::VertexSet::iterator Vset_it;
///Type to iterate over graph map
typedef std::map<int, Node* >::iterator graph_it;
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;
typedef g2o::HyperGraph::EdgeSet::iterator EdgeSet_it;
typedef std::map<int, Node* >::iterator graph_it;
//#define ROSCONSOLE_SEVERITY_INFO
/*
class GraphNode {
  GraphNode() : backend_node(NULL), frontend_node(NULL), index(-1) {}

  g2o::VertexSE3* backend_node;
  Node* frontend_node;
  int index;
}
*/

//!Computes a globally optimal trajectory from transformations between Node-pairs
class GraphManager : public QObject {
    Q_OBJECT
    Q_SIGNALS:
      ///Connect to this signal to get the transformation matrix from the last frame as QString
      void newTransformationMatrix(QString);
      void sendFinished();
      void setGUIInfo(QString message);
      void setGUIStatus(QString message);
      void setPointCloud(pointcloud_type * pc, QMatrix4x4 transformation);
      void setFeatures(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >*);
      void updateTransforms(QList<QMatrix4x4>* transformations);
      void setGUIInfo2(QString message);
      void setGraphEdges(const QList<QPair<int, int> >* edge_list);
      void deleteLastNode();
      void resetGLViewer();
      void setGraph(const g2o::OptimizableGraph*);
      void iamBusy(int id, const char* message, int max);
      void progress(int id, const char* message, int val);
      void renderableOctomap(Renderable* r);
    
    public Q_SLOTS:
      /// Start over with new graph
      void reset();
      ///Throw the last node out, reoptimize
      void deleteLastFrame(); 
      void clearPointCloud(pointcloud_type const * pc);
      void clearPointClouds();
      void reducePointCloud(pointcloud_type const * pc);
      ///Calls optimizeGraphImpl either in fore- or background depending on parameter concurrent_optimization
      ///Returns chi2
      double optimizeGraph(double iter = -1, bool nonthreaded=false);
      void printEdgeErrors(QString);
      ///Actually only discounts the edges drastically
      ///Returns the number of edges that have been discounted
      ///Thresh corresponds to a squared error
      unsigned int pruneEdgesWithErrorAbove(float);
      void sanityCheck(float);
      void toggleMapping(bool);
      void filterNodesByPosition(float x, float y, float z);

      //The following SLOT methods are in graph_mgr_io.cpp:
      ///iterate over all Nodes, sending their transform and pointcloud
      void sendAllClouds(bool threaded_if_available=true);
      ///iterate over all Nodes, writing their transform and pointcloud to a bagfile. 
      ///If parameter concurrent_io is set, do it in a background thread
      void saveBagfileAsync(QString filename);
      ///iterate over all Nodes, writing their transform and pointcloud to a bagfile
      void saveBagfile(QString filename);
      ///Call saveIndividualCloudsToFile, as background thread if threaded=true and possible 
      void saveIndividualClouds(QString file_basename, bool threaded=true);
      ///Call saveAllCloudsToFile,  as background thread if threaded=true and possible 
      void saveAllClouds(QString filename, bool threaded=true);
      ///Call saveAllFeaturesToFile,  as background thread if threaded=true and possible 
      void saveAllFeatures(QString filename, bool threaded = true);
      ///Save the cloud as octomap by using the (internal) octomap server to render the point clouds into the voxel map
      void saveOctomap(QString filename, bool threaded = true);
      ///Save trajectory of computed motion and ground truth (if available)
      void saveTrajectory(QString filebasename, bool with_ground_truth = false);
      ///Save graph in g2o format
      void saveG2OGraph(QString filename);
      ///Remove points in free space for all nodes (requires octomapping to be done)
      void occupancyFilterClouds();

    public:
      GraphManager();
      ~GraphManager();

    //! Add new node to the graph.
    /// Node will be included, if a valid transformation to one of the former nodes
    /// can be found. If appropriate, the graph is optimized
    /// graphmanager owns newNode after this call. Do no delete the object
    /// \callergraph
    bool addNode(Node* newNode); 

    //! Check if all nodes include the odometry. If not, add a new edge.
    /// \callergraph
    void addOdometry(ros::Time timestamp, tf::TransformListener* listener);
    //! Try to compute transformations to previous nodes
    /// getPotentialEdgeTargetsWithDijkstra is used to select
    /// previous nodes to match against, then the comparison
    /// of nodes is called, possibly in parallel.
    /// \callergraph
    bool nodeComparisons(Node* newNode, 
                         QMatrix4x4& curr_motion_estimate,
                         bool& edge_to_keyframe);///Output:contains the best-yet of the pairwise motion estimates for the current node
    /// Adds the first node
    void firstNode(Node* new_node);

    ///Pose vertices (in camera coordinate system)
    g2o::HyperGraph::VertexSet camera_vertices;
    ///"Regular" edges from camera to camera as found form feature correspondeces
    g2o::HyperGraph::EdgeSet cam_cam_edges_;
    ///Contains those motions that represent absence of information. 
    ///Only there to prevent the graph from becoming unconnected.
    //g2o::HyperGraph::EdgeSet dummy_edges_; 
    ///Edges added by addOdometryEdgeToG2O(...)
    g2o::HyperGraph::EdgeSet odometry_edges_;
    g2o::HyperGraph::EdgeSet current_match_edges_;

#ifdef DO_FEATURE_OPTIMIZATION
    g2o::HyperGraph::EdgeSet cam_lm_edges;
    g2o::HyperGraph::VertexSet landmark_vertices;
#endif

    ///Draw the features's motions onto the canvas
    void drawFeatureFlow(cv::Mat& canvas,
                         cv::Scalar line_color = cv::Scalar(255,0,0,0), 
                         cv::Scalar circle_color = cv::Scalar(0,0,255,0)); 

    //Juergen: overloading function to draw features in different image
    void drawFeatureFlow(cv::Mat& canvas_flow, cv::Mat& canvas_features,
                         cv::Scalar line_color = cv::Scalar(255,0,0,0), 
                         cv::Scalar circle_color = cv::Scalar(0,0,255,0)); 

    ///Used by OpenNIListener. Indicates whether long running computations are running
    bool isBusy();
    
    //!Warning: This is a dangerous way to save memory. Some methods will behave undefined after this.
    ///Notable exception: optimizeGraph()
    void deleteFeatureInformation();
    ///Only write (not create, not clear) existing octomap
    void writeOctomap(QString filename) const;
    void setOptimizerVerbose(bool verbose);

    ///Compute information matrix from empirical variances.
    ///If graph_filename is given, compute variances from 
    void setEmpiricalCovariances();
protected:
        
    ///Start over
    void resetGraph();
    ///Applies g2o for optimization returns chi2
    double optimizeGraphImpl(double max_iter);

    ///Write current position estimate to the node's point cloud's origin
    ///Make sure to acquire the optimizer_mutex_ before calling
    bool updateCloudOrigin(Node* node);
    ///Instanciate the optimizer with the desired backend
    void createOptimizer(std::string backend, g2o::SparseOptimizer* optimizer = NULL);
    ///will contain the motion to the best matching node
    MatchingResult curr_best_result_; 

    ///Compute the tranformation between (sensor) Base and Fixed (Map) frame
    tf::StampedTransform computeFixedToBaseTransform(Node* node, bool invert);
    /// Suggest nodes for comparison. Suggests <sequential_targets> direct predecessors in the time sequence
    /// <geodesic_targets> nodes from the graph-neighborhood and <sample_targets> randomly chosen from the keyframes
    /// Using the graph neighborhood, has the advantage that once a loop closure is found by sampling, the edge 
    /// will be used to find more closures, where the first one was found 
    QList<int> getPotentialEdgeTargetsWithDijkstra(const Node* new_node, int sequential_targets, int geodesic_targets, int sampled_targets = 5, int predecessor_id = -1, bool include_predecessor = false);
    
    //std::vector<int> getPotentialEdgeTargetsFeatures(const Node* new_node, int max_targets);

    ///use matching results to update location
    void localizationUpdate(Node* new_node, QMatrix4x4 motion_estimate);
#ifdef DO_FEATURE_OPTIMIZATION
    std::vector<Landmark> landmarks;
    void updateLandmarks(const MatchingResult& match_result, Node* old_node, Node* new_node);
    int next_landmark_id;
    // merge lm_2 into lm_1
    void mergeLandmarks(Landmark *lm_1, Landmark *lm_2);
    void printLandmarkStatistic();
    void updateProjectionEdges();
    void updateLandmarkInGraph(Landmark* lm);
    void removeFeaturesFromGraph();
#endif


#ifdef DO_LOOP_CLOSING
    void createSearchTree(); // use all images in the GraphManager
    void createSearchTree(const std::vector<int>& node_ids); // as stored in GraphManager::graph_
    // find nodes with similar images, returns sorted list of images with their score (more is better)
    void getNeighbours(int node_id, uint neighbour_cnt, std::vector<std::pair<int,float> >& neighbours);

    cv::flann::Index *tree;
    cv::Mat all_descriptors;

    std::set<std::pair<int,int> > tested_pairs;
    std::set<std::pair<int,int> > matched_pairs;

    void loopClosingTest();
    uint descriptor_length;
    uint total_descriptor_count;
    int *descriptor_to_node;
#endif
    
    ///Adds an visual-feature-correspondence edge to the optimizer
    bool addEdgeToG2O(const LoadedEdge3D& edge, Node* n1, Node* n2, bool good_edge, bool set_estimate, QMatrix4x4& motion_estimate);
    ///Adds an odometry edge to the optimizer
    bool addOdometryEdgeToG2O(const LoadedEdge3D& edge, Node* n1, Node* n2, QMatrix4x4& motion_estimate);

    //Delete a camera frame. Be careful, this might split the graph!
    void deleteCameraFrame(int id);

    ///Broadcast given transform
    void broadcastTransform(const tf::StampedTransform& computed_motion);

    ///Broadcast cached transform
    void broadcastLatestTransform(const ros::TimerEvent& event) const;

    ///Compute the transform between the fixed frame (usually the initial position of the cam) 
    /// and the node from the motion (given in sensor frame)
    tf::StampedTransform stampedTransformInWorldFrame(const Node* node, 
                                                      const tf::Transform& computed_motion) const;

    ///Add a keyframe to the list (and log keyframes)
    void addKeyframe(int id);

    int last_added_cam_vertex_id(){
      return graph_[graph_.size()-1]->vertex_id_;
    }

    int nodeId2VertexId(int node_id){
     assert(graph_.find(node_id) != graph_.end());
     return graph_[node_id]->vertex_id_;
    }
    //! Return pointer to a list of the optimizers graph poses on the heap(!)
    QList<QMatrix4x4>* getAllPosesAsMatrixList() const;
    //! Return pointer to a list of the optimizers graph edges on the heap(!)
    QList<QPair<int, int> >* getGraphEdges(); 

    // MEMBER VARIABLES
    QList<QPair<int, int> > current_edges_;
    //QMutex current_edges_lock_;
    //QList<QMatrix4x4> current_poses_;

    //void mergeAllClouds(pointcloud_type & merge);
    double geodesicDiscount(g2o::HyperDijkstra& hypdij, const MatchingResult& mr);
    ///See setEmpiricalCovariances
    void setEmpiricalCovariancesForEdgeSet(EdgeSet& edges);
    
    g2o::SparseOptimizer* optimizer_;

    ros::Publisher marker_pub_; 
    ros::Publisher ransac_marker_pub_;
    ros::Publisher whole_cloud_pub_;
    ros::Publisher batch_cloud_pub_;
    ros::Publisher online_cloud_pub_;
    
    //!Used to start the broadcasting of the pose estimate regularly
    ros::Timer timer_;
    //!Used to broadcast the pose estimate
    mutable tf::TransformBroadcaster br_;
    tf::Transform computed_motion_; ///<transformation of the last frame to the first frame (assuming the first one is fixed)
    tf::Transform  init_base_pose_;
    tf::StampedTransform latest_transform_cache_;//base_frame -> optical_frame 

    //!Map from node id to node. Assumption is, that ids start at 0 and are consecutive
    typedef std::pair<int, Node*> GraphNodeType;
    //QMap<int, Node* > graph_;
    std::map<int, Node* > graph_;
    bool reset_request_;
    unsigned int marker_id_;
    bool batch_processing_runs_;
    bool process_node_runs_;
    bool localization_only_;
    //!This mutex restricts access to the optimizer's data structures
    QMutex optimizer_mutex_;
    //!This mutex restricts the optimization computation itself
    QMutex optimization_mutex_;
    //cv::FlannBasedMatcher global_flann_matcher;
    QList<int> keyframe_ids_;//Keyframes are added, if no previous keyframe was matched
    //NEW std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > feature_coords_;  
    //NEW cv::Mat feature_descriptors_;         
    unsigned int loop_closures_edges, sequential_edges;
    unsigned int next_seq_id;
    unsigned int next_vertex_id;
    std::string current_backend_;
    int earliest_loop_closure_node_;
    ColorOctomapServer co_server_;
    
    //The following methods are defined in graph_mgr_io.cpp:
    void sendAllCloudsImpl();
    ///iterate over all Nodes, transform them to the fixed frame, aggregate and save
    void saveAllCloudsToFile(QString filename);
    ///Transform all feature positions to global coordinates and save them together with the belonging descriptors
    void saveAllFeaturesToFile(QString filename);
    ///iterate over all Nodes, save each in one pcd-file
    void saveIndividualCloudsToFile(QString filename);
    void saveOctomapImpl(QString filename);
    void renderToOctomap(Node* node);
    void pointCloud2MeshFile(QString filename, pointcloud_type full_cloud);
    void savePlyFile(QString filename, pointcloud_normal_type& full_cloud);

    //RVIZ visualization stuff (also in graph_mgr_io.cpp)
    ///Send markers to visualize the graph edges (cam transforms) in rviz (if somebody subscribed)
    void visualizeGraphEdges() const;
    ///Send markers to visualize the graph nodes (cam positions) in rviz (if somebody subscribed)
    void visualizeGraphNodes() const;
    ///Send markers to visualize the graph ids in rviz (if somebody subscribed)
    void visualizeGraphIds() const;
    ///Send markers to visualize the last matched features in rviz (if somebody subscribed)
    void visualizeFeatureFlow3D(unsigned int marker_id = 0, bool draw_outlier = true);
    
    g2o::RobustKernelHuber robust_kernel_;
    //g2o::RobustKernelDCS robust_kernel_;
    g2o::SparseOptimizer::Vertex* calibration_vertex_;
    //g2o::SparseOptimizer::Vertex* odom_calibration_vertex_;


    //Odometry variables needed
    unsigned int last_odom_time_;

};

///Send node's pointcloud with given publisher and timestamp
void publishCloud(Node* node, ros::Time timestamp, ros::Publisher pub);


#endif /* GRAPH_MANAGER_H_ */

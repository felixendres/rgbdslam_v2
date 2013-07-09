/*
 * loop_closing.cpp
 *
 *  Created on: Jun 11, 2012
 *      Author: Nikolas Engelhard
 */


#include "graph_manager.h"

using namespace std;


#ifdef DO_LOOP_CLOSING

void GraphManager::createSearchTree(){

 vector<int> all_node_ids;
 for (map<int,Node*>::iterator it = graph_.begin(); it != graph_.end(); ++it){
  all_node_ids.push_back(it->first);
 }

 createSearchTree(all_node_ids);
}


int findClosestNeighbour(cv::Mat all, cv::Mat row){

 int best = -1;
 float dist = 1e10;

 for (int i=0; i<all.rows; ++i){
  float sum = 0;

  //  float norm = cv::norm(all.row(i)-row);
  for (int j=0; j<row.cols; ++j){
   sum += abs(all.at<float>(i,0)-row.at<float>(0,0));
  }
  if (sum < dist){
   dist = sum;
   best = i;
  }
 }

 return best;

}


void GraphManager::createSearchTree(const std::vector<int>& node_ids){

 // get descriptor size:
 descriptor_length = graph_.begin()->second->feature_descriptors_.cols;

 total_descriptor_count = 0;

 // get total number of features
 for (uint i=0; i<node_ids.size(); ++i)
  total_descriptor_count += graph_.at(node_ids[i])->feature_descriptors_.rows;

 // TODO: memleak?
 descriptor_to_node = new int[total_descriptor_count]; // map from index-position to image

 // storage for all descriptors
 all_descriptors = cv::Mat(total_descriptor_count,descriptor_length, CV_32FC1);


 /*
  *  int descriptor_size = graph_[0]->feature_descriptors_.cols;
 int descriptor_type = graph_[0]->feature_descriptors_.type();
 cv::Mat alldescriptors(0, descriptor_size,  descriptor_type);
 alldescriptors.reserve(feat_count);
 for (unsigned int i = 0; i < graph_.size(); ++i) {
  alldescriptors.push_back(graph_[i]->feature_descriptors_);
 }
  */

 ROS_INFO("Building a tree for %i descriptors of length %i", total_descriptor_count, descriptor_length);

 uint pos=0;
 for (uint i=0; i<node_ids.size(); ++i){

  Node* node = graph_.at(node_ids[i]);
  int desc_cnt = node->feature_descriptors_.rows;

  // copy desciptors into large matrix
  //all_descriptors.rowRange(pos, pos+desc_cnt) = node->feature_descriptors_;
  for (int l = 0; l<desc_cnt; ++l){
   for (uint d = 0; d<descriptor_length; d++){
    all_descriptors.at<float>(pos+l,d) = node->feature_descriptors_.at<float>(l,d);
    descriptor_to_node[pos+l]= node->id_;
   }
  }
  // ROS_INFO("img %i has %i descriptors", node->id_, desc_cnt);
  pos += desc_cnt;
 }


 // Brute force matching
 // tree = new cv::flann::Index(all_descriptors, cv::flann::LinearIndexParams());.

 // Random Forest
 tree = new cv::flann::Index(all_descriptors, cv::flann::KDTreeIndexParams(16));


 // ROS_INFO("FLANN");
 // for (uint i=0; i<total_descriptor_count; ++i){
 //  for (uint j=0; j<descriptor_length; ++j){
 //   cout << all_descriptors.at<float>(i,j) << " ";
 //   assert(all_descriptors.at<float>(i,j) == all_descriptors.at<float>(i,j));
 //  }
 //  cout << endl;
 //}


 ROS_INFO("Tree was build");

}

bool higherScore(const pair<int,float>& A,const pair<int,float>& B){
 return A.second > B.second;
}


void GraphManager::loopClosingTest(){

 // if (graph_.size() != 3) return;
 //
 //cv::Mat f0(10,1,CV_32FC1);
 //cv::Mat f1(10,1,CV_32FC1);
 //cv::Mat f2(10,1,CV_32FC1);
 //
 //for (uint i=0; i<10; ++i){
 // f0.at<float>(i,0) = i;
 // f1.at<float>(i,0) = 100+i;
 // f2.at<float>(i,0) = 1000+i;
 //}
 //
 //graph_[0]->feature_descriptors_ = f0;
 //graph_[1]->feature_descriptors_ = f1;
 //graph_[2]->feature_descriptors_ = f2;




 createSearchTree();


 ROS_INFO("Test: best neighbour is image itself");


 // best match should be image itself and score should be one point for each descriptor;
 for (std::map<int, Node* >::iterator it = graph_.begin(); it!=graph_.end(); ++it){
  std::vector<std::pair<int,float> > neighbours;
  getNeighbours(it->first, 1, neighbours);
  // ROS_INFO("best match %i, id %i", neighbours[0].first,it->first);
  assert(neighbours[0].first == it->first);
  // ROS_INFO("%f %i",neighbours[0].second,it->second->feature_descriptors_.rows);
  //  assert(neighbours[0].second == it->second->feature_descriptors_.rows);
 }
 ROS_INFO("Test: k=1: PASSED");

 // again for more neighbours
 for (std::map<int, Node* >::iterator it = graph_.begin(); it!=graph_.end(); ++it){
  std::vector<std::pair<int,float> > neighbours;
  getNeighbours(it->first, 2, neighbours);
  // ROS_INFO("best match %i, id %i", neighbours[0].first,it->first);
  assert(neighbours[0].first == it->first);
  //ROS_INFO("%f %i",neighbours[0].second,it->second->feature_descriptors_.rows);
  //  assert(neighbours[0].second == it->second->feature_descriptors_.rows);
 }
 ROS_INFO("Test: k=2: PASSED");
 //
 // again for more neighbours
 for (std::map<int, Node* >::iterator it = graph_.begin(); it!=graph_.end(); ++it){
  std::vector<std::pair<int,float> > neighbours;
  getNeighbours(it->first, graph_.size(), neighbours);
  // ROS_INFO("best match %i, id %i", neighbours[0].first,it->first);
  assert(neighbours[0].first == it->first);
  // ROS_INFO("%f %i",neighbours[0].second,it->second->feature_descriptors_.rows);
  //  assert(neighbours[0].second == it->second->feature_descriptors_.rows);
 }
 ROS_INFO("Test: k=graph_.size(): PASSED");



}


void GraphManager::getNeighbours(int node_id, uint neighbour_cnt, std::vector<std::pair<int,float> >& neighbours){

 // ROS_INFO("Finding neighbours for node %i", node_id);

 assert(tree);
 assert(descriptor_to_node);

 neighbours.clear();

 Node *node = graph_.at(node_id);

 // ROS_INFO("Features of node %i", node_id);
 //  for (int i=0; i<node->feature_descriptors_.rows; ++i)
 //   cout << node->feature_descriptors_.at<float>(i,0) << " ";
 //  cout << endl;

 cv::Mat indices(node->feature_descriptors_.rows, neighbour_cnt, CV_32S);
 cv::Mat dists(node->feature_descriptors_.rows, neighbour_cnt, CV_32FC1);

 std::vector<int> index;
 std::vector<float> dist;

 // for (int i=0; i<node->feature_descriptors_.rows; ++i){
 //
 //  int best = findClosestNeighbour(all_descriptors,node->feature_descriptors_.row(i) );
 //
 //  tree->knnSearch(node->feature_descriptors_.row(i), index, dist, 1);//, cv::flann::SearchParams(100));
 //
 //  //ROS_INFO("row %i: index: %i (own: %i)", i,index[0], best);
 //
 // }


 tree->knnSearch(node->feature_descriptors_, indices, dists, neighbour_cnt);//, cv::flann::SearchParams(100));

 map<int, float> scores;

 for (int desc=0; desc<indices.rows; ++desc){
  for (int neighbour = 0; neighbour<indices.cols; neighbour++){

   int desc_id = indices.at<int>(desc, neighbour); // pos of similiar descriptor in tree
   int best_node_id = descriptor_to_node[desc_id]; // descriptor belongs to this image

   // for DEBUG:
   // if the descriptors of the node are in the tree, the best neighbour should be the descriptor itself
   // if (neighbour == 0)  assert(best_node_id == node_id);

   // linear decreasing score
   float value = neighbour_cnt-neighbour;

   if (scores.find(best_node_id) == scores.end()){
    scores[best_node_id] = value;
   }else{
    scores[best_node_id] += value;
   }

  }

  //  cout << endl;

 }



 // ROS_INFO("%zu images have similar descs (k was %i)", scores.size(), neighbour_cnt);
 std::vector<pair<int,float> > all_neighbours;
 float score_sum = 0;

 for (map<int, float>::iterator it = scores.begin(); it != scores.end(); ++it){

  //  all_neighbours.push_back(make_pair(it->first, it->second));

  // normalizing the score with the number of features in the corresponding image
  all_neighbours.push_back(make_pair(it->first, it->second/graph_.at(it->first)->feature_descriptors_.rows));

  //  ROS_INFO("node: %i,score: %f", it->first, it->second);
  score_sum+=it->second;
 }

 sort(all_neighbours.begin(), all_neighbours.end(),higherScore);

 for (uint i=0; i< uint(all_neighbours.size()) && i<neighbour_cnt; ++i){
  //  ROS_INFO("img: %i, score: %f", all_neighbours[i].first,all_neighbours[i].second);
  neighbours.push_back(all_neighbours[i]);
 }



}


#endif


#include "matching_result.h"
#include "sstream"
const char* MatchingResult::toString(){
  std::stringstream ss;
  ss << "edge id1: " << edge.id1 << "; ";
  ss << "edge id2: " << edge.id2 << "; ";
  ss << "rmse: " << rmse << "; ";
  if(inlier_points != 0) ss << "inlier_points: " << inlier_points << "; ";
  if(outlier_points != 0) ss << "outlier_points: " << outlier_points << "; ";
  if(occluded_points != 0) ss << "occluded_points: " << occluded_points << "; ";
  if(all_points != 0) ss << "all_points: " << all_points << "; ";
  if(inlier_matches.size() != 0) ss << "inlier matches size: " << inlier_matches.size() << "; ";
  if(all_matches.size() != 0) ss << "all matches size: " << all_matches.size() << "; ";
  return ss.str().c_str();
}


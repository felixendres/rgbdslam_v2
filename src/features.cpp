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
#include <cv.h>
#if CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION >= 4
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#ifdef CV_NONFREE
#include "opencv2/nonfree/nonfree.hpp"
#endif
#endif

#include "feature_adjuster.h"
#include "parameter_server.h"
#include "ros/ros.h"
//For tolower and transform
#include <string>
#include <algorithm>

using namespace cv;

StatefulFeatureDetector* adjusterWrapper(cv::Ptr<DetectorAdjuster> detadj, int min, int max)
{
  int iterations = ParameterServer::instance()->get<int>("adjuster_max_iterations");
  ROS_INFO("Using adjusted keypoint detector with %d maximum iterations, keeping the number of keypoints between %d and %d", iterations, min, max);
  return new  VideoDynamicAdaptedFeatureDetector(detadj, min, max, iterations);
}

StatefulFeatureDetector* adjustedGridWrapper(cv::Ptr<DetectorAdjuster> detadj)
{
  ParameterServer* params = ParameterServer::instance();
  //Try to get between "max" keypoints and 1.5 times of that.
  //We actually want exactly "max_keypoints" keypoints. Therefore
  //it's better to overshoot and then cut away the excessive keypoints
  int min = params->get<int>("max_keypoints"); //Shall not get below max
  int max = min * 1.5; //

  int gridRes = params->get<int>("detector_grid_resolution");
  int gridcells = gridRes*gridRes;
  int gridmin = round(min/static_cast<float>(gridcells));
  int gridmax =  round(max/static_cast<float>(gridcells));
  ROS_INFO("Using gridded keypoint detector with %dx%d cells, keeping %d keypoints in total.", gridRes, gridRes, max);
  
  StatefulFeatureDetector* detector = adjusterWrapper(detadj, gridmin, gridmax);

  return new VideoGridAdaptedFeatureDetector(detector, max, gridRes, gridRes);
}

//Use Grid or Dynamic or GridDynamic as prefix of FAST, SIFT, SURF or AORB
Feature2D* createDetector(const std::string& detectorName){
  //For anything but SIFTGPU
  DetectorAdjuster* detAdj = NULL;

  ROS_INFO_STREAM("Using " << detectorName << " keypoint detector.");
  if( detectorName == "SIFTGPU" ) {
    return NULL;// Will not be used
  } 
  else if(detectorName == "FAST") {
     detAdj = new DetectorAdjuster("FAST", 20);
  }
#ifdef CV_NONFREE
  else if(detectorName == "SURF" || detectorName == "SURF128") {
     detAdj = new DetectorAdjuster("SURF", 200);
  }
  else if(detectorName == "SIFT") {
     detAdj = new DetectorAdjuster("SIFT", 0.04, 0.0001);
  }
#else
  else if(detectorName == "SURF" || detectorName == "SURF128" || detectorName == "SIFT") 
  {
     ROS_ERROR("OpenCV non-free functionality (%s) not built in.", detectorName.c_str());
     ROS_ERROR("To enable non-free functionality build with CV_NONFREE set.");
     ROS_WARN("Using ORB feature detection instead.");
     ParameterServer::instance()->set("feature_detector_type", std::string("ORB"));
     detAdj = new DetectorAdjuster("ORB", 20);
  }
#endif
  else if(detectorName == "ORB") {
     detAdj = new DetectorAdjuster("ORB", 20);
  } 
  else {
    ROS_ERROR("Unsupported Keypoint Detector. Using ORB as fallback.");
    return createDetector("ORB");
  }
  assert(detAdj != NULL && "No valid detector aduster");

  ParameterServer* params = ParameterServer::instance();
  bool gridWrap = (params->get<int>("detector_grid_resolution") > 1);
  bool dynaWrap = (params->get<int>("adjuster_max_iterations") > 0);

  if(dynaWrap && gridWrap){
    return adjustedGridWrapper(detAdj);
  }
  else if(dynaWrap){
    int min = params->get<int>("max_keypoints");
    int max = min * 1.5; //params->get<int>("max_keypoints");
    return adjusterWrapper(detAdj, min, max);
  }
  else return detAdj;
}

cv::Ptr<DescriptorExtractor> createDescriptorExtractor(std::string descriptorType) 
{
    if(descriptorType == "ORB") {
        return ORB::create();
    }
#ifdef CV_NONFREE
    else if(descriptorType == "SIFT") {
        return cv::xfeatures2d::SiftDescriptorExtractor::create();/*( double magnification=SIFT::DescriptorParams::GET_DEFAULT_MAGNIFICATION(), bool isNormalize=true, bool recalculateAngles=true, int nOctaves=SIFT::CommonParams::DEFAULT_NOCTAVES, int nOctaveLayers=SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS, int firstOctave=SIFT::CommonParams::DEFAULT_FIRST_OCTAVE, int angleMode=SIFT::CommonParams::FIRST_ANGLE )*/
    }
    else if(descriptorType == "SURF") {
        return cv:xfeatures2d::SURF();/*( int nOctaves=4, int nOctaveLayers=2, bool extended=false )*/
    }
    else if(descriptorType == "SURF128") {
        auto extractor = cv:xfeatures2d::SURF();
        extractor->setExtended(true);
        return extractor;
    }
    else if(descriptorType == "BRIEF") {
        return cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if(descriptorType == "FREAK") {
        return cv::xfeatures2d::FREAK::create();
    }
#else
    else if(descriptorType == "SURF128" || descriptorType == "SIFT" || 
            descriptorType == "SURF" || descriptorType == "BRIEF" ||
            descriptorType == "FREAK") 
    {
        ROS_ERROR("OpenCV non-free functionality (%s) not built in.", descriptorType.c_str());
        ROS_ERROR("To enable non-free functionality build with CV_NONFREE set.");
        ROS_WARN("Using ORB feature detection instead.");
        ParameterServer::instance()->set("feature_extractor_type", std::string("ORB"));
        return createDescriptorExtractor("ORB");
    }
#endif
    else if(descriptorType == "BRISK") {
        return cv::BRISK::create();/*brisk default: (int thresh=30, int octaves=3, float patternScale=1.0f)*/
    }
    else if(descriptorType == "SIFTGPU") {
      ROS_DEBUG("%s is to be used as extractor, creating ORB descriptor extractor as fallback.", descriptorType.c_str());
      return createDescriptorExtractor("ORB");
    }
    else {
      ROS_ERROR("No valid descriptor-matcher-type given: %s. Using ORB", descriptorType.c_str());
      return createDescriptorExtractor("ORB");
    }
}

static inline int hamming_distance_orb32x8_popcountll(const uint64_t* v1, const uint64_t* v2) {
  return (__builtin_popcountll(v1[0] ^ v2[0]) + __builtin_popcountll(v1[1] ^ v2[1])) +
         (__builtin_popcountll(v1[2] ^ v2[2]) + __builtin_popcountll(v1[3] ^ v2[3]));
}

int bruteForceSearchORB(const uint64_t* v, const uint64_t* search_array, const unsigned int& size, int& result_index){
  //constexpr unsigned int howmany64bitwords = 4;//32*8/64;
  const unsigned int howmany64bitwords = 4;//32*8/64;
  assert(search_array && "Nullpointer in bruteForceSearchORB");
  result_index = -1;//impossible
  int min_distance = 1 + 256;//More than maximum distance
  for(unsigned int i = 0; i < size-1; i+=1, search_array+=4){
    int hamming_distance_i = hamming_distance_orb32x8_popcountll(v, search_array);
    if(hamming_distance_i < min_distance){
      min_distance = hamming_distance_i;
      result_index = i;
    }
  } 
  return min_distance;
}

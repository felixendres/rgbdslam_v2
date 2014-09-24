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
#include "opencv2/nonfree/nonfree.hpp"
#endif
#include "aorb.h"

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
  ROS_WARN("Using adjusted keypoint detector with %d maximum iterations, keeping the number of keypoints between %d and %d", iterations, min, max);
  return new  VideoDynamicAdaptedFeatureDetector(detadj, min, max, iterations);
}

StatefulFeatureDetector* adjustedGridWrapper(cv::Ptr<DetectorAdjuster> detadj)
{
  ParameterServer* params = ParameterServer::instance();
  int gridRes = params->get<int>("detector_grid_resolution");
  int gridcells = gridRes*gridRes;
  int min = params->get<int>("max_keypoints"); //Shall not get below max
  int max = min * 1.5; //params->get<int>("max_keypoints");
  int gridmin = round(min/static_cast<float>(gridcells));
  int gridmax =  round(max/static_cast<float>(gridcells));
  ROS_WARN("Using gridded keypoint detector with %dx%d cells, keeping %d keypoints in total.", gridRes, gridRes, max);
  
  StatefulFeatureDetector* detector = adjusterWrapper(detadj, gridmin, gridmax);

  return new VideoGridAdaptedFeatureDetector(detector, max, gridRes, gridRes);
}

//Use Grid or Dynamic or GridDynamic as prefix of FAST, SIFT, SURF or AORB
FeatureDetector* create(std::string detectorName){
  //For anything but SIFTGPU
  DetectorAdjuster* detAdj = NULL;

  ROS_WARN_STREAM("Using " << detectorName << " keypoint detector.");
  if( detectorName == "SIFTGPU" ) {
    return NULL;// Will not be used
  } 
  else if(detectorName == "FAST") {
     detAdj = new DetectorAdjuster("FAST", 20);
  }
  else if(detectorName == "SURF" || detectorName == "SURF128") {
     detAdj = new DetectorAdjuster("SURF", 200);
  }
  else if(detectorName == "SIFT") {
     detAdj = new DetectorAdjuster("SIFT", 0.04, 0.0001);
  }
  else if(detectorName == "ORB") {
     detAdj = new DetectorAdjuster("AORB", 20);
  } 
  else {
    ROS_ERROR("Unsupported Keypoint Detector. Using GridDynamicORB as fallback.");
    return create("GridDynamicORB");
  }
  assert(detAdj != NULL);

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

///Analog to opencv example file and modified to use adjusters
FeatureDetector* createDetector( const string& detectorType ) 
{
  return create(detectorType);
//	ParameterServer* params = ParameterServer::instance();
//	FeatureDetector* fd = 0;
//    if( !detectorType.compare( "FAST" ) ) {
//        fd = adjustedGridWrapper(new DetectorAdjuster("FAST", 20));
//    }
//    else if( !detectorType.compare( "STAR" ) ) {
//        fd = new StarFeatureDetector( 16/*max_size*/, 5/*response_threshold*/, 10/*line_threshold_projected*/,
//                                      8/*line_threshold_binarized*/, 5/*suppress_nonmax_size*/ );
//    }
//    else if( !detectorType.compare( "SIFT" ) ) {
//#if CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION <= 3
//        fd = new SiftFeatureDetector(SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
//                                     SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());
//        ROS_INFO("Default SIFT threshold: %f, Default SIFT Edge Threshold: %f", 
//                 SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
//                 SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());
//#else
//        DetectorAdjuster* detadj = new DetectorAdjuster("SIFT", 0.04, 0.0001);
//        fd = adjustedGridWrapper(detadj);
//#endif
//    }
//    else if( !detectorType.compare( "SURF" ) || !detectorType.compare( "SURF128" ) ) {
//      /* fd = new SurfFeatureDetector(200.0, 6, 5); */
//        fd = adjustedGridWrapper(new DetectorAdjuster("SURF", 200));
//    }
//    else if( !detectorType.compare( "MSER" ) ) {
//        fd = new MserFeatureDetector( 1/*delta*/, 60/*min_area*/, 114400/*_max_area*/, 0.35f/*max_variation*/,
//                0.2/*min_diversity*/, 200/*max_evolution*/, 1.01/*area_threshold*/, 0.003/*min_margin*/,
//                5/*edge_blur_size*/ );
//    }
//    else if( !detectorType.compare( "HARRIS" ) ) {
//        ROS_INFO("Creating GFTT detector with HARRIS.");
//        fd = new GoodFeaturesToTrackDetector( params->get<int>("max_keypoints"), 0.0001, 2.0, 9, true);
//    }
//    else if( !detectorType.compare( "GFTT" ) ) {
//        ROS_INFO("Creating GFTT detector.");
//        fd = new GoodFeaturesToTrackDetector( params->get<int>("max_keypoints"), 0.0001, 2.0, 9);
//    }
//    else if( !detectorType.compare( "BRISK" ) ) {
//        ROS_INFO("Creating BRISK detector.");
//        fd = fd->create("BRISK");
//    }
//    else if( !detectorType.compare( "ORB" ) ) {
//#if CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION == 3
//        fd = new OrbFeatureDetector(params->get<int>("max_keypoints")+1500,
//                ORB::CommonParams(1.2, ORB::CommonParams::DEFAULT_N_LEVELS, 31, ORB::CommonParams::DEFAULT_FIRST_LEVEL));
//#elif CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION >= 4
//        ROS_INFO("Using Adapted ORB");
//        //fd = new AorbFeatureDetector(params->get<int>("max_keypoints"), 1.1, 8, 31, 0, 4, 0, 31, 10);
//        fd = adjustedGridWrapper(new DetectorAdjuster("AORB"));
//    //CV_WRAP explicit ORB(int nfeatures = 500, float scaleFactor = 1.2f, int nlevels = 8, int edgeThreshold = 31,
//     //   int firstLevel = 0, int WTA_K=2, int scoreType=ORB::HARRIS_SCORE, int patchSize=31 );
//#else
//        ROS_ERROR("ORB features are not implemented in your version of OpenCV");
//        ROS_DEBUG("Creating FAST detector as fallback.");
//        fd = createDetector("FAST"); //recursive call with correct parameter
//#endif
//    }
//    else if( !detectorType.compare( "SIFTGPU" ) ) {
//      ROS_INFO("%s is to be used", detectorType.c_str());
//      ROS_DEBUG("Creating SURF detector as fallback.");
//      fd = createDetector("SURF"); //recursive call with correct parameter
//    }
//    else {
//      ROS_WARN("No valid detector-type given: %s. Using SURF.", detectorType.c_str());
//      fd = createDetector("SURF"); //recursive call with correct parameter
//    }
//    ROS_ERROR_COND(fd == 0, "No detector could be created");
//    return fd;
}

DescriptorExtractor* createDescriptorExtractor( const string& descriptorType ) 
{
    DescriptorExtractor* extractor = 0;
    if( !descriptorType.compare( "SIFT" ) ) {
        extractor = new SiftDescriptorExtractor();/*( double magnification=SIFT::DescriptorParams::GET_DEFAULT_MAGNIFICATION(), bool isNormalize=true, bool recalculateAngles=true, int nOctaves=SIFT::CommonParams::DEFAULT_NOCTAVES, int nOctaveLayers=SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS, int firstOctave=SIFT::CommonParams::DEFAULT_FIRST_OCTAVE, int angleMode=SIFT::CommonParams::FIRST_ANGLE )*/
    }
    else if( !descriptorType.compare( "BRIEF" ) ) {
        extractor = new BriefDescriptorExtractor();
    }
    else if( !descriptorType.compare( "BRISK" ) ) {
        extractor = new cv::BRISK();/*brisk default: (int thresh=30, int octaves=3, float patternScale=1.0f)*/
    }
    else if( !descriptorType.compare( "FREAK" ) ) {
        extractor = new cv::FREAK();
    }
    else if( !descriptorType.compare( "SURF" ) ) {
        extractor = new SurfDescriptorExtractor();/*( int nOctaves=4, int nOctaveLayers=2, bool extended=false )*/
    }
    else if( !descriptorType.compare( "SURF128" ) ) {
        extractor = new SurfDescriptorExtractor();/*( int nOctaves=4, int nOctaveLayers=2, bool extended=false )*/
        extractor->set("extended", 1);
    }
#if CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION >= 3
    else if( !descriptorType.compare( "ORB" ) ) {
        extractor = new OrbDescriptorExtractor();
    }
#endif
    else if( !descriptorType.compare( "SIFTGPU" ) ) {
      ROS_DEBUG("%s is to be used as extractor, creating SURF descriptor extractor as fallback.", descriptorType.c_str());
      extractor = new SurfDescriptorExtractor();/*( int nOctaves=4, int nOctaveLayers=2, bool extended=false )*/
    }
    else {
      ROS_ERROR("No valid descriptor-matcher-type given: %s. Using SURF", descriptorType.c_str());
      extractor = createDescriptorExtractor("SURF");
    }
    ROS_ERROR_COND(extractor == 0, "No extractor could be created");
    return extractor;
}


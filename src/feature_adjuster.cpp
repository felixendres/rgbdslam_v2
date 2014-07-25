/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this license.
 //  If you do not agree to this license, do not download, install,
 //  copy or use the software.
 //
 //
 //                           License Agreement
 //                For Open Source Computer Vision Library
 //
 // Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
 // Copyright (C) 2009-2010, Willow Garage Inc., all rights reserved.
 // Third party copyrights are property of their respective owners.
 //
 // Redistribution and use in source and binary forms, with or without modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright notice,
 //     this list of conditions and the following disclaimer in the documentation
 //     and/or other materials provided with the distribution.
 //
 //   * The name of the copyright holders may not be used to endorse or promote products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is" and
 // any express or implied warranties, including, but not limited to, the implied
 // warranties of merchantability and fitness for a particular purpose are disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/

#include "feature_adjuster.h"
//#include "opencv2/features2d/precomp.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <ros/ros.h>
using namespace cv;


DetectorAdjuster::DetectorAdjuster(const char* detector_name, double initial_thresh, double min_thresh, double max_thresh, double increase_factor, double decrease_factor ) :
    thresh_(initial_thresh), init_thresh_(initial_thresh),
    min_thresh_(min_thresh), max_thresh_(max_thresh),
    increase_factor_(increase_factor), decrease_factor_(decrease_factor),
    detector_name_(detector_name)
{
  if(detector_name_ == NULL){
    ROS_ERROR("No detector name given, using SURF");
    detector_name_ = "SURF";
  } 
}

void DetectorAdjuster::detectImpl(const Mat& image, std::vector<KeyPoint>& keypoints, const Mat& mask) const
{
    Ptr<FeatureDetector> detector = FeatureDetector::create(detector_name_);
    if(strcmp(detector_name_, "SURF") == 0){
      detector->set("hessianThreshold", thresh_);
    }
    else if(strcmp(detector_name_, "SIFT") == 0){
      detector->set("contrastThreshold", thresh_);
    }
    else if(strcmp(detector_name_, "FAST") == 0){
      detector->set("threshold", static_cast<int>(thresh_));
    }
    else {
      ROS_ERROR("Unknown Descriptor, not setting threshold");
    }
    detector->detect(image, keypoints, mask);
}

void DetectorAdjuster::setDecreaseFactor(double new_factor){
  decrease_factor_ = new_factor;
}
void DetectorAdjuster::setIncreaseFactor(double new_factor){
  increase_factor_ = new_factor;
}

void DetectorAdjuster::tooFew(int, int)
{
    thresh_ *= decrease_factor_;
    if (thresh_ < min_thresh_)
            thresh_ = min_thresh_;
    ROS_INFO("Too Few Features, Decreased Threshold. New Threshold: %f", thresh_);
}

void DetectorAdjuster::tooMany(int, int)
{
    thresh_ *= increase_factor_;
    if (thresh_ > max_thresh_)
            thresh_ = max_thresh_;
    ROS_INFO("Too Many Features, Increased Threshold. New Threshold: %f", thresh_);
}

//return whether or not the threshhold is beyond
//a useful point
bool DetectorAdjuster::good() const
{
    return (thresh_ > min_thresh_) && (thresh_ < max_thresh_);
}

Ptr<AdjusterAdapter> DetectorAdjuster::clone() const
{
    Ptr<AdjusterAdapter> cloned_obj(new DetectorAdjuster(detector_name_, init_thresh_, min_thresh_, max_thresh_, increase_factor_, decrease_factor_ ));
    return cloned_obj;
}

DynamicAdaptedFeatureDetectorWithStorage::DynamicAdaptedFeatureDetectorWithStorage(Ptr<AdjusterAdapter> a,
                                         int min_features, int max_features, int max_iters ) :
        escape_iters_(max_iters), min_features_(min_features), max_features_(max_features), adjuster_(a)
{}

bool DynamicAdaptedFeatureDetectorWithStorage::empty() const
{
    return !adjuster_ || adjuster_->empty();
}

void DynamicAdaptedFeatureDetectorWithStorage::detectImpl(const cv::Mat& _image, std::vector<KeyPoint>& keypoints, const cv::Mat& _mask) const
{
    //Mat image = _image, mask = _mask;

    //for oscillation testing
    bool down = false;
    bool up = false;

    //flag for whether the correct threshhold has been reached
    bool thresh_good = false;

    //break if the desired number hasn't been reached.
    int iter_count = escape_iters_;

    do { // detect at least once
        keypoints.clear();

        //the adjuster takes care of calling the detector with updated parameters
        adjuster_->detect(_image, keypoints,_mask);
        ROS_INFO("Detected %d Keypoints", int(keypoints.size()));
        if( int(keypoints.size()) < min_features_ )
        {
            down = true;
            adjuster_->tooFew(min_features_, (int)keypoints.size());
        }
        else if( int(keypoints.size()) > max_features_ )
        {
            up = true;
            adjuster_->tooMany(max_features_, (int)keypoints.size());
        }
        else
            thresh_good = true;

        iter_count--;
    } while( iter_count > 0 && !(down && up) && !thresh_good && adjuster_->good() );

}



#include <cv.h>
#include <string>

/// Creates Feature Detector Objects accordingt to the type.
/// Possible detectorTypes: FAST, STAR, SIFT, SURF, GFTT
/// Some features are the self-adjusting versions (see http://opencv.willowgarage.com/documentation/cpp/features2d_common_interfaces_of_feature_detectors.html#DynamicAdaptedFeatureDetector)
cv::FeatureDetector* createDetector( const std::string& detectorType );
/// Create an object to extract features at keypoints. The Exctractor is passed to the Node constructor and must be the same for each node.
cv::DescriptorExtractor* createDescriptorExtractor( const std::string& descriptorType );

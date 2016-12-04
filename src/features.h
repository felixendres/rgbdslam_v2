#include <cv.h>
#include <string>
#include <stdint.h>

/// Creates Feature Detector Objects accordingt to the type.
/// Possible: FAST, SURF, SIFT, ORB
/// The features are the self-adjusting versions (see
///http://opencv.willowgarage.com/documentation/cpp/features2d_common_interfaces_of_feature_detectors.html#DynamicAdaptedFeatureDetector)
cv::Feature2D* createDetector(const std::string& detectorType);
/// Create an object to extract features at keypoints. The Exctractor is passed
///to the Node constructor and must be the same for each node.
cv::Ptr<cv::DescriptorExtractor> createDescriptorExtractor(std::string descriptorType);

int bruteForceSearchORB(const uint64_t* v, const uint64_t* search_array, const unsigned int& size, int& result_index);

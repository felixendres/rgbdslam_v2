#ifndef FEATUREADJUSTER_H
#define FEATUREADJUSTER_H
#include <opencv2/features2d/features2d.hpp>

/** \brief an detector adjuster optimized for image sequences (video).
 * Use this Adjuster with the DynamicAdaptedFeatureDetector. 
 * It lets you set the increase/decrease factor for faster adaptation.
 * It works for SURF, SIFT, FAST and the adjustable ORB variant "AORB" 
 * which exposes its FAST threshold.
 */
class DetectorAdjuster: public cv::AdjusterAdapter
{
public:
    ///Initial values are for SURF detector
    DetectorAdjuster(const char* detector_name, double initial_thresh=200.f, double min_thresh=2, double max_thresh=10000, double increase_factor=1.3, double decrease_factor=0.7 );
    
    virtual void tooFew(int minv, int n_detected);
    virtual void tooMany(int maxv, int n_detected);
    virtual bool good() const;

    virtual cv::Ptr<cv::AdjusterAdapter> clone() const;

    void setIncreaseFactor(double new_factor);
    void setDecreaseFactor(double new_factor);
protected:
    virtual void detectImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat() ) const;

    double thresh_, init_thresh_, min_thresh_, max_thresh_;
    double increase_factor_, decrease_factor_;
    const char* detector_name_;
};

/** \brief an adaptively adjusting detector that iteratively detects until the desired number
 * of features are detected.
 *  Beware that this is not thread safe - as the adjustment of parameters breaks the const
 *  of the detection routine...
 *  /TODO Make this const correct and thread safe
 *
 *  sample usage:
 //will create a detector that attempts to find 100 - 110 FAST Keypoints, and will at most run
 //FAST feature detection 10 times until that number of keypoints are found
 Ptr<FeatureDetector> detector(new DynamicAdaptedFeatureDetector(new FastAdjuster(20,true),100, 110, 10));

 In contrast to the original DynamicAdaptedFeatureDetector, this variant is enhanced for
 processing of video sequences. It is meant to work with the DetectorAdjuster.
 It keeps the DetectorAdjuster alive, so that the final threshold will be retained
 throughout detection calls. For video sequences the "good" threshold will in
 general be similar for successive frames, therefore the last "good" threshold
 is a good starting point for the next frame.

 In case of too many features, this variant will just decrease the threshold of the
 DetectorAdjuster without triggering redetection, so the user needs to get rid of the
 superfluous keypoints. Mostly the keypoints are scored anyway, so this avoids
 costly and unnecessary redetections. 
 */
class DynamicAdaptedFeatureDetectorWithStorage: public cv::FeatureDetector
{
public:

    /** \param adjuster an AdjusterAdapter that will do the detection and parameter adjustment
     *  \param max_features the maximum desired number of features
     *  \param max_iters the maximum number of times to try to adjust the feature detector params
     *          for the FastAdjuster this can be high, but with Star or Surf this can get time consuming
     *  \param min_features the minimum desired features
     */
    DynamicAdaptedFeatureDetectorWithStorage( cv::Ptr<cv::AdjusterAdapter> adjuster, int min_features=400, int max_features=500, int max_iters=5 );

    virtual bool empty() const;

protected:
    virtual void detectImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat() ) const;

private:
    DynamicAdaptedFeatureDetectorWithStorage& operator=(const DynamicAdaptedFeatureDetectorWithStorage&);
    DynamicAdaptedFeatureDetectorWithStorage(const DynamicAdaptedFeatureDetectorWithStorage&);

    int escape_iters_;
    int min_features_, max_features_;
    mutable cv::Ptr<cv::AdjusterAdapter> adjuster_;
};
#endif

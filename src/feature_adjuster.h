#ifndef FEATUREADJUSTER_H
#define FEATUREADJUSTER_H
#include <opencv2/features2d.hpp>
#include <string>
#include <iostream>

/** \brief an detector adjuster optimized for image sequences (video).
 * Use this Adjuster with the DynamicAdaptedFeatureDetector. 
 * It lets you set the increase/decrease factor for faster adaptation.
 * It works for SURF, SIFT, FAST and the adjustable ORB variant "AORB" 
 * which exposes its FAST threshold.
 */
class DetectorAdjuster: public cv::Feature2D
{
public:
    ///Initial values are for SURF detector
    DetectorAdjuster(std::string detector_name, double initial_thresh=200.f, double min_thresh=2, double max_thresh=10000, double increase_factor=1.3, double decrease_factor=0.7 );
    
    virtual void tooFew(int minv, int n_detected);
    virtual void tooMany(int maxv, int n_detected);
    virtual bool good() const;

    virtual cv::Ptr<DetectorAdjuster> clone() const;

    void setIncreaseFactor(double new_factor);
    void setDecreaseFactor(double new_factor);
    //virtual void detect( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat() ) const;
    virtual void detect(cv::InputArray _image, std::vector<cv::KeyPoint>& keypoints, cv::InputArray _mask);
protected:

    double thresh_, min_thresh_, max_thresh_;
    double increase_factor_, decrease_factor_;
    const std::string detector_name_;
};

/** A Feature Detector that saves some state. 
 * Common base class for VideoDynamicAdaptedFeatureDetector and VideoGridAdaptedFeatureDetector */
class StatefulFeatureDetector : public cv::Feature2D {
  public:
    virtual cv::Ptr<StatefulFeatureDetector> clone() const = 0;
    CV_WRAP virtual void detect( cv::InputArray image,
                                 CV_OUT std::vector<cv::KeyPoint>& keypoints,
                                 cv::InputArray mask=cv::noArray() )
    {std::cerr << "StatefulFeatureDetector::detect\n";}
};

/** 
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
class VideoDynamicAdaptedFeatureDetector: public StatefulFeatureDetector
{
public:

    /** \param adjuster an AdjusterAdapter that will do the detection and parameter adjustment
     *  \param max_features the maximum desired number of features
     *  \param max_iters the maximum number of times to try to adjust the feature detector params
     *          for the FastAdjuster this can be high, but with Star or Surf this can get time consuming
     *  \param min_features the minimum desired features
     */
     VideoDynamicAdaptedFeatureDetector( cv::Ptr<DetectorAdjuster> adjuster, int min_features=400, int max_features=500, int max_iters=5);

    virtual cv::Ptr<StatefulFeatureDetector> clone() const;

    //virtual void detect( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat() ) const;
    CV_WRAP virtual void detect( cv::InputArray image,
                                 CV_OUT std::vector<cv::KeyPoint>& keypoints,
                                 cv::InputArray mask=cv::noArray() );

private:
    VideoDynamicAdaptedFeatureDetector& operator=(const VideoDynamicAdaptedFeatureDetector&);
    VideoDynamicAdaptedFeatureDetector(const VideoDynamicAdaptedFeatureDetector&);

    int escape_iters_;
    int min_features_, max_features_;
    mutable cv::Ptr<DetectorAdjuster> adjuster_;
};






/*
 * Adapts a detector to partition the source image into a grid and detect
 * points in each cell. Considers the overlap between cells required for
 * not losing keypoints
 */
class VideoGridAdaptedFeatureDetector : public StatefulFeatureDetector
{
public:
    /*
     * \param detector            Detector that will be adapted.
     * \param maxTotalKeypoints   Maximum count of keypoints detected on the image. Only the strongest keypoints will be keeped.
     *  \param gridRows           Grid rows count.
     *  \param gridCols           Grid column count.
     *  \param edgeThreshold     how much overlap is needed, to not lose keypoints at the inner borders of the grid (should be the same value, e.g., as edgeThreshold for ORB)
     */
    VideoGridAdaptedFeatureDetector(const cv::Ptr<StatefulFeatureDetector>& detector,
                                    int maxTotalKeypoints=1000, int gridRows=4, 
                                    int gridCols=4, int edgeThreshold=31 );
    
    // TODO implement read/write
    virtual cv::Ptr<StatefulFeatureDetector> clone() const;
    //virtual void detect(cv::InputArray _image, std::vector<cv::KeyPoint>& keypoints, cv::InputArray _mask) const;
     // detect( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat() ) const;
    CV_WRAP virtual void detect( cv::InputArray image,
                                 CV_OUT std::vector<cv::KeyPoint>& keypoints,
                                 cv::InputArray mask=cv::noArray() );

protected:
    VideoGridAdaptedFeatureDetector& operator=(const VideoGridAdaptedFeatureDetector&);
    VideoGridAdaptedFeatureDetector(const VideoGridAdaptedFeatureDetector&);


    std::vector<cv::Ptr<StatefulFeatureDetector> > detectors;
    int maxTotalKeypoints;
    int gridRows;
    int gridCols;
    int edgeThreshold; 
};


#endif

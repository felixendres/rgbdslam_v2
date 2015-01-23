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

    double thresh_, min_thresh_, max_thresh_;
    double increase_factor_, decrease_factor_;
    const char* detector_name_;
};

/** A Feature Detector that saves some state.
 * The copy constructor of */
class StatefulFeatureDetector : public cv::FeatureDetector {
  public:
    virtual cv::Ptr<StatefulFeatureDetector> clone() const = 0;
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
     VideoDynamicAdaptedFeatureDetector( cv::Ptr<cv::AdjusterAdapter> adjuster, int min_features=400, int max_features=500, int max_iters=5);

    virtual cv::Ptr<StatefulFeatureDetector> clone() const;
    virtual bool empty() const;

protected:
    virtual void detectImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat() ) const;

private:
    VideoDynamicAdaptedFeatureDetector& operator=(const VideoDynamicAdaptedFeatureDetector&);
    VideoDynamicAdaptedFeatureDetector(const VideoDynamicAdaptedFeatureDetector&);

    int escape_iters_;
    int min_features_, max_features_;
    mutable cv::Ptr<cv::AdjusterAdapter> adjuster_;
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
    virtual bool empty() const;
    virtual cv::Ptr<StatefulFeatureDetector> clone() const;

protected:
    VideoGridAdaptedFeatureDetector& operator=(const VideoGridAdaptedFeatureDetector&);
    VideoGridAdaptedFeatureDetector(const VideoGridAdaptedFeatureDetector&);

    virtual void detectImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat() ) const;

    std::vector<cv::Ptr<StatefulFeatureDetector> > detectors;
    int maxTotalKeypoints;
    int gridRows;
    int gridCols;
    int edgeThreshold; 
};


#endif

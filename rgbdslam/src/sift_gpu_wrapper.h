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


#ifndef SIFT_GPU_FEATURE_DETECTOR_H
#define SIFT_GPU_FEATURE_DETECTOR_H

//#ifdef USE_SIFT_GPU
#include <opencv2/features2d/features2d.hpp>
#include <opencv/cv.h>
#include <SiftGPU/SiftGPU.h>
#include <QMutex>

/*!
 * \brief Interface for SiftGPU
 * The class is used as an interface to SiftGPU.
 * It's a singleton class
 */
class SiftGPUWrapper {
public:
    /*!
     * Destructor
     */
	virtual ~SiftGPUWrapper();

	/*!
	 * Method, which is used for calculating the features and descriptors.
	 * The first parameter is the image, the second one a reference to a keypoint vector
	 * and the third parameter can be used for defining a mask
	 *
	 * \param  image        the image
	 * \param  keypoints    a cv::vector of cv::Keypoints, which is used for storing the keypoints
	 * \param  mask         a mask (see OpenCV)
	 * \return a pointer to the descriptor values
	 */
	void detect(const cv::Mat& image, cv::vector<cv::KeyPoint>& keypoints, std::vector<float>& descriptors, const cv::Mat& mask = cv::Mat()) const;

	/*!
	 * Is used for matching two descriptors
	 *
	 * \param  descriptors1     the first descriptor
	 * \param  num1             size of the first descriptor
	 * \param  descriptors2     the second descriptor
	 * \param  num2             size of the second descriptor
	 * \param  matches          is used to store the matches
	 * \return the summed distance of the corresponding descriptors
	 */
	int match(const std::vector<float>& descriptors1, int num1, const std::vector<float>& descriptors2, int num2, std::vector<cv::DMatch>* matches);

	/*!
	 * Return instance of the singleton class
	 */
	static SiftGPUWrapper* getInstance();
	static void destroyInstance();

private:
	/*!
	 * private constructor, because of singleton
	 */
	SiftGPUWrapper();

	void initializeMatcher();

	/*!
	 * Building a siftgpu compatible unsigned char pointer out of the image cv::Mat
	 * (converts a cv matrix into an OpenGL texture array)
	 *
	 * \param  image        the image
	 * \param  siftImage    the transformed image (output)
	 */
	void cvMatToSiftGPU(const cv::Mat& image, unsigned char* siftImage) const;

	/*!
	 * For testing purposes: write a .pgm file of the SiftGPU image
	 *
	 * \param   fp      a filepointer
	 * \param   data    the imagedata (e.g. OpenGL texture)
	 * \param   width   width
	 * \param   height  height
	 */
	void writePGM(FILE *fp, unsigned char* data, int width, int height);

	mutable int imageWidth;          ///<width of the image constant for Kinect
	mutable int imageHeight;         ///<height of the image constant for Kinect
	mutable unsigned char* data;     ///<image as texture

	static SiftGPUWrapper* instance;    ///<singleton instance
	SiftGPU* siftgpu;                           ///<siftgpu instance
	SiftMatchGPU *matcher;                      ///<siftgpu matcher
	bool isMatcherInitialized;                  ///<true, if matcher was initialized
	bool error;                                 ///<error happened?
  mutable QMutex gpu_mutex;
};

#endif
//#endif

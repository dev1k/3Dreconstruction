#define _CRT_SECURE_NO_WARNINGS
#include <iostream> 
#include <string> 
#include <iomanip>  
#include <sstream> 

#include "opencv/cv.h"

using namespace std;
using namespace cv;

class Matcher {
private:
	
	cv::Ptr<cv::FeatureDetector> detector;
	
	cv::Ptr<cv::DescriptorExtractor> extractor;
	float ratio; 
	double distance; 
	double confidence; 
	Mat fundamentalMatrics;
public:
	Matcher();

	Mat getFundamentalMatrix();

	void setFeatureDetector(Ptr<FeatureDetector> &detect);

	void setDescriptorExtractor(Ptr<DescriptorExtractor> & desExtract);

	void setMinDistanceToEpipolar(double distance);

	void setRatio(float ratio);

	void match(Mat &image1, Mat &image2, vector<DMatch> &matches, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2);

	void ratioTest(std::vector<std::vector<cv::DMatch>> &matches);

	void symmetryTest(const std::vector<std::vector<cv::DMatch>>& matches1, const std::vector<std::vector<cv::DMatch>>& matches2, std::vector<cv::DMatch>& symMatches);

	void setConfidenceLevel(double confidence); //доверительный уровень

	cv::Mat ransacTest(
		const std::vector<cv::DMatch>& matches,
		const std::vector<cv::KeyPoint>& keypoints1,
		const std::vector<cv::KeyPoint>& keypoints2,
		std::vector<cv::DMatch>& outMatches);


};



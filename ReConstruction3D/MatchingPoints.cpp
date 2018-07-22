#define _CRT_SECURE_NO_WARNINGS
#include <iostream> 
#include <string>   
#include <iomanip> 
#include <sstream> 

#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/core/core.hpp>      
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv/cv.h"
#include "Match.h"
#include "MatchingPoints.h"
#include "MatrixOperations.h"

using namespace std;
using namespace cv;

MatchingPoints::MatchingPoints(Mat image1, Mat image2) {
	Matcher SURFMatcher;
	SURFMatcher.setConfidenceLevel(0.98); //доверительный уровень
	SURFMatcher.setMinDistanceToEpipolar(1.0); 
	SURFMatcher.setRatio(0.65f); //расстояние
	cv::Ptr<cv::FeatureDetector> SFD = new cv::SurfFeatureDetector(10); 
	SURFMatcher.setFeatureDetector(SFD);

	
	SURFMatcher.match(image1, image2,
		matches, fullKeypoints1, fullKeypoints2);

	fundamentalMatrics = SURFMatcher.getFundamentalMatrix();

	//заполняем фильтрованные кейпоинты
	for (int i = 0; i < matches.size(); i++) {
		DMatch match = matches.at(i);
		keyPoints1.push_back(fullKeypoints1.at(match.queryIdx));
		keyPoints2.push_back(fullKeypoints2.at(match.trainIdx));
	}
}

vector<PointColors> MatchingPoints::getCLRS(Mat image) {
	

	vector<PointColors> clrs;
	KeyPoint point;
	
	for (int i = 0; i < keyPoints1.size(); i++) {

		point = keyPoints1.at(i);
		int x = int(point.pt.x + 0.5);
		int y = int(point.pt.y + 0.5);
		
		Point3_<uchar> *p = image.ptr< Point3_<uchar>>(y, x);
		
		PointColors pointColour;
		
		pointColour.blue = int(p->x);
		pointColour.green = int(p->y);
		pointColour.red = int(p->z);
		


		clrs.push_back(pointColour);
		
	}
	pointCLRS = clrs;

	return clrs;
}

Mat MatchingPoints::getFundamentalMatrix()
{
	return fundamentalMatrics;
}

vector<KeyPoint> MatchingPoints::getKeyPoints1()
{
	return keyPoints1;
}

vector<KeyPoint> MatchingPoints::getKeyPoints2()
{
	return keyPoints2;
}




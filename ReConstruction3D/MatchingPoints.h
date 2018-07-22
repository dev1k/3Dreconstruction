#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <string> 
#include <iomanip>
#include <sstream> 

#include "opencv/cv.h"

using namespace std;
using namespace cv;

struct PointColors {
	int red;
	int blue;
	int green;
};

class MatchingPoints {
private:
	vector<KeyPoint> keyPoints1;
	vector<KeyPoint> keyPoints2;
	vector<PointColors> pointCLRS;
	Mat fundamentalMatrics;

	vector<DMatch> matches;
	vector<KeyPoint> fullKeypoints1, fullKeypoints2;
	
public:
	MatchingPoints(Mat image1, Mat image2);
	vector<KeyPoint> getKeyPoints1();
	vector<KeyPoint> getKeyPoints2();
	Mat getFundamentalMatrix();
	
	
	vector<PointColors> getCLRS(Mat frame1);
};


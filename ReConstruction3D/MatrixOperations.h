#define _CRT_SECURE_NO_WARNINGS
#include "StructsTypes.h";

using namespace std;
using namespace cv;



class MxOperations {
private:
	Mat K;
public:
	MxOperations();

	Mat findMatrixK(Mat image);

	Mat_<double> FindEssentialMatrix(Mat F);

	

	bool DecomposeE(Mat_<double>& E,Mat_<double>& R1,Mat_<double>& t1);

	void FindProjectionMatrices(
		vector<KeyPoint> featurePoints1,
		vector<KeyPoint> featurePoints2,
		Mat F,
		Matx34d& P,
		Matx34d& P1
		
	);

	vector<SpacePoint> triangulation(vector<KeyPoint> keypoints1,
		vector<KeyPoint> keypoints2,
		Mat &K,
		Matx34d &P,
		Matx34d & P1,
		vector<SpacePoint> pointCloud);


	Mat_<double> IterativeTriangulation(Point3d u,Matx34d P,Point3d u1,	Matx34d P1);

	Mat_<double> LinearLSTriangulation(Point3d u,Matx34d P,	Point3d u1,	Matx34d P1);


};
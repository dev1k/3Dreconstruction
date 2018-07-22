#define _CRT_SECURE_NO_WARNINGS
#include <iostream> 
#include <string>   
#include <iomanip>  
#include <sstream>  
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp>      
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv/cv.h"

#include "StructsTypes.h";
#include "MatrixOperations.h";
#include "MakeCloud.h";

#include <math.h>

using namespace std;
using namespace cv;

# define M_PI         3.141592653589





MxOperations::MxOperations()
{

}

// essential matrix
Mat_<double> MxOperations::FindEssentialMatrix(Mat F) {
	Mat_<double> E = K.t() * F * K; 
	
	return E;
}

bool MxOperations::DecomposeE(Mat_<double>& E,Mat_<double>& R1,	Mat_<double>& t1)
{
    //разложение E методом SVD
	
	SVD svd(E, SVD::MODIFY_A);

	Matx33d W(0, -1, 0,	
		1, 0, 0,
		0, 0, 1);
	Matx33d Wt(0, 1, 0,
		-1, 0, 0,
		0, 0, 1);

	R1 = svd.u * Mat(W) * svd.vt; 
	t1 = -svd.u.col(2); 
	 

	return true;
}


Mat MxOperations::findMatrixK(Mat image)
{

	
	double pX = image.cols / 2;
	double pY = image.rows / 2;
	Mat C = (Mat_<double>(3, 3) << 1000, 0, pX, 0, 1000, pY, 0, 0, 1);
	K = C;

	

	return K;
}

void MxOperations::FindProjectionMatrices(
	vector<KeyPoint> featurePoints1,
	vector<KeyPoint> featurePoints2,
	Mat F,
	Matx34d& P,
	Matx34d& P1
	
)
{

	

	Mat_<double> R1(3, 3);
	Mat_<double> R2(3, 3);
	Mat_<double> t1(1, 3);
	Mat_<double> t2(1, 3);
	Matx33d tempik; //матрицы дл€ визуального просмотра данных
	Matx33d tempik2;
	F.copyTo(tempik);
	Mat_<double> E = FindEssentialMatrix(F);
	E.copyTo(tempik2);
	DecomposeE(E, R1, t1); 

	Mat Ptemp = (Mat_<double>(3, 4) << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);


	

	P = Matx34d(Ptemp);

	


	Mat P2s2 = (Mat_<double>(3, 4) << R1(0, 0), R1(0, 1), R1(0, 2), t1(0),
		R1(1, 0), R1(1, 1), R1(1, 2), t1(1),
		R1(2, 0), R1(2, 1), R1(2, 2), t1(2));

	P1 = Matx34d(P2s2);
	
}

vector<SpacePoint> MxOperations::triangulation(vector<KeyPoint> keypoints1,
	vector<KeyPoint> keypoints2,
	Mat &K,
	Matx34d &P,
	Matx34d & P1,
	vector<SpacePoint> pointCloud)
{
	Matx33d Kcheck, Kcheck2;
	K.copyTo(Kcheck);
	Mat kInverse = K.inv();
	kInverse.copyTo(Kcheck2);
	vector<SpacePoint> tempCloud = pointCloud;

	

	for (int i = 0; i < keypoints1.size(); i++)
	{
		KeyPoint keypoint1 = keypoints1.at(i);
		Point3d point3D1(keypoint1.pt.x, keypoint1.pt.y, 1);

		Mat_<double> mapping3D1 = kInverse * Mat_<double>(point3D1);
		point3D1.x = mapping3D1(0);
		point3D1.y = mapping3D1(1);
		point3D1.z = mapping3D1(2);

		KeyPoint keypoint2 = keypoints2.at(i);
		Point3d point3D2(keypoint2.pt.x, keypoint2.pt.y, 1);
		Mat_<double> mapping3D2 = kInverse * Mat_<double>(point3D2);
		point3D2.x = mapping3D2(0);
		point3D2.y = mapping3D2(1);
		point3D2.z = mapping3D2(2);

		

		Mat_<double> X = IterativeTriangulation(point3D1, P, point3D2, P1);

		

		SpacePoint Location3D;
		Location3D.point.x = X(0);
		Location3D.point.y = X(1);
		Location3D.point.z = X(2);

		tempCloud.push_back(Location3D);

	}

	pointCloud = tempCloud;

	
	
	cout << "–азмер облака точек :  " << pointCloud.size() << endl;
	

	return tempCloud;


}


Mat_<double> MxOperations::LinearLSTriangulation(Point3d u,	Matx34d P,		Point3d u1,	Matx34d P1)
{

	Matx43d A(u.x*P(2, 0) - P(0, 0), u.x*P(2, 1) - P(0, 1), u.x*P(2, 2) - P(0, 2),
		u.y*P(2, 0) - P(1, 0), u.y*P(2, 1) - P(1, 1), u.y*P(2, 2) - P(1, 2),
		u1.x*P1(2, 0) - P1(0, 0), u1.x*P1(2, 1) - P1(0, 1), u1.x*P1(2, 2) - P1(0, 2),
		u1.y*P1(2, 0) - P1(1, 0), u1.y*P1(2, 1) - P1(1, 1), u1.y*P1(2, 2) - P1(1, 2)
	);
	Matx41d B(-(u.x*P(2, 3) - P(0, 3)),
		-(u.y*P(2, 3) - P(1, 3)),
		-(u1.x*P1(2, 3) - P1(0, 3)),
		-(u1.y*P1(2, 3) - P1(1, 3)));

	Mat_<double> X;
	Matx31d ttt;
	solve(A, B, X, DECOMP_SVD);
	X.copyTo(ttt);
	return X;

}



Mat_<double> MxOperations::IterativeTriangulation(Point3d u,	Matx34d P,	Point3d u1,	Matx34d P1
)
{
	double wi = 1, wi1 = 1;
	Mat_<double> X(4, 1);

	for (int i = 0; i<2; i++) { 
		Mat_<double> X_ = LinearLSTriangulation(u, P, u1, P1);

		X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;

		

		double p2x = Mat_<double>(Mat_<double>(P).row(2)*X)(0);
		double p2x1 = Mat_<double>(Mat_<double>(P1).row(2)*X)(0);


		wi = p2x;
		wi1 = p2x1;
		
		Matx43d A((u.x*P(2, 0) - P(0, 0)) / wi, (u.x*P(2, 1) - P(0, 1)) / wi, (u.x*P(2, 2) - P(0, 2)) / wi,
			(u.y*P(2, 0) - P(1, 0)) / wi, (u.y*P(2, 1) - P(1, 1)) / wi, (u.y*P(2, 2) - P(1, 2)) / wi,
			(u1.x*P1(2, 0) - P1(0, 0)) / wi1, (u1.x*P1(2, 1) - P1(0, 1)) / wi1, (u1.x*P1(2, 2) - P1(0, 2)) / wi1,
			(u1.y*P1(2, 0) - P1(1, 0)) / wi1, (u1.y*P1(2, 1) - P1(1, 1)) / wi1, (u1.y*P1(2, 2) - P1(1, 2)) / wi1
		);

	
		Mat_<double> B = (Mat_<double>(4, 1) << -(u.x*P(2, 3) - P(0, 3)) / wi,
												-(u.y*P(2, 3) - P(1, 3)) / wi,
												-(u1.x*P1(2, 3) - P1(0, 3)) / wi1,
												-(u1.y*P1(2, 3) - P1(1, 3)) / wi1
						 );

		solve(A, B, X_, DECOMP_SVD);
		X(0) = X_(0);
		X(1) = X_(1);
		X(2) = X_(2);
		X(3) = 1.0;
	}

	return X;
}

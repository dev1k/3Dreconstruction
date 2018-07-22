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

using namespace std;
using namespace cv;

Matcher::Matcher() : ratio(0.65f),  confidence(0.99), distance(3.0)
{
	
	detector = new cv::SurfFeatureDetector();
	extractor = new cv::SurfDescriptorExtractor();
}

Mat Matcher::getFundamentalMatrix()
{
	return fundamentalMatrics;
}


void Matcher::setFeatureDetector(cv::Ptr<cv::FeatureDetector>& detect)
{
	detector = detect;
}

void Matcher::setDescriptorExtractor(cv::Ptr<cv::DescriptorExtractor>& desc)
{
	extractor = desc;
}

void Matcher::setConfidenceLevel(double confidenceIn)
{
	confidence = confidenceIn;
}

void Matcher::setMinDistanceToEpipolar(double distanceIn)
{
	distance = distanceIn;
}

void Matcher::setRatio(float ratioIn)
{
	ratio = ratioIn;
}



void Matcher::match(cv::Mat& image1,cv::Mat& image2,std::vector<cv::DMatch>& matches,
	std::vector<cv::KeyPoint>& keypoints1,	std::vector<cv::KeyPoint>& keypoints2)
{

	
	
	detector->detect(image1, keypoints1); //�������� �������� �����
	detector->detect(image2, keypoints2);
	
	cv::Mat descriptors1, descriptors2;
	extractor->compute(image1, keypoints1, descriptors1); //����������� ����������� �� ��������� �������� �����
	extractor->compute(image2, keypoints2, descriptors2);
	
	cv::BruteForceMatcher<cv::L2<float>> matcher;
	
	// BruteForceMatcher � k =2 �������� 	

	std::vector<std::vector<cv::DMatch>> matches1;
	matcher.knnMatch(descriptors1, descriptors2,matches1,2); 
	std::vector<std::vector<cv::DMatch>> matches2;
	matcher.knnMatch(descriptors2, descriptors1,matches2,2); 
	// ������� ���������� �  [0]/[1]>ratio
	   
	ratioTest(matches1);
	
	ratioTest(matches2);
	//������� �������������� ����������
	std::vector<cv::DMatch> symMatches;
	symmetryTest(matches1, matches2, symMatches);
	
	
	fundamentalMatrics = ransacTest(symMatches,
			keypoints1, keypoints2, matches);
		
	
	
	
}

void Matcher::ratioTest(std::vector<std::vector<cv::DMatch>> &matches)
{
	
	
	for (std::vector<std::vector<cv::DMatch>>::iterator
		matchIterator = matches.begin();
		matchIterator != matches.end(); ++matchIterator)
	{
		// ���� � ����� ������� ���� � 2�� �������
		if (matchIterator->size() > 1)
		{
			// �������� �� ���������
			if ((*matchIterator)[0].distance / (*matchIterator)[1].distance > ratio)
			{
				matchIterator->clear(); // ������� ���� ������� �����������
			
			}
		}
		else
		{ // � ��������� ������ ������ ������� ����������
			matchIterator->clear(); 
			
		}
	}
	
}


void Matcher::symmetryTest(
	const std::vector<std::vector<cv::DMatch>>& matches1,
	const std::vector<std::vector<cv::DMatch>>& matches2,
	std::vector<cv::DMatch>& symMatches)
{
	
	for (std::vector<std::vector<cv::DMatch>>::
		const_iterator matchIterator1 = matches1.begin();
		matchIterator1 != matches1.end(); ++matchIterator1)
	{
		
		if (matchIterator1->size() < 2)
			continue;
		
		for (std::vector<std::vector<cv::DMatch>>::
			const_iterator matchIterator2 = matches2.begin();
			matchIterator2 != matches2.end();
			++matchIterator2)
		{
			// ���������� ������ ����������
			if (matchIterator2->size() < 2)
				continue;
			//  symmetry test
			if ((*matchIterator1)[0].queryIdx ==
				(*matchIterator2)[0].trainIdx &&
				(*matchIterator2)[0].queryIdx ==
				(*matchIterator1)[0].trainIdx)
			{ 
				// ��������� ��������� ������������ ������
				symMatches.push_back(
					cv::DMatch((*matchIterator1)[0].queryIdx,
					(*matchIterator1)[0].trainIdx,
						(*matchIterator1)[0].distance));
				break; // next match in image 1 -> image 2
			}
		}
	}
}

// ���� ������� ���������� � ������� RANSAC � ��������� 8 �����
// ���������� fundemental matrix
cv::Mat Matcher::ransacTest(
	const std::vector<cv::DMatch>& symMatches,
	const std::vector<cv::KeyPoint>& keypoints1,
	const std::vector<cv::KeyPoint>& keypoints2,
	std::vector<cv::DMatch>& outMatches) {

	// ��������� ����� � Point2f
	std::vector<cv::Point2f> points1, points2;
	for (std::vector<cv::DMatch>::
		const_iterator it = symMatches.begin();
		it != symMatches.end(); ++it) {
		// ����� �������� ����� ��������
		float x = keypoints1[it->queryIdx].pt.x;
		float y = keypoints1[it->queryIdx].pt.y;
		points1.push_back(cv::Point2f(x, y));
		// ����� �������� ������ ��������
		x = keypoints2[it->trainIdx].pt.x;
		y = keypoints2[it->trainIdx].pt.y;
		points2.push_back(cv::Point2f(x, y));
	}
	//  RANSAC
	
	std::vector<uchar> inliers(points1.size(), 0);
	cv::Mat fundemental = cv::findFundamentalMat(cv::Mat(points1), cv::Mat(points2),inliers, CV_FM_RANSAC,distance,	confidence); 
			 
	Matx33f temp2;
	fundemental.copyTo(temp2);
	std::vector<uchar>::const_iterator
		itIn = inliers.begin();
	std::vector<cv::DMatch>::const_iterator
		itM = symMatches.begin();
	// for all matches
	for (;itIn != inliers.end(); ++itIn, ++itM) {
		if (*itIn) { // ���� �� 0 ���������
			outMatches.push_back(*itM);
		}
	}
	
		//���������� �������� 8 ����� ��� ���������
		points1.clear();
		points2.clear();
		for (std::vector<cv::DMatch>::
			const_iterator it = outMatches.begin();
			it != outMatches.end(); ++it) {
			
			float x = keypoints1[it->queryIdx].pt.x;
			float y = keypoints1[it->queryIdx].pt.y;
			points1.push_back(cv::Point2f(x, y));
			
			x = keypoints2[it->trainIdx].pt.x;
			y = keypoints2[it->trainIdx].pt.y;
			points2.push_back(cv::Point2f(x, y));
		}
		
		fundemental = cv::findFundamentalMat(cv::Mat(points1), cv::Mat(points2),CV_FM_8POINT); 
	
	return fundemental;
}
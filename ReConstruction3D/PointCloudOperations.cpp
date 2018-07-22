#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <string>  
#include <iomanip> 
#include <sstream> 
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp>        
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv/cv.h"
#include "PointCloudOperations.h"




void Table::init()
{

	table.resize(500); //определяет новый размер вектора
	numberEntry = 0;

}


int Table::tableSize()
{
	return numberEntry;
}

void Table::FormTable(vector<KeyPoint> Vector2Dpoints, vector<SpacePoint> Vector3Dpoints) //заполнение таблицы
{
	int size2d = Vector2Dpoints.size();
	int Point3Dcoords_Start = Vector3Dpoints.size() - size2d;

	


	for (int i = 0; i < size2d; i++)
	{
		Point2d *Point2Dcoords = (Point2d *)malloc(sizeof(Point2d));
		Point3d *Point3Dcoords = (Point3d *)malloc(sizeof(Point3d));

		Point2Dcoords->x = Vector2Dpoints.at(i).pt.x;
		Point2Dcoords->y = Vector2Dpoints.at(i).pt.y;

		Point3Dcoords->x = Vector3Dpoints.at(Point3Dcoords_Start + i).point.x;
		Point3Dcoords->y = Vector3Dpoints.at(Point3Dcoords_Start + i).point.y;
		Point3Dcoords->z = Vector3Dpoints.at(Point3Dcoords_Start + i).point.z;

		entryToTable(Point3Dcoords, Point2Dcoords);
	}

	

}

void Table::entryToTable(Point3d *Point3Dcoords, Point2d *Point2Dcoords)
{

	if (numberEntry >= table.size())
	{
		table.resize(500 + table.size());
	}

	Entry record;
	record.Point2Dcoords = Point2Dcoords;
	record.Point3Dcoords = Point3Dcoords;
	table[numberEntry] = record;
	numberEntry++;


}


Table::Table()
{
}

Point3d* Table::get3D(Point2d Point2Dcoords)
{


	int i;
	Point3d *p = NULL;

	for (i = 0; i < numberEntry; i++)
	{

		if (table[i].Point2Dcoords->x == Point2Dcoords.x && table[i].Point2Dcoords->y == Point2Dcoords.y)
		{
			p = table[i].Point3Dcoords;
			return p;
		}
	}

	return p;
}


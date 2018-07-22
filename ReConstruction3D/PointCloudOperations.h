#define _CRT_SECURE_NO_WARNINGS
#include <iostream> 
#include <string>  
#include <iomanip>  
#include <sstream>  
#include <vector>

#include "opencv/cv.h"
#include "StructsTypes.h"

using namespace std;
using namespace cv;


typedef struct STRUCTTYPE {
	Point2d *Point2Dcoords;
	Point3d *Point3Dcoords;
}Entry;


class Table {
public:
	Table();
	void init();
	void entryToTable(Point3d *Point3Dcoords, Point2d *Point2Dcoords);  // добавбление в конец списка 
	Point3d* get3D(Point2d Point2Dcoords);				//  получение 3ей координаты из 2ух
	void FormTable(vector<KeyPoint> Vector2Dpoints, vector<SpacePoint> Vector3Dpoints);
	int tableSize();

private:
	vector <Entry> table;				
	int numberEntry;

};

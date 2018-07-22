#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>
using namespace std;

class CloudBuilding
{
public:
	void WritePoint(double x, double y, double z, int b, int g, int r, int index);
	void Writeheader(int pointCloudSize, int index);

};

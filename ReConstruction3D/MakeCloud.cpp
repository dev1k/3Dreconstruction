#define _CRT_SECURE_NO_WARNINGS
#include <sstream>  
#include "MakeCloud.h"

void CloudBuilding::Writeheader(int pointCloudSize, int index)
{

	string number = static_cast<ostringstream*>(&(ostringstream() << index))->str();
	string name = "output" + number + ".ply";

	ofstream outfile(name);
	
	
		outfile << "ply\n";
		outfile << "format ascii 1.0\n";
		outfile << "element vertex " << pointCloudSize + 1 << "\n";
		outfile << "property float x\n";
		outfile << "property float y\n";
		outfile << "property float z\n";
		outfile << "property uchar red\n";
		outfile << "property uchar green\n";
		outfile << "property uchar blue\n";
		outfile << "end_header\n";

		outfile << "0 0 0 255 0 0\n";

		outfile.close();
	
	



}


void CloudBuilding::WritePoint(double x, double y, double z, int b, int g, int r, int index)
{

	string number = static_cast<ostringstream*>(&(ostringstream() << index))->str();
	string name = "output" + number + ".ply";

	ofstream outfile;
	outfile.open(name, ios::app);
	outfile << x << " ";
	outfile << y << " ";
	outfile << z << " ";
	outfile << b << " ";
	outfile << g << " ";
	outfile << r << endl;
	outfile.close();


}


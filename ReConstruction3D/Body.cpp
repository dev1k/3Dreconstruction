#define _CRT_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#include <iostream> 
#include <string>   
#include <iomanip>  
#include <sstream>  
#include <math.h>
#include <conio.h>

#include <string>
#include <experimental\filesystem>
#include <filesystem> 

#include <opencv2\imgproc\imgproc.hpp>  
#include <opencv2\core\core.hpp>        
#include <opencv2\highgui\highgui.hpp>  
#include <opencv2\nonfree\features2d.hpp>
#include <opencv2\legacy\legacy.hpp>
#include <opencv2\calib3d\calib3d.hpp>
#include <opencv\cv.h>

#include <pcl\point_types.h>
#include <pcl\io\pcd_io.h>
#include <pcl\io\ply_io.h>
#include <pcl\kdtree\kdtree_flann.h>
#include <pcl\features\normal_3d.h>
#include <pcl\surface\gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl\filters\radius_outlier_removal.h>

#include "StructsTypes.h"
#include "Match.h"
#include "MatchingPoints.h"
#include "MatrixOperations.h"
#include "MakeCloud.h"
#include "PointCloudOperations.h"

using namespace std::experimental::filesystem::v1;

using namespace cv;
using std::cout;

string getFileExt(string s);
void PCLwork(string filename);
void downsample(Mat *image);
Matx34d FindNewProjectionMxP1(Matx34d P1,
	vector<KeyPoint> newKeyPoints,
	vector<KeyPoint> oldKeyPoints,
	Table *current,
	Table *previous, Mat K);

int fileNumber=1;

int main(int argc, char *argv[]) {
	string imagesCount; //количество файлов
	string extension;   //расширение файлов

	setlocale(LC_ALL, "Russian");
	std::string path = argv[1];
	int numberPictures = 0;
	
	for (auto & pa2 : directory_iterator(path)) //подсчет количества изображений и их расширения
	{
		if (numberPictures == 1)
		{
			std::stringstream buffer;
			buffer << pa2;
			extension = getFileExt(buffer.str());
		}
		numberPictures++;
	}
	


	int pictureNumber1 = 1; //порядковый номер одной сравниваемой фотографии
	int pictureNumber2 = 2;  //порядковый номер второй


	string imageName1 = "1" + extension; //полное название одной картинки
	string imageName2 = "2" + extension;  //второй



	MxOperations MatrixOperations; //создаем экземпляр класса матрикскалк
	CloudBuilding CLoud3D; //создание модели .ply 

	vector<SpacePoint> pointCloud; 
	Matx34d P; // проекционные матрицы 3х4
	Matx34d P1;

	int LastStepAddedPoints = 0; //количетсво добавленных точек на предыдущем шаге

	bool FirstTime= true; //для первого прохождения цикла

	Table table1;
	Table table2;
	Table table3, table4; table3.init();table4.init();
	vector<PointColors> keyPointCLRS;
	vector<vector<PointColors>> keyPointCLRSALL;
	table2.init(); //инициализация, размер таблиц становится 500
	table1.init();
	Mat frame1, frame2;
	Table *current = &table1; //указатель на текущую таблицу
	Table *previous = &table2; //указатель на предыдущую
	bool f = true;//проверка на корректность введенных данных
	Mat K;
	cout << "|-----------------------------------------|" << endl;
	cout << "|  .... Построение модели началось....    |" << endl;
	cout << "|-----------------------------------------|" << endl;
	
	while (fileNumber < numberPictures)
	{
		frame1 = imread(path+"\\"+imageName1, -1); 
		frame2 = imread(path+"\\" + imageName2, -1);

		if ((frame1.rows == 0) || (frame2.rows == 0))
		{
			cout << "|-------------------------------------------|" << endl;
			cout << "|                                           |" << endl;
			cout << "|   Проверьте правильность введенных данных |" << endl;
			cout << "|                                           |" << endl;
			cout << "|-------------------------------------------|" << endl;
			f = false;
			system("pause");
			return 0;
		}

		downsample(&frame1); //функция сжатия
		downsample(&frame2);
		cout <<"Текущая пара изображений:"<<imageName1<<" и " << imageName2 << endl;
	

		
		MatchingPoints PointsMatcher(frame1, frame2);//сопоставление точек на 2ух фреймах
		vector<KeyPoint> keypoints1 = PointsMatcher.getKeyPoints1(); //затаскиваем фильтрованные ключевые точки первого фрема
		vector<KeyPoint> keypoints2 = PointsMatcher.getKeyPoints2(); //затаскиваем фильтрованные ключевые точки второго фрема
		
		keyPointCLRS=PointsMatcher.getCLRS(frame1); //раскрашиваем точки

		keyPointCLRSALL.push_back(keyPointCLRS);
			
			
			cout<< "Найдено " << keypoints1.size() << " совпадений" << endl;
			
			//добавление точек в облако

		//	
			if (FirstTime)
			{
				K = MatrixOperations.findMatrixK(frame1);
				MatrixOperations.FindProjectionMatrices(keypoints1, keypoints2, PointsMatcher.getFundamentalMatrix(), P, P1);

				
				pointCloud = MatrixOperations.triangulation(keypoints1, keypoints2, K, P, P1, pointCloud);
				(*current).FormTable(keypoints2, pointCloud);
				
				FirstTime= false;
			}
			else
			{   
				previous->init();
				previous = current;
				
				if (current == &table2)
				{
					current = &table1;
				}
				else
				{
					current = &table2;
				}
				P = P1; //так как картинки по порядку
				P1 = FindNewProjectionMxP1(P1, keypoints2, keypoints1, current, previous, K);
				pointCloud = MatrixOperations.triangulation(keypoints1, keypoints2, K, P, P1, pointCloud);
				current->FormTable(keypoints2, pointCloud);
			}

			int CurrentPointsAdded = keypoints1.size();
			
		
			fileNumber++;

			LastStepAddedPoints = CurrentPointsAdded + LastStepAddedPoints;

			cout << "Добавление точек завершено" << endl;
					
			pictureNumber1++;
			pictureNumber2++;
			auto s = std::to_string(pictureNumber1);
			auto s1 = std::to_string(pictureNumber2);
		char key = (char)waitKey(20); //приостановка для прорисовки

		
		
		
		imageName1 = s + extension;
		imageName2 =  s1 + extension;
		
		cout << "|--------------------------------------------|" << endl;
		cout << "|                                            |" << endl;
		cout << "|Обработка текущей пары изображений завершена|" << endl;
		cout << "|                                            |" << endl;
		cout << "|--------------------------------------------|" << endl;

		
		f = true;
	}

	

	CLoud3D.Writeheader(pointCloud.size(), fileNumber-1); //запись заголовка
	keyPointCLRS.clear();
	for (int afk = 0;afk < fileNumber-1;afk++)
	{
		for (int yu = 0;yu < keyPointCLRSALL[afk].size();yu++)
		{
			keyPointCLRS.push_back(keyPointCLRSALL[afk][yu]);
		}

	}
	//запись в файл точек
	for (int i = 0; i < pointCloud.size(); i++)
	{
		Point3d point = pointCloud.at(i).point;
		CLoud3D.WritePoint(point.x, point.y, point.z, keyPointCLRS[i].red, keyPointCLRS[i].green, keyPointCLRS[i].blue, fileNumber-1);

	}
	cout << ">>>> Облако точек записано в output" << fileNumber - 1 << ".ply <<<<" << endl;

	string ss = to_string(numberPictures - 1); //так как пар меньше на 1 чем изображений
	string lastPLYfile = "output" + ss + ".ply";
		PCLwork(lastPLYfile);//триангуляция
	
	if (f) {
		cout << "|--------------------------------------------|" << endl;
		cout << "|                                            |" << endl;
		cout << "|   .....:::::Процесс завершен:::::.....     |" << endl;
		cout << "|                                            |" << endl;
		cout << "|--------------------------------------------|" << endl;
		cout << " Нажмите любую кнопку чтобы завершить программу" << endl;
	}
	system("pause");
	
	return 0;
}

Matx34d FindNewProjectionMxP1(Matx34d P1,
	vector<KeyPoint> newKeyPoints,
	vector<KeyPoint> oldKeyPoints,
	Table *current,
	Table *previous,
	Mat K)
{
	Point3d *found;
	vector<Point2d> foundPoints2d;
	vector<Point3d> foundPoints3d;
	

	for (int i = 0; i < oldKeyPoints.size(); i++)
	{
		found = previous->get3D(oldKeyPoints.at(i).pt);

		if (found != NULL)
		{
			Point3d newPoint;
			newPoint.x = found->x;
			newPoint.y = found->y;
			newPoint.z = found->z;
			Point2d newPoint2;
			newPoint2.x = newKeyPoints.at(i).pt.x;
			newPoint2.y = newKeyPoints.at(i).pt.y;
			foundPoints3d.push_back(newPoint);
			foundPoints2d.push_back(newPoint2);
			current->entryToTable(&newPoint, &newPoint2);
		}
	}



	int size = foundPoints3d.size();

	

	Mat_<double> found3dPoints(size, 3);
	Mat_<double> found2dPoints(size, 2);

	for (int i = 0; i < size; i++)
	{

		found3dPoints(i, 0) = foundPoints3d.at(i).x;
		found3dPoints(i, 1) = foundPoints3d.at(i).y;
		found3dPoints(i, 2) = foundPoints3d.at(i).z;

		found2dPoints(i, 0) = foundPoints2d.at(i).x;
		found2dPoints(i, 1) = foundPoints2d.at(i).y;

	}

	Mat_<double> temp1(found3dPoints);
	Mat_<double> temp2(found2dPoints);

	Mat P(P1);
	Mat r(P, Rect(0, 0, 3, 3));
	Mat t(P, Rect(3, 0, 1, 3));
	Mat r_rog;
	cv::Rodrigues(r, r_rog);
	Mat dist = Mat::zeros(1, 4, CV_32F);
	double _dc[] = { 0, 0, 0, 0 };
	Mat(1, 4, CV_64FC1, _dc);
	cv::solvePnP(found3dPoints, found2dPoints, K, Mat(1, 4, CV_64FC1, _dc), r_rog, t, false);
	Mat_<double> R1(3, 3);
	Mat_<double> t1(t);
	cv::Rodrigues(r_rog, R1);
	Mat camera = (Mat_<double>(3, 4) << R1(0, 0), R1(0, 1), R1(0, 2), t1(0),
										R1(1, 0), R1(1, 1), R1(1, 2), t1(1),
										R1(2, 0), R1(2, 1), R1(2, 2), t1(2));
	return Matx34d(camera);
}




void downsample(Mat *image) //сжатие картинки пирамаидами Гаусса
{ 
	int maxRows = 1800; //устанавливаем макс колво строк
	int maxCols = 1600;  //стобцов
	Mat ImageModified = *image;  
	int height = ImageModified.rows; //высота переданной картинки
	int width = ImageModified.cols; //ширина

								  
	if (height % 2 != 0)
	{
		height--;
	}
	if (width % 2 != 0)
	{
		width--;
	}
	//формируем новые картинки
	Mat CurrentSize(ImageModified, Rect(0, 0, width - 1, height - 1)); //создали прямоугольник
	Mat downSize;
	while (height * width > maxRows * maxCols)  
	{
		pyrDown(CurrentSize, downSize, Size(width / 2, height / 2)); 
																 
		*image = downSize;
		
		height = downSize.rows;
		width = downSize.cols;

		if (height % 2 != 0)
		{
			height--;
		}
		if (width % 2 != 0)
		{
			width--;
		}
		Mat next(downSize, Rect(0, 0, width - 1, height - 1));
		CurrentSize = next;		
	}
}

string getFileExt(string s)
{
	std::string fn = s;
	fn = fn.substr(fn.find_last_of("."));

	return fn;
}

void PCLwork(string filename)
{
	cout << "|--------------------------------------------|" << endl;
	cout << "|                                            |" << endl;
	cout << "|       Начинается процесс триангуляции      |" << endl;
	cout << "|                                            |" << endl;
	cout << "|--------------------------------------------|" << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PLYReader Reader;
	
	
	Reader.read(filename, *cloud);
	/*
	//filter
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> rorfilter(true); // Initializing with true will allow us to extract the removed indices
	rorfilter.setInputCloud(cloud);
	rorfilter.setRadiusSearch(0.1);
	rorfilter.setMinNeighborsInRadius(3);
	rorfilter.setNegative(true);
	rorfilter.filter(*cloud_out);
	// The resulting cloud_out contains all points of cloud_in that have 4 or less neighbors within the 0.1 search radius
	//indices_rem = rorfilter.getRemovedIndices();
	*/

	//нормали
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	//объединили точки с нормалями
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	
	// kdtree
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud(cloud_with_normals);
	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
	pcl::PolygonMesh triangles;
	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.5);
	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);
	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);
	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
	pcl::io::savePLYFile("mesh.ply", triangles);


}
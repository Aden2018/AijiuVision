#ifndef COORDINATETRANSFORM_VISION_H
#define COORDINATETRANSFORM_VISION_H
//opencv2.4.9 vs2012
#include <opencv2\opencv.hpp>
#include <fstream>
#include <iostream>

using namespace std;
using namespace cv;

Point2f xyz2uv(Point3f worldPoint,float intrinsic[3][3],float translation[1][3],float rotation[3][3]);
Point3f uv2xyz(Point2f uvLeft,Point2f uvRight);
#endif // COORDINATETRANSFORM_VISION_H

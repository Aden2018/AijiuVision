#ifndef COORDINATETRANSFORM_VISION_H
#define COORDINATETRANSFORM_VISION_H
//opencv2.4.9 vs2012
#include <opencv2\opencv.hpp>
#include <fstream>
#include <iostream>

using namespace std;
using namespace cv;


//左相机内参数矩阵
extern float leftIntrinsic[3][3];
//左相机畸变系数
extern float leftDistortion[1][5];
//左相机旋转矩阵
extern float leftRotation[3][3];
//左相机平移向量
extern float leftTranslation[1][3];

//右相机内参数矩阵
extern float rightIntrinsic[3][3];
//右相机畸变系数
extern float rightDistortion[1][5];
//右相机旋转矩阵
extern float rightRotation[3][3];
//右相机平移向量
extern  float rightTranslation[1][3];


Point2f xyz2uv(Point3f worldPoint,float intrinsic[3][3],float translation[1][3],float rotation[3][3]);
Point3f uv2xyz(Point2f uvLeft,Point2f uvRight);
#endif // COORDINATETRANSFORM_VISION_H

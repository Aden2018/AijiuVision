#ifndef GLOBAL_H
#define GLOBAL_H

#include <QImage>
#include <QPixmap>
#include <QDebug>
#include<opencv2/opencv.hpp>
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;

Mat QImage2cvMat(QImage image);    //QImage-->Mat
QImage Mat2QImage(const Mat& mat); //Mat-->QImage
//颜色转换：RGB-->HSV
void RGB2HSV(double red, double green, double blue, double& hue, double& saturation, double& intensity);
//找出图像中红色的物体
Mat DetectRedTarget(Mat input);
vector<Point2f> GetTargetCoordinate(Mat in); //获取红色目标坐标
#endif // GLOBAL_H

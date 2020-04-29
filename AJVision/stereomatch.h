#ifndef STEREOMATCH_H
#define STEREOMATCH_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

const int imageWidth = 1920;                             //摄像头的分辨率
const int imageHeight = 1024;
//双目校正、立体匹配类
class StereoMatch
{
public:
    StereoMatch();
    void saveXYZ(const char* filename, const Mat& mat);
    void GenerateFalseMap(cv::Mat &src, cv::Mat &disp);
    Mat stereo_match(Mat rgbImageL, Mat rgbImageR,Mat &rectifyImageL,Mat &rectifyImageR,Mat &disp,Mat &xyz);
public:
    Mat rgbImageL, grayImageL;//
    Mat rgbImageR, grayImageR;//左边彩色图像，右边彩色图像
//    Mat rectifyImageL, rectifyImageR;

    Rect validROIL;//图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域
    Rect validROIR;

    Mat mapLx, mapLy, mapRx, mapRy;     //映射表
    Mat Rl, Rr, Pl, Pr, Q;              //校正旋转矩阵R，投影矩阵P 重投影矩阵Q
    Mat xyz;              //三维坐标

    Point origin;         //鼠标按下的起始点
    Rect selection;      //定义矩形选框
    bool selectObject = false;    //是否选择对象

  //  Size imageSize;// = Size(imageWidth, imageHeight);
    Ptr<StereoSGBM> sgbm;// = StereoSGBM::create(0, 16, 3);

    /*
    事先标定好的相机的参数
    fx 0 cx
    0 fy cy
    0 0  1
    */
    Mat cameraMatrixL; /*= (Mat_<double>(3, 3) << 4334.09568, 0, 959.50000,
        0, 4334.09568, 511.50000,
        0, 0, 1.0);*/
    Mat distCoeffL;// = (Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);

    Mat cameraMatrixR;/* = (Mat_<double>(3, 3) << 4489.55770, 0, 801.86552,
        0, 4507.13412, 530.72579,
        0, 0, 1.0);*/
    Mat distCoeffR;// = (Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);

    Mat T;// = (Mat_<double>(3, 1) << -518.97666, 01.20629, 9.14632);//T平移向量
    Mat rec;// = (Mat_<double>(3, 1) << 0.04345, -0.05236, -0.01810);//rec旋转向量
    Mat R;//R 旋转矩阵

};

#endif // STEREOMATCH_H

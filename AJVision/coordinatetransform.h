#ifndef COORDINATETRANSFORM_H
#define COORDINATETRANSFORM_H
//双目视觉坐标转换
#include <opencv2\opencv.hpp>
#include <fstream>
#include <iostream>

using namespace std;
using namespace cv;

Point2f xyz2uv(Point3f worldPoint,float intrinsic[3][3],float translation[1][3],float rotation[3][3]);
Point3f uv2xyz(Point2f uvLeft,Point2f uvRight);

/*
左相机内参数：
[7894.302084242289, 0, 1216.117060275077;
 0, 7893.980963601273, 974.4183691184483;
 0, 0, 1]
右相机内参数：
[7814.733093843312, 0, 1207.05438822664;
 0, 7813.5503046064, 978.1758114163888;
 0, 0, 1]
左相机畸变系数：
[-0.1204948302747099, 8.59226892990853, -0.001685124849829532, -0.001618605090039412, -292.2458695418636]
右相机畸变系数：
[0.1324023769580313, -13.7183505415937, -0.002134485429285803, -0.002917982804290732, 482.622663640412]
旋转矩阵：
[0.9978746882647606, 0.01591960681515056, 0.06318759877822494;
 -0.01495118133138012, 0.9997638633046674, -0.01576958486842791;
 -0.06342372345820696, 0.01479134033741945, 0.9978770703617352]
平移向量：
[-72.57675901257728;
 -0.2948569428772342;
 10.6640080096793]
*/
//左相机内参数矩阵
float leftIntrinsic[3][3] = {7894.302084242289, 0, 1216.117060275077,
                             0, 7893.980963601273, 974.4183691184483,
                             0,                 0,          1};
//左相机畸变系数
float leftDistortion[1][5] = {-0.1204948302747099, 8.59226892990853, -0.001685124849829532, -0.001618605090039412, -292.2458695418636};
//opencv标定完都是在左相机上建立世界坐标系
//左相机旋转矩阵
float leftRotation[3][3] = {1,0,0,0,1,0,0,0,1};
//左相机平移向量
float leftTranslation[3][1] = {0,0,0};

//右相机内参数矩阵
float rightIntrinsic[3][3] = {7814.733093843312, 0, 1207.05438822664,
                              0, 7813.5503046064, 978.1758114163888,
                              0, 0, 1};
//右相机畸变系数
float rightDistortion[1][5] = {0.1324023769580313, -13.7183505415937, -0.002134485429285803, -0.002917982804290732, 482.622663640412};
//右相机旋转矩阵
float rightRotation[3][3] = {  0.9978746882647606, 0.01591960681515056, 0.06318759877822494,
                             -0.01495118133138012, 0.9997638633046674 , -0.01576958486842791,
                             -0.06342372345820696, 0.01479134033741945, 0.9978770703617352};
//右相机平移向量
float rightTranslation[3][1] = {-72.57675901257728,
                                -0.2948569428772342,
                                10.6640080096793};

//int main()
//{
//    //已知空间坐标求成像坐标
//    Point3f point(700,220,530);
//    cout<<"左相机中坐标："<<endl;
//    cout<<xyz2uv(point,leftIntrinsic,leftTranslation,leftRotation)<<endl;
//    cout<<"右相机中坐标："<<endl;
//    cout<<xyz2uv(point,rightIntrinsic,rightTranslation,rightRotation)<<endl;

//    //已知左右相机成像坐标求空间坐标
//    Point2f l = xyz2uv(point,leftIntrinsic,leftTranslation,leftRotation);
//    Point2f r = xyz2uv(point,rightIntrinsic,rightTranslation,rightRotation);
//    Point3f worldPoint;
//    worldPoint = uv2xyz(l,r);
//    cout<<"空间坐标为:"<<endl<<uv2xyz(l,r)<<endl;

//    system("pause");

//    return 0;
//}


//************************************
// Description: 根据左右相机中成像坐标求解空间坐标
// Method:    uv2xyz
// FullName:  uv2xyz
// Access:    public
// Parameter: Point2f uvLeft
// Parameter: Point2f uvRight
// Returns:   cv::Point3f
// Author:    小白
// Date:      2017/01/10
// History:
//************************************
Point3f uv2xyz(Point2f uvLeft,Point2f uvRight)
{
    //  [u1]      |X|					  [u2]      |X|
    //Z*|v1| = Ml*|Y|					Z*|v2| = Mr*|Y|
    //  [ 1]      |Z|					  [ 1]      |Z|
    //			  |1|								|1|
    Mat mLeftRotation = Mat(3,3,CV_32F,leftRotation);
    Mat mLeftTranslation = Mat(3,1,CV_32F,leftTranslation);
    Mat mLeftRT = Mat(3,4,CV_32F);//左相机M矩阵
    hconcat(mLeftRotation,mLeftTranslation,mLeftRT);
    Mat mLeftIntrinsic = Mat(3,3,CV_32F,leftIntrinsic);
    Mat mLeftM = mLeftIntrinsic * mLeftRT;
    //cout<<"左相机M矩阵 = "<<endl<<mLeftM<<endl;

    Mat mRightRotation = Mat(3,3,CV_32F,rightRotation);
    Mat mRightTranslation = Mat(3,1,CV_32F,rightTranslation);
    Mat mRightRT = Mat(3,4,CV_32F);//右相机M矩阵
    hconcat(mRightRotation,mRightTranslation,mRightRT);
    Mat mRightIntrinsic = Mat(3,3,CV_32F,rightIntrinsic);
    Mat mRightM = mRightIntrinsic * mRightRT;
    //cout<<"右相机M矩阵 = "<<endl<<mRightM<<endl;

    //最小二乘法A矩阵
    Mat A = Mat(4,3,CV_32F);
    A.at<float>(0,0) = uvLeft.x * mLeftM.at<float>(2,0) - mLeftM.at<float>(0,0);
    A.at<float>(0,1) = uvLeft.x * mLeftM.at<float>(2,1) - mLeftM.at<float>(0,1);
    A.at<float>(0,2) = uvLeft.x * mLeftM.at<float>(2,2) - mLeftM.at<float>(0,2);

    A.at<float>(1,0) = uvLeft.y * mLeftM.at<float>(2,0) - mLeftM.at<float>(1,0);
    A.at<float>(1,1) = uvLeft.y * mLeftM.at<float>(2,1) - mLeftM.at<float>(1,1);
    A.at<float>(1,2) = uvLeft.y * mLeftM.at<float>(2,2) - mLeftM.at<float>(1,2);

    A.at<float>(2,0) = uvRight.x * mRightM.at<float>(2,0) - mRightM.at<float>(0,0);
    A.at<float>(2,1) = uvRight.x * mRightM.at<float>(2,1) - mRightM.at<float>(0,1);
    A.at<float>(2,2) = uvRight.x * mRightM.at<float>(2,2) - mRightM.at<float>(0,2);

    A.at<float>(3,0) = uvRight.y * mRightM.at<float>(2,0) - mRightM.at<float>(1,0);
    A.at<float>(3,1) = uvRight.y * mRightM.at<float>(2,1) - mRightM.at<float>(1,1);
    A.at<float>(3,2) = uvRight.y * mRightM.at<float>(2,2) - mRightM.at<float>(1,2);

    //最小二乘法B矩阵
    Mat B = Mat(4,1,CV_32F);
    B.at<float>(0,0) = mLeftM.at<float>(0,3) - uvLeft.x * mLeftM.at<float>(2,3);
    B.at<float>(1,0) = mLeftM.at<float>(1,3) - uvLeft.y * mLeftM.at<float>(2,3);
    B.at<float>(2,0) = mRightM.at<float>(0,3) - uvRight.x * mRightM.at<float>(2,3);
    B.at<float>(3,0) = mRightM.at<float>(1,3) - uvRight.y * mRightM.at<float>(2,3);

    Mat XYZ = Mat(3,1,CV_32F);
    //采用SVD最小二乘法求解XYZ
    solve(A,B,XYZ,DECOMP_SVD);

    //cout<<"空间坐标为 = "<<endl<<XYZ<<endl;

    //世界坐标系中坐标
    Point3f world;
    world.x = XYZ.at<float>(0,0);
    world.y = XYZ.at<float>(1,0);
    world.z = XYZ.at<float>(2,0);

    return world;
}

//************************************
// Description: 将世界坐标系中的点投影到左右相机成像坐标系中
// Method:    xyz2uv
// FullName:  xyz2uv
// Access:    public
// Parameter: Point3f worldPoint
// Parameter: float intrinsic[3][3]
// Parameter: float translation[1][3]
// Parameter: float rotation[3][3]
// Returns:   cv::Point2f
// Author:    小白
// Date:      2017/01/10
// History:
//************************************
Point2f xyz2uv(Point3f worldPoint,float intrinsic[3][3],float translation[1][3],float rotation[3][3])
{
    //    [fx s x0]							[Xc]		[Xw]		[u]	  1		[Xc]
    //K = |0 fy y0|       TEMP = [R T]		|Yc| = TEMP*|Yw|		| | = —*K *|Yc|
    //    [ 0 0 1 ]							[Zc]		|Zw|		[v]	  Zc	[Zc]
    //													[1 ]
    Point3f c;
    c.x = rotation[0][0]*worldPoint.x + rotation[0][1]*worldPoint.y + rotation[0][2]*worldPoint.z + translation[0][0]*1;
    c.y = rotation[1][0]*worldPoint.x + rotation[1][1]*worldPoint.y + rotation[1][2]*worldPoint.z + translation[0][1]*1;
    c.z = rotation[2][0]*worldPoint.x + rotation[2][1]*worldPoint.y + rotation[2][2]*worldPoint.z + translation[0][2]*1;

    Point2f uv;
    uv.x = (intrinsic[0][0]*c.x + intrinsic[0][1]*c.y + intrinsic[0][2]*c.z)/c.z;
    uv.y = (intrinsic[1][0]*c.x + intrinsic[1][1]*c.y + intrinsic[1][2]*c.z)/c.z;

    return uv;
}



#endif // COORDINATETRANSFORM_H

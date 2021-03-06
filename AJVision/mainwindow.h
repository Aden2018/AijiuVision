﻿#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMainWindow>
#include <QTimer>
#include <QDebug>
#include <QVector>
#include <QUdpSocket>
#include <QBuffer>
#include <QFile>
#include <QTime>
#include <QMessageBox>
#include <QException>
#include <QMouseEvent>
#include <vector>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/opencv.hpp>
#include "imagestittch.h"
#include "stereomatch.h"
#include "global.h"          //目标检测算法，使用算法计算左右相机图像同一点的坐标值
#include "transfrom.h"       //坐标转换

using namespace cv;
using namespace std;

const double LENGTH_PER_PIXEL =  400/273.0;//每像素转换成mm

//拍摄照片时x轴电机位置
const int FIRST_CAPTURE_POSITION   = 100;//第一张照片拍摄位置
const int SECOND_CAPTURE_POSITION  = 300;//第二张照片拍摄位置
const int THIRD_CAPTURE_POSITION   = 500;//第三张照片拍摄位置
const int FOURTH_CAPTURE_POSITION  = 700;//第四张照片拍摄位置


#pragma pack(1)  //指定一字节对齐
//治疗方案结构体
struct Cute_Solution
{
    int no;        //治疗顺序
    float x;      //中心点x坐标
    float y;      //中心点y坐标
    int cute;      //治疗方案：1:点按、2:画圆、3:雀琢
    int time;      //治疗时间，单位分钟
    Cute_Solution()
    {
        no = 0;
        x = 0.0;
        y = 0.0;
        cute = 1;
        time = 0;
    }
};

#pragma pack()  //取消指定对齐，恢复缺省对齐

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
protected:
//     void mousePressEvent(QMouseEvent *event);//鼠标点击事件
//     void mouseMoveEvent(QMouseEvent *event); //鼠标移动事件
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    Mat imageRotate90(Mat src);//选择90度
    Mat imageRatateNegative90(Mat src);//选择-90度
    Mat ImageStitchByHconcat(Mat left,Mat right,int nums); //横向拼接
    Mat ImageStitchByVconcat(Mat left,Mat right,int nums);//纵向拼接
    void ShowImage(Mat srcImage);//显示图像

    //将二维坐标转换成三维坐标
    Vec3f xy2XYZ(Mat imgLeft,Mat imgRight,int x,int y);
    //计算视差图
    bool computeDisparityImage(Mat img1, Mat img2, Mat& img1_rectified,
        Mat& img2_rectified, Mat& mapl1, Mat& mapl2, Mat& mapr1, Mat& mapr2, Rect validRoi[2], Mat& disparity);

    //立体校正
    void stereoRectification(Mat& cameraMatrix1, Mat& distCoeffs1, Mat& cameraMatrix2, Mat& distCoeffs2,
        Size& imageSize, Mat& R, Mat& T, Mat& R1, Mat& R2, Mat& P1, Mat& P2, Mat& Q, Mat& mapl1, Mat& mapl2, Mat& mapr1, Mat& mapr2);

    //读取相机标定参数，包括内参、外参
    void ReadParameters();
       //发送图片
    void SendPictureByUdp(QString path,QString ip,qint16 port);
    //添加内容到列表
    void addToList(QString str);
    //初始化串口
    void InitialCom();
    //串口接收数据校验
    unsigned char SerialCheckSum(unsigned char *buf, unsigned char len);
    //处理串口数据包
    void processSerialBuffer(const char* data);

    Mat jiaozheng( Mat image,Mat intrinsic_matrix,Mat distortion_coeffs );

    //计算校验值
    unsigned char jiaoyan(unsigned char* data);
    //电机复位
    void SendMotorResetCmd(int id);
    //电机运动
    void SendMotorRunCmd(int id,int speed,int pos);
    //温度控制
    void SendTempretureControlCmd(int value);
    //动作控制指令帧，画圆
    void SendHuayuanCmd(bool bEnable,int speed,int radius);
    //动作控制指令帧，雀琢
    void SendQuezuoCmd(bool bEnable,int speed,int pos,int maxResrved);
    //故障帧，不定时，有故障即发 (有)
    void SendFaultCmd(bool bFault);
    //实时温度级别帧，1Hz
    void SendTempretureLevelCmd(int level);

    //Mat转QImage
    QImage mat2QImage(Mat srcImg);

public slots:
    void handleTimeout();  //超时处理函数
    void handleTimeout1();
    void handleTimeout2();
    void handleTimeout3();
    void handleTimeout4();

    void Read_Data();      //串口读写类
private slots:
    void on_pushButton_clicked();
    void slotUDPReadyRead();
    void on_deleteAll_clicked();

    void on_saveButton_clicked();

    void on_checkBox_clicked(bool checked);

    void on_checkBox_2_clicked(bool checked);

    void on_moveButton_clicked();

    void on_resetButton_clicked();

    void on_pushButton_capture_clicked();

private:
    Ui::MainWindow *ui;

    QUdpSocket *mSocket;    //UDP

    QTimer *timer;          //定时器
    QTimer *timer_fisrt;           //定时器，拍摄第一张图片
    QTimer *timer_second;          //定时器，拍摄第二张图片
    QTimer *timer_third;           //定时器，拍摄第三张图片
    QTimer *timer_fourth;          //定时器，拍摄第四张图片
    QSerialPort *serial;    //串口
    QVector<QString> vComs;

    VideoCapture capture;//摄像头
    Mat frame;

    Mat cameraMatrix_L;// = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 相机的内参数
    Mat cameraMatrix_R;// = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 初始化相机的内参数
    Mat distCoeffs_L;// = Mat(1, 5, CV_32FC1, Scalar::all(0)); // 相机的畸变系数
    Mat distCoeffs_R;// = Mat(1, 5, CV_32FC1, Scalar::all(0)); // 初始化相机的畸变系数
    Mat R, T, E, F; // 立体标定参数
    Mat R1, R2, P1, P2, Q; // 立体校正参数
    Mat mapl1, mapl2, mapr1, mapr2; // 图像重投影映射表
    Mat img1_rectified, img2_rectified, disparity, result3DImage; // 校正图像 视差图 深度图
    Size imageSize; // 图像尺寸
    Rect validRoi[2];

    bool m_bSaveImage;
    bool m_bShowImage;
    bool m_bRotateImage;

    Cute_Solution cuteArray[50];//治疗方案
    int acupointNum;            //穴位数目


    Mat distortion;//矫正后的图像

    StereoMatch *pStereoMatch;

};


extern bool x_step_finish;     //X轴步进电机是否完成
extern bool y_step_finish;     //Y轴步进电机是否完成
extern unsigned short coarse_x;//x轴电机位置
extern unsigned short coarse_y;//y轴电机位置
extern unsigned short coarse_z;//z轴电机位置
extern bool bUseEnableShutdown;//使能停机标志
extern bool bDistanceDone;     //距离完成标志
extern bool bTimeUseup;     //时间耗尽标志
extern bool bToAppiontPos;     //到达指定位置标志
extern bool bToNegativeLimit;  //到达负限位标志
extern bool bDirectionError;  //方向错误标志
extern bool bToLimit;          //到达限位标志

#endif // MAINWINDOW_H

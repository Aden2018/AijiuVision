#ifndef MAINWINDOW_H
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
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "imagestittch.h"

using namespace cv;

//状态宏定义
#define START 0 // 治疗初始状态
#define COARSE_MOVE 1 //粗定位过程
#define PRECISE_MOVE 2 //精定位过程
#define WAIT_FOR_TREATMENT 3 //等待治疗
#define IN_TREATMENT 4 //治疗中
#define TIME_OUT 5 // 时间耗尽
#define PAUSE 6 //治疗暂停
#define DC_MOTOR_DOWN 7 // 直流电机向下运动中
#define DC_MOTOR_UP 8 // 直流电机向上运动中
#define PRECISE_FINISH 9 // 精定位完成
#define MOVE_TO_POS 10 // 精定位完成后，艾灸头移动到穴位
#define STOP 11 // 结束治疗
#define WAIT_FOR_RESET 12 // 等待复位完成
#define WAIT_FOR_START 13 //等待开始完成

struct ImageFrameHead {
    unsigned int funCode;               //功能码，用于区分数据包中的内容
    unsigned int uTransFrameHdrSize;    //sizeof(ImageFrameHead)，数据包头的大小
    unsigned int uTransFrameSize;       //Data Size，数据包中数据的大小
    //数据帧变量
    unsigned int uDataFrameSize;        //图片数据总大小
    unsigned int uDataFrameTotal;       //图片数据被分数据帧个数
    unsigned int uDataFrameCurr;        //数据帧当前的帧号
    unsigned int uDataInFrameOffset;    //数据帧在整帧的偏移
};

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    Mat imageRotate90(Mat src);//选择90度
    Mat imageRatateNegative90(Mat src);//选择-90度

    //计算视差图
    bool computeDisparityImage(const char* imageName1, const char* imageName2, Mat& img1_rectified,
        Mat& img2_rectified, Mat& mapl1, Mat& mapl2, Mat& mapr1, Mat& mapr2, Rect validRoi[2], Mat& disparity);

    //立体校正
    Rect stereoRectification(Mat& cameraMatrix1, Mat& distCoeffs1, Mat& cameraMatrix2, Mat& distCoeffs2,
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

public slots:
    void handleTimeout();  //超时处理函数
    void Read_Data();      //串口读写类
private slots:
    void on_pushButton_clicked();
    void read_data();
    void on_deleteAll_clicked();
    void on_deleteSingleItem_clicked();

private:
    Ui::MainWindow *ui;

    QUdpSocket *mSocket;    //UDP

    QTimer *timer;          //定时器
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
};

#endif // MAINWINDOW_H

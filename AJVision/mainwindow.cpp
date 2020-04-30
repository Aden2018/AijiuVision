#include "mainwindow.h"
#include "ui_mainwindow.h"

bool x_step_finish = false;//X轴步进电机是否完成
bool y_step_finish = false;//Y轴步进电机是否完成
unsigned short coarse_x = 0;//x轴电机位置
unsigned short coarse_y = 0;//y轴电机位置
unsigned short coarse_z = 0;//z轴电机位置
bool bUseEnableShutdown = false;//使能停机标志
bool bDistanceDone = false;     //距离完成标志
bool bTimeUseup    = false;     //时间耗尽标志
bool bToAppiontPos = false;     //到达指定位置标志
bool bToNegativeLimit = false;  //到达负限位标志
bool bDirectionError  = false;  //方向错误标志
bool bToLimit = false;          //到达限位标志

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->setMouseTracking(false);//设置窗体可响应 Mouse Move

    pStereoMatch = new StereoMatch();//初始化，加载内参、外参以及其他相关参数

    m_bSaveImage = false;
    m_bRotateImage = false;

    QStringList text;
    text<<tr("SURF")<<tr("ORB")<<tr("SIFT");
    ui->comboBox_Algorihm->addItems(text);
    ui->listWidget->setViewMode(QListView::ListMode);   //设置显示模式为列表模式

    //udp初始化
    mSocket = new QUdpSocket(this);
    connect(mSocket,SIGNAL(readyRead()),this,SLOT(slotUDPReadyRead()));
    qDebug()<< mSocket->bind(QHostAddress::Any, 65520);

    //初始化定时器
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(handleTimeout()));
    timer->start(500);

    InitialCom();//初始化串口

    ReadParameters();
}

Mat MainWindow::jiaozheng( Mat image,Mat intrinsic_matrix,Mat distortion_coeffs )
{
    Size image_size = image.size();
    Mat R = Mat::eye(3,3,CV_32F);
    Mat mapx = Mat(image_size,CV_32FC1);
    Mat mapy = Mat(image_size,CV_32FC1);
    Mat new_intrinsic_matrix;
    initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,R,new_intrinsic_matrix,image_size,CV_32FC1,mapx,mapy);
    Mat t = image.clone();
    cv::remap( image, t, mapx, mapy, INTER_LINEAR);
    return t;

}

//鼠标点击事件
 void MainWindow::mousePressEvent(QMouseEvent *event)
 {
     if(event->button()==Qt::LeftButton)
     {

         if(!distortion.data)
             return;
         int ImageWidth = distortion.cols;
         int ImageHeight = distortion.rows;

         QPoint viewPoint = event->globalPos();  //获取全局位置
         viewPoint = ui->label->mapFromGlobal(viewPoint);

         float w = ui->label->contentsRect().width();
         float h = ui->label->contentsRect().height();
         int col = int(viewPoint.x() * (ImageWidth / w));  //获取当前Label的坐标，按比例缩放为图片坐标
         int row = int(viewPoint.y() * (ImageHeight / h));

         if (row > 0 && row < ImageHeight && col > 0 && col < ImageWidth) //超出图像范围不显示坐标
         {
             QString strPosX = QString::number(col);
             QString strPosY = QString::number(row);

             cvtColor(distortion, distortion, CV_BGR2RGB);
             QImage img_2 = QImage((const unsigned char*)distortion.data, // uchar* data
                 distortion.cols, distortion.rows, distortion.step, QImage::Format_RGB888);

             QRgb rgbPixel = img_2.pixel(col,row); //读入的图像为灰度图，任意通道像素值一样
             int gray = qRed(rgbPixel);
             QString strGray = QString::number(gray);

             QString strShowData;// = strPosX + "," + strPosY + "," + strGray;
             strShowData.append("x:");
             strShowData.append(strPosX);

             strShowData.append(",");
             strShowData.append("y:");
             strShowData.append(strPosY);

             strShowData.append(",");
             strShowData.append("灰度:");
             strShowData.append(strPosX);

             addToList(strShowData);
         }
     }
     else if(event->button()==Qt::RightButton)
     {
         qDebug()<<"RightButton clicked!";

     }
 }

// //第二步：移动鼠标获取图像坐标和灰度值（按下左键移动鼠标）
// void MainWindow::mouseMoveEvent(QMouseEvent *event)
// {

// }

MainWindow::~MainWindow()
{
    if(pStereoMatch)
        delete pStereoMatch;
    //关闭相机
    if(capture.isOpened())
    {
        capture.release();
    }

    //关闭定时器
    if(timer)
    {
       if(timer->isActive())
       {
           timer->stop();
       }

       delete timer;
    }

    //关闭udp
    if(mSocket)
        delete mSocket;

    //关闭串口
    if(serial)
    {
        if(serial->isOpen())
        {
            serial->close();//关闭串口
        }

        delete serial;
    }


    delete ui;
}

Vec3f  MainWindow::xy2XYZ(Mat imgLeft,Mat imgRight, int x,int y)
{
    //视差图建立
    computeDisparityImage(imgLeft, imgRight, img1_rectified, img2_rectified, mapl1, mapl2, mapr1, mapr2, validRoi, disparity);
    // 从三维投影获得深度映射
    reprojectImageTo3D(disparity, result3DImage, Q);

    Point point;
    point.x = x;
    point.y = y;
    Vec3f xyz = result3DImage.at<Vec3f>(point);

    return xyz;
}

void MainWindow::InitialCom()
{
    serial = new QSerialPort;
    serial->setPortName("COM1");      //设置串口名
    serial->open(QIODevice::ReadWrite);      //打开串口
    serial->setBaudRate(9600);      //设置波特率
    serial->setDataBits(QSerialPort::Data8);      //设置数据位数
    serial->setParity(QSerialPort::NoParity);        //设置奇偶校验
    serial->setStopBits(QSerialPort::OneStop);      //设置停止位
    serial->setFlowControl(QSerialPort::NoFlowControl);    //设置流控制
    connect(serial,SIGNAL(readyRead()),this,SLOT(Read_Data()));
}

void MainWindow::addToList(QString str)
{
    QListWidgetItem *item = new QListWidgetItem;
    item->setText(str);                     //设置列表项的文本
    ui->listWidget->addItem(item);          //加载列表项到列表框
}

void MainWindow::slotUDPReadyRead()
{
    while(mSocket->hasPendingDatagrams())  //有未处理的报文
    {
        QByteArray recvMsg;
        qint64 length = mSocket->pendingDatagramSize();//接收数据包长度
        int size = sizeof (Cute_Solution);

        acupointNum = static_cast<int>(length/size);

        recvMsg.resize(static_cast<int>(length));

        mSocket->readDatagram(recvMsg.data(),recvMsg.size());

        if(acupointNum > 0)
        {
            memcpy((char*)cuteArray,recvMsg.data(),static_cast<size_t>(length));

            static int cnt = 0;
            cnt++;

            for(int i=0;i<acupointNum;i++)
            {
                qDebug()<< cnt << ":" << i+1 << "," << cuteArray[i].no << "," << cuteArray[i].x << "," << cuteArray[i].y << "," << cuteArray[i].cute << "," << cuteArray[i].time << endl;
            }
        }

        //像素坐标转三维坐标
        /*
        //左右相机像素坐标
        Point2f l;
        l.x = cuteArray[0].x;
        l.y = cuteArray[0].y;

        Mat image_r;//右相机图像

        //右相机像素坐标
        vector<Point2f> targetPoints = GetTargetCoordinate(image_r);
        Point2f r;
        r.x = targetPoints[0].x;
        r.y = targetPoints[0].y;

        Point3f worldPoint = uv2xyz(l,r);
        */

    }

}


void MainWindow::handleTimeout()
{
    if(!m_bShowImage)
        return;
    if(capture.isOpened())
    {
        capture >> frame;


        Rect rectLeft(0, 0, frame.cols / 2, frame.rows);
        Rect rectRight(frame.cols / 2, 0, frame.cols / 2, frame.rows);
        Mat image_l = Mat(frame, rectLeft);
        Mat image_r = Mat(frame, rectRight);



        if(m_bRotateImage)
        {
            image_l = imageRatateNegative90(image_l);
            image_r = imageRatateNegative90(image_r);
        }

      //  imshow("left",jiaozheng(image_l,cameraMatrix_L,distCoeffs_L));
     //   imshow("right",jiaozheng(image_r,cameraMatrix_R,distCoeffs_R));

        imshow("left",image_l);
        imshow("right",image_r);

       if(m_bSaveImage)
       {
           static int cnt = 0;
           cnt++;

           m_bSaveImage = false;

           cvtColor(distortion, distortion, CV_RGB2BGR);
           string path1 = std::to_string(cnt) + ".jpg";
           imwrite(path1, distortion);

           string path2 = std::to_string(cnt+25) + ".jpg";
           imwrite(path2, image_r);
       }
    }
    else
    {
        capture.open(0);
        capture.set(CV_CAP_PROP_FRAME_WIDTH,  640);//
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    }


//            if(!capture.isOpened())
//                 capture.open(0);
//            Mat frame,srcmat2,gray,dst_x,dst_y,dst;
//            capture>>frame;
//         //   imshow("origin", frame);
//            cvtColor(frame, srcmat2, CV_BGR2RGB);
//            cvtColor(frame, gray, CV_BGR2GRAY);
//            //sobel 边缘检测
//            Sobel(gray, dst_x, gray.depth(), 1, 0);
//            Sobel(gray, dst_y, gray.depth(), 0, 1);
//            convertScaleAbs(dst_x, dst_x);
//            convertScaleAbs(dst_y, dst_y);
//            addWeighted(dst_x, 0.5, dst_y, 0.5, 0, dst);
//        //    imshow("dst", dst);
//            //原始图像的显示
//            QImage img = QImage((const unsigned char*)srcmat2.data, // uchar* data
//                srcmat2.cols, srcmat2.rows, srcmat2.step, QImage::Format_RGB888);//格式转换
//            int a = ui->label->width();
//            int b = ui->label->height();
//            QImage ime = img.scaled(a, b);//自定义缩放
//            ui->label->setPixmap(QPixmap::fromImage(ime));
//            ui->label->show();
//            //处理后图像的显示
//            QImage img_2 = QImage((const unsigned char*)dst.data, // uchar* data
//                dst.cols, dst.rows, dst.step, QImage::Format_Grayscale8);

//            int a2 = ui->label->width();
//            int b2 = ui->label->height();
//            QImage imgg = img_2.scaled(a2, b2);
//            ui->label->setPixmap(QPixmap::fromImage(imgg));
//            ui->label->show();


}

void MainWindow::ShowImage(Mat srcImage)
{
    QImage img;
    cvtColor(srcImage, srcImage, CV_BGR2RGB);
    img = QImage((const unsigned char*)srcImage.data, // uchar* data
                 static_cast<int>(srcImage.cols), static_cast<int>(srcImage.rows), static_cast<int>(srcImage.step), QImage::Format_RGB888);


//   int a2 = ui->label->width();
//   int b2 = ui->label->height();
//   QImage imgg = img.scaled(a2, b2);
    ui->label->setPixmap(QPixmap::fromImage(img));
    ui->label->show();
}

void MainWindow::Read_Data()//串口读取函数
{
    QByteArray buf;
    buf = serial->readAll();
    //QByteArray 转换为 char *
    char *rxData;//不要定义成ch[n];
    rxData = buf.data();
    //int size = buf.size();

    processSerialBuffer(rxData);

}

//顺时针旋转90度
Mat MainWindow::imageRotate90(Mat src)
{
    Mat temp,dst;
    transpose(src, temp);
    flip(temp, dst, 1); //旋转90度

    return dst;
}

//逆时针旋转90度
Mat MainWindow::imageRatateNegative90(Mat src)
{
    Mat temp, dst;
    transpose(src, temp);
    flip(temp, dst, 0); //旋转-90度

    return dst;
}

//读取相机标定参数，包括内参、外参
void MainWindow::ReadParameters()
{
    cv::FileStorage fs("calibParams.yml", cv::FileStorage::READ);
    if (fs.isOpened())
    {
        fs["cameraMatrix_L"] >> cameraMatrix_L;
        fs["distCoeffs_L"] >> distCoeffs_L;
        fs["cameraMatrix_R"] >> cameraMatrix_R;
        fs["distCoeffs_R"] >> distCoeffs_R;
        fs["R"] >> R;
        fs["T"] >> T;
        fs["imageSize"] >> imageSize;
        fs.release();

//        cout << cameraMatrix_L << endl;
//        cout << distCoeffs_L << endl;
//        cout << cameraMatrix_R << endl;
//        cout << distCoeffs_R << endl;
//        cout << R << endl;
//        cout << T << endl;
    }
}

/*
立体校正
参数：
    cameraMatrix			相机内参数
    distCoeffs				相机畸变系数
    imageSize				图像尺寸
    R						左右相机相对的旋转矩阵
    T						左右相机相对的平移向量
    R1, R2					行对齐旋转校正
    P1, P2					左右投影矩阵
    Q						重投影矩阵
    map1, map2				重投影映射表
*/
void MainWindow::stereoRectification(Mat& cameraMatrix1, Mat& distCoeffs1, Mat& cameraMatrix2, Mat& distCoeffs2,
    Size& imageSize, Mat& R, Mat& T, Mat& R1, Mat& R2, Mat& P1, Mat& P2, Mat& Q, Mat& mapl1, Mat& mapl2, Mat& mapr1, Mat& mapr2)
{
    Rect validRoi[2];

    stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize,
        R, T, R1, R2, P1, P2, Q, 0, -1, imageSize, &validRoi[0], &validRoi[1]);

    initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize, CV_32FC1, mapl1, mapl2);
    initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize, CV_32FC1, mapr1, mapr2);
  //  return validRoi[0], validRoi[1];
}

/*
计算视差图
参数：
    imageName1	左相机拍摄的图像
    imageName2	右相机拍摄的图像
    img1_rectified	重映射后的左侧相机图像
    img2_rectified	重映射后的右侧相机图像
    map	重投影映射表
*/
bool MainWindow::computeDisparityImage(Mat img1, Mat img2, Mat& img1_rectified,
    Mat& img2_rectified, Mat& mapl1, Mat& mapl2, Mat& mapr1, Mat& mapr2, Rect validRoi[2], Mat& disparity)
{
    // 首先，对左右相机的两张图片进行重构
//    Mat img1 = imread(imageName1);
//    Mat img2 = imread(imageName2);
    if (img1.empty() | img2.empty())
    {
        qDebug() << "图像为空" << endl;
        return false;
    }
    Mat gray_img1, gray_img2;
    cvtColor(img1, gray_img1, COLOR_BGR2GRAY);
    cvtColor(img2, gray_img2, COLOR_BGR2GRAY);
    imshow("gray_img1",gray_img1);
    imshow("gray_img2",gray_img2);
    Mat canvas(imageSize.height, imageSize.width * 2, CV_8UC1); // 注意数据类型
    Mat canLeft = canvas(Rect(0, 0, imageSize.width, imageSize.height));
    Mat canRight = canvas(Rect(imageSize.width, 0, imageSize.width, imageSize.height));
  //  gray_img1.copyTo(canLeft);
  //  gray_img2.copyTo(canRight);
 //   imshow("canvas",canvas);
    imwrite("校正前左右相机图像.jpg", canvas);
    remap(gray_img1, img1_rectified, mapl1, mapl2, INTER_LINEAR);
    remap(gray_img2, img2_rectified, mapr1, mapr2, INTER_LINEAR);
    imwrite("左相机校正图像.jpg", img1_rectified);
    imwrite("右相机校正图像.jpg", img2_rectified);
 //   imshow("img1_rectified",img1_rectified);
  //  imshow("img2_rectified",img2_rectified);
    img1_rectified.copyTo(canLeft);
    img2_rectified.copyTo(canRight);
    rectangle(canLeft, validRoi[0], Scalar(255, 255, 255), 5, 8);
    rectangle(canRight, validRoi[1], Scalar(255, 255, 255), 5, 8);
    for (int j = 0; j <= canvas.rows; j += 16)
        line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
    imwrite("校正后左右相机图像.jpg", canvas);
    // 进行立体匹配
    Ptr<StereoBM> bm = StereoBM::create(16, 9); // Ptr<>是一个智能指针
    bm->compute(img1_rectified, img2_rectified, disparity); // 计算视差图
  //  imshow("img1_rectified",img1_rectified);
 //   imshow("img2_rectified",img2_rectified);
    disparity.convertTo(disparity, CV_32F, 1.0 / 16);
    // 归一化视差映射
    normalize(disparity, disparity, 0, 256, NORM_MINMAX, -1);
    return true;
}
//两幅图片横向拼接
/*
  参数：
    left	左边图像
    right	右边图像
    roi	右边图像拼接区域
  返回：
   mat 拼接之后的图像
*/
Mat MainWindow::ImageStitchByHconcat(Mat left,Mat right,Rect  roi)
{
    Mat dst;
  //   Rect rectRight(cols, 0, right.cols - cols, right.rows);
    right = Mat(right,roi);

    hconcat(left,right,dst);
    //         vconcat（B,C，A）; // 等同于A=[B  C] 横向拼接
    //         hconcat（B,C，A）; // 等同于A=[B  C] 纵向拼接
    return dst;
}
void MainWindow::on_pushButton_clicked()
{
    try {
         Mat dst;
         Mat right = imread("d:\\2.jpg");
         Mat left  = imread("d:\\1.jpg");

         Rect rectLeft(0, 0, left.cols / 2, left.rows);
         Rect rectRight(right.cols / 2, 0, right.cols / 2, right.rows);
         Mat image_l = Mat(left, rectLeft);
         Mat image_r = Mat(right, rectRight);

         ShowImage(ImageStitchByHconcat(left,right,rectRight));

//         return;
//         if(ui->comboBox_Algorihm->currentIndex()==0)
//         {
//             dst = StittchBySurf(right,left);
//         }
//         else if(ui->comboBox_Algorihm->currentIndex()==1)
//         {
//             dst = StitchImageByOrb(right,left);
//         }
//         else if(ui->comboBox_Algorihm->currentIndex()==2)
//         {
//             dst = StitchImageBySift(right,left);
//         }

        //   imshow("dst",dst);

    } catch (QString exception) {
        QMessageBox::about(this,"Error",exception);
    }
}

void MainWindow::SendPictureByUdp(QString path,QString ip,qint16 port)
{
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

    while(true) {
        QFile file(path);
        if (!file.open(QIODevice::ReadOnly))
            return;
        char *m_sendBuf = new char[1024];

        int size = file.size();
        int num = 0;
        unsigned int count = 0;
        int endSize = size%996;//996=1024-sizeof(ImageFrameHead)
        if (endSize == 0) {
            num = size/996;
        }
        else {
            num = size/996+1;
        }


        while (count < num) {
            memset(m_sendBuf, 0, 1024);

            ImageFrameHead mes;
            mes.funCode = 24;
            mes.uTransFrameHdrSize = sizeof(ImageFrameHead);
            if ((count+1) != num) {
                mes.uTransFrameSize = 996;
            }
            else {
                mes.uTransFrameSize =static_cast<unsigned int>(endSize);
            }
            //qDebug()<<size;
            mes.uDataFrameSize  = static_cast<unsigned int>(size);
            mes.uDataFrameTotal = static_cast<unsigned int>(num);
            mes.uDataFrameCurr = count+1;
            mes.uDataInFrameOffset = count*(1024 - sizeof(ImageFrameHead));

            file.read(m_sendBuf+sizeof(ImageFrameHead), 1024-sizeof(ImageFrameHead));

            memcpy(m_sendBuf, (char *)&mes, sizeof(ImageFrameHead));
            mSocket->writeDatagram(m_sendBuf, mes.uTransFrameSize+mes.uTransFrameHdrSize, QHostAddress(ip)/*QHostAddress("192.168.56.1")*/, static_cast<quint16>(port));
            QTime dieTime = QTime::currentTime().addMSecs(1);
            while( QTime::currentTime() < dieTime )
                QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
            count++;
        }
        file.close();
        QTime dieTime = QTime::currentTime().addMSecs(10);
        while( QTime::currentTime() < dieTime )
            QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
    }
}

void MainWindow::on_deleteAll_clicked()
{
    int num = ui->listWidget->count();  //获取列表项的总数目
     ui->listWidget->setFocus(); //将光标设置到列表框上，若注释该语句，则删除时，要手动将焦点设置到列表框，即点击列表项
     for(int i=0;i<num;i++)
     {   //逐个获取列表项的指针，并删除
         QListWidgetItem *item = ui->listWidget->takeItem(ui->listWidget->currentRow());
         delete item;
     }
}


//串口数据检查求和
unsigned char MainWindow::SerialCheckSum(unsigned char *buf, unsigned char len)
{
    unsigned char i, ret = 0;

    for(i=0; i<len; i++)
    {
        ret += buf[i];
    }
    return ret;
}


//处理串口缓冲区数据
void MainWindow::processSerialBuffer(const char* data)
{
    if (SerialCheckSum((unsigned char*)data, 10) != (unsigned char)data[10])
    {
        cout << "CRC校验和不符!" << endl;
        return;
    }

    unsigned char device_id, command;
    device_id = data[1];//设备id
    command   = data[2];//命令id

    //完成帧
    if (command == 0x0f)//完成帧
    {
        if (device_id == 0x00)//横轴完成
        {
            addToList("X轴位移完成");
          //  cout << "X轴位移完成" << endl;
            coarse_x = 0;
            unsigned short temp;

            temp =  static_cast<unsigned char>(data[5]);
            coarse_x |= temp;
            temp =  static_cast<unsigned char>(data[6]);
            coarse_x |= temp << 8;
            //printf("%x %x\n", udata[5], udata[6]);
            QString strPos("x轴当前位置:");
            strPos.append(QString::number(coarse_x));
            addToList(strPos);
            //cout << "当前x=" << coarse_x << endl;
            x_step_finish = true;
        }
       else if (device_id == 0x01)
        {
            addToList("Y轴位移完成");
            coarse_y = 0;
            unsigned short temp;
            temp = (unsigned char)data[5];
            coarse_y |= temp;
            temp = (unsigned char)data[6];
            coarse_y |= temp << 8;
            //printf("%x %x\n", udata[5], udata[6]);
            //cout << "当前y=" << coarse_y << endl;
            QString strPos("y轴当前位置:");
            strPos.append(QString::number(coarse_y));
            addToList(strPos);

            y_step_finish = true;
        }
        else if(device_id == 0x02)//直流电机完成帧
        {
            addToList("Z轴位移完成");

            coarse_z = 0;
            unsigned short temp;
            temp = (unsigned char)data[5];
            coarse_z |= temp;
            temp = (unsigned char)data[6];
            coarse_z |= temp << 8;
            //printf("%x %x\n", udata[5], udata[6]);
            //cout << "当前y=" << coarse_y << endl;
            QString strPos("z轴当前位置:");
            strPos.append(QString::number(coarse_z));
            addToList(strPos);
        }

        unsigned char status = (unsigned char)data[4];
        if((status&0x1) == 1) //bit0 使能停机标志
        {
            addToList("不是使用使能位停机");
            bUseEnableShutdown = true;
        }
        else
        {
            addToList("不是使用使能位停机");
            bUseEnableShutdown = false;
        }

        if((status&0x2) == 1) //bit1 距离完成标志
        {
            addToList("不是使用使能位停机");
            bDistanceDone = true;
        }
        else
        {
            addToList("不是使用使能位停机");
            bDistanceDone = false;
        }

        if((status&0x4) == 1)//时间耗尽标志
        {
            addToList("时间耗尽");
            bTimeUseup = true;
        }
        else
        {
            addToList("时间未耗尽");
            bTimeUseup = false;
        }

        if((status&0x8) == 1)//到达指定位置标志
        {
            addToList("到达指定位置");
            bToAppiontPos = true;
        }
        else
        {
            addToList("未到达指定位置");
            bToAppiontPos = false;
        }

        if((status&0x10) == 1)//到达负限位标志
        {
            addToList("到达负限位");
            bToNegativeLimit = true;
        }
        else
        {
            addToList("未到达负限位");
            bToNegativeLimit = false;
        }

        if((status&0x20) == 1)//方向错误标志
        {
            addToList("方向无误");
            bDirectionError = true;
        }
        else
        {
            addToList("方向错误");
            bDirectionError = false;
        }


        if((status&0x40) == 1)//到达限位标志
        {
            addToList("到达限位");
            bToLimit = true;
        }
        else
        {
            addToList("未到限位");
            bToLimit = false;
        }
    }
    else if (command == 0x0c)//应答帧
    {
        if(device_id == 0x0)
        {
            addToList("x轴电机应答");
        }
        else if(device_id == 0x1)
        {
            addToList("y轴电机应答");
        }
        else if(device_id == 0x2)
        {
            addToList("z轴电机应答");
        }
        else if(device_id == 0x5)
        {
            addToList("温度控制应答");
        }
        else if(device_id == 0x6)
        {
            addToList("动作控制应答");
        }
        else if(device_id == 0x7)
        {
            addToList("故障帧应答");
        }
    }

}

void MainWindow::on_saveButton_clicked()
{
    m_bSaveImage = true;//开始保存图像
}

//计算校验值
unsigned char MainWindow::jiaoyan(unsigned char* data)
{
    int sum = 0;
    for(int i=0;i<10;i++)
    {
        sum += data[i];
    }

    return sum&0xff;//取低8位
}

//电机复位指令帧
void MainWindow::SendMotorResetCmd(int id,int speed,int pos)
{
    unsigned char txData[20] = {0};

    txData[0] = 0x5a;
    txData[1] = (unsigned char)id; //包括0:横轴电机、1:纵轴电机、2:垂向电机
    txData[2] = 0x21;//下行数据帧：复位指令
    txData[4] = speed&0xff;//低8位
    txData[5] = (speed>>8)&0xff;//高8位
    txData[6] = pos&0xff;
    txData[7] = (pos>>8)&0xff;
    txData[10] = jiaoyan(txData);

    if(serial)
        serial->write((const char*)txData,11);

}

//电机运动指令帧
void MainWindow::SendMotorRunCmd(int id,int speed,int pos)
{
    unsigned char txData[20] = {0};

    txData[0] = 0x5a;
    txData[1] = id; //包括0:横轴电机、1:纵轴电机、2:垂向电机
    txData[2] = 0x01;//下行数据帧：运动指令
    txData[4] = speed&0xff;//低8位
    txData[5] = (speed>>8)&0xff;//高8位
    txData[6] = pos&0xff;
    txData[7] = (pos>>8)&0xff;
    txData[10] = jiaoyan(txData);

    if(serial)
        serial->write((const char*)txData,11);
}


//温度控制指令帧
void MainWindow::SendTempretureControlCmd(int value)
{
    unsigned char txData[20] = {0};

    txData[0] = 0x5a;
    txData[1] = 5; //包括0:横轴电机、1:纵轴电机、2:垂向电机
    txData[2] = 0x11;//下行数据帧：温度控制
    txData[6] = value&0xff;
    txData[7] = (value>>8)&0xff;
    txData[10] = jiaoyan(txData);

    if(serial)
        serial->write((const char*)txData,11);
}
//动作控制指令帧，画圆
void MainWindow::SendHuayuanCmd(bool bEnable,int speed,int radius)
{
    unsigned char txData[20] = {0};

    txData[0] = 0x5a;
    txData[1] = 6; //动作控制
    txData[2] = 0x31;//下行数据帧：画圆
    txData[3] = bEnable?1:0;
    txData[4] = speed&0xff;//低8位
    txData[5] = (speed>>8)&0xff;//高8位
    txData[6] = radius&0xff;
    txData[7] = (radius>>8)&0xff;
    txData[10] = jiaoyan(txData);

    if(serial)
        serial->write((const char*)txData,11);
}

//动作控制指令帧，雀琢
void MainWindow::SendQuezuoCmd(bool bEnable,int speed,int pos,int maxResrved)
{
    unsigned char txData[20] = {0};

    txData[0] = 0x5a;
    txData[1] = 6; //动作控制
    txData[2] = 0x32;//下行数据帧：动作指令
    txData[3] = bEnable?1:0;
    txData[4] = speed&0xff;//低8位
    txData[5] = (speed>>8)&0xff;//高8位
    txData[6] = pos&0xff;
    txData[7] = (pos>>8)&0xff;
    txData[8] = maxResrved&0xff;
    txData[9] = (maxResrved>>8)&0xff;
    txData[10] = jiaoyan(txData);

    if(serial)
        serial->write((const char*)txData,11);
}

//下行故障帧，不定时，有故障即发 (有)
void MainWindow::SendFaultCmd(bool bFault)
{
    unsigned char txData[20] = {0};

    txData[0] = 0x5a;
    txData[1] = 7; //动作控制
    txData[2] = 0x2f;//下行数据帧：画圆
    txData[4] =bFault?0:1;//低8位

    txData[10] = jiaoyan(txData);

    if(serial)
        serial->write((const char*)txData,11);
}

//实时温度级别帧，1Hz
void MainWindow::SendTempretureLevelCmd(int level)
{
    unsigned char txData[20] = {0};

    txData[0] = 0x5a;
    txData[1] = 87; //动作控制
    txData[2] = 0x40;//下行数据帧：画圆
    txData[4] = level;//

    txData[10] = jiaoyan(txData);

    if(serial)
        serial->write((const char*)txData,11);
}

void MainWindow::on_checkBox_clicked(bool checked)
{
    m_bShowImage = checked;
}



void MainWindow::on_checkBox_2_clicked(bool checked)
{
    m_bRotateImage = checked;
}

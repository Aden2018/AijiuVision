//                   _ooOoo_
//                  o8888888o
//                  88" . "88
//                  (| -_- |)
//                  O\  =  /O
//               ____/`---'\____
//             .'  \\|     |//  `.
//            /  \\|||  :  |||//  \
//           /  _||||| -:- |||||-  \
//           |   | \\\  -  /// |   |
//           | \_|  ''\---/''  |   |
//           \  .-\__  `-`  ___/-. /
//         ___`. .'  /--.--\  `. . __
//      ."" '<  `.___\_<|>_/___.'  >'"".
//     | | :  `- \`.;`\ _ /`;.`/ - ` : | |
//     \  \ `-.   \_ __\ /__ _/   .-` /  /
//======`-.____`-.___\_____/___.-`____.-'======
//                   `=---='
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//         佛祖保佑       永无BUG
//  本模块已经经过开光处理，绝无可能再产生bug
//=============================================

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

     ui->listWidget->setViewMode(QListView::ListMode);   //设置显示模式为列表模式

    //udp初始化
    mSocket = new QUdpSocket(this);
    connect(mSocket,SIGNAL(readyRead()),this,SLOT(read_data()),Qt::DirectConnection);
    qDebug()<< mSocket->bind(QHostAddress::Any, 8888);

    //初始化定时器
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(handleTimeout()));
    timer->start(100);

    InitialCom();//初始化串口

//    Mat_<float> invec = (Mat_<float>(3, 1) << 0.04345, -0.05236, -0.01810);
//    Mat  outMat;
//    Rodrigues(invec, outMat);
//    cout << outMat << endl;//打印矩阵

//    Mat outvec;
//    Rodrigues(outMat, outvec);
//    cout << outvec << endl;

}

MainWindow::~MainWindow()
{
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

void MainWindow::InitialCom()
{
//    //查找可用的串口
//    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
//    {
//        QSerialPort temp;
//        temp.setPort(info);
//        if(temp.open(QIODevice::ReadWrite))
//        {
//            vComs.push_back(temp.portName());
//            temp.close();
//            qDebug()<<temp.portName()<<endl;
//        }
//    }

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

void MainWindow::read_data()
{
    QByteArray array;
    QHostAddress address;
    quint16 port;
    array.resize(mSocket->bytesAvailable());//根据可读数据来设置空间大小
    mSocket->readDatagram(array.data(),array.size(),&address,&port); //读取数据
    ui->listWidget->addItem(array);//显示数据
   //发送反馈数据

    QString strRecv = "hello";
    qDebug()<<strRecv<<endl;
}


void MainWindow::handleTimeout()
{
    if(capture.isOpened())
    {
        capture >> frame;
//        cvtColor(frame, frame, CV_BGR2RGB);//only RGB of Qt
//        QImage srcQImage = QImage((uchar*)(frame.data), frame.cols, frame.rows, QImage::Format_RGB888);

//        Rect rectLeft(0, 0, frame.cols / 2, frame.rows);
//        Rect rectRight(frame.cols / 2, 0, frame.cols / 2, frame.rows);
//        Mat image_l = Mat(frame, rectLeft);
//        Mat image_r = Mat(frame, rectRight);

      // imshow("left",image_l);
        imshow("right",frame);

//        cvtColor(image_l, image_l, CV_BGR2RGB);//only RGB of Qt
//        QImage srcQImage = QImage((uchar*)(image_l.data), image_l.cols, image_l.rows, QImage::Format_RGB888);

//        ui->label->setPixmap(QPixmap::fromImage(srcQImage));
//        ui->label->resize(srcQImage.size());
//        ui->label->show();
    }
    else
    {
        capture.open(0);
        capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, 960);
    }

}

void MainWindow::Read_Data()//串口读取函数
{
    QByteArray buf;
    buf = serial->readAll();

    //QByteArray<->char数组
    char recv[100];//数组
    int len_array = buf.size();
    int len_buf = sizeof(buf);
    int len = qMin( len_array, len_buf );
    // 转化
    memcpy( recv, buf,  len );
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
    cv::FileStorage fs("d:\\calibParams.yml", cv::FileStorage::READ);
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
    }
}

/*
立体校正
参数：
    stereoRectifyParams	存放立体校正结果的txt
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
Rect MainWindow::stereoRectification(Mat& cameraMatrix1, Mat& distCoeffs1, Mat& cameraMatrix2, Mat& distCoeffs2,
    Size& imageSize, Mat& R, Mat& T, Mat& R1, Mat& R2, Mat& P1, Mat& P2, Mat& Q, Mat& mapl1, Mat& mapl2, Mat& mapr1, Mat& mapr2)
{
    Rect validRoi[2];

    stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize,
        R, T, R1, R2, P1, P2, Q, 0, -1, imageSize, &validRoi[0], &validRoi[1]);

    initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize, CV_32FC1, mapl1, mapl2);
    initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize, CV_32FC1, mapr1, mapr2);
    return validRoi[0], validRoi[1];
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
bool MainWindow::computeDisparityImage(const char* imageName1, const char* imageName2, Mat& img1_rectified,
    Mat& img2_rectified, Mat& mapl1, Mat& mapl2, Mat& mapr1, Mat& mapr2, Rect validRoi[2], Mat& disparity)
{
    // 首先，对左右相机的两张图片进行重构
    Mat img1 = imread(imageName1);
    Mat img2 = imread(imageName2);
    if (img1.empty() | img2.empty())
    {
        qDebug() << "图像为空" << endl;
    }
    Mat gray_img1, gray_img2;
    cvtColor(img1, gray_img1, COLOR_BGR2GRAY);
    cvtColor(img2, gray_img2, COLOR_BGR2GRAY);
    Mat canvas(imageSize.height, imageSize.width * 2, CV_8UC1); // 注意数据类型
    Mat canLeft = canvas(Rect(0, 0, imageSize.width, imageSize.height));
    Mat canRight = canvas(Rect(imageSize.width, 0, imageSize.width, imageSize.height));
    gray_img1.copyTo(canLeft);
    gray_img2.copyTo(canRight);
    imwrite("校正前左右相机图像.jpg", canvas);
    remap(gray_img1, img1_rectified, mapl1, mapl2, INTER_LINEAR);
    remap(gray_img2, img2_rectified, mapr1, mapr2, INTER_LINEAR);
    imwrite("左相机校正图像.jpg", img1_rectified);
    imwrite("右相机校正图像.jpg", img2_rectified);
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
    disparity.convertTo(disparity, CV_32F, 1.0 / 16);
    // 归一化视差映射
    normalize(disparity, disparity, 0, 256, NORM_MINMAX, -1);
    return true;
}

void MainWindow::on_pushButton_clicked()
{
    //加载源图像和模板图像
    char imagename[20];
    const cv::Mat ComFrame(480 , 640 * 6, CV_8UC1);
    cv::Mat image1, image2;
    for (int i = 0; i < 3; i++)
    {
        sprintf_s(imagename, "d:\\%u.bmp", i);
        image2 = cv::imread(imagename, IMREAD_GRAYSCALE);
        ImageStitch(i, image1, image2, ComFrame);
        Mat display;
        cv::resize(ComFrame, display, Size(1800, 225), 0, 0, INTER_NEAREST);
        imshow("result", display);
        waitKey(1000);
    }

////    SendPictureByUdp("d:\\1.jpg",QString("127.0.0.1"), 65522);
////    return;
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
        int count = 0;
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
                mes.uTransFrameSize = endSize;
            }
            //qDebug()<<size;
            mes.uDataFrameSize = size;
            mes.uDataFrameTotal = num;
            mes.uDataFrameCurr = count+1;
            mes.uDataInFrameOffset = count*(1024 - sizeof(ImageFrameHead));

            file.read(m_sendBuf+sizeof(ImageFrameHead), 1024-sizeof(ImageFrameHead));

            memcpy(m_sendBuf, (char *)&mes, sizeof(ImageFrameHead));
            mSocket->writeDatagram(m_sendBuf, mes.uTransFrameSize+mes.uTransFrameHdrSize, QHostAddress(ip)/*QHostAddress("192.168.56.1")*/, port);
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

void MainWindow::on_deleteSingleItem_clicked()
{
    //获取列表项的指针
    QListWidgetItem *item = ui->listWidget->takeItem(ui->listWidget->currentRow());
    delete item;        //释放指针所指向的列表项
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



int status = 0;
//处理串口缓冲区数据
void MainWindow::processSerialBuffer(const char* data)
{
    if (SerialCheckSum((unsigned char*)data, 10) != (unsigned char)data[10])
    {
        cout << "CRC校验和不符!" << endl;
        return;
    }

//    unsigned char device_id, command;
//    device_id = data[1];
//    command   = data[2];

//    //完成帧
//    if (command == 0x0f)//完成帧
//    {
//        if (device_id == 0x00)//横轴完成
//        {
//            cout << "X轴位移完成" << endl;
//            coarse_x = 0;
//            unsigned short temp;
//            temp = (unsigned char)data[5];
//            coarse_x |= temp;
//            temp = (unsigned char)data[6];
//            coarse_x |= temp << 8;
//            //printf("%x %x\n", udata[5], udata[6]);
//            //cout << "当前x=" << coarse_x << endl;
//            x_step_finish = true;
//        }

//        if (device_id == 0x01)
//        {
//            cout << "Y轴位移完成" << endl;
//            coarse_y = 0;
//            unsigned short temp;
//            temp = (unsigned char)data[5];
//            coarse_y |= temp;
//            temp = (unsigned char)data[6];
//            coarse_y |= temp << 8;
//            //printf("%x %x\n", udata[5], udata[6]);
//            //cout << "当前y=" << coarse_y << endl;
//            y_step_finish = true;
//        }

//        if (device_id == 0x02)//直流电机完成帧
//        {
//            if (status == DC_MOTOR_DOWN)//直流电机向下移动完成，开始治疗
//            {
////                if (acu_data_pack.actions[action_index].method == 1)
////                    sendQuezhuoCommand(true);
////                else if (acu_data_pack.actions[action_index].method == 2)
////                    sendDiananCommand(true);
////                else if (acu_data_pack.actions[action_index].method == 3)
////                    sendHuayuanCommand(true);
////                status = WAIT_FOR_TREATMENT;

////                if (timeRemaining == 0)
////                    timeRemaining = acu_data_pack.actions[action_index].time * 10;//开始计时，时间单位为0.1秒
//            }

//            if (status == DC_MOTOR_UP)//直流电机向上移动完成
//            {
//                if (timeRemaining == 0)
//                {
//                    action_index = 0;
//                    finished_one_acupoint = true; //将标志位设为True，进行下一个穴位的按摩
//                    ++id;
//                }
//                else
//                {
//                    cout << "直流电机上提完成" << endl;
//                    /*if (acu_data_pack.actions[action_index].method == 1)
//                    sendQuezhuoCommand(false);
//                    else if (acu_data_pack.actions[action_index].method == 2)
//                    sendDiananCommand(false);
//                    else if (acu_data_pack.actions[action_index].method == 3)
//                    sendHuayuanCommand(false);*/
//                    if (!stopFlag)
//                        status = PAUSE;
//                    else
//                        status = STOP;
//                }
//            }
//        }

//        if (x_step_finish && y_step_finish)
//        {
//            if (status == WAIT_FOR_RESET)  //复位状态
//            {
//                cout << "步进电机复位完成" << endl;
//                sendResetDuojiCommand(duoji_rst);
//                //status = WAIT_FOR_START;
//                //sendMoveCommand(200, 5900, 15000);
//            }
//            else if (status == WAIT_FOR_START)
//            {
//                status = START;
//                startFlag = true;
//            }

//            if (status == COARSE_MOVE)
//                //status = PRECISE_MOVE;

//            if (status == PRECISE_FINISH)
//            {
//                sendMoveCommand(50, coarse_x, coarse_y - 380);
//                status = MOVE_TO_POS;
//            }
//            else if (status == MOVE_TO_POS)
//            {
//                status = WAIT_FOR_TREATMENT;//执行第一个按摩手法
//                cout << "开始执行第" << action_index + 1 << "个按摩手法" << endl;
//                //开始对当前手法设定温度，等待治疗
//                sendSetTemperature(acu_data_pack.actions[action_index].temperature * 10);
//            }

//            if (status == PRECISE_MOVE)
//            {
//                int x = 0, y = 0;
//                unsigned short pres_x, pres_y;

//                ImageProc(frame, x, y);//进行图像处理，进行穴位细定位，得到精确位置
//                cout << "x= " << x << endl;
//                cout << "y= " << y << endl;
//                cout << "coarse_x=" << coarse_x << endl;
//                cout << "coarse_y=" << coarse_y << endl;

//                pres_x = x > 0 ? coarse_x + (unsigned short)abs(x) : coarse_x - (unsigned short)abs(x);
//                pres_y = y > 0 ? coarse_y + (unsigned short)abs(y) : coarse_y - (unsigned short)abs(y);

//                sendMoveCommand(100, pres_x, pres_y);
//            }
//            else if (status == TIME_OUT)//画圆治疗结束的完成帧，通过计时完成结束
//            {
//                if (action_index == acu_data_pack.action_num)
//                {
//                    /*action_index = 0;
//                    finished_one_acupoint = true; //将标志位设为True，进行下一个穴位的按摩
//                    ++id;*/
//                    cout << "直流电机提起" << endl;
//                    sendDcmotorCommand(2500, 900, false, true);//直流电机提起
//                    status = DC_MOTOR_UP;
//                }
//                else
//                {
//                    //发送指令进行下一个按摩手法
//                    cout << "开始执行第" << action_index + 1 << "个按摩手法" << endl;
//                    //开始对当前手法设定温度,等待治疗
//                    sendSetTemperature(acu_data_pack.actions[action_index].temperature * 10);
//                    status = WAIT_FOR_TREATMENT;
//                }
//            }
//            else if (status == IN_TREATMENT && acu_data_pack.actions[action_index].method == 3)//画圆治疗结束的完成帧，通过按键
//            {
//                sendDcmotorCommand(2500, 900, false, true);
//                status = DC_MOTOR_UP;
//            }
//            else if (status == STOP)
//            {
//                //system("shutdown -s -t 10"); 治疗完成关闭计算机
//            }

//            x_step_finish = false;
//            y_step_finish = false;
//        }
//    }

//    //应答帧
//    else if (command == 0x0c)//应答帧
//    {
//        //cout<<"收到设备"<<device_id<<"的应答帧"<<endl;
//        if (device_id == 0x04)//舵机复位完成
//        {
//            cout << "舵机复位完成" << endl;
//            status = WAIT_FOR_START;
//            sendMoveCommand(200, 5900, 15000);//走到床头
//        }

//        if (device_id == 0x06)//动作控制帧：以当前位置为圆心画圆
//        {
//            if (status == WAIT_FOR_TREATMENT)
//                status = IN_TREATMENT;//进入治疗中状态，开始计时
//            else if (status == IN_TREATMENT)//按键暂停的应答帧
//            {
//                cout << "method=" << acu_data_pack.actions[action_index].method << endl;
//                if (acu_data_pack.actions[action_index].method != 3)//若当前手法不是画圆，则收到应答帧即可认为电机停止
//                {
//                    sendDcmotorCommand(2500, 900, false, true);
//                    status = DC_MOTOR_UP;
//                    //status = PAUSE;
//                }
//            }
//            else if (status == PAUSE)
//            {
//                status = IN_TREATMENT;
//            }
//            else if (status == TIME_OUT)
//            {
//                if (acu_data_pack.actions[action_index - 1].method != 3)//若当前手法不是画圆，则收到应答帧即可认为电机停止
//                {
//                    if (action_index == acu_data_pack.action_num)
//                    {
//                        cout << "直流电机提起" << endl;
//                        sendDcmotorCommand(2500, 900, false, true);//直流电机提起
//                        status = DC_MOTOR_UP;
//                    }
//                    else
//                    {
//                        //发送指令进行下一个按摩手法
//                        cout << "开始执行第" << action_index + 1 << "个按摩手法" << endl;
//                        //开始对当前手法设定温度，等待治疗
//                        sendSetTemperature(acu_data_pack.actions[action_index].temperature * 10);
//                        status = WAIT_FOR_TREATMENT;
//                    }
//                }
//            }

//            cout << "治疗手法应答后,status=" << status << endl;
//        }
//    }

//    //按键帧
//    else if (command == 0x3f)//按键帧
//    {
//        if (device_id == 0x08)
//        {
//            unsigned char key = data[4];
//            cout << "收到按键，键值为" << (int)key << endl;
//            if (key == 0)//按键值为WORK,重新开启工作状态
//            {
//                cout << "收到WORK按键" << endl;
//                /*if (acu_data_pack.actions[action_index].method == 1)
//                sendQuezhuoCommand(true);
//                else if (acu_data_pack.actions[action_index].method == 2)
//                sendDiananCommand(true);
//                else if (acu_data_pack.actions[action_index].method == 3)
//                sendHuayuanCommand(true);*/
//                if (status == PAUSE)
//                {
//                    sendDcmotorCommand(1500, 400, true, true);
//                    status = DC_MOTOR_DOWN;
//                }
//            }
//            else if (key == 1)//按键值为PAUSE,暂停工作
//            {
//                cout << "收到PAUSE按键" << endl;
//                if (status == IN_TREATMENT)
//                {
//                    if (acu_data_pack.actions[action_index].method == 1)
//                        sendQuezhuoCommand(false);
//                    else if (acu_data_pack.actions[action_index].method == 2)
//                        sendDiananCommand(false);
//                    else if (acu_data_pack.actions[action_index].method == 3)
//                        sendHuayuanCommand(false);
//                }
//            }
//            else if (key == 2)//按键值为STOP，停止工作
//            {
//                cout << "收到STOP按键" << endl;
//                if (acu_data_pack.actions[action_index].method == 1)
//                    sendQuezhuoCommand(false);
//                else if (acu_data_pack.actions[action_index].method == 2)
//                    sendDiananCommand(false);
//                else if (acu_data_pack.actions[action_index].method == 3)
//                    sendHuayuanCommand(false);
//                stopFlag = true;
//                //sendDcmotorCommand(2500, 900, false, true);
//                //status = DC_MOTOR_UP;
//            }
//        }
//    }

//    //温度反馈
//    else if (command == 0x1f)//温度反馈
//    {
//        unsigned short temp;
//        unsigned char level;
//        temprature = 0;
//        temp = (unsigned char)data[3];
//        temprature |= temp;
//        temp = (unsigned char)data[4];
//        temprature |= temp << 8;
//        cout << "当前温度值为" << (float)temprature / 10 << endl;
//        level = temp2Level(temprature / 10);
//        sendTempLevel(level);

//        if (status == WAIT_FOR_TREATMENT)
//        {
//            cout << "检查温度是否符合要求" << endl;
//            if (abs(temprature / 10 - acu_data_pack.actions[action_index].temperature) < 10)//如果当前温度与设定温度差值小于一定值
//            {
//                /*if (acu_data_pack.actions[action_index].method == 1)
//                sendQuezhuoCommand(true);
//                else if (acu_data_pack.actions[action_index].method == 2)
//                sendDiananCommand(true);
//                else if (acu_data_pack.actions[action_index].method == 3)
//                sendHuayuanCommand(true);
//                //status = IN_TREATMENT;

//                timeRemaining = acu_data_pack.actions[action_index].time * 10;//开始计时，时间单位为0.1秒*/
//                cout << "温度满足要求" << endl;
//                if (action_index == 0)
//                {
//                    sendDcmotorCommand(1500, 400, true, true);
//                    status = DC_MOTOR_DOWN;
//                }
//                else
//                {
//                    if (acu_data_pack.actions[action_index].method == 1)
//                        sendQuezhuoCommand(true);
//                    else if (acu_data_pack.actions[action_index].method == 2)
//                        sendDiananCommand(true);
//                    else if (acu_data_pack.actions[action_index].method == 3)
//                        sendHuayuanCommand(true);
//                    //status = IN_TREATMENT;

//                    timeRemaining = acu_data_pack.actions[action_index].time * 10;
//                }

//            }
//        }
//    }

//    //设定温度级别
//    else if (command == 0x40)//设定温度级别
//    {
//        if (device_id == 0x08)
//        {
//            unsigned short temp;
//            temp = level2Temp(data[4]);
//            cout << "收到设定温度级别，设定温度为" << temp << "度" << endl;
//            sendSetTemperature(temp * 10);
//        }
//    }
}


Mat MainWindow::jiaozheng( Mat image )
{
    Size image_size = image.size();
//    float intrinsic[3][3] = {589.2526583947847,0,321.8607532099886,0,585.7784771038199,251.0338528599469,0,0,1};
//    float distortion[1][5] = {-0.5284205687061442, 0.3373615384253201, -0.002133029981628697, 0.001511983002864886, -0.1598661778309496};
//    Mat intrinsic_matrix = Mat(3,3,CV_32FC1,intrinsic);
//    Mat distortion_coeffs = Mat(1,5,CV_32FC1,distortion);
    Mat R = Mat::eye(3,3,CV_32F);
    Mat mapx = Mat(image_size,CV_32FC1);
    Mat mapy = Mat(image_size,CV_32FC1);
    Mat intrinsic_matrix;
    initUndistortRectifyMap(cameraMatrix_L,distCoeffs_L,R,intrinsic_matrix,image_size,CV_32FC1,mapx,mapy);
    Mat t = image.clone();
    cv::remap( image, t, mapx, mapy, INTER_LINEAR);
    return t;
}

/***************************************
 * //已知空间坐标求成像坐标
    Point3f point(700,220,530);
    cout<<"左相机中坐标："<<endl;
    cout<<xyz2uv(point,leftIntrinsic,leftTranslation,leftRotation)<<endl;
    cout<<"右相机中坐标："<<endl;
    cout<<xyz2uv(point,rightIntrinsic,rightTranslation,rightRotation)<<endl;

    //已知左右相机成像坐标求空间坐标
    Point2f l = xyz2uv(point,leftIntrinsic,leftTranslation,leftRotation);
    Point2f r = xyz2uv(point,rightIntrinsic,rightTranslation,rightRotation);
    Point3f worldPoint;
    worldPoint = uv2xyz(l,r);
    cout<<"空间坐标为:"<<endl<<uv2xyz(l,r)<<endl;
 * ************************************/

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
Point3f MainWindow::uv2xyz(Point2f uvLeft,Point2f uvRight)
{
    //  [u1]      |X|					  [u2]      |X|
    //Z*|v1| = Ml*|Y|					Z*|v2| = Mr*|Y|
    //  [ 1]      |Z|					  [ 1]      |Z|
    //			  |1|								|1|

//    Mat cameraMatrix_L = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 相机的内参数
//    Mat cameraMatrix_R = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 初始化相机的内参数
//    Mat distCoeffs_L = Mat(1, 5, CV_32FC1, Scalar::all(0)); // 相机的畸变系数
//    Mat distCoeffs_R = Mat(1, 5, CV_32FC1, Scalar::all(0)); // 初始化相机的畸变系数
//    Mat R, T, E, F; // 立体标定参数

    //将mat转换成float数组
    //左相机内参数矩阵
    float leftIntrinsic[3][3] = {4037.82450,			 0,		947.65449,
                                          0,	3969.79038,		455.48718,
                                          0,			 0,				1};
    //左相机畸变系数
    float leftDistortion[1][5] = {0.18962, -4.05566, -0.00510, 0.02895, 0};
    //左相机旋转矩阵
    float leftRotation[3][3] = {0.912333,		-0.211508,		 0.350590,
                                0.023249,		-0.828105,		-0.560091,
                                0.408789,		 0.519140,		-0.750590};
    //左相机平移向量
    float leftTranslation[1][3] = {-127.199992, 28.190639, 1471.356768};

    //右相机内参数矩阵
    float rightIntrinsic[3][3] = {3765.83307,			 0,		339.31958,
                                            0,	3808.08469,		660.05543,
                                            0,			 0,				1};
    //右相机畸变系数
    float rightDistortion[1][5] = {-0.24195, 5.97763, -0.02057, -0.01429, 0};
    //右相机旋转矩阵
    float rightRotation[3][3] = {-0.134947,		 0.989568,		-0.050442,
                                  0.752355,		 0.069205,		-0.655113,
                                 -0.644788,		-0.126356,		-0.753845};
    //右相机平移向量
    float rightTranslation[1][3] = {50.877397, -99.796492, 1507.312197};


    Mat mLeftRotation = Mat(3,3,CV_32F,leftRotation);//左相机旋转矩阵
    Mat mLeftTranslation = Mat(3,1,CV_32F,leftTranslation);//左相机平移向量
    Mat mLeftRT = Mat(3,4,CV_32F);//左相机M矩阵
    hconcat(mLeftRotation,mLeftTranslation,mLeftRT);
    Mat mLeftIntrinsic = Mat(3,3,CV_32F,leftIntrinsic);//右相机内参数矩阵
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
Point2f MainWindow::xyz2uv(Point3f worldPoint,float intrinsic[3][3],float translation[1][3],float rotation[3][3])
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




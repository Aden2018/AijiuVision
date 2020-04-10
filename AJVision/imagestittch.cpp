#include "imagestittch.h"

void OptimizeSeam(Mat& img1, Mat& trans, Mat& dst);

typedef struct
{
    Point2f left_top;
    Point2f left_bottom;
    Point2f right_top;
    Point2f right_bottom;
}four_corners_t;

four_corners_t corners;

void CalcCorners(const Mat& H, const Mat& src)
{
    double v2[] = { 0, 0, 1 };//左上角
    double v1[3];//变换后的坐标值
    Mat V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    Mat V1 = Mat(3, 1, CV_64FC1, v1);  //列向量

    V1 = H * V2;
//    //左上角(0,0,1)
//    cout << "V2: " << V2 << endl;
//    cout << "V1: " << V1 << endl;
    corners.left_top.x = v1[0] / v1[2];
    corners.left_top.y = v1[1] / v1[2];

    //左下角(0,src.rows,1)
    v2[0] = 0;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.left_bottom.x = v1[0] / v1[2];
    corners.left_bottom.y = v1[1] / v1[2];

    //右上角(src.cols,0,1)
    v2[0] = src.cols;
    v2[1] = 0;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.right_top.x = v1[0] / v1[2];
    corners.right_top.y = v1[1] / v1[2];

    //右下角(src.cols,src.rows,1)
    v2[0] = src.cols;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.right_bottom.x = v1[0] / v1[2];
    corners.right_bottom.y = v1[1] / v1[2];

}


Mat ImageStittch(Mat a,Mat b) //a：右边图像，b：左边图像
{
    Ptr<SURF> surf;            //创建方式和OpenCV2中的不一样,并且要加上命名空间xfreatures2d
                               //否则即使配置好了还是显示SURF为未声明的标识符
    surf = SURF::create(800);

    BFMatcher matcher;         //实例化一个暴力匹配器
    Mat c, d;
    vector<KeyPoint>key1, key2;
    vector<DMatch> matches;    //DMatch是用来描述匹配好的一对特征点的类，包含这两个点之间的相关信息
                               //比如左图有个特征m，它和右图的特征点n最匹配，这个DMatch就记录它俩最匹配，并且还记录m和n的
                               //特征向量的距离和其他信息，这个距离在后面用来做筛选

    surf->detectAndCompute(a, Mat(), key1, c);//输入图像，输入掩码，输入特征点，输出Mat，存放所有特征点的描述向量
    surf->detectAndCompute(b, Mat(), key2, d);//这个Mat行数为特征点的个数，列数为每个特征向量的尺寸，SURF是64（维）

    matcher.match(d, c, matches);             //匹配，数据来源是特征向量，结果存放在DMatch类型里面

                                              //sort函数对数据进行升序排列
    sort(matches.begin(), matches.end());     //筛选匹配点，根据match里面特征对的距离从小到大排序
    vector< DMatch > good_matches;
    int ptsPairs = std::min(50, (int)(matches.size() * 0.15));
 //   cout << ptsPairs << endl;
    for (int i = 0; i < ptsPairs; i++)
    {
        good_matches.push_back(matches[i]);//距离最小的50个压入新的DMatch
    }
    Mat outimg;                                //drawMatches这个函数直接画出摆在一起的图
    drawMatches(b, key2, a, key1, good_matches, outimg, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);  //绘制匹配点


  //  imshow("桌面", outimg);

    ///////////////////////图像配准及融合////////////////////////

    vector<Point2f> imagePoints1, imagePoints2;

    for (size_t i = 0; i<good_matches.size(); i++)
    {
        imagePoints2.push_back(key2[good_matches[i].queryIdx].pt);
        imagePoints1.push_back(key1[good_matches[i].trainIdx].pt);
    }

    //获取图像1到图像2的投影映射矩阵 尺寸为3*3
    Mat homo = findHomography(imagePoints1, imagePoints2, CV_RANSAC);
    ////也可以使用getPerspectiveTransform方法获得透视变换矩阵，不过要求只能有4个点，效果稍差
    //Mat   homo=getPerspectiveTransform(imagePoints1,imagePoints2);
    //cout << "变换矩阵为：\n" << homo << endl << endl; //输出映射矩阵

                                                //计算配准图的四个顶点坐标
    CalcCorners(homo, a);
//    cout << "left_top:" << corners.left_top << endl;
//    cout << "left_bottom:" << corners.left_bottom << endl;
//    cout << "right_top:" << corners.right_top << endl;
//    cout << "right_bottom:" << corners.right_bottom << endl;

                                                //图像配准
    Mat imageTransform1, imageTransform2;
    warpPerspective(a, imageTransform1, homo, Size(MAX(corners.right_top.x, corners.right_bottom.x), b.rows));
    //warpPerspective(a, imageTransform2, adjustMat*homo, Size(b.cols*1.3, b.rows*1.8));
   // imshow("直接经过透视矩阵变换", imageTransform1);
  //  imwrite("trans1.jpg", imageTransform1);

    //创建拼接后的图,需提前计算图的大小
    int dst_width = imageTransform1.cols;  //取最右点的长度为拼接图的长度
    int dst_height = b.rows;

    Mat dst(dst_height, dst_width, CV_8UC3);
    dst.setTo(0);

    imageTransform1.copyTo(dst(Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));
    b.copyTo(dst(Rect(0, 0, b.cols, b.rows)));

 //   imshow("b_dst", dst);


    OptimizeSeam(b, imageTransform1, dst);

    return dst;


   // imshow("dst", dst);
   // imwrite("dst.jpg", dst);
}
//优化两图的连接处，使得拼接自然
void OptimizeSeam(Mat& img1, Mat& trans, Mat& dst)
{
    int start = MIN(corners.left_top.x, corners.left_bottom.x);//开始位置，即重叠区域的左边界

    double processWidth = img1.cols - start;//重叠区域的宽度
    int rows = dst.rows;
    int cols = img1.cols; //注意，是列数*通道数
    double alpha = 1;//img1中像素的权重
    for (int i = 0; i < rows; i++)
    {
        uchar* p = img1.ptr<uchar>(i);  //获取第i行的首地址
        uchar* t = trans.ptr<uchar>(i);
        uchar* d = dst.ptr<uchar>(i);
        for (int j = start; j < cols; j++)
        {
            //如果遇到图像trans中无像素的黑点，则完全拷贝img1中的数据
            if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
            {
                alpha = 1;
            }
            else
            {
                //img1中像素的权重，与当前处理点距重叠区域左边界的距离成正比，实验证明，这种方法确实好
                alpha = (processWidth - (j - start)) / processWidth;
            }

            d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1 - alpha);
            d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
            d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);

        }
    }

}

//image1、image2分别为待拼接的左右两张图像，index为image1的编号。BIGframe为存放的拼接图像。每执行一次放置一张图像。
//当输入index为0时，即为第一张图像时，仅仅存入当前帧，不拼接；当index不等于0时，执行拼接操作。
//该实现功能：读取第一帧，当下一帧到来时，使用拼接、平滑计算，重新计算上一帧，并去掉上一阵左边重叠区域，得到更新的上一帧。
//第一张与最后一张图像之间不执行拼接操作
void ImageStitch(int index, Mat &image1, Mat &image2, const Mat &ComFrame)
{
    static int image_source_width = 120;
    static int image_template_width = 100;
    static int image_template_height = 580;
    static int deltaY = 0;
    static Point PreMaxLoc;
    static const int BigCols = 6;//拼接的图像每行6张
    static const int BigRows = 2;//拼接的图像每列2张
    static const int FirstFrame = 0;
    if (!index)
    {
        PreMaxLoc.x = 5;//偏移量x属于0-20,x取任意位置均可以。保持第一张图片宽度为800-120+5=685
        PreMaxLoc.y = 10;//偏移量y属于0-20,当y=0时，为理想值，无偏移量(即两帧图像无上下偏移)。
        deltaY = 0;//第一张图像上下无偏移量
    }
    else
    {
        Mat image_source = image1(Rect(800 - image_source_width, 0, image_source_width, 600));
        Mat image_template = image2(Rect(0, 10, image_template_width, image_template_height));
        cv::Mat image_matched;
        //模板匹配
        cv::matchTemplate(image_source, image_template, image_matched, cv::TM_CCORR_NORMED); //TM_SQDIFF=0, TM_SQDIFF_NORMED=1, TM_CCORR=2, TM_CCORR_NORMED=3, TM_CCOEFF=4, TM_CCOEFF_NORMED=5
        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        //寻找最佳匹配位置
        cv::minMaxLoc(image_matched, &minVal, &maxVal, &minLoc, &maxLoc);
        Mat overlay1 = image1(Rect(800 - image_source_width + maxLoc.x, 0, image_source_width - maxLoc.x, image1.rows - 10 + maxLoc.y));  //左边区域
        Mat overlay2 = image2(Rect(0, 10 - maxLoc.y, image_source_width - maxLoc.x, image2.rows - 10 + maxLoc.y)); //右边区域
        Mat overlay = Mat::zeros(overlay2.rows, overlay2.cols, CV_8UC1);
        for (int i = 0; i< overlay2.rows; i++)
        {
            uchar* dataoverlay1 = overlay1.ptr(i);
            uchar* dataoverlay2 = overlay2.ptr(i);
            uchar* dataoverlay = overlay.ptr(i);
            for (int j = 0; j<overlay2.cols; j++)
            {
                double weight;
                weight = (double)j / overlay2.cols;  //随距离改变而改变的叠加系数
                dataoverlay[j] = (1 - weight)*dataoverlay1[j] + weight*dataoverlay2[j];
            }
        }
        overlay.copyTo(image1(Rect(800 - image_source_width + maxLoc.x, 0, overlay.cols, overlay.rows)));
        deltaY = deltaY + (10 - PreMaxLoc.y);
        Mat newframe1 = image1(Rect(image_source_width - PreMaxLoc.x, deltaY, image1.cols - image_source_width + PreMaxLoc.x, 514));//高度给定值514，否则由于下面的resize语句，造成错位
        cv::resize(newframe1, newframe1, Size(640, 480), 0, 0, INTER_NEAREST);
        newframe1.copyTo(ComFrame(Rect(((index - 1) % BigCols)* newframe1.cols, ((index - 1) / BigCols)*newframe1.rows, newframe1.cols, newframe1.rows)));
        if (index == (BigCols*BigRows + FirstFrame - 1))//最后一张图像不执行拼接操作，与上一张图像同时显示
        {
            Mat newframe2 = image2(Rect(image_source_width - maxLoc.x, deltaY + (10 - maxLoc.y), image2.cols - image_source_width + maxLoc.x, 514));//高度给定值514，否则由于下面的resize语句，造成错位
            cv::resize(newframe2, newframe2, Size(640, 480), 0, 0, INTER_NEAREST);
            newframe2.copyTo(ComFrame(Rect((index % BigCols)* newframe1.cols, ((index - 1) / BigCols)*newframe1.rows, newframe2.cols, newframe2.rows)));
        }
        PreMaxLoc = maxLoc;
        if (!(index % BigCols))  //每行6列，起始index为0
        {
            deltaY = 0;
        }
    }
    image1 = image2;  //将第二张图片赋值给第一张图片，继续进行下一循环。
}

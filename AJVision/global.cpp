#include "global.h"

Mat QImage2cvMat(QImage image)    //QImage-->Mat
{
    Mat mat;
    qDebug() << image.format();
    switch(image.format())
    {
    case QImage::Format_ARGB32:
    case QImage::Format_RGB32:
    case QImage::Format_ARGB32_Premultiplied:
        mat = Mat(image.height(), image.width(), CV_8UC4, (void*)image.constBits(), static_cast<size_t>(image.bytesPerLine()));
        break;
    case QImage::Format_RGB888:
        mat = Mat(image.height(), image.width(), CV_8UC3, (void*)image.constBits(), static_cast<size_t>(image.bytesPerLine()));
        cvtColor(mat, mat, CV_BGR2RGB);
        break;
    case QImage::Format_Indexed8:
        mat = Mat(image.height(), image.width(), CV_8UC1, (void*)image.constBits(), static_cast<size_t>(image.bytesPerLine()));
        break;
    }
    return mat;
}

QImage Mat2QImage(const Mat& mat) //Mat-->QImage
{
    // 8-bits unsigned, NO. OF CHANNELS = 1
    if(mat.type() == CV_8UC1)
    {
        QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
        // Set the color table (used to translate colour indexes to qRgb values)
        image.setColorCount(256);
        for(int i = 0; i < 256; i++)
        {
            image.setColor(i, qRgb(i, i, i));
        }
        // Copy input Mat
        uchar *pSrc = mat.data;
        for(int row = 0; row < mat.rows; row ++)
        {
            uchar *pDest = image.scanLine(row);
            memcpy(pDest, pSrc, mat.cols);
            pSrc += mat.step;
        }
        return image;
    }
    // 8-bits unsigned, NO. OF CHANNELS = 3
    else if(mat.type() == CV_8UC3)
    {
        // Copy input Mat
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped();
    }
    else if(mat.type() == CV_8UC4)
    {
  //      qDebug() << "CV_8UC4";
        // Copy input Mat
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        return image.copy();
    }
    else
    {
    //    qDebug() << "ERROR: Mat could not be converted to QImage.";
        return QImage();
    }
}

//颜色转换：RGB-->HSV
void RGB2HSV(double red, double green, double blue, double& hue, double& saturation, double& intensity)
{

#define PI 3.1415926

    double r, g, b;
    double h, s, i;

    double sum;
    double minRGB, maxRGB;
    double theta;

    r = red / 255.0;
    g = green / 255.0;
    b = blue / 255.0;

    minRGB = ((r<g) ? (r) : (g));
    minRGB = (minRGB<b) ? (minRGB) : (b);

    maxRGB = ((r>g) ? (r) : (g));
    maxRGB = (maxRGB>b) ? (maxRGB) : (b);

    sum = r + g + b;
    i = sum / 3.0;

    if (i<0.001 || maxRGB - minRGB<0.001)
    {
        h = 0.0;
        s = 0.0;
    }
    else
    {
        s = 1.0 - 3.0*minRGB / sum;
        theta = sqrt((r - g)*(r - g) + (r - b)*(g - b));
        theta = acos((r - g + r - b)*0.5 / theta);
        if (b <= g)
            h = theta;
        else
            h = 2 * PI - theta;
        if (s <= 0.01)
            h = 0;
    }

    hue = static_cast<int>(h * 180 / PI);
    saturation = static_cast<int>(s * 100);
    intensity  = static_cast<int>(i * 100);
}
//找出图像中红色的物体
Mat DetectRedTarget(Mat input)
{

    Mat frame;
    Mat srcImg = input;
    frame = srcImg;
  //  waitKey(1);
    int width = srcImg.cols;
    int height = srcImg.rows;

    int x, y;
    double B = 0.0, G = 0.0, R = 0.0, H = 0.0, S = 0.0, V = 0.0;
    Mat vec_rgb = Mat::zeros(srcImg.size(), CV_8UC1);
    for (x = 0; x < height; x++)
    {
        for (y = 0; y < width; y++)
        {
            B = srcImg.at<Vec3b>(x, y)[0];
            G = srcImg.at<Vec3b>(x, y)[1];
            R = srcImg.at<Vec3b>(x, y)[2];
            RGB2HSV(R, G, B, H, S, V);
            //红色范围，范围参考的网上。可以自己调
            if ((H >= 312 && H <= 360) && (S >= 17 && S <= 100) && (V>18 && V < 100))
                vec_rgb.at<uchar>(x, y) = 255;
            /*cout << H << "," << S << "," << V << endl;*/
        }
    }

    return vec_rgb;
}

vector<Point2f> GetTargetCoordinate(Mat in) //获取红色目标坐标
{
    Mat matSrc = in;

    GaussianBlur(matSrc, matSrc, Size(5, 5), 0);//高斯滤波，除噪点

    vector<vector<Point> > contours;//contours的类型，双重的vector

    vector<Vec4i> hierarchy;//Vec4i是指每一个vector元素中有四个int型数据。

    //阈值
    threshold(matSrc, matSrc, 100, 255, THRESH_BINARY);//图像二值化

    //寻找轮廓，这里注意，findContours的输入参数要求是二值图像，二值图像的来源大致有两种，第一种用threshold，第二种用canny
    findContours(matSrc.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    if(contours.size() == 0)
    {
        return vector<Point2f>();
    }
    // 计算矩
    vector<Moments> mu(contours.size());

    for (size_t i = 0; i < contours.size(); i++)
    {
        mu[i] = moments(contours[i], false);
    }

    ///  计算矩中心:

    vector<Point2f> mc(contours.size());

    for (size_t i = 0; i < contours.size(); i++)
    {
        mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    }

    return mc;
}

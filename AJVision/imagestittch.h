#ifndef IMAGESTITTCH_H
#define IMAGESTITTCH_H
#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include"opencv2/flann.hpp"
#include"opencv2/xfeatures2d.hpp"
#include"opencv2/ml.hpp"

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;
using namespace cv::ml;

//a：右边图像，b：左边图像
Mat ImageStittch(Mat a,Mat b);

void ImageStitch(int index, Mat &image1, Mat &image2, const Mat &ComFrame);

#endif // IMAGESTITTCH_H

#include "stereomatch.h"

StereoMatch::StereoMatch()
{
    sgbm = StereoSGBM::create(0, 16, 3);

    Size  imageSize = Size(imageWidth, imageHeight);

    cv::FileStorage fs("calibParams.yml", cv::FileStorage::READ);
    if (fs.isOpened())
    {
        fs["cameraMatrix_L"] >> cameraMatrixL;
        fs["distCoeffs_L"] >> distCoeffL;
        fs["cameraMatrix_R"] >> cameraMatrixR;
        fs["distCoeffs_R"] >> distCoeffR;
        fs["R"] >> rec;
        fs["T"] >> T;
        fs["imageSize"] >> imageSize;
        fs.release();
    }

    if(cameraMatrixL.data && distCoeffL.data && cameraMatrixR.data && distCoeffR.data && rec.data && T.data)
    {
        /*  立体校正    */
        Rodrigues(rec, R); //Rodrigues变换
        stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
            0, imageSize, &validROIL, &validROIR);
        initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_16SC2, mapLx, mapLy);
        initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_16SC2, mapRx, mapRy);
    }
    else {
        cout << "参数读取失败" << endl;
    }

}

void StereoMatch::saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 16.0e4;
    FILE* fp = fopen(filename, "wt");
    printf("%d %d \n", mat.rows, mat.cols);
    for (int y = 0; y < mat.rows; y++)
    {
        for (int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);

        }
    }
    fclose(fp);
}

/*给深度图上色*/
void StereoMatch::GenerateFalseMap(cv::Mat &src, cv::Mat &disp)
{
    // color map
    float max_val = 255.0f;
    float map[8][4] = { { 0,0,0,114 },{ 0,0,1,185 },{ 1,0,0,114 },{ 1,0,1,174 },
    { 0,1,0,114 },{ 0,1,1,185 },{ 1,1,0,114 },{ 1,1,1,0 } };
    float sum = 0;
    for (int i = 0; i < 8; i++)
        sum += map[i][3];

    float weights[8]; // relative   weights
    float cumsum[8];  // cumulative weights
    cumsum[0] = 0;
    for (int i = 0; i < 7; i++) {
        weights[i] = sum / map[i][3];
        cumsum[i + 1] = cumsum[i] + map[i][3] / sum;
    }

    int height_ = src.rows;
    int width_ = src.cols;
    // for all pixels do
    for (int v = 0; v < height_; v++) {
        for (int u = 0; u < width_; u++) {

            // get normalized value
            float val = std::min(std::max(src.data[v*width_ + u] / max_val, 0.0f), 1.0f);

            // find bin
            int i;
            for (i = 0; i < 7; i++)
                if (val < cumsum[i + 1])
                    break;

            // compute red/green/blue values
            float   w = 1.0 - (val - cumsum[i])*weights[i];
            uchar r = (uchar)((w*map[i][0] + (1.0 - w)*map[i + 1][0]) * 255.0);
            uchar g = (uchar)((w*map[i][1] + (1.0 - w)*map[i + 1][1]) * 255.0);
            uchar b = (uchar)((w*map[i][2] + (1.0 - w)*map[i + 1][2]) * 255.0);
            //rgb内存连续存放
            disp.data[v*width_ * 3 + 3 * u + 0] = b;
            disp.data[v*width_ * 3 + 3 * u + 1] = g;
            disp.data[v*width_ * 3 + 3 * u + 2] = r;
        }
    }
}

/*****立体匹配*****/
/*

    rgbImageL = imread("left_cor.bmp", CV_LOAD_IMAGE_COLOR);//CV_LOAD_IMAGE_COLOR
    rgbImageR = imread("right_cor.bmp", -1);
    Mat disparity;
    xyz=stereo_match(rgbImageL,rgbImageR);
 */
Mat StereoMatch::stereo_match(Mat rgbImageL, Mat rgbImageR,Mat &rectifyImageL,Mat &rectifyImageR,Mat &disp8,Mat &xyz)
{
    if(rgbImageL.empty() || rgbImageR.empty())
    {
        cout << "图像数据为空" << endl;
        return Mat();
    }

    if(mapLx.empty() || mapLy.empty() || mapRx.empty() || mapRy.empty())
    {
        cout << "映射表数据为空" << endl;
        return Mat();
    }
    /*  经过remap之后，左右相机的图像已经共面并且行对准了 */
    remap(rgbImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);//INTER_LINEAR
    remap(rgbImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

    sgbm->setPreFilterCap(63);
    int sgbmWinSize = 5;//根据实际情况自己设定
    int NumDisparities = 416;//根据实际情况自己设定
    int UniquenessRatio = 6;//根据实际情况自己设定
    sgbm->setBlockSize(sgbmWinSize);
    int cn = rectifyImageL.channels();

    sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(NumDisparities);
    sgbm->setUniquenessRatio(UniquenessRatio);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(10);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setMode(StereoSGBM::MODE_SGBM);
    Mat disp, dispf;
    sgbm->compute(rectifyImageL, rectifyImageR, disp);
    //去黑边
    Mat img1p, img2p;
    copyMakeBorder(rectifyImageL, img1p, 0, 0, NumDisparities, 0, IPL_BORDER_REPLICATE);
    copyMakeBorder(rectifyImageR, img2p, 0, 0, NumDisparities, 0, IPL_BORDER_REPLICATE);
    dispf = disp.colRange(NumDisparities, img2p.cols - NumDisparities);

    dispf.convertTo(disp8, CV_8U, 255 / (NumDisparities *16.));
    reprojectImageTo3D(dispf, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
    xyz = xyz * 16;
    imshow("disparity", disp8);
    Mat color(dispf.size(), CV_8UC3);
    GenerateFalseMap(disp8, color);//转成彩图

    return xyz;
 //   imshow("disparity", color);
 //   saveXYZ("xyz.xls", xyz);
}

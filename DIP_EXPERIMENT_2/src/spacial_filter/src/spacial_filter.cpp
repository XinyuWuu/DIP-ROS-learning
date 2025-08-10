#include <stdlib.h>
#include <string>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#define READIMAGE_ONLY
#ifndef READIMAGE_ONLY
#include <geometry_msgs/Twist.h>
#define CAMERIA 1
#endif
#ifdef READIMAGE_ONLY
#define CAMERIA 0
#endif

using namespace cv;
using namespace std;

double **GaussianKernalGenerator(double sigma, unsigned int kernal_size);
void myGaussianBlur(Mat *Src, Mat *Dst, double **kernal, unsigned int kernal_size);
void myErode(Mat src, Mat dst, bool **kernal, int kernal_size, int center_x, int center_y);
void myDilate(Mat src, Mat dst, bool **kernal, int kernal_size, int center_x, int center_y);
int main(int argc, char *argv[])
{
    ROS_WARN("****START*****");
    ros::init(argc, argv, "trafficLaneTrack"); //初始化ROS节点
    ros::NodeHandle n;

    VideoCapture capture;
    capture.open(CAMERIA); // 1 打开zed相机，如果要打开笔记本上的摄像头，需要改为0
    waitKey(100);
    if (!capture.isOpened())
    {
        printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
        return 0;
    }
    waitKey(1000);

#ifndef READIMAGE_ONLY
    ros::Rate loop_rate(10);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5); //定义dashgo机器人的速度发布器
#endif

    Mat src_frame;
    Mat gray_frame;
    // int nFrames = 0;
    int frameWidth = capture.get(cv::CAP_PROP_FRAME_WIDTH);
    int frameHeight = capture.get(cv::CAP_PROP_FRAME_HEIGHT);
    Mat Gaussian_blured(frameHeight, frameWidth, CV_8UC1);
    Mat Eroded(frameHeight, frameWidth, CV_8UC1);
    Mat Dilated(frameHeight, frameWidth, CV_8UC1);
    Mat test;

    double **gaussian_kernal = GaussianKernalGenerator(5.0, 7);
    unsigned int ED_kernal_size = 7;
    bool **ED_kernal = new bool *[ED_kernal_size];
    for (int i = 0; i < 7; i++)
    {
        ED_kernal[i] = new bool[ED_kernal_size];
    }
    for (unsigned int i = 0; i < ED_kernal_size; i++)
    {
        for (unsigned j = 0; j < ED_kernal_size; j++)
        {
            ED_kernal[i][j] = true;
        }
    }
    Mat binary;
    Mat frln;
    // double sum = 0;
    // for (int i = 0; i < 7; i++)
    // {
    //     for (int j = 0; j < 7; j++)
    //     {
    //         sum += gaussian_kernal[i][j];
    //         printf("%f ", gaussian_kernal[i][j]);
    //     }
    //     printf("\r\n");
    // }
    // printf("%f\r\n", sum);
    while (ros::ok())
    {
        capture.read(src_frame);
        if (src_frame.empty())
        {
            break;
        }
        cvtColor(src_frame, gray_frame, cv::COLOR_BGR2GRAY);
#ifndef READIMAGE_ONLY
        frln = gray_frame(cv::Rect(0, 0, gray_frame.cols / 2, gray_frame.rows));
#endif
#ifdef READIMAGE_ONLY
        frln = gray_frame.clone();
#endif
        imshow("src", frln);
        myGaussianBlur(&frln, &Gaussian_blured, gaussian_kernal, 7);
        imshow("guassian blured", Gaussian_blured);
        // GaussianBlur(frln, test, cv::Size(7, 7), 5, 5);
        // imshow("test", test);

        //threshold(frln, binary, 32, 255, THRESH_BINARY);
        threshold(frln, binary, 32, 255, THRESH_BINARY_INV);
        // adaptiveThreshold(frln, binary, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 11, 1);
        // adaptiveThreshold(frln, binary, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 11, 1);
        imshow("binary", binary);

        myErode(binary, Eroded, ED_kernal, ED_kernal_size, 2, 2);
        imshow("Eroded", Eroded);

        myDilate(binary, Dilated, ED_kernal, ED_kernal_size, 2, 2);
        imshow("Dilated", Dilated);
#ifndef READIMAGE_ONLY
        ros::spinOnce();
#endif
        waitKey(5);
    }

    return 0;
}

double **GaussianKernalGenerator(double sigma, unsigned int kernal_size)
{
    unsigned int max_kernal_size = ceil<unsigned int>(6 * sigma);
    if (kernal_size > max_kernal_size)
        kernal_size = max_kernal_size;
    if (kernal_size % 2 == 0)
    {
        kernal_size += 1;
    }
    double **kernal = new double *[kernal_size];
    for (unsigned int i = 0; i < kernal_size; i++)
    {
        kernal[i] = new double[kernal_size];
    }
    unsigned int dis_i, dis_j;
    unsigned int center_index = (kernal_size - 1) / 2;
    double pi = 3.141592653589793;
    double sum = 0.0f;

    for (unsigned int i = 0; i < kernal_size; i++)
    {

        for (unsigned int j = 0; j < kernal_size; j++)
        {
            dis_i = i > center_index ? i - center_index : center_index - i;
            dis_j = j > center_index ? j - center_index : center_index - j;
            kernal[i][j] =
                exp(
                    -(1.0f) * ((dis_i * dis_i + dis_j * dis_j) /
                               (2.0f * sigma * sigma)));
            sum += kernal[i][j];
        }
    }
    for (unsigned int i = 0; i < kernal_size; i++)
    {
        for (unsigned int j = 0; j < kernal_size; j++)
        {

            kernal[i][j] /= sum;
        }
    }
    return kernal;
}

void myGaussianBlur(Mat *Src, Mat *Dst, double **kernal, unsigned int kernal_size)
{
    double sum;
    unsigned int center_index = (kernal_size - 1) / 2;
    for (unsigned int i = 0; i < Src->rows; i++)
    {
        for (unsigned int j = 0; j < Src->cols; j++)
        {
            sum = 0;
            for (unsigned int m = 0; m <= center_index; m++)
            {
                for (unsigned int n = 0; n <= center_index; n++)
                {
                    if (n == 0 && m == 0)
                    {
                        sum += Src->at<uchar>(i, j) * kernal[center_index][center_index];
                        continue;
                    }
                    else if (m == 0)
                    {
                        if (j >= n)
                        {
                            sum += Src->at<uchar>(i, j - n) * kernal[center_index][center_index - n];
                        }
                        if (j + n < Src->cols)
                        {
                            sum += Src->at<uchar>(i, j + n) * kernal[center_index][center_index + n];
                        }
                    }
                    else if (n == 0)
                    {
                        if (i >= m)
                        {
                            sum += Src->at<uchar>(i - m, j) * kernal[center_index - m][center_index];
                        }
                        if (i + m < Src->rows)
                        {
                            sum += Src->at<uchar>(i + m, j) * kernal[center_index + m][center_index];
                        }
                    }
                    else
                    {
                        if (i >= m && j >= n)
                        {
                            sum += Src->at<uchar>(i - m, j - n) * kernal[center_index - m][center_index - n];
                        }
                        if (i >= m && j + n < Src->cols)
                        {
                            sum += Src->at<uchar>(i - m, j + n) * kernal[center_index - m][center_index + n];
                        }
                        if (i + m < Src->rows && j + n < Src->cols)
                        {
                            sum += Src->at<uchar>(i + m, j + n) * kernal[center_index + m][center_index + n];
                        }
                        if (i + m < Src->rows && j >= n)
                        {
                            sum += Src->at<uchar>(i + m, j - n) * kernal[center_index + m][center_index - n];
                        }
                    }
                }
            }
            Dst->at<uchar>(i, j) = uchar(sum + 0.5);
        }
    }
}

void myErode(Mat src, Mat dst, bool **kernal, int kernal_size, int center_x, int center_y)
{
    bool flag;
    unsigned int i, j, m, n;
    for (i = 0; i < src.rows; i++)
    {
        for (j = 0; j < src.cols; j++)
        {
            flag = true;
            for (m = 0; m < kernal_size; m++)
            {
                for (n = 0; n < kernal_size; n++)
                {
                    if (kernal[m][n] && i + m >= center_x && i + m - center_x < src.rows && j + n >= center_y && j + n - center_y < src.cols)
                    {
                        flag = bool(src.at<uchar>(i + m - center_x, j + n - center_y)) && flag;
                    }
                }
            }
            if (flag)
            {
                dst.at<uchar>(i, j) = 255;
            }
            else
            {
                dst.at<uchar>(i, j) = 0;
            }
        }
    }
}

void myDilate(Mat src, Mat dst, bool **kernal, int kernal_size, int center_x, int center_y)
{
    bool flag;
    unsigned int i, j, m, n;
    for (i = 0; i < src.rows; i++)
    {
        for (j = 0; j < src.cols; j++)
        {
            flag = false;
            for (m = 0; m < kernal_size; m++)
            {
                for (n = 0; n < kernal_size; n++)
                {
                    if (kernal[m][n] && i + m >= center_x && i + m - center_x < src.rows && j + n >= center_y && j + n - center_y < src.cols)
                    {
                        flag = bool(src.at<uchar>(i + m - center_x, j + n - center_y)) || flag;
                    }
                }
            }
            if (flag)
            {
                dst.at<uchar>(i, j) = 255;
            }
            else
            {
                dst.at<uchar>(i, j) = 0;
            }
        }
    }
}

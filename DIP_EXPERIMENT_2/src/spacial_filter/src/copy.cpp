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

void Gaussian(Mat input, Mat output, double sigma)
{
    double g1 = exp(-1 / (2 * sigma * sigma));
    double g2 = exp(-2 / (2 * sigma * sigma));
    double data[3][3] = {{g2, g1, g2}, {g1, 1, g1}, {g2, g1, g2}};
    double value = 0;
    for (int i = 1; i < input.rows - 1; i++)
    {
        uchar *outp = output.ptr<uchar>(i);
        for (int j = 1; j < input.cols - 1; j++)
        {
            value = 0;
            for (int a = 0; a < 3; a++)
            {
                uchar *p = input.ptr<uchar>(a + i - 1);
                for (int b = 0; b < 3; b++)
                {
                    value += p[b + j - 1] * data[a][b] / (4 * g1 + 4 * g2 + 1);
                }
            }
            outp[j] = value;
        }
    }
}

void Dilate(Mat Src, Mat Tem, Mat Dst, int x, int y)
{
    Mat Dst_inv = Dst.clone();
    int y_inv = Dst.cols - y - 1;
    int x_inv = Dst.rows - x - 1;
    int dst_inv[Dst.rows][Dst.cols];
    for (int i = 0; i < Dst.rows; i++)
    {
        uchar *p = Dst.ptr<uchar>(i);
        uchar *q = Dst_inv.ptr<uchar>(x_inv - i + x);
        for (int j = 0; j < Dst.cols; j++)
        {
            q[y_inv - j + y] = p[j];
        }
    }

    int count = 0;

    int count1 = 0;
    for (int i = 0; i < Src.rows; i++)
    {
        uchar *outptr = Src.ptr<uchar>(i);
        for (int j = 0; j < Src.cols; j++)
        {
            if (outptr[j] == 255)
            {
                count++;
            }
            if (outptr[j] == 0)
            {
                count1++;
            }
            //ROS_INFO("src:%d", outptr[j]);
        }
    }

    for (int i = x_inv; i < Src.rows - Dst_inv.rows + x_inv + 1; i++)
    {
        uchar *outptr = Tem.ptr<uchar>(i);
        for (int j = y_inv; j < Src.cols - Dst_inv.cols + y_inv + 1; j++)
        {
            count = 0;
            for (int a = 0; a < Dst_inv.rows; a++)
            {
                uchar *p = Src.ptr<uchar>(i + a - x_inv);
                uchar *q = Dst_inv.ptr<uchar>(a);
                for (int b = 0; b < Dst_inv.cols; b++)
                {
                    count += p[j + b - y_inv] * q[b];
                }
            }
            if (count > 0)
            {
                outptr[j] = 255;
            }
            else
            {
                outptr[j] = 0;
            }
        }
    }
}

void Erode(Mat Src, Mat Tem, Mat Dst, int x, int y)
{

    bool flag = false;
    for (int i = x; i < Src.rows - Dst.rows + x + 1; i++)
    {
        uchar *outptr = Tem.ptr<uchar>(i);
        for (int j = y; j < Src.cols - Dst.cols + y + 1; j++)
        {
            flag = false;
            for (int a = 0; a < Dst.rows; a++)
            {
                uchar *p = Src.ptr<uchar>(i + a - x);
                uchar *q = Dst.ptr<uchar>(a);
                for (int b = 0; b < Dst.cols; b++)
                {
                    if (p[j + b - y] * q[b] < 127 && q[b] == 1)
                    {
                        flag = true;
                        break;
                    }
                }
            }
            if (flag)
            {
                outptr[j] = 0;
            }
            else
            {
                outptr[j] = 255;
            }
        }
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ROS_WARN("*****START*****");
    ros::init(argc, argv, "trafficLaneTrack"); //初始化ROS节点
    ros::NodeHandle n;

    //Before the use of camera, you can test ur program with images first: imread()
    VideoCapture capture;
    ROS_INFO("ros is running");
    capture.open(0); //打开zed相机，如果要打开笔记本上的摄像头，需要改为0
    ROS_INFO("camera is opening");
    waitKey(100);
    if (!capture.isOpened())
    {
        printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
        return 0;
    }

#ifndef READIMAGE_ONLY
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);
    //定义dashgo机器人的速度发布器
#endif
    Mat src_frame;
    Mat grey;
    Mat guassian;
    Mat dilate_image;
    Mat erode_image;
    Mat bin_image;
    Mat dilate_kernel = Mat::ones(5, 5, CV_8UC1);
    Mat erode_kernel = Mat::ones(5, 5, CV_8UC1);
    while (ros::ok())
    {
        capture.read(src_frame);
        if (src_frame.empty())
        {
            break;
        }
        cvtColor(src_frame, grey, COLOR_BGR2GRAY);

        imshow("src_pic", grey);

        guassian = grey.clone();
        Gaussian(grey, guassian, 1);
        imshow("filter_pic", guassian);
        bin_image = grey.clone();
        threshold(grey, bin_image, 100, 255, cv::THRESH_BINARY);
        imshow("bin_pic", bin_image);

        int structdata[2][2] = {{1, 1}, {1, 0}};
        // Mat dilate_kernel=Mat(2,2,CV_8UC1,structdata);
        
        // Mat dilate_kernel = (Mat_<int>(2, 2) << 1, 1, 1, 1);
        // for(int i=0;i<dilate_kernel.rows;i++){
        //     const uchar* p=dilate_kernel.ptr<uchar>(i);
        //     for(int j=0;j<dilate_kernel.cols;j++){
        //         ROS_INFO("p:%d",p[j]);
        //     }
        // }
        dilate_image = bin_image.clone();
        Dilate(bin_image, dilate_image, dilate_kernel, 0, 0);
        imshow("dilate_pic", dilate_image);

        
        // erode_kernel.at<uchar>(0,1)=1;
        erode_image = bin_image.clone();
        Erode(bin_image, erode_image, erode_kernel, 1, 1);
        imshow("erode_pic", erode_image);

#ifndef READIMAGE_ONLY
        //以下代码可设置机器人的速度值，从而控制机器人运动
        geometry_msgs::Twist cmd_red;
        cmd_red.linear.x = 0;
        cmd_red.linear.y = 0;
        cmd_red.linear.z = 0;
        cmd_red.angular.x = 0;
        cmd_red.angular.y = 0;
        cmd_red.angular.z = 0.2;
        pub.publish(cmd_red);
#endif
#ifndef READIMAGE_ONLY
        ros::spinOnce();
#endif
        waitKey(5);
    }
    return 0;
}

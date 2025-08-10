#include <stdlib.h>
#include <string>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <boost/thread.hpp>

// #define READIMAGE_ONLY
#ifndef READIMAGE_ONLY
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#define CAMERIA 1
#define SPEED 1
#define SPIN 0.3
#endif
#ifdef READIMAGE_ONLY
#define CAMERIA 0
#endif

using namespace cv;
using namespace std;
void RGB_to_HSI(Mat src, Mat dst);
void RGB_to_HSV(Mat src, Mat dst);
void HSV_threshold(Mat **HSV_frame);
void find_color_block(Mat **HSV_frame, Scalar low, Scalar upper);
void HSVcallBack(int, void *);
// int HSV_H_min, HSV_S_min, HSV_V_min, HSV_H_max = 360, HSV_S_max = 255, HSV_V_max = 255;
// int HSV_H_min = 270, HSV_S_min = 80, HSV_V_min = 30, HSV_H_max = 320, HSV_S_max = 250, HSV_V_max = 70; //face
int HSV_H_min = 100, HSV_S_min = 43, HSV_V_min = 46, HSV_H_max = 180, HSV_S_max = 255, HSV_V_max = 255; //red
int HSI_H_min, HSI_S_min, HSI_V_min, HSI_H_max, HSI_S_max, HSI_V_max;
int main(int argc, char **argv)
{

    ROS_WARN("*****START*******");
    ros::init(argc, argv, "colorDetection"); //初始化 ROS 节点
    ros::NodeHandle n;
#ifndef READIMAGE_ONLY
    ros::Rate loop_rate(10);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5); //定义dashgo机器人的速度发布器
    geometry_msgs::Twist cmd_red;
#endif
    VideoCapture capture;
    capture.open(CAMERIA); // 1 打开zed相机，如果要打开笔记本上的摄像头，需要改为0
    if (!capture.isOpened())
    {
        printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
        return 0;
    }
    waitKey(100);
    Mat src_frame;                                        //当前帧图片
    int nFrames = 0;                                      //图片帧数
    int frameWidth = capture.get(CAP_PROP_FRAME_WIDTH);   //图片宽
    int frameHeight = capture.get(CAP_PROP_FRAME_HEIGHT); //图片高
    Mat HSI_frame(frameHeight, frameWidth, CV_64FC3);
    Mat HSV_frame(frameHeight, frameWidth, CV_64FC3);
    Mat HSI_test, HSV_test;
    Mat HSV_index, HSV_Adjust(frameHeight, frameWidth, CV_8UC3), HSV_Adjust_gray;
    Mat *HSV_data[] = {&HSV_frame, &src_frame, &HSV_index, &HSV_Adjust};
    vector<vector<Point>> HSV_contour;
    vector<Vec4i> HSV_hierarchy;
    vector<Rect> boundRect;
    unsigned int max_rect_size, max_rect_index, red, green, blue, yellow, max_color;
    Mat Histogram(550, 500, CV_8UC3);
    namedWindow("HSV Adjust", WINDOW_GUI_EXPANDED);
    createTrackbar("hmin", "HSV Adjust", &HSV_H_min, 180);
    createTrackbar("hmax", "HSV Adjust", &HSV_H_max, 180);
    createTrackbar("smin", "HSV Adjust", &HSV_S_min, 255);
    createTrackbar("smax", "HSV Adjust", &HSV_S_max, 255);
    createTrackbar("vmin", "HSV Adjust", &HSV_V_min, 255);
    createTrackbar("vmax", "HSV Adjust", &HSV_V_max, 255);
    while (ros::ok())
    {
        capture.read(src_frame);
        if (src_frame.empty())
        {
            break;
        }
        imshow("src", src_frame);
        RGB_to_HSI(src_frame, HSI_frame);
        imshow("HSI", HSI_frame);
        RGB_to_HSV(src_frame, HSV_frame);
        imshow("HSV", HSV_frame);
        HSV_threshold(HSV_data);                               //根据滑动条的值进行分割
        cvtColor(HSV_Adjust, HSV_Adjust_gray, COLOR_BGR2GRAY); //将分割后的图像转为灰度图
        HSV_contour.clear();
        findContours(HSV_Adjust_gray, HSV_contour, HSV_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); //轮廓检测
        boundRect.clear();
        max_rect_size = 0;
        for (int i = 0; i < HSV_contour.size(); i++)
        {
            boundRect.push_back(boundingRect(HSV_contour[i])); //对轮廓生成矩形边界
            if (boundRect[i].area() > max_rect_size)
            {
                max_rect_size = boundRect[i].area();
                max_rect_index = i;
            }
        }
        rectangle(HSV_Adjust, boundRect[max_rect_index].tl(), boundRect[max_rect_index].br(), (0, 0, 255), 2); //画出最大的矩形边界
        imshow("HSV Adjust", HSV_Adjust);

        red = green = blue = yellow = 0;
        Histogram = Mat::zeros(550, 500, CV_8UC3);
        //*****统计最大矩形边界内的像素颜色
        for (unsigned int i = boundRect[max_rect_index].tl().y; i < boundRect[max_rect_index].br().y; i++)
        {
            for (unsigned int j = boundRect[max_rect_index].tl().x; j < boundRect[max_rect_index].br().x; j++)
            {
                // cout << HSV_frame.at<Vec3b>(i, j)[1] << "," << HSV_frame.at<Vec3b>(i, j)[2] << endl;
                if ((HSV_frame.at<Vec3d>(i, j)[1] > double(43) / 255 && HSV_frame.at<Vec3d>(i, j)[1] < 1) && (HSV_frame.at<Vec3d>(i, j)[2] > double(46) / 255 && HSV_frame.at<Vec3d>(i, j)[2] < 1))
                {
                    // cout << HSV_frame.at<Vec3d>(i, j)[0] * 180 << endl;
                    if ((HSV_frame.at<Vec3d>(i, j)[0] * 180 > 150.0))
                    {
                        red += 1;
                    }
                    else if ((HSV_frame.at<Vec3d>(i, j)[0] * 180 < 91 && HSV_frame.at<Vec3d>(i, j)[0] * 180 > 40))
                    {
                        green += 1;
                    }
                    else if ((HSV_frame.at<Vec3d>(i, j)[0] * 180 < 140 && HSV_frame.at<Vec3d>(i, j)[0] * 180 > 100))
                    {
                        blue += 1;
                    }
                    else if (HSV_frame.at<Vec3d>(i, j)[0] * 180 < 60)
                    {
                        yellow += 1;
                    }
                }
            }
        }
        max_color = MAX(red, MAX(green, MAX(blue, yellow)));
        rectangle(Histogram, Rect(50, 550 - int(double(red) / max_color * 500.0 + 0.5), 100, int(double(red) / max_color * 500.0 + 0.5)), CV_RGB(255, 0, 0), 5);
        rectangle(Histogram, Rect(150, 550 - int(double(green) / max_color * 500.0 + 0.5), 100, int(double(green) / max_color * 500.0 + 0.5)), CV_RGB(0, 255, 0), 5);
        rectangle(Histogram, Rect(250, 550 - int(double(blue) / max_color * 500.0 + 0.5), 100, int(double(blue) / max_color * 500.0 + 0.5)), CV_RGB(0, 0, 255), 5);
        rectangle(Histogram, Rect(350, 550 - int(double(yellow) / max_color * 500.0 + 0.5), 100, int(double(yellow) / max_color * 500.0 + 0.5)), CV_RGB(255, 255, 0), 5);
        imshow("histogram", Histogram);

        find_color_block(HSV_data, Scalar(152.0 / 180, 120.0 / 255, 46.0 / 255), Scalar(1, 1, 1));
        cvtColor(HSV_Adjust, HSV_Adjust_gray, COLOR_BGR2GRAY); //将分割后的图像转为灰度图
        HSV_contour.clear();
        findContours(HSV_Adjust_gray, HSV_contour, HSV_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); //轮廓检测
        boundRect.clear();
        max_rect_size = 0;
        for (int i = 0; i < HSV_contour.size(); i++)
        {
            boundRect.push_back(boundingRect(HSV_contour[i])); //对轮廓生成矩形边界
            if (boundRect[i].area() > max_rect_size)
            {
                max_rect_size = boundRect[i].area();
                max_rect_index = i;
            }
        }
        rectangle(HSV_Adjust, boundRect[max_rect_index].tl(), boundRect[max_rect_index].br(), CV_RGB(255, 0, 0), 2); //画出最大的矩形边界
        imshow("red", HSV_Adjust);
        red = max_rect_size;

        find_color_block(HSV_data, Scalar(40.0 / 180, 120.0 / 255, 46.0 / 255), Scalar(91.0 / 180, 1, 1));
        cvtColor(HSV_Adjust, HSV_Adjust_gray, COLOR_BGR2GRAY); //将分割后的图像转为灰度图
        HSV_contour.clear();
        findContours(HSV_Adjust_gray, HSV_contour, HSV_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); //轮廓检测
        boundRect.clear();
        max_rect_size = 0;
        for (int i = 0; i < HSV_contour.size(); i++)
        {
            boundRect.push_back(boundingRect(HSV_contour[i])); //对轮廓生成矩形边界
            if (boundRect[i].area() > max_rect_size)
            {
                max_rect_size = boundRect[i].area();
                max_rect_index = i;
            }
        }
        rectangle(HSV_Adjust, boundRect[max_rect_index].tl(), boundRect[max_rect_index].br(), CV_RGB(255, 0, 0), 2); //画出最大的矩形边界
        imshow("green", HSV_Adjust);
        green = max_rect_size;

        find_color_block(HSV_data, Scalar(93.0 / 180, 120.0 / 255, 46.0 / 255), Scalar(115.0 / 180, 1, 1));
        cvtColor(HSV_Adjust, HSV_Adjust_gray, COLOR_BGR2GRAY); //将分割后的图像转为灰度图
        HSV_contour.clear();
        findContours(HSV_Adjust_gray, HSV_contour, HSV_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); //轮廓检测
        boundRect.clear();
        max_rect_size = 0;
        for (int i = 0; i < HSV_contour.size(); i++)
        {
            boundRect.push_back(boundingRect(HSV_contour[i])); //对轮廓生成矩形边界
            if (boundRect[i].area() > max_rect_size)
            {
                max_rect_size = boundRect[i].area();
                max_rect_index = i;
            }
        }
        rectangle(HSV_Adjust, boundRect[max_rect_index].tl(), boundRect[max_rect_index].br(), CV_RGB(255, 0, 0), 2); //画出最大的矩形边界
        imshow("blue", HSV_Adjust);
        blue = max_rect_size;

        find_color_block(HSV_data, Scalar(0, 120.0 / 255, 46.0 / 255), Scalar(10.0 / 180, 1, 1));
        cvtColor(HSV_Adjust, HSV_Adjust_gray, COLOR_BGR2GRAY); //将分割后的图像转为灰度图
        HSV_contour.clear();
        findContours(HSV_Adjust_gray, HSV_contour, HSV_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); //轮廓检测
        boundRect.clear();
        max_rect_size = 0;
        for (int i = 0; i < HSV_contour.size(); i++)
        {
            boundRect.push_back(boundingRect(HSV_contour[i])); //对轮廓生成矩形边界
            if (boundRect[i].area() > max_rect_size)
            {
                max_rect_size = boundRect[i].area();
                max_rect_index = i;
            }
        }
        rectangle(HSV_Adjust, boundRect[max_rect_index].tl(), boundRect[max_rect_index].br(), CV_RGB(255, 0, 0), 2); //画出最大的矩形边界
        imshow("yellow", HSV_Adjust);
        yellow = max_rect_size;
#ifndef READIMAGE_ONLY
        // 车的速度值设置
        max_rect_size = MAX(red, MAX(green, MAX(blue, yellow)));
        if (max_rect_size > 100000)
        {
            if (max_rect_size == red)
            {
                cmd_red.linear.x = SPEED;
                cmd_red.linear.y = 0;
                cmd_red.linear.z = 0;
                cmd_red.angular.x = 0;
                cmd_red.angular.y = 0;
                cmd_red.angular.z = 0.0;
            }
            else if (max_rect_size == green)
            {
                cmd_red.linear.x = -SPEED;
                cmd_red.linear.y = 0;
                cmd_red.linear.z = 0;
                cmd_red.angular.x = 0;
                cmd_red.angular.y = 0;
                cmd_red.angular.z = 0.0;
            }
            else if (max_rect_size == blue)
            {
                cmd_red.linear.x = SPEED;
                cmd_red.linear.y = 0;
                cmd_red.linear.z = 0;
                cmd_red.angular.x = 0;
                cmd_red.angular.y = 0;
                cmd_red.angular.z = SPIN;
            }
            else if (max_rect_size == yellow)
            {
                cmd_red.linear.x = SPEED;
                cmd_red.linear.y = 0;
                cmd_red.linear.z = 0;
                cmd_red.angular.x = 0;
                cmd_red.angular.y = 0;
                cmd_red.angular.z = -SPIN;
            }
        }
        else
        {
            cmd_red.linear.x = 0;
            cmd_red.linear.y = 0;
            cmd_red.linear.z = 0;
            cmd_red.angular.x = 0;
            cmd_red.angular.y = 0;
            cmd_red.angular.z = 0.0;
        }
        pub.publish(cmd_red);
        ros::spinOnce();
#endif
        // char key;
        // while (key != 'p')
        // {
        //     key = cv::waitKey(0);
        // }
        waitKey(5);
        // return 0;
    }
    return 0;
}

void RGB_to_HSI(Mat src, Mat dst)
{
    double theta;
    for (unsigned int i = 0; i < src.rows; i++)
    {
        for (unsigned int j = 0; j < src.cols; j++)
        {
            theta = acos(((2.0 * src.at<Vec3b>(i, j)[2] - src.at<Vec3b>(i, j)[1] - src.at<Vec3b>(i, j)[0]) / 2) /
                         sqrt(double(src.at<Vec3b>(i, j)[2] - src.at<Vec3b>(i, j)[1]) * (src.at<Vec3b>(i, j)[2] - src.at<Vec3b>(i, j)[1]) + (src.at<Vec3b>(i, j)[2] - src.at<Vec3b>(i, j)[0]) * (src.at<Vec3b>(i, j)[1] - src.at<Vec3b>(i, j)[0])));
            dst.at<Vec3d>(i, j)[0] = (src.at<Vec3b>(i, j)[1] > src.at<Vec3b>(i, j)[0] ? theta : 2 * 3.1415926535 - theta) / 2 / 3.1415926535;
            dst.at<Vec3d>(i, j)[1] = 1.0 - 3.0 / (src.at<Vec3b>(i, j)[2] + src.at<Vec3b>(i, j)[1] + src.at<Vec3b>(i, j)[0]) * MIN(src.at<Vec3b>(i, j)[2], MIN(src.at<Vec3b>(i, j)[1], src.at<Vec3b>(i, j)[0]));
            dst.at<Vec3d>(i, j)[2] = double(src.at<Vec3b>(i, j)[2] + src.at<Vec3b>(i, j)[1] + src.at<Vec3b>(i, j)[0]) / 3 / 255;
        }
    }
}

void RGB_to_HSV(Mat src, Mat dst)
{
    unsigned int min_value, max_value;

    for (unsigned int i = 0; i < src.rows; i++)
    {
        for (unsigned int j = 0; j < src.cols; j++)
        {
            min_value = MIN(src.at<Vec3b>(i, j)[0], MIN(src.at<Vec3b>(i, j)[1], src.at<Vec3b>(i, j)[2]));
            max_value = MAX(src.at<Vec3b>(i, j)[0], MAX(src.at<Vec3b>(i, j)[1], src.at<Vec3b>(i, j)[2]));
            dst.at<Vec3d>(i, j)[2] = max_value / 255.0;
            dst.at<Vec3d>(i, j)[1] = (dst.at<Vec3d>(i, j)[2] - min_value / 255.0) / dst.at<Vec3d>(i, j)[2];
            if (src.at<Vec3b>(i, j)[2] == max_value)
            {
                if (src.at<Vec3b>(i, j)[1] == min_value)
                {
                    dst.at<Vec3d>(i, j)[0] = 1.0 / 6 * (5 + src.at<Vec3b>(i, j)[0] / 255.0);
                }
                else
                {
                    dst.at<Vec3d>(i, j)[0] = 1.0 / 6 * (1 - src.at<Vec3b>(i, j)[1] / 255.0);
                }
            }
            else if (src.at<Vec3b>(i, j)[1] == max_value)
            {
                if (src.at<Vec3b>(i, j)[0] == min_value)
                {
                    dst.at<Vec3d>(i, j)[0] = 1.0 / 6 * (1 + src.at<Vec3b>(i, j)[2] / 255.0);
                }
                else
                {
                    dst.at<Vec3d>(i, j)[0] = 1.0 / 6 * (3 - src.at<Vec3b>(i, j)[0] / 255.0);
                }
            }
            else
            {
                if (src.at<Vec3b>(i, j)[2] == min_value)
                {
                    dst.at<Vec3d>(i, j)[0] = 1.0 / 6 * (3 + src.at<Vec3b>(i, j)[1] / 255.0);
                }
                else
                {
                    dst.at<Vec3d>(i, j)[0] = 1.0 / 6 * (5 - src.at<Vec3b>(i, j)[2] / 255.0);
                }
            }
        }
    }
}

void HSV_threshold(Mat **HSV_frame)
{
    inRange(*(HSV_frame[0]), Scalar(HSV_H_min / 180.0, HSV_S_min / 255.0, HSV_V_min / 180.0), Scalar(HSV_H_max / 180.0, HSV_S_max / 255.0, HSV_V_max / 255.0), *(HSV_frame[2]));
    for (int r = 0; r < ((Mat **)(HSV_frame))[0]->rows; r++)
    {

        for (int c = 0; c < HSV_frame[0]->cols; c++)
        {
            if (HSV_frame[2]->at<uchar>(r, c) == 255)
            {
                HSV_frame[3]->at<Vec3b>(r, c) = HSV_frame[1]->at<Vec3b>(r, c);
            }
            else
            {
                HSV_frame[3]->at<Vec3b>(r, c) = {0, 0, 0};
            }
        }
    }
}

void find_color_block(Mat **HSV_frame, Scalar low, Scalar upper)
{
    inRange(*(HSV_frame[0]), low, upper, *(HSV_frame[2]));
    for (int r = 0; r < HSV_frame[0]->rows; r++)
    {
        for (int c = 0; c < HSV_frame[0]->cols; c++)
        {
            if (HSV_frame[2]->at<uchar>(r, c) == 255)
            {
                HSV_frame[3]->at<Vec3b>(r, c) = HSV_frame[1]->at<Vec3b>(r, c);
            }
            else
            {
                HSV_frame[3]->at<Vec3b>(r, c) = {0, 0, 0};
            }
        }
    }
}

void HSVcallBack(int, void *HSV_frame)
{
    inRange(*((Mat **)(HSV_frame))[0], Scalar(HSV_H_min / 180.0, HSV_S_min / 255.0, HSV_V_min / 255.0), Scalar(HSV_H_max / 180.0, HSV_S_max / 255.0, HSV_V_max / 255.0), *((Mat **)(HSV_frame))[2]);
    for (int r = 0; r < ((Mat **)(HSV_frame))[0]->rows; r++)
    {

        for (int c = 0; c < ((Mat **)(HSV_frame))[0]->cols; c++)
        {
            if (((Mat **)(HSV_frame))[2]->at<uchar>(r, c) == 255)
            {
                ((Mat **)(HSV_frame))[3]->at<Vec3b>(r, c) = ((Mat **)(HSV_frame))[1]->at<Vec3b>(r, c);
            }
            else
            {
                ((Mat **)(HSV_frame))[3]->at<Vec3b>(r, c) = {0, 0, 0};
            }
        }
    }
}

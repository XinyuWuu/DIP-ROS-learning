#include <stdlib.h>
// #include <cv.h>
// #include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"
#include <algorithm>
using namespace cv;
using namespace std;

int h_low = 0, h_high = 180, s_low = 0, s_high = 255, v_low = 0, v_high = 255;
int h_max = 180;
int h_min = 0;
int s_max = 255;
int s_min = 0;
int v_max = 255;
int v_min = 0;

Mat hsv; //全局变量，保证回调函数的调用
Mat seg; //分割后的图像

int th[6] = {137, 180, 141, 255, 0, 50};
int Max = 0;
int submax = 0;
double area_pro;
double aim_area_pro = 0.1;
RotatedRect MAX_RECT, SUB_MAX_RECT;
Point2f midpoint(0, 0);
void on_ThreshChange_h_low(int h_low, void *)
{
	seg = Mat::zeros(hsv.size(), CV_32FC3);
	inRange(hsv, Scalar(h_low, s_low, v_low), Scalar(h_high, s_high, v_high), seg);
	imshow("hsv", seg);
}
void on_ThreshChange_h_high(int h_high, void *)
{
	seg = Mat::zeros(hsv.size(), CV_32FC3);
	inRange(hsv, Scalar(h_low, s_low, v_low), Scalar(h_high, s_high, v_high), seg);
	imshow("hsv", seg);
}
void on_ThreshChange_s_low(int s_low, void *)
{
	seg = Mat::zeros(hsv.size(), CV_32FC3);
	inRange(hsv, Scalar(h_low, s_low, v_low), Scalar(h_high, s_high, v_high), seg);
	imshow("hsv", seg);
}
void on_ThreshChange_s_high(int s_high, void *)
{
	seg = Mat::zeros(hsv.size(), CV_32FC3);
	inRange(hsv, Scalar(h_low, s_low, v_low), Scalar(h_high, s_high, v_high), seg);
	imshow("hsv", seg);
}
void on_ThreshChange_v_low(int v_low, void *)
{
	seg = Mat::zeros(hsv.size(), CV_32FC3);
	inRange(hsv, Scalar(h_low, s_low, v_low), Scalar(h_high, s_high, v_high), seg);
	imshow("hsv", seg);
}
void on_ThreshChange_v_high(int v_high, void *)
{
	seg = Mat::zeros(hsv.size(), CV_32FC3);
	inRange(hsv, Scalar(h_low, s_low, v_low), Scalar(h_high, s_high, v_high), seg);
	imshow("hsv", seg);
}

void trackbar()
{
	createTrackbar("h_low", "hsv", &h_low, h_max, on_ThreshChange_h_low, &hsv);
	createTrackbar("h_high", "hsv", &h_high, h_max, on_ThreshChange_h_high, &hsv);
	createTrackbar("s_low", "hsv", &s_low, s_max, on_ThreshChange_s_low, &hsv);
	createTrackbar("s_high", "hsv", &s_high, s_max, on_ThreshChange_s_high, &hsv);
	createTrackbar("v_low", "hsv", &v_low, v_max, on_ThreshChange_v_low, &hsv);
	createTrackbar("v_high", "hsv", &v_high, v_max, on_ThreshChange_v_high, &hsv);
}

void Contours(Mat src, Mat dst, Mat open, Mat kernel, Mat imageContours, Mat Contours_)
{
	// 开运算
	morphologyEx(src, open, MORPH_OPEN, kernel);
	//获取轮廓
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(open, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
	// for (int i = 0; i < contours.size(); i++)
	// {
	// 	//contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
	// 	for (int j = 0; j < contours[i].size(); j++)
	// 	{
	// 		//绘制出contours向量内所有的像素点
	// 		Point P = Point(contours[i][j].x, contours[i][j].y);
	// 		Contours_.at<uchar>(P) = 255;
	// 	}
	// 	//绘制轮廓
	// 	// drawContours(imageContours, contours, -1, Scalar(255), 1, 8, hierarchy);
	// }

	vector<RotatedRect> minRect(contours.size()); //存储拟合点集
	//拟合
	for (int i = 0; i < contours.size(); i++)
	{
		//拟合
		minRect[i] = minAreaRect(contours[i]);
	}
	//get the max 2 rect
	int area[minRect.size()];
	Max = 0;
	submax = 0;
	for (int i = 0; i < minRect.size(); i++)
	{
		area[i] = minRect[i].size.width * minRect[i].size.height;
		Max = (area[i] > Max) ? (area[i]) : Max;
		MAX_RECT = (area[i] == Max) ? minRect[i] : MAX_RECT;
		submax = (area[i] < Max && area[i] > submax) ? area[i] : submax;
		SUB_MAX_RECT = (area[i] == submax) ? minRect[i] : SUB_MAX_RECT;
	}
}

void colorseg(Mat src, Mat dst, Mat open, Mat kernel, Mat imageContours, Mat Contours_, Mat img_th)
{
	inRange(hsv, Scalar(th[0], th[2], th[4]), Scalar(th[1], th[3], th[5]), img_th);
	int row = img_th.rows;
	int col = img_th.cols;
	int step = row / 10;
	//imshow("hsv",hsv);
	//imshow("dst",img_th);

	for (int i = 0; i < 10; i++)
	{
		Mat cut = img_th(Rect(0, i * step, col, row - i * step));
		Contours(cut, dst, open, kernel, imageContours, Contours_);
		if (Max > 2 * submax)
		{
			submax = 0;
		}
		//画max矩形
		Point2f rect_point[4];
		midpoint = Point2f(0, 0);
		Scalar color = Scalar(0, 0, 255);
		if (submax != 0)
		{
			MAX_RECT.points(rect_point);
			for (int j = 0; j < 4; j++)
			{
				line(dst, Point2f(rect_point[j].x, rect_point[j].y + i * step), Point2f(rect_point[(j + 1) % 4].x, rect_point[(j + 1) % 4].y + i * step), color, 2, 8);
				midpoint.x += rect_point[j].x;
				midpoint.y += rect_point[j].y;
			}

			SUB_MAX_RECT.points(rect_point);
			for (int j = 0; j < 4; j++)
			{
				line(dst, Point2f(rect_point[j].x, rect_point[j].y + i * step), Point2f(rect_point[(j + 1) % 4].x, rect_point[(j + 1) % 4].y + i * step), color, 2, 8);
				midpoint.x += rect_point[j].x;
				midpoint.y += rect_point[j].y;
			}
			area_pro = double(Max) / double((cut.rows * cut.cols));
			midpoint.x /= 8;
			midpoint.y /= 8;

			break;
		}
		else if (i == 9)
		{
			MAX_RECT.points(rect_point);
			for (int j = 0; j < 4; j++)
			{
				line(dst, Point2f(rect_point[j].x, rect_point[j].y + i * step), Point2f(rect_point[(j + 1) % 4].x, rect_point[(j + 1) % 4].y + i * step), color, 2, 8);
				midpoint.x += rect_point[j].x;
				midpoint.y += rect_point[j].y;
			}
			area_pro = double(Max) / 1.3 / double((cut.rows * cut.cols));
			midpoint.x /= 4;
			midpoint.y /= 4;
		}

		//imshow("dst", dst);//展示拟合后的轮廓
		// return dst;
	}
}

int main(int argc, char **argv)
{
	VideoCapture capture;
	capture.open(0); // 1 打开zed相机，如果要打开笔记本上的摄像头，需要改为0
	ROS_WARN("*****START");
	ros::init(argc, argv, "trafficLaneTrack"); //初始化 ROS 节点
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5); //定义dashgo机器人的速度发布器
	geometry_msgs::Twist cmd_red;

	if (!capture.isOpened())
	{
		printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
		return 0;
	}
	waitKey(1000);
	Mat frame;		 //当前帧图片
	int nFrames = 0; //图片帧数
	// int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);	  //图片宽
	// int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT); //图片高
	int frameWidth = capture.get(CAP_PROP_FRAME_WIDTH);	  //图片宽
	int frameHeight = capture.get(CAP_PROP_FRAME_HEIGHT); //图片高
	//开运算
	Mat open;
	Mat kernel = getStructuringElement(MORPH_RECT, Size(7, 7)); //创建结构元素大小为3*3
	Mat dst = Mat::zeros(frameHeight, frameWidth, CV_8UC3);
	Mat imageContours = Mat::zeros(frameHeight, frameWidth, CV_8UC1);
	Mat Contours_ = Mat::zeros(frameHeight, frameWidth, CV_8UC1);
	Mat img_th;
	while (ros::ok())
	{
		capture.read(frame);
		if (frame.empty())
		{
			break;
		}
		Mat src = frame.clone();

		imshow("src", src);
		// cvtColor(src, hsv, BGR2HSV); //RGB TO HSV
		cvtColor(src, hsv, COLOR_BGR2HSV); //RGB TO HSV
		dst = Mat::zeros(frameHeight, frameWidth, CV_8UC3);
		namedWindow("hsv", WINDOW_NORMAL);
		trackbar();

		colorseg(hsv, dst, open, kernel, imageContours, Contours_, img_th);
		imshow("dst", dst);
		cout << "x:" << midpoint.x << " "
			 << "y:" << midpoint.y << " " << area_pro << endl;

		cmd_red.linear.x = (aim_area_pro - area_pro);
		cmd_red.linear.y = 0;
		cmd_red.linear.z = 0;
		cmd_red.angular.x = 0;
		cmd_red.angular.y = 0;
		cmd_red.angular.z = -(midpoint.x - frameWidth / 2.0) / 150.0;
		pub.publish(cmd_red);
		ros::spinOnce();

		waitKey(5);
	}
}

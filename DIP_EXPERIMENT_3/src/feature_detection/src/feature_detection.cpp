
#include <stdlib.h>
#include <string>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#define READIMAGE_ONLY
#ifndef READIMAGE_ONLY
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#define CAMERIA 1
#define LINEAR_X 0
#endif
#ifdef READIMAGE_ONLY
#define CAMERIA 0
#endif
#define READFROMFILE

using namespace cv;
using namespace std;
double **GaussianKernalGenerator(double sigma, unsigned int kernal_size);
void myGaussianBlur(Mat *Src, Mat *Dst, double **kernal, unsigned int kernal_size);

//////////////////////边缘检测//////////////////
//边缘检测函数
void mysobel(Mat src, Mat sobel_value, Mat sobel_alpha, double **kernalx, double **kernaly);
void mycanny(Mat output, Mat sobel_value, Mat sobel_alpha, unsigned int th, unsigned int tl);
//////////////////////霍夫线变换//////////////////
void myhough_lines(Mat src, Mat dst, Mat cannyed, Mat votes, Mat tem);
//////////////////////霍夫圆变换//////////////////
void NMS(Mat src, Mat dst, int box_size, double skip);
void myhough_circles(Mat src, Mat dst, Mat cannyed, Mat sobelx, Mat sobely, Mat circle_center_votes, Mat tem);

int main(int argc, char **argv)
{

    ROS_WARN("*****START*******");
    ros::init(argc, argv, "featureDetection"); //初始化 ROS 节点
    ros::NodeHandle n;
#ifndef READIMAGE_ONLY
    ros::Rate loop_rate(10);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5); //定义dashgo机器人的速度发布器
#endif
#ifndef READFROMFILE
    VideoCapture capture;
    capture.open(CAMERIA); // 1 打开zed相机，如果要打开笔记本上的摄像头，需要改为0
    if (!capture.isOpened())
    {
        printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
        return 0;
    }
    waitKey(100);
#endif
    Mat src_frame; //当前帧图片
    Mat gray_frame;
    Mat frln;
    int nFrames = 0; //图片帧数
#ifdef READFROMFILE
    src_frame = imread("/home/wuxinyu/Documents/ROS/DIP_EXPERIMENT_3/src/feature_detection/src/Lenna_(test_image).png");
    int frameWidth = src_frame.cols;  //图片宽
    int frameHeight = src_frame.rows; //图片高
    cvtColor(src_frame, gray_frame, cv::COLOR_BGR2GRAY);
#endif
#ifndef READFROMFILE
    int frameWidth = capture.get(CAP_PROP_FRAME_WIDTH);   //图片宽
    int frameHeight = capture.get(CAP_PROP_FRAME_HEIGHT); //图片高
#endif
    Mat gaussian_blured(frameHeight, frameWidth, CV_8UC1);
    Mat edge_detected(frameHeight, frameWidth, CV_8UC1);
    Mat sobel_value(frameHeight, frameWidth, CV_8UC1);
    Mat sobel_alpha(frameHeight, frameWidth, CV_8UC1);

    double gaussian_sigma = 2.0;
    int gaussian_kernal_size = int(ceil(int(ceil(gaussian_sigma * 6)) % 2 == 0 ? gaussian_sigma * 6 + 1 : gaussian_sigma * 6));
    double **gaussian_kernal = GaussianKernalGenerator(gaussian_sigma, gaussian_kernal_size);

    double **Gx = new double *[3];
    Gx[0] = new double[3];
    Gx[1] = new double[3];
    Gx[2] = new double[3];
    Gx[0][0] = Gx[2][0] = -1;
    Gx[1][0] = -2;
    Gx[0][1] = Gx[1][1] = Gx[2][1] = 0;
    Gx[0][2] = Gx[2][2] = 1;
    Gx[1][2] = 2;

    double **Gy = new double *[3];
    Gy[0] = new double[3];
    Gy[1] = new double[3];
    Gy[2] = new double[3];
    Gy[0][0] = Gy[0][2] = -1;
    Gy[0][1] = -2;
    Gy[1][0] = Gy[1][1] = Gy[1][2] = 0;
    Gy[2][0] = Gy[2][2] = 1;
    Gy[2][1] = 2;

    // for (int i = 0; i < 3; i++)
    // {
    //     for (int j = 0; j < 3; j++)
    //     {
    //         cout << Gx[i][j] << " ";
    //     }
    //     cout << endl;
    // }
    // for (int i = 0; i < 3; i++)
    // {
    //     for (int j = 0; j < 3; j++)
    //     {
    //         cout << Gy[i][j] << " ";
    //     }
    //     cout << endl;
    // }

    while (ros::ok())
    {
#ifndef READFROMFILE
        capture.read(src_frame);
        if (src_frame.empty())
        {
            break;
        }
        cvtColor(src_frame, gray_frame, cv::COLOR_BGR2GRAY);
#endif
#ifndef READIMAGE_ONLY
        frln = gray_frame(cv::Rect(0, 0, gray_frame.cols / 2, gray_frame.rows));
#endif
#ifdef READIMAGE_ONLY
        frln = gray_frame.clone();
#endif
        imshow("gray src", frln);
        myGaussianBlur(&frln, &gaussian_blured, gaussian_kernal, gaussian_kernal_size);
        imshow("gussian blur", gaussian_blured);
        mysobel(gaussian_blured, sobel_value, sobel_alpha, Gx, Gy);
        imshow("sobel", sobel_value);
#ifndef READFROMFILE
        mycanny(edge_detected, sobel_value, sobel_alpha, 10, 5);
        imshow("mycanny", edge_detected);
#endif
#ifdef READFROMFILE
        mycanny(edge_detected, sobel_value, sobel_alpha, 20, 7);
        imshow("mycanny", edge_detected);
#endif
        // Canny(src_frame, gaussian_blured, 200, 100);
        // imshow("test", gaussian_blured);
        // // 线检测
        // Hough_Line();
        //************圆检测
#ifdef READFROMFILE
        Mat img;
        img = imread("/home/wuxinyu/Documents/drafts/draft_C_C++/C++/lines_and_circles.jpg");
        Mat gray;
        cvtColor(img, gray, COLOR_RGB2GRAY);
        imshow("Gray", gray);
#endif

#ifndef READFROMFILE
        Mat img = src_frame.clone();
        Mat gray;
        cvtColor(img, gray, COLOR_RGB2GRAY);
#endif

        //*******计算sobel梯度
        Mat sobelx, sobely, cannyed;
        Mat sobelx_kernal = (Mat_<double>(3, 3) << -1, -2, -1, 0, 0, 0, 1, 2, 1);
        filter2D(gray, sobelx, CV_64F, sobelx_kernal);
        Mat sobely_kernal = (Mat_<double>(3, 3) << -1, 0, 1, -2, 0, 2, -1, 0, 1);
        filter2D(gray, sobely, CV_64F, sobely_kernal);
        Canny(gray, cannyed, 150, 70);
        imshow("canny", cannyed);
        Mat houghcircles, circle_center_votes, houghlines, tem;
        houghcircles = img.clone();
        houghlines = img.clone();
        myhough_circles(img, houghcircles, cannyed, sobelx, sobely, circle_center_votes, tem);
        imshow("houghcircles", houghcircles);
        Mat votes;
        myhough_lines(img, houghlines, cannyed, votes, tem);
        imshow("hough line", houghlines);
#ifndef READIMAGE_ONLY
        geometry_msgs::Twist cmd_red;
        // 车的速度值设置
        cmd_red.linear.x = LINEAR_X;
        cmd_red.linear.y = 0;
        cmd_red.linear.z = 0;
        cmd_red.angular.x = 0;
        cmd_red.angular.y = 0;
        cmd_red.angular.z = 0.2;
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

void mysobel(Mat src, Mat sobel_value, Mat sobel_alpha, double **kernalx, double **kernaly)
{
    unsigned int i, j, m, n;
    int valuex, valuey;
    double angle_tan;
    for (i = 0; i < src.rows; i++)
    {
        for (j = 0; j < src.cols; j++)
        {
            valuex = valuey = 0;
            for (m = 0; m < 3; m++)
            {
                for (n = 0; n < 3; n++)
                {
                    if (kernalx[m][n] != 0 && i + m >= 1 && i + m - 1 < src.rows && j + n >= 1 && j + n - 1 < src.cols)
                    {
                        valuex += (kernalx[m][n] * src.at<uchar>(i + m - 1, j + n - 1));
                    }
                    if (kernaly[m][n] != 0 && i + m >= 1 && i + m - 1 < src.rows && j + n >= 1 && j + n - 1 < src.cols)
                    {
                        valuey += (kernaly[m][n] * src.at<uchar>(i + m - 1, j + n - 1));
                    }
                }
            }
            sobel_value.at<uchar>(i, j) = uchar(sqrt(valuex * valuex / 4 + valuey * valuey / 4) + 0.5);
            if (valuex != 0)
            {
                angle_tan = double(valuey) / valuex;
                sobel_alpha.at<uchar>(i, j) = abs(angle_tan) < 0.4142135624 ? 1 : (abs(angle_tan) > 2.414213562 ? 4 : (angle_tan > 0 ? 2 : 3));
            }
            else
            {
                sobel_alpha.at<uchar>(i, j) = 4;
            }
            //         *     * 3
            //         *   *
            //         * *
            //   * * * * * * * 4
            //         * *
            //         *   *
            //         *     * 2
            //         * 1
            // 边缘法线方向编码
        }
    }
}

void mycanny(Mat output, Mat sobel_value, Mat sobel_alpha, unsigned int th, unsigned int tl)
{
    unsigned int i, j, m, n;
    bool flag;
    //非极大值抑制
    for (i = 0; i < sobel_value.rows; i++)
    {
        for (j = 0; j < sobel_value.cols; j++)
        {
            flag = true;
            switch (sobel_alpha.at<uchar>(i, j))
            {
            case 1:
                if (i > 1)
                    flag = flag && sobel_value.at<uchar>(i, j) > sobel_value.at<uchar>(i - 1, j);
                if (i + 1 < sobel_value.rows)
                    flag = flag && sobel_value.at<uchar>(i, j) > sobel_value.at<uchar>(i + 1, j);
                break;
            case 2:
                if (i > 1 && j > 1)
                    flag = flag && sobel_value.at<uchar>(i, j) > sobel_value.at<uchar>(i - 1, j - 1);
                if (i + 1 < sobel_value.rows && j + 1 < sobel_value.cols)
                    flag = flag && sobel_value.at<uchar>(i, j) > sobel_value.at<uchar>(i + 1, j + 1);
                break;
            case 3:
                if (i > 1 && j + 1 < sobel_value.cols)
                    flag = flag && sobel_value.at<uchar>(i, j) > sobel_value.at<uchar>(i - 1, j + 1);
                if (i + 1 < sobel_value.rows && j > 1)
                    flag = flag && sobel_value.at<uchar>(i, j) > sobel_value.at<uchar>(i + 1, j - 1);
                break;
            case 4:
                if (j > 1)
                    flag = flag && sobel_value.at<uchar>(i, j) > sobel_value.at<uchar>(i, j - 1);
                if (j + 1 < sobel_value.cols)
                    flag = flag && sobel_value.at<uchar>(i, j) > sobel_value.at<uchar>(i, j + 1);
                break;
            default:
                break;
            }
            if (flag)
                output.at<uchar>(i, j) = sobel_value.at<uchar>(i, j);
            else
                output.at<uchar>(i, j) = 0;
        }
    }
    imshow("非极大抑制", output);
    //计算两个阀值
    for (i = 0; i < sobel_value.rows; i++)
    {
        for (j = 0; j < sobel_value.cols; j++)
        {
            sobel_alpha.at<uchar>(i, j) = output.at<uchar>(i, j) > tl ? 255 : 0;
            sobel_value.at<uchar>(i, j) = output.at<uchar>(i, j) > th ? 255 : 0;
        }
    }
    //填充高阀值间隙
    for (i = 0; i < sobel_value.rows; i++)
    {
        for (j = 0; j < sobel_value.cols; j++)
        {
            output.at<uchar>(i, j) = 0;
        }
    }
    for (i = 0; i < sobel_value.rows; i++)
    {
        for (j = 0; j < sobel_value.cols; j++)
        {
            if (sobel_value.at<uchar>(i, j) == 255)
            {
                for (m = 0; m < 3; m++)
                {
                    for (n = 0; n < 3; n++)
                    {
                        if (i + m >= 1 && i + m - 1 < sobel_value.rows && j + n >= 1 && j + n - 1 < sobel_value.cols && output.at<uchar>(i + m - 1, j + n - 1) == 0)
                        {
                            output.at<uchar>(i + m - 1, j + n - 1) = sobel_alpha.at<uchar>(i + m - 1, j + n - 1) == 0 ? 0 : 255;
                        }
                    }
                }
            }
        }
    }
}

void myhough_circles(Mat src, Mat dst, Mat cannyed, Mat sobelx, Mat sobely, Mat circle_center_votes, Mat tem)
{
    //*********对圆心进行投票
    circle_center_votes = Mat::zeros(cannyed.rows, cannyed.cols, CV_64F);
    int i, j, n, m;
    for (i = 0; i < cannyed.rows; i++)
    {
        for (j = 0; j < cannyed.cols; j++)
        {
            if (cannyed.at<uchar>(i, j))
            {
                for (n = 0; n < cannyed.rows; n++)
                {
                    m = double(n - i) / sobelx.at<double>(i, j) * sobely.at<double>(i, j) + j + 0.5;
                    if (m >= 0 && m < cannyed.cols)
                    {
                        circle_center_votes.at<double>(n, m) += 1.0;
                    }
                }
            }
        }
    }

    //找到票数最大值用于自动调整阀值
    double min_value, max_value;
    int min_idx[2], max_idx[2];
    minMaxIdx(circle_center_votes, &min_value, &max_value, min_idx, max_idx);

    //将投票结果normalize以进行观察
    normalize(circle_center_votes, tem, 0, 255, NORM_MINMAX, CV_8UC1);
    imshow("real votes normalized circles", tem);

    //进行非极大值抑制，并以最大票数的一半进行阀值处理
    tem = circle_center_votes.clone();
    NMS(circle_center_votes, tem, 55, max_value / 2);
    threshold(tem, circle_center_votes, max_value / 2, 255, THRESH_BINARY);
    imshow("votes circles", circle_center_votes);

    vector<int> centerx_index;
    vector<int> centery_index;
    vector<int> radius;
    double *radius_votes = new double[circle_center_votes.cols + circle_center_votes.rows];
    memset(radius_votes, 0, sizeof(radius_votes));
    for (i = 0; i < circle_center_votes.rows; i++)
    {
        for (j = 0; j < circle_center_votes.cols; j++)
        {
            if (circle_center_votes.at<double>(i, j))
            {
                centerx_index.push_back(i);
                centery_index.push_back(j);
            }
        }
    }

    for (m = 0; m < centerx_index.size(); m++)
    {
        for (i = 0; i < cannyed.rows; i++)
        {
            for (j = 0; j < cannyed.cols; j++)
            {
                if (cannyed.at<uchar>(i, j))
                {
                    radius_votes[int(sqrt((i - centerx_index[m]) * (i - centerx_index[m]) + (j - centery_index[m]) * (j - centery_index[m])) + 0.5)] += 1;
                }
            }
        }
        int maxvotes = 0;
        radius.push_back(0);
        for (i = 0; i < circle_center_votes.cols + circle_center_votes.rows; i++)
        {
            if (radius_votes[i] > maxvotes)
            {
                maxvotes = radius_votes[i];
                radius[m] = i;
            }
        }
    }

    //将找到的圆用蓝色线画出
    for (m = 0; m < centerx_index.size(); m++)
    {
        circle(dst, Point2d(centery_index[m], centerx_index[m]), radius[m], (0, 0, 255), 2);
    }
}

void NMS(Mat src, Mat dst, int box_size, double skip)
{
    bool flag;
    int i, j, m, n;
    for (i = 0; i < src.rows; i++)
    {
        for (j = 0; j < src.cols; j++)
        {
            if (src.at<double>(i, j) < skip)
            {
                dst.at<double>(i, j) = 0;
            }
            else
            {
                flag = true;
                for (m = 0; m < box_size; m++)
                {
                    for (n = 0; n < box_size; n++)
                    {
                        if (!(m == ((box_size - 1) / 2) && n == ((box_size - 1) / 2)) && i + m - ((box_size - 1) / 2) >= 0 && i + m - ((box_size - 1) / 2) < dst.rows && j + n - ((box_size - 1) / 2) >= 0 && j + n - ((box_size - 1) / 2) < dst.cols)
                        {

                            flag = flag && (src.at<double>(i, j) > src.at<double>(i + m - ((box_size - 1) / 2), j + n - ((box_size - 1) / 2)));
                        }
                    }
                }
                if (flag)
                    dst.at<double>(i, j) = src.at<double>(i, j);
                else
                    dst.at<double>(i, j) = 0;
            }
        }
    }
}

void myhough_lines(Mat src, Mat dst, Mat cannyed, Mat votes, Mat tem)
{
    //*********投票
    int votelength = int(floor(sqrt(cannyed.cols * cannyed.cols + cannyed.rows * cannyed.rows) + 0.5)) % 2 == 0 ? int(floor(sqrt(cannyed.cols * cannyed.cols + cannyed.rows * cannyed.rows) + 0.5)) : int(floor(sqrt(cannyed.cols * cannyed.cols + cannyed.rows * cannyed.rows) + 0.5)) + 1;
    votelength = votelength * 2 + 1;
    votes = Mat::zeros(votelength, 181, CV_64F);
    int half_votelength = (votelength - 1) / 2;
    int i, j, m;
    for (i = 0; i < cannyed.rows; i++)
    {
        for (j = 0; j < cannyed.cols; j++)
        {
            if (cannyed.at<uchar>(i, j))
            {
                for (m = 0; m < 181; m++)
                {
                    votes.at<double>(int(floor(i * cos(double(m - 90) / 180 * 3.1415926535) + j * sin(double(m - 90) / 180 * 3.1415926535) + half_votelength + 0.5)), m) += 1.0;
                }
            }
        }
    }
    //找到票数最大值用于自动调整阀值
    double min_value, max_value;
    int min_idx[2], max_idx[2];
    minMaxIdx(votes, &min_value, &max_value, min_idx, max_idx);

    //将投票结果normalize以进行观察
    normalize(votes, tem, 0, 255, NORM_MINMAX, CV_8UC1);
    imshow("real votes normalized line", tem.t());

    //进行非极大值抑制，并以最大票数的一半进行阀值处理
    tem = votes.clone();
    NMS(votes, tem, 45, max_value / 3);
    threshold(tem, votes, max_value / 3, 255, THRESH_BINARY);
    imshow("votes line", votes.t());
    vector<int> rho;
    vector<int> theta;
    for (i = 0; i < votes.rows; i++)
    {
        for (j = 0; j < votes.cols; j++)
        {
            if (votes.at<double>(i, j))
            {
                rho.push_back(i - half_votelength);
                theta.push_back(j - 90);
            }
        }
    }
    for (m = 0; m < rho.size(); m++)
    {
        // cout << theta[m] << endl;
        if (theta[m] == 0)
        {
            line(dst, Point2d(0, rho[m]), Point2d(dst.cols, rho[m]), (0, 0, 255), 5);
        }
        else if (theta[m] - 90 == 0 || theta[m] + 90 == 0)
        {
            line(dst, Point2d(rho[m], 0), Point2d(rho[m], dst.rows), (0, 0, 255), 5);
        }
        else
        {
            int merge2 = int(floor(rho[m] / sin(double(theta[m]) / 180 * 3.1415926535) + 0.5));
            int merge4 = int(floor((rho[m] - double(dst.rows - 1) * cos(double(theta[m]) / 180 * 3.1415926535)) / sin(double(theta[m]) / 180 * 3.1415926535) + 0.5));
            int merge1 = int(floor(rho[m] / cos(double(theta[m]) / 180 * 3.1415926535) + 0.5));
            int merge3 = int(floor((rho[m] - double(dst.cols - 1) * sin(double(theta[m]) / 180 * 3.1415926535)) / cos(double(theta[m]) / 180 * 3.1415926535) + 0.5));
            line(dst, Point2d(merge2, 0), Point2d(merge4, dst.rows - 1), (0, 0, 255), 3);
            line(dst, Point2d(0, merge1), Point2d(dst.cols - 1, merge3), (0, 0, 255), 3);
            line(dst, Point2d(0, merge1), Point2d(merge2, 0), (0, 0, 255), 3);
            line(dst, Point2d(dst.cols - 1, merge3), Point2d(merge4, dst.rows - 1), (0, 0, 255), 3);
            line(dst, Point2d(0, merge1), Point2d(merge4, dst.rows), (0, 0, 255), 3);
            line(dst, Point2d(merge2, 0), Point2d(dst.cols - 1, merge3), (0, 0, 255), 3);
        }
    }
}

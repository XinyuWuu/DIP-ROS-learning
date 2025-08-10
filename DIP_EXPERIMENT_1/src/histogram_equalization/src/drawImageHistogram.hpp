//此代码仅供参考
#include <iostream>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

void openCVHist(const Mat src);
void drawHist(vector<int> nums,const char* name);
void calHist(const string img);

#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

#define DIS 30
#define ALG 3

using namespace std;
using namespace cv;

Mat match_by_line(Mat inputa, Mat inputb);
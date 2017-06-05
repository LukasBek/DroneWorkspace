#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;

bool Dcompare(const RotatedRect &a, const RotatedRect &b);
void sobel(Mat src_gray, Mat *grad);
Mat minBoundingRotatedBoxes (Mat src);
vector<Vec3f> getCircles(Mat src);
int histogramCalculation(Mat src);
Mat redFilter(Mat src);

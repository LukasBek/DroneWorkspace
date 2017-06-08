#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <math.h>

using namespace cv;

bool Dcompare(const Rect &a, const Rect &b);
void sobel(Mat src_gray, Mat *grad);
void minBoundingBoxes (Mat src, int *width, int *height, int *x, int *y);
void getCircles(Mat src, std::vector<Vec3f> *dest);
int histogramCalculation(Mat src);
Mat redFilter(Mat src);
void isCircle(cv::Mat src, int *width, int *height, int *x, int *y, bool *res);

int morphologyFilter(cv::Mat *src, int filterSize);
int erodeImage(cv::Mat *src, int erosionSize);
int dilateImage(cv::Mat *src, int dilationSize);

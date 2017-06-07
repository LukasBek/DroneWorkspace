#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;

bool Dcompare(const Rect &a, const Rect &b);
void sobel(Mat src_gray, Mat *grad);
Mat minBoundingBoxes (Mat src);
void getCircles(Mat src, vector<Vec3f> *dest);
int histogramCalculation(Mat src);
Mat redFilter(Mat src);

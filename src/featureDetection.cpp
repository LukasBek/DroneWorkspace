#include "featureDetection.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <math.h>

using namespace cv;
using namespace std;

// Blur and gray the image before call //

bool Dcompare(const Rect &a, const Rect &b){
 // TODO Change to area
  return b.area() < a.area();
}

void sobel(Mat src_gray, Mat *grad){

  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;

  int c;

  /// Generate grad_x and grad_y
  Mat grad_x, grad_y;
  Mat abs_grad_x, abs_grad_y;

  /// Gradient X
  //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
  Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
  convertScaleAbs( grad_x, abs_grad_x );

  /// Gradient Y
  //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
  Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
  convertScaleAbs( grad_y, abs_grad_y );

  /// Total Gradient (approximate)
  addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, *grad );
}

// -------------------------------------------------- //

// Blur and gray the image before call //

void minBoundingBoxes (Mat src, int *width, int *height, int *x, int *y){

  // imshow("Før moF", src);

  int moF = morphologyFilter(&src, 8);

  // imshow("Efter MoF", src);

  blur(src, src, Size(5, 5));

  //namedWindow("Blur from minBoundingBoxes");
  //imshow("Blur from minBoundingBoxes ", src);

  RNG rng(12345);

  Mat threshold_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  int thresh = 100;
  int max_thresh = 255;
  /// Detect edges using Threshold
  threshold(src, threshold_output, thresh, max_thresh, THRESH_BINARY );

  /// Find contours
  findContours( threshold_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Find the rotated rectangles and ellipses for each contour
  vector<Rect> boundRect( contours.size() );
  vector<vector<Point> > contours_poly( contours.size() );

  // vector<Rect> minEllipse( contours.size() );

  if (contours.size() > 0){
    for( int i = 0; i < contours.size(); i++ )
    { approxPolyDP( contours[i], contours_poly[i], 3, true );
      boundRect[i] = boundingRect( contours_poly[i] );
    }
    std::sort(boundRect.begin(),boundRect.end(),Dcompare);
  }

  int rectWidth   = 0;
  int rectHeight  = 0;
  int rectY       = 0;
  int rectX       = 0;
  bool isCircleRes = false;

  /// Draw polygonal contour + bonding rects + circles
  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  if (contours.size() > 0){
  for( int i = 0; i < 1; i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
     }
     rectWidth   = boundRect[0].width;
     rectHeight  = boundRect[0].height;
     rectY       = boundRect[0].y;
     rectX       = boundRect[0].x;
     isCircle(src, &rectWidth, &rectHeight, &rectX, &rectY, &isCircleRes);

}

  namedWindow("Boxes from minBoundingBoxes");
  imshow("Boxes from minBoundingBoxes", drawing);

  if (isCircleRes == false){
    rectWidth   = 0;
    rectHeight  = 0;
    rectY       = 0;
    rectX       = 0;
  }

  /// retrun to pointers
   *width   = rectWidth;
   *height  = rectHeight;
   *x       = rectX;
   *y       = rectY;

}

// -------------------------------------------------- //

void getCircles(Mat src, std::vector<Vec3f> *dest){

  vector<Vec3f> circles;
  int dp = 1;           // The inverse ratio of resolution
  int min_dist = 100;   // Minimum distance between detected centers
  int param_1 = 100;    // Upper threshold for the internal Canny edge detector
  int param_2 =170;     // Threshold for center detection
                        // 200 Has hard time finding perfect circles
                        // 100 Only very clear circles
                        // 80 detects random round suff
                        // 60 begins detecting faces
                        // 50 sees faces
                        // 40 sees circles in square objects
                        // 20 or below, finds circles everywhere
  int min_radius = 20;  // Minimum radio to be detected. If unknown, put zero as default
  int max_radius = 200; // Maximum radius to be detected. If unknown, put zero as default
  HoughCircles(src, circles, CV_HOUGH_GRADIENT, dp, min_dist, param_1, param_2, min_radius, max_radius);
  for (size_t i = 0; i < circles.size(); i++)
  {
    Vec3i c = circles[i];
    circle(src, Point(c[0], c[1]), c[2], Scalar(255, 255, 255), 3, CV_AA);
    circle(src, Point(c[0], c[1]), 2,    Scalar(255, 255, 255), 3, CV_AA);
  }

  // cout << "Amount of circles: " << circles.size() << endl;
  //namedWindow("Detected circles from getCircles");
  imshow("Detected circles from getCircles", src);

  *dest = circles;

}

  // Histogram Calculation //
  // Source - http://docs.opencv.org/2.4/doc/tutorials/imgproc/histograms/histogram_calculation/histogram_calculation.html
int histogramCalculation(Mat src)
{
cout << "DONT WORK" << endl;
/*
  Mat dst;

  /// Separate the image in 3 places ( B, G and R )
  vector<Mat> bgr_planes;
  split( src, bgr_planes );

  /// Establish the number of bins
  int histSize = 256;

  /// Set the ranges ( for B,G,R) )
  float range[] = { 0, 256 } ;
  const float* histRange = { range };

  bool uniform = true;
  bool accumulate = false;

  Mat b_hist, g_hist, r_hist;
  /// Compute the histograms:
  calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

  /// Normalize the result to [ 0, histImage.rows ]
  //normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
  //normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
  //normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

  int hist_w = 256;

  float top = 0;
  int indeks = 0;
  for (int i = 1 + 30; i < hist_w - 30; i++)
  {
    if (cvRound(r_hist.at<float>(i)) > top){
        top = (cvRound(r_hist.at<float>(i)));
        indeks = i;
    }
}
   // cout << "Top: " << top << ", indeks: " << indeks << endl;
*/
   return -1;
}

Mat redFilter(Mat src){

  Mat threshold_output;

  int thresh = 25;
  int max_thresh = 255;

  Mat bgr[3];     //destination array
  split(src,bgr); //split source

  Mat temp;

  subtract(bgr[2],bgr[1],temp);

  //namedWindow("Rødfilter");
  //imshow("Rødfilter",temp);

  threshold(temp, threshold_output, thresh, max_thresh, THRESH_BINARY );

  // int moF = morphologyFilter(&threshold_output, 3);

  //Mat sobelGrad;
  //sobel(temp, &sobelGrad);
  //namedWindow("Sobel");
  //imshow("Sobel", sobelGrad);

  return threshold_output;

}

void isCircle(cv::Mat src, int *width, int *height, int *x, int *y, bool *res) {

  if (*width > 0 && *height > 0 && *x > 0 && *y > 0){

  int rectWidth   = *width;
  int rectHeight  = *height;

  int offsetX = *x;
  int offsetY = *y;

    cv::Point p1  = Point(rectWidth*10/20   ,rectHeight*10/20 );
    cv::Point p2  = Point(rectWidth*10/20   ,rectHeight*9/20  );
    cv::Point p3  = Point(rectWidth*11/20   ,rectHeight*10/20 );
    cv::Point p4  = Point(rectWidth*10/20   ,rectHeight*11/20 );
    cv::Point p5  = Point(rectWidth*9/20    ,rectHeight*10/20 );
    cv::Point p6  = Point(rectWidth*11/20   ,rectHeight*9/20  );
    cv::Point p7  = Point(rectWidth*11/20   ,rectHeight*11/20 );
    cv::Point p8  = Point(rectWidth*9/20    ,rectHeight*11/20 );
    cv::Point p9  = Point(rectWidth*9/20    ,rectHeight*9/20  );
    cv::Point p10 = Point(rectWidth*10/20   ,rectHeight*8/20  );
    cv::Point p11 = Point(rectWidth*12/20   ,rectHeight*10/20 );
    cv::Point p12 = Point(rectWidth*10/20   ,rectHeight*12/20 );
    cv::Point p13 = Point(rectWidth*8/20    ,rectHeight*10/20 );
    cv::Point p14 = Point(rectWidth*14/20   ,rectHeight*10/20 );
    cv::Point p15 = Point(rectWidth*10/20   ,rectHeight*14/20 );
    cv::Point p16 = Point(rectWidth*6/20    ,rectHeight*10/20 );
    cv::Point p17 = Point(rectWidth*10/20   ,rectHeight*6/20  );
    cv::Point p22 = Point(rectWidth*12/20   ,rectHeight*7/20  );
    cv::Point p23 = Point(rectWidth*13/20   ,rectHeight*8/20  );
    cv::Point p24 = Point(rectWidth*13/20   ,rectHeight*12/20 );
    cv::Point p25 = Point(rectWidth*12/20   ,rectHeight*13/20 );
    cv::Point p26 = Point(rectWidth*8/20    ,rectHeight*13/20 );
    cv::Point p27 = Point(rectWidth*7/20    ,rectHeight*12/20 );
    cv::Point p28 = Point(rectWidth*7/20    ,rectHeight*8/20  );
    cv::Point p29 = Point(rectWidth*8/20    ,rectHeight*7/20  );
/*
    cv::Point B = Point(rectWidth/4     ,rectHeight/2     );
    cv::Point C = Point(rectWidth*3/8   ,rectHeight/2     );
    cv::Point D = Point(rectWidth/2     ,rectHeight/2     );
    cv::Point E = Point(rectWidth*5/8   ,rectHeight/2     );
    cv::Point F = Point(rectWidth*3/4   ,rectHeight/2     );
    cv::Point H = Point(rectWidth/2     ,rectHeight*5/8   );
    cv::Point J = Point(rectWidth/2     ,rectHeight*3/4   );
    cv::Point M = Point(rectWidth/2     ,rectHeight*3/8   );
    cv::Point O = Point(rectWidth/2     ,rectHeight/4     );
*/
  cv::Point2f points [25] = {p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17, p22, p23, p24, p25, p26, p27, p28, p29};

  for (int i = 0 ; i < 25 ; i++){
    points[i].x = points[i].x + offsetX;
    points[i].y = points[i].y + offsetY;
  }
  float f1 = 1.0;
  float f2 = 0.8;
  float f3 = 0.4;
  float f4 = 0.2;

  float weight [] = {f1, f1, f1, f1, f1, f1, f1, f1, f1, f2, f2, f2, f2, f3, f3, f3, f3, f3, f3, f3, f3, f3, f3, f3, f3};
  int   value [25];

  for (int i = 0 ; i < 25 ; i++){

    Scalar intensity = src.at<uchar>(points[i]);
    value[i] = ceil(intensity[0] * weight[i]);

  }

  for (int i = 0 ; i < 25 ; i++){
    circle(src, points[i], 1, Scalar(150, 150, 150), 2, CV_AA);
  }

    int sum   = 0;
    int nSum  = 0;
    for (int i = 0 ; i < 25 ; i++){
      sum = (sum + value[i]);
    }
    nSum = ceil(sum / 25);

    cout << " -------------nSUM " << nSum << endl;

    if (nSum <= 10){
      *res = true;
    }

  imshow("Point", src);
  }
}

// Author: Niclas Atzen //
// Methods: morphologyFilter, erodeImage, dilateImage

int morphologyFilter(cv::Mat *src, int filterSize = 1) {
  dilateImage(src, filterSize);
	//erodeImage(src, filterSize);
	return 0;
}

int erodeImage(cv::Mat *src, int erosionSize = 1) {
	// Erodes image(src) with ellipse element //
	int erosion_type = cv::MORPH_RECT;
	int const max_kernel_size = 21;

	cv::Mat element = cv::getStructuringElement(erosion_type, cv::Size(erosionSize * 2 + 1, erosionSize * 2 + 1), cv::Point(erosionSize, erosionSize));
	erode(*src, *src, element);
	element.release();
	return 0;
}

int dilateImage(cv::Mat *src, int dilationSize = 1) {
	// dilates image(src) with ellipse element //
	int dilation_type = cv::MORPH_ELLIPSE;
	int const max_kernel_size = 21;

	cv::Mat element = cv::getStructuringElement(dilation_type, cv::Size(dilationSize * 2 + 1, dilationSize * 2 + 1), cv::Point(dilationSize, dilationSize));
	dilate(*src, *src, element);
	return 0;
}

#include "featureDetection.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

// Blur and gray the image before call //
Mat sobel(Mat src){

  Mat src_gray;
  Mat grad;
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
  addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

  return grad;
}

// -------------------------------------------------- //

// Blur and gray the image before call //
Mat minBoundingRotatedBoxes (Mat src){


  RNG rng(12345);

  Mat threshold_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  int thresh = 100;
  int max_thresh = 255;
  /// Detect edges using Threshold
  threshold(src, threshold_output, thresh, max_thresh, THRESH_BINARY );

  /// Find contours
  findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Find the rotated rectangles and ellipses for each contour
  vector<RotatedRect> minRect( contours.size() );
  // vector<RotatedRect> minEllipse( contours.size() );

  for( int i = 0; i < contours.size(); i++ )
  { minRect[i] = minAreaRect( Mat(contours[i]) );
    // if( contours[i].size() > 5 )
    //   { minEllipse[i] = fitEllipse( Mat(contours[i]) ); }
  }

  /// Draw contours + rotated rects + ellipses
  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
  {
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    // contour
    drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
    // ellipse
    // ellipse( drawing, minEllipse[i], color, 2, 8 );
    // rotated rectangle
    Point2f rect_points[4]; minRect[i].points( rect_points );
    for( int j = 0; j < 4; j++ )
    line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
  }

  /// Show in a window
  return drawing;
}

// -------------------------------------------------- //

vector<Vec3f> getCircles(Mat src){

  vector<Vec3f> circles;
  int dp = 1;           // The inverse ratio of resolution
  int min_dist = 100;   // Minimum distance between detected centers
  int param_1 = 100;    // Upper threshold for the internal Canny edge detector
  int param_2 = 100;    // Threshold for center detection
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

  return circles;
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

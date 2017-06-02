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
  char* window_name = "Sobel Demo - Simple Edge Detector";
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;

  int c;

  if( !src.data )
  { return NULL; }

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

Mat minBoundingRotatedBoxes (Mat src){

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

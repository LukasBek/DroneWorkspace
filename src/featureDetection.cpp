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

  blur(src, src, Size(5, 5));

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
  // TODO Perhaps the first if shuld be extended to enclose the whole lot

  if (contours.size() >= 5){
    for( int i = 0; i < 5; i++ )
    { approxPolyDP( contours[i], contours_poly[i], 3, true );
      boundRect[i] = boundingRect( contours_poly[i] );
    }
  }

  if (boundRect.size() > 0){
    std::sort(boundRect.begin(),boundRect.end(),Dcompare);
  }

  /// Draw polygonal contour + bonding rects + circles
  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  if (contours.size() >= 5){
  for( int i = 0; i < 5; i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
     }
}
  int rectWidth = 0;
  int rectHeight = 0;
  int rectY = 0;
  int rectX = 0;


  imshow("Boxes from minBoundingBoxes", drawing);

  /// retrun to pointers
   *width   = rectWidth;
   *height  = rectHeight;
   *x       = rectX;
   *y       = rectY;

}

// -------------------------------------------------- //

void getCircles(Mat src, vector<Vec3f> *dest){

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

  //Note: OpenCV uses BGR color order
  // imwrite("blue.png",bgr[0]);   //blue channel
  // imwrite("green.png",bgr[1]);  //green channel
  // imwrite("red.png",bgr[2]);    //red channel
  Mat temp;
  //Mat temp2;
  //addWeighted(bgr[0],0.5,bgr[1],0.5,temp2,0);

  subtract(bgr[2],bgr[0],temp);

  imshow("Rødfilter",temp);

  threshold(temp, threshold_output, thresh, max_thresh, THRESH_BINARY );

  Mat sobelGrad;
  sobel(temp, &sobelGrad);
  imshow("Sobel", sobelGrad);

  return threshold_output;

}

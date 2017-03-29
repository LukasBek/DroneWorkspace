#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>
#include <sstream>
#include <iostream>
#include <queue>
#include <deque>
#include <string>
#include <vector>
using namespace cv;
using namespace std;

// Primary working frame //
Mat frame;



// qrReader qr = qrReader();

RNG rng(12345);
int lowThreshold = 60;
int const max_lowThreshold = 200;
int ratio = 3;
int kernel_size = 3;

double eDistance (Vec3i a, Vec3i b)
{
  return sqrt(pow(a[0] - b[0],2) + pow(a[1] - b[1],2));
}


Mat cannyEdgeDetector(Mat frame)
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  Canny( frame, canny_output, lowThreshold, lowThreshold*ratio, kernel_size);
  findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( size_t i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, Point() );
     }
  return canny_output;
}

void qrDetector(Mat frame)
{





}


int main(int argc, char** argv)
{
    VideoCapture cap;

    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    queue <Vec3i> circleQueue; /* Declare a queue */
    struct videoSize {
      int width;
      int height;
    };
    struct acceptSize {
      int minHeight;
      int maxHeight;
      int maxLeft;
      int maxRight;
    };

    videoSize vSize;
    vSize.width  = 0;
    vSize.height = 0;

    acceptSize aSize;
    aSize.minHeight = 0;
    aSize.maxHeight = 0;
    aSize.maxLeft   = 0;
    aSize.maxRight  = 0;

    if(!cap.open(0))
        return 0;
    for(;;)
    {
          cap >> frame;
          if( frame.empty() ) break; // end of video stream

          if (vSize.width == 0 || vSize.height == 0){
            vSize.width = frame.cols;
            vSize.height = frame.rows;

            aSize.minHeight = vSize.height / 3;
            aSize.maxHeight = vSize.height / 3 * 2;
            aSize.maxLeft  = vSize.width / 3.2;
            aSize.maxRight = vSize.width / 1.4545;

            cout << "Frame Dimensions - "  << vSize.width     << " " << vSize.height << endl;
            cout << "Minimum Height   - "  << aSize.minHeight << endl;
            cout << "Maximum Height   - "  << aSize.maxHeight << endl;
            cout << "Maximum Left     - "  << aSize.maxLeft   << endl;
            cout << "Maximum Right    - "  << aSize.maxRight  << endl;

          }


          blur(frame, frame, Size(3,3));
          cvtColor(frame,frame,CV_RGB2GRAY);
/*
          Mat3b hsv;
          cvtColor(frame, hsv, COLOR_BGR2HSV);

          Mat1b mask1, mask2;
          inRange(hsv, Scalar(0, 70, 50), Scalar(10, 255, 255), mask1);
          inRange(hsv, Scalar(170, 70, 50), Scalar(180, 255, 255), mask2);

          Mat1b mask = mask1 | mask2;
*/



          // putText(frame, "Test", cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,255,255), 1, CV_AA);


          vector<Vec3f> circles;
          int dp = 1;             // The inverse ratio of resolution
          int min_dist = 100;     // Minimum distance between detected centers
          int param_1 = 100;      // Upper threshold for the internal Canny edge detector
          int param_2 = 150;      // Threshold for center detection
                                  // 200 Has hard time finding perfect circles
                                  // 100 Only very clear circles
                                  // 80 detects random round suff
                                  // 60 begins detecting faces
                                  // 50 sees faces
                                  // 40 sees circles in square objects
                                  // 20 or below, finds circles everywhere
          int min_radius = 20;    // Minimum radio to be detected. If unknown, put zero as default
          int max_radius = 200;   // Maximum radius to be detected. If unknown, put zero as default
          HoughCircles(frame, circles, CV_HOUGH_GRADIENT, dp, min_dist, param_1, param_2, min_radius, max_radius);
          for( size_t i = 0; i < circles.size(); i++ )
          {
              Vec3i c = circles[i];
              circle(frame, Point(c[0], c[1]), c[2], Scalar(255,255,255), 3, CV_AA);
              circle(frame, Point(c[0], c[1]), 2,    Scalar(255,255,255), 3, CV_AA);
          }


        //  frame = cannyEdgeDetector(frame);


          // Setting text on screen start //

          // Finds the amount of circles in the image
          unsigned long ul = circles.size();
          std::string numberOfCircles;
          std::stringstream strstream;
          strstream << ul;
          strstream >> numberOfCircles;


          if (ul != 0){
            Vec3i c = circles[0];

            // Queue start //

            if (circleQueue.size() < 5){
              circleQueue.push(c);  //  Add some values to the queue
            } else {

                Vec3i p1 = circleQueue.front();
                Vec3i p2 = circleQueue.back();

                // cout << "The result is " << eDistance (p2,p1) << endl;

                circleQueue.pop();

            }

            // Queue end //

            // Command section start //
            /*
            std::string message;

            if (c[0] < aSize.maxLeft){
              // Go Left
              message = "Go left ";
            }
            if (c[0] > aSize.maxRight){
              // Go Right
              message = "Go right ";
            }
            if (c[1] > aSize.maxHeight){
              // GO Down
              message = "Go down ";
            }
            if (c[1] < aSize.minHeight){
              // Go Up
              message = "Go up ";
            }

            if(message != "" && message != "DEF"){
                cout << message << endl;
                message = "";
            } else {
              message = "DEF";
            }
            */
            // Command section end //

            std::string number0;
            std::string number1;
            std::string number2;

            std::stringstream strstream0;
            std::stringstream strstream1;
            std::stringstream strstream2;

            strstream0 << c[0];
            strstream0 >> number0;

            strstream1 << c[1];
            strstream1 >> number1;

            strstream2 << c[2];
            strstream2 >> number2;

            std::string number = "Horizontal " + number0 + " Vertical " + number1 + " Size " + number2;

            putText(frame, "Circles " + numberOfCircles, cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,255,255), 1, CV_AA);
            putText(frame, number, cvPoint(30,60), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,255,255), 1, CV_AA);

            // Setting text on screen end //

          }

          imshow("detected circles", frame);

          if( waitKey(10) == 27 ) break; // stop capturing by pressing ESC
    }
    // the camera will be closed automatically upon exit
    //cap.close();
    return 0;
}
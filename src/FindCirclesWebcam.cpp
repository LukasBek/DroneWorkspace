#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>
#include <zbar.h>
#include <sstream>
#include <iostream>
#include <queue>
#include <deque>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <cstdlib>
#include <stdio.h>
#include <std_msgs/String.h>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#include "qrScan.h"
#include "featureDetection.h"
#include "DroneMovement.h"

// #include <chrono>

using namespace cv;
using namespace std;
using namespace zbar;

// typedef std::chrono::high_resolution_clock Clock;

Mat frameRBG; // Primary blur working frame from capture //
Mat frame;    // Primary gray working frame //
Mat noBlurRGB; //

// -------------------------------------------------------------------------------------------------------------
//Getting the video stream and converting it to openCV

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{

public:
  cv_bridge::CvImagePtr cv_ptr;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
      : it_(nh_)
  {

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ardrone/front/image_raw", 1,
                               &ImageConverter::imageCb, this);
    // image_pub_ = it_.advertise("/image_converter/output_video", 1);

    // cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    // cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    // public cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Update GUI Window
    frameRBG = cv_ptr->image;

    cv::waitKey(3);

    // Output modified video stream
    // image_pub_.publish(cv_ptr->toImageMsg());
  }
};

// ----------------------  vvv  -----------------------  vvv   OpenCV   vvv  ----------------------  vvv  ----------------------- //

RNG rng(12345);
int lowThreshold = 60;
int const max_lowThreshold = 200;
int ratio = 3;
int kernel_size = 3;


// eDistance - Calculates distance between two vectors //
double eDistance(Vec3i a, Vec3i b)
{
  return sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2));
}

int main(int argc, char **argv)
{

  // Main scope variables //
  queue<Vec3i> circleQueue; // Que for circle positions //
  std::string message;      // Message for console prints //
  int variance20 = 20;


  ros::init(argc, argv, "image_converter");
  ImageConverter ic;

  ros::NodeHandle node;

  DroneMovement move;
  Bool isCentered;

  move.init(node);

  ros::Rate loop_rate(10);


  struct videoSize // Struct for video size //
  {
    int width;
    int height;
  };
  struct acceptSize // Dimensions for accept //
  {
    int minHeight;
    int maxHeight;
    int maxLeft;
    int maxRight;
    int maxSize;
    int minSize;
  };
 // Ini of structs //
  videoSize vSize;
    vSize.width  = 0;
    vSize.height = 0;

  acceptSize aSize;
    aSize.minHeight = 0;
    aSize.maxHeight = 0;
    aSize.maxLeft   = 0;
    aSize.maxRight  = 0;
    aSize.maxSize   = 0;
    aSize.minSize   = 0;

  ros::Duration(1).sleep();
  int b = 0;
  // auto t1 = Clock::now();
  // cout << "clock: " << t1 << endl;
  while (ros::ok())
  {
    ros::spinOnce();

       //while ((double)ros::Time::now().toSec() < start_time + takeoff_time + 2)
    //while ((double)ros::Time::now().toSec() < 7 + 2)
    //{ //takeoff

    //cout << "Variabler " << move.takeoff_time << endl;
    while (b < 100)
    {
      ros::spinOnce();
      //// Denne skal udkommenteres for kun at teste kamera og så dronen ikke letter
    //  move.takeoff();

      if (b == 99)
      {
        cout << "Taking off" << endl;
        ros::Duration(6).sleep();
      }
      b++;
    }
    while (b < 101)
    {
      cout << "OPSADASSE!" << endl;
      ros::spinOnce();
      move.goUp(1.6);
      b++;
    }

    // cout << "TRY ME" << endl;
    if (frameRBG.empty()){
      cout << "No frame from capture, end og video stream" << endl;
      break; // end of video stream
    }

    if (vSize.width == 0 || vSize.height == 0)
    {
      vSize.width     = frameRBG.cols;
      vSize.height    = frameRBG.rows;

      aSize.minHeight = vSize.height / 2.5;
      aSize.maxHeight = vSize.height / 7;
      aSize.maxLeft   = vSize.width  / 2.5;
      aSize.maxRight  = vSize.width  / 1.6;
      aSize.maxSize   = vSize.height / 4;
      //Jo mindre tal, jo tættere på kommer dronen
      aSize.minSize   = vSize.height / 12;

      cout << "Frame Dimensions - " << vSize.width      << " " << vSize.height << endl;
      cout << "Minimum Height   - " << aSize.minHeight  << endl;
      cout << "Maximum Height   - " << aSize.maxHeight  << endl;
      cout << "Maximum Left     - " << aSize.maxLeft    << endl;
      cout << "Maximum Right    - " << aSize.maxRight   << endl;
      cout << "Maximum Size     - " << aSize.maxSize    << endl;
      cout << "Minimum Size     - " << aSize.minSize    << endl;
    }

    noBlurRGB = frameRBG.clone();

    // Blur and convertion to grayscale //
    blur(frameRBG, frameRBG, Size(5, 5));

    cvtColor(frameRBG, frame, CV_RGB2GRAY);

    // Zbar Start //
    string res = zbarScan(frame, vSize.width, vSize.height);
    if (!res.empty()){
      cout << "Symbol: " << zbarScan(frame, vSize.width, vSize.height) << endl;
    }
    // Zbar End //

    // Red Filter start //

    imshow("Rødfilter threshold", redFilter(noBlurRGB));

    // Sobel //

  //  imshow("Sobel", sobel(frame));

    // Sobel //


    // Red Filter start //

    // pRect //
     imshow("Boxes",minBoundingRotatedBoxes(redFilter(noBlurRGB)));
    // pRect //

    // putText(frame, "Test", cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,255,255), 1, CV_AA);

    // HoughCircles Start //
    vector<Vec3f> circles;
    circles = getCircles(frame);
    // HoughCircles End //

    // Finds the amount of circles in the image
    unsigned long ul = circles.size();
    std::string numberOfCircles;
    std::stringstream strstream;
    strstream << ul;
    strstream >> numberOfCircles;

    if (ul != 0)
    {
      Vec3i c = circles[0];

      // Queue start // // TODO - May need a check if logic is correct //

      if (circleQueue.size() < 5)
      {
        circleQueue.push(c); //  Add some values to the queue
      }
      else
      {
        queue<Vec3i> circleQueueTemp;
        circleQueueTemp = circleQueue;
        double iterator = 1;
        // cout << "Accept" << endl;

        while (circleQueueTemp.size() > 0){

          // cout << "Round " << iterator << ", Distance " << eDistance(circleQueueTemp.front(),c) << ", max distance " << (iterator/20)*vSize.width << endl;

          if (eDistance(circleQueueTemp.front(),c) < iterator/20*vSize.width){
            //cout << "Accept circle into que" << endl;
            //cout << "Accept" << endl;
            circleQueue.pop();
            circleQueue.push(c);
            break;
          }
          else {
            //cout << "Deny circle into que" << endl;
            //cout << "Deny" << endl;
            circleQueueTemp.pop();
          }
          iterator++;
        }
      }

      // Queue end //

      // Command section start //

/*
            int H = c[0] - 320;
            int V = c[1] - 180;

            if (!(H > 320 - 20 && H < 320 + 20)){
              if (H < 0){
                move.goRight(baseTime * |H|);
                break;
              }
              if (H > 0){
                move.goLeft(baseTime * |H|);
                break;
              }
            }

            if (!(V > 180 - 20 && V < 180 + 20)){
              if (V < 0){
                move.goUp(baseTime * |V|);
                break;
              }
              if (V > 0){
                move.goDown(baseTime * |V|);
                break;
              }
            }
*/
            isCentered = false;
            if (c[2] > aSize.maxSize){
              //Go through
              cout << "Go through" << endl;
              //move.goThrough(1.7);
            }

            else if (c[2] < aSize.minSize){
              // Go Forward
              cout << "Go forward" << endl;
              move.forwardx();
            }
            else if (c[0] < aSize.maxLeft){
              // Go Left
              cout << "Go left" << endl;
              move.goLeft();
            }
            else if (c[0] > aSize.maxRight){
              // Go Right
              cout << "Go right" << endl;
              move.goRight();
            }
            else if (c[1] > aSize.maxHeight){
              // GO Down
              cout << "Go down" << endl;
              move.goDown();
            }
            else if (c[1] < aSize.minHeight){
              // Go Up
              cout << "Go up" << endl;
              move.goUp(0.1);
            }else{
              isCentered = true;
            }
            // if(message != "" && message != "DEF"){
            if(isCentered){
              cout << "Going through the circle" << endl;
              message = "DEF";
              move.hover();
              //move.goThrough(1.0);
            } else {
              move.hover();
            }

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

      putText(frame, "Circles " + numberOfCircles, cvPoint(30, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);
      putText(frame, number, cvPoint(30, 60), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);

      // Setting text on screen end //
    } else {
      // cout << "Hover" << endl;
      move.hover();
      // cout << "Hover" << endl;
    //  move.turnAround(0.1);
    }

    imshow("Detected circles", frame);
    loop_rate.sleep();
    // if (waitKey(10) == 27)
    //   break; // stop capturing by pressing ESC
  }
  // the camera will be closed automatically upon exit
  //cap.close();
  return 0;
}

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

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <cstdlib>
#include <stdio.h>
#include <std_msgs/String.h>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

using namespace cv;
using namespace std;

// Primary working frame //
Mat frameRBG;

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

// -------------------------------------------------------------------------------------------------------------


// -------------------------------------------------------------------------------------------------------------
//DroneMovements

std_msgs::Empty emp_msg;
geometry_msgs::Twist twist_msg;
geometry_msgs::Twist twist_msg_neg;
geometry_msgs::Twist twist_msg_hover;
geometry_msgs::Twist twist_msg_up;

ros::Publisher pub_empty_takeoff;
ros::Publisher pub_empty_land;
ros::Publisher pub_cmd_vel;

double start_time;

//command message
float takeoff_time = 5.0;
float fly_time = 7.0;
float land_time = 3.0;
float kill_time = 2.0;

geometry_msgs::Twist changeTwist(float x, float y, float z, float turn)
{
    geometry_msgs::Twist msg_vel;
    msg_vel.angular.x = 0;
    msg_vel.angular.y = 0;
    msg_vel.angular.z = turn;
    msg_vel.linear.x = x;
    msg_vel.linear.y = y;
    msg_vel.linear.z = z;
    return (msg_vel);
}

 void takeoff(void)
{
    std_msgs::Empty empty;
    geometry_msgs::Twist msg_vel;
    pub_empty_takeoff.publish(empty);
    ROS_INFO("Starter");
    msg_vel = changeTwist(0, 0, 0, 0);
    pub_cmd_vel.publish(msg_vel);
    // ros::Duration(3).sleep();
    ROS_INFO("Starter");
}

void land(void)
{
    std_msgs::Empty empty;
    pub_empty_land.publish(empty);
    ros::Duration(2).sleep();
}

void forwardx(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(1, 0, 0, 0);
    pub_cmd_vel.publish(msg_vel);
}

void backwardx(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(-1, 0, 0, 0);
    pub_cmd_vel.publish(msg_vel);
}

void goLeft(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 1, 0, 0);
    pub_cmd_vel.publish(msg_vel);
}

void goRight(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, -1, 0, 0);
    pub_cmd_vel.publish(msg_vel);
}

void goUp(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 0, 1, 0);
    pub_cmd_vel.publish(msg_vel);
}

void goDown(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 0, -1, 0);
    pub_cmd_vel.publish(msg_vel);
}

void turnAround(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 0, 0, 1);
    pub_cmd_vel.publish(msg_vel);
}

void hover(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 0, 0, 0);
    pub_cmd_vel.publish(msg_vel);
}




// -------------------------------------------------------------------------------------------------------------

// qrReader qr = qrReader();

RNG rng(12345);
int lowThreshold = 60;
int const max_lowThreshold = 200;
int ratio = 3;
int kernel_size = 3;

double eDistance(Vec3i a, Vec3i b)
{
  return sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2));
}

// Mat cannyEdgeDetector(Mat frame)
// {
//   Mat canny_output;
//   vector<vector<Point>> contours;
//   vector<Vec4i> hierarchy;
//   Canny(frame, canny_output, lowThreshold, lowThreshold * ratio, kernel_size);
//   findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
//   Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
//   for (size_t i = 0; i < contours.size(); i++)
//   {
//     Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
//     drawContours(drawing, contours, (int)i, color, 2, 8, hierarchy, 0, Point());
//   }
//   return canny_output;
// }

void qrDetector(Mat frame)
{
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;

  ros::NodeHandle node;

  pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
  pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1);
  pub_cmd_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  ros::Rate loop_rate(10);
  // VideoCapture cap;

  // open the default camera, use something different from 0 otherwise;
  // Check VideoCapture documentation.
  queue<Vec3i> circleQueue; /* Declare a queue */
  struct videoSize
  {
    int width;
    int height;
  };
  struct acceptSize
  {
    int minHeight;
    int maxHeight;
    int maxLeft;
    int maxRight;
  };

  videoSize vSize;
  vSize.width = 0;
  vSize.height = 0;

  acceptSize aSize;
  aSize.minHeight = 0;
  aSize.maxHeight = 0;
  aSize.maxLeft = 0;
  aSize.maxRight = 0;

  // if(!cap.open(0))
  //     return 0;
  std::string message;
  ros::Duration(1).sleep();
  while (ros::ok())
  {
    ros::spinOnce();

    // while ((double)ros::Time::now().toSec() < start_time + takeoff_time)
    // { //takeoff
    //     ros::spinOnce();
    //     takeoff();
    //     ROS_INFO("Drone taking off - Nicki");
    // }

    if (frameRBG.empty())
      break; // end of video stream

    if (vSize.width == 0 || vSize.height == 0)
    {
      vSize.width = frameRBG.cols;
      vSize.height = frameRBG.rows;

      aSize.minHeight = vSize.height / 3;
      aSize.maxHeight = vSize.height / 3 * 2;
      aSize.maxLeft = vSize.width / 3.2;
      aSize.maxRight = vSize.width / 1.4545;

      cout << "Frame Dimensions - " << vSize.width << " " << vSize.height << endl;
      cout << "Minimum Height   - " << aSize.minHeight << endl;
      cout << "Maximum Height   - " << aSize.maxHeight << endl;
      cout << "Maximum Left     - " << aSize.maxLeft << endl;
      cout << "Maximum Right    - " << aSize.maxRight << endl;
    }

    blur(frameRBG, frameRBG, Size(3, 3));
    Mat frame;
    cvtColor(frameRBG, frame, CV_RGB2GRAY);
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
    HoughCircles(frame, circles, CV_HOUGH_GRADIENT, dp, min_dist, param_1, param_2, min_radius, max_radius);
    for (size_t i = 0; i < circles.size(); i++)
    {
      Vec3i c = circles[i];
      circle(frame, Point(c[0], c[1]), c[2], Scalar(255, 255, 255), 3, CV_AA);
      circle(frame, Point(c[0], c[1]), 2, Scalar(255, 255, 255), 3, CV_AA);
    }

    //  frame = cannyEdgeDetector(frame);

    // Setting text on screen start //

    // Finds the amount of circles in the image
    unsigned long ul = circles.size();
    std::string numberOfCircles;
    std::stringstream strstream;
    strstream << ul;
    strstream >> numberOfCircles;

    if (ul != 0)
    {
      Vec3i c = circles[0];

      // Queue start //

      if (circleQueue.size() < 5)
      {
        circleQueue.push(c); //  Add some values to the queue
      }
      else
      {

        Vec3i p1 = circleQueue.front();
        Vec3i p2 = circleQueue.back();

        // cout << "The result is " << eDistance (p2,p1) << endl;

        circleQueue.pop();
      }

      // Queue end //

      // Command section start //
      
            

            if (c[0] < aSize.maxLeft){
              // Go Left
              message = "Go left ";
              goLeft();
            }
             if (c[0] > aSize.maxRight){
              // Go Right
              message = "Go right ";
              goRight();
            }
             if (c[1] > aSize.maxHeight){
              // GO Down
              message = "Go down ";
              goDown();
            }
            if (c[1] < aSize.minHeight){
              // Go Up
              message = "Go up ";
              goUp();
            } 
            // if(message != "" && message != "DEF"){
            if(message != "DEF"){
                cout << message << endl;
                message = "DEF";
            } else {
               cout << "Hover inside" << endl;
              message = "DEF";
              hover();
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
       cout << "Hover" << endl;
      hover();
    }

    imshow("detected circles", frame);
    loop_rate.sleep();
    // if (waitKey(10) == 27)
    //   break; // stop capturing by pressing ESC
  }
  // the camera will be closed automatically upon exit
  //cap.close();
  return 0;
}
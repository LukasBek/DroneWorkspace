#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <sstream>
#include <iostream>
#include <queue>
#include <deque>
#include <string>



#include <cstdlib>
#include <stdio.h>
#include <std_msgs/String.h>
#include <sstream>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>



using namespace cv;
using namespace std;
// #include "opencv2/imgcodecs.hpp"
public void sendImage (cv_bridge::CvImagePtr cv_ptr);
int thresh = 100;
int max_thresh = 255;
 
cv_bridge::CvImagePtr cv_ptr;



int main(int argc, char** argv)
{

}
  ros::init(argc, argv, "hejsa");
  ros::NodeHandle node;
    //  VideoCapture cap;

image_transport::ImageTransport it(node);
    image_transport::Subscriber sub = it.subscribe("/ardrone/front/image_raw", 1, 
      imageCb);

    
public void sendImage (cv_bridge::CvImagePtr cv_ptr){

cv::Mat HSVImage;
    cv::cvtColor(cv_ptr -> image, HSVImage, CV_BGR2HSV);
    cv::Size size = HSVImage.size();
    cv::Mat mask = cvCreateMat(size.height, size.width, CV_8UC1);
    cv::inRange(HSVImage, cv::Scalar(1,115,119), cv::Scalar(8,255,255), mask);

dilate(mask, mask, getStructuringElement(MORPH_RECT, Size(21,21)));
erode(mask, mask, getStructuringElement(MORPH_RECT, Size(10,10)));


erode(mask, mask, getStructuringElement(MORPH_RECT, Size(11,11)));
dilate(mask, mask, getStructuringElement(MORPH_RECT, Size(5,5)));

GaussianBlur(mask, mask, Size(15,15),2,2);








    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    // queue <Vec3i> circleQueue; /* Declare a queue */
    // struct videoSize {
    //   int width;
    //   int height;
    // };
    // struct acceptSize {
    //   int minHeight;
    //   int maxHeight;
    //   int maxLeft;
    //   int maxRight;
    // };

    // videoSize vSize;
    // vSize.width  = 0;
    // vSize.height = 0;

    // acceptSize aSize;
    // aSize.minHeight = 0;
    // aSize.maxHeight = 0;
    // aSize.maxLeft   = 0;
    // aSize.maxRight  = 0;

    // if(!cap.open(0))
    //     return 0;
    for(;;)
    {
      ros::spinOnce();
          if( mask.empty() ) break; // end of video stream

          // if (vSize.width == 0 || vSize.height == 0){
          //   vSize.width = frame.cols;
          //   vSize.height = frame.rows;

          //   aSize.minHeight = vSize.height / 3;
          //   aSize.maxHeight = vSize.height / 3 * 2;
          //   aSize.maxLeft  = vSize.width / 3.2;
          //   aSize.maxRight = vSize.width / 1.4545;

          //   cout << "Frame Dimensions - "  << vSize.width     << " " << vSize.height << endl;
          //   cout << "Minimum Height   - "  << aSize.minHeight << endl;
          //   cout << "Maximum Height   - "  << aSize.maxHeight << endl;
          //   cout << "Maximum Left     - "  << aSize.maxLeft   << endl;
          //   cout << "Maximum Right    - "  << aSize.maxRight  << endl;

          // }

          Mat cimg;

          // blur(frame, frame, Size(3,3));
          // cvtColor(frame,frame,CV_RGB2GRAY);

          // // const char* source_window = "Source";


          // cimg = frame;

          vector<Vec3f> circles;
          int dp = 1;             // The inverse ratio of resolution
          int min_dist = 100;     // Minimum distance between detected centers
          int param_1 = 100;      // Upper threshold for the internal Canny edge detector
          int param_2 = 90;       // Threshold for center detection
                                  // 200 Has hard time finding perfect circles
                                  // 100 Only very clear circles
                                  // 80 detects random round suff
                                  // 60 begins detecting faces
                                  // 50 sees faces
                                  // 40 sees circles in square objects
                                  // 20 or below, finds circles everywhere
          int min_radius = 20;    // Minimum radio to be detected. If unknown, put zero as default
          int max_radius = 200;   // Maximum radius to be detected. If unknown, put zero as default
          
          HoughCircles(mask, circles, CV_HOUGH_GRADIENT, dp, min_dist, param_1, param_2, min_radius, max_radius);
          for( size_t i = 0; i < circles.size(); i++ )
          {
              Vec3i c = circles[i];
              circle(cimg, Point(c[0], c[1]), c[2], Scalar(0,0,255), 3, CV_AA);
              circle(cimg, Point(c[0], c[1]), 2, Scalar(0,255,0), 3, CV_AA);
          }

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
            /*
            if (circleQueue.size() < 10){
              circleQueue.push(c);  //  Add some values to the queue
              cout << "Pushed element. Size = " << circleQueue.size() << endl;
            } else {
              for (int i = 0; i <= circleQueue.size() ; ++i) {
              cout << "Element "<< i << " accessed with [] : "<< 2 << " size " << circleQueue.size() << endl;
              circleQueue.pop();
              }
            }
            */
            // Queue end //

            // Command section start //

            // std::string message;

            // if (c[0] < aSize.maxLeft){
            //   // Go Left
            //   message = "Go left ";
            // }
            // if (c[0] > aSize.maxRight){
            //   // Go Right
            //   message = "Go right ";
            // }
            // if (c[1] > aSize.maxHeight){
            //   // GO Down
            //   message = "Go down ";
            // }
            // if (c[1] < aSize.minHeight){
            //   // Go Up
            //   message = "Go up ";
            // }

            // if(message != "" && message != "DEF"){
            //     cout << message << endl;
            //     message = "";
            // } else {
            //   message = "DEF";
            // }
            
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

            putText(cimg, "Circles " + numberOfCircles, cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,255,0), 1, CV_AA);
            putText(cimg, number, cvPoint(30,60), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,255,0), 1, CV_AA);

            // Setting text on screen end //

          }

          imshow("detected circles", cimg);

          if( waitKey(10) == 27 ) break; // stop capturing by pressing ESC
    }
    // the camera will be closed automatically upon exit
    //cap.close();
    return 0;
}

/*
void thresh_callback(int, void* )
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  Canny( src_gray, canny_output, thresh, thresh*2, 3 );
  findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( size_t i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, Point() );
     }
  namedWindow( "Contours", WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );
}
*/

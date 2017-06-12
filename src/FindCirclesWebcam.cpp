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

#include <cmath>

// #include <chrono>

using namespace cv;
using namespace std;
using namespace zbar;

// typedef std::chrono::high_resolution_clock Clock;

Mat frameRBG;  // Primary blur working frame from capture //
Mat frame;     // Primary gray working frame //
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
    bool isCentered;

    move.init(node);

    ros::Rate loop_rate(500);

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
    vSize.width = 0;
    vSize.height = 0;

    acceptSize aSize;
    aSize.minHeight = 0;
    aSize.maxHeight = 0;
    aSize.maxLeft = 0;
    aSize.maxRight = 0;
    aSize.maxSize = 0;
    aSize.minSize = 0;

    ros::Duration(1).sleep();
    int b = 0;
    double baseTime = 0.001;

    double H;
    double V;

    double Htime;
    double Vtime;

    // true = clockwise

    bool bGoLeft = false;
    double rectLastWidth = 0;
    double rectLastHeight = 0;
    double rectLastComparison = 0;
    double rectComparison = 0;
    bool bFindRect = false;

    int turnCounter = 0;

    while (ros::ok())
    {
	loop_rate.sleep();
	ros::spinOnce();

	//while ((double)ros::Time::now().toSec() < start_time + takeoff_time + 2)
	//while ((double)ros::Time::now().toSec() < 7 + 2)
	//{ //takeoff

	// while (b < 50)
	// {
	//     ros::spinOnce();
	//     //// Denne skal udkommenteres for kun at teste kamera og så dronen ikke letter
	//     move.takeoff();

	//     if (b == 49)
	//     {
	// 	cout << "Taking off" << endl;
	// 	ros::Duration(4).sleep();
	//     }
	//     b++;
	// }
	// while (b < 51)
	// {
	//     cout << "OPSADASSE!" << endl;
	//     ros::spinOnce();
	//     move.goUp(1.6);
	//     b++;
	// }

	// cout << "TRY ME" << endl;
	if (frameRBG.empty())
	{
	    cout << "No frame from capture, end og video stream" << endl;
	    break; // end of video stream
	}

	if (vSize.width == 0 || vSize.height == 0)
	{
	    vSize.width = frameRBG.cols;
	    vSize.height = frameRBG.rows;

	    aSize.minHeight = vSize.height / 2.5;
	    aSize.maxHeight = vSize.height / 7;
	    aSize.maxLeft = vSize.width / 2.5;
	    aSize.maxRight = vSize.width / 1.6;
	    aSize.maxSize = vSize.height / 4;
	    //Jo mindre tal, jo tættere på kommer dronen
	    aSize.minSize = vSize.height / 10;

	    cout << "Frame Dimensions - " << vSize.width << " " << vSize.height << endl;
	    cout << "Minimum Height   - " << aSize.minHeight << endl;
	    cout << "Maximum Height   - " << aSize.maxHeight << endl;
	    cout << "Maximum Left     - " << aSize.maxLeft << endl;
	    cout << "Maximum Right    - " << aSize.maxRight << endl;
	    cout << "Maximum Size     - " << aSize.maxSize << endl;
	    cout << "Minimum Size     - " << aSize.minSize << endl;
	}

	noBlurRGB = frameRBG.clone();

	// Blur and convertion to grayscale //
	blur(frameRBG, frameRBG, Size(5, 5));

	cvtColor(frameRBG, frame, CV_RGB2GRAY);

	// Zbar Start //
	string res = zbarScan(frame, vSize.width, vSize.height);
	if (!res.empty())
	{
	    cout << "Symbol: " << zbarScan(frame, vSize.width, vSize.height) << endl;
	}
	// Zbar End //

	bool rectFoundCircle;
	bool houghFoundCircle;
	int rectWidth;
	int rectHeight;
	int rectPosX;
	int rectPosY;
	int houghPosX;
	int houghPosY;
	int houghSize;

	minBoundingBoxes(redFilter(noBlurRGB), frame, &rectWidth, &rectHeight, &rectPosX, &rectPosY, &houghPosX, &houghPosY, &houghSize, &rectFoundCircle, &houghFoundCircle);

	// cout << "--------------" << endl;
	// cout << "Width:  " << rectWidth   << endl;
	// cout << "Height: " << rectHeight  << endl;
	// cout << "Y:      " << rectPosY       << endl;
	// cout << "X:      " << rectPosX       << endl;
	// cout << "--------------" << endl;

	// pRect //

	// putText(frame, "Test", cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,255,255), 1, CV_AA);

	// HoughCircles Start //
	std::vector<Vec3f> circles;
	getCircles(frame, &circles);
	// HoughCircles End //

	// Finds the amount of circles in the image
	unsigned long ul = circles.size();
	std::string numberOfCircles;
	std::stringstream strstream;
	strstream << ul;
	strstream >> numberOfCircles;

	bFindRect = false;
	if (!(rectWidth == 0 || rectHeight == 0))
	{
	    rectComparison = (double)rectHeight / (double)rectWidth;
	    bFindRect = true;
	}

	if (ul != 0)
	{
	    turnCounter = 0;

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

		while (circleQueueTemp.size() > 0)
		{

		    // cout << "Round " << iterator << ", Distance " << eDistance(circleQueueTemp.front(),c) << ", max distance " << (iterator/20)*vSize.width << endl;

		    if (eDistance(circleQueueTemp.front(), c) < iterator / 20 * vSize.width)
		    {
			//cout << "Accept circle into que" << endl;
			//cout << "Accept" << endl;
			circleQueue.pop();
			circleQueue.push(c);
			break;
		    }
		    else
		    {
			//cout << "Deny circle into que" << endl;
			//cout << "Deny" << endl;
			circleQueueTemp.pop();
		    }
		    iterator++;
		}
	    }

	    // Queue end //

	    // Command section start //

	    // Midten af cirklens placering i framen
	    H = c[0] - 320;
	    V = c[1] - 180;
	    // Makes sure that even though V or H is negative, it will be a possitive number
	    Htime = std::abs(H);
	    Vtime = std::abs(V);
	    // Circle end

	    if (H < -40 || H > 40)
	    {
		if (H > 0)
		{
		    // cout << "Go right: H=" << H << " time=" << baseTime * Htime << " c0=" << c[0] << endl;
		    move.goRight(baseTime * Htime);
		    continue;
		}
		else if (H < 0)
		{
		    // cout << "Go left: H=" << H << " time=" << baseTime * Htime << " c0=" << c[0] << endl;
		    move.goLeft(baseTime * Htime);
		    continue;
		}
	    }

	    else if (V < -20 || V > 20)
	    {
		if (V < 0)
		{
		    // cout << "Go up: V=" << V << " time=" << baseTime * Vtime << " c1=" << c[1] << endl;
		    move.goUp(baseTime * Vtime);
		    continue;
		}
		if (V > 0)
		{
		    // cout << "Go down: V=" << V << " time=" << baseTime * Vtime << " c1=" << c[1] << endl;
		    move.goDown(baseTime * Vtime);
		    continue;
		}
	    }
	    else if (c[2] < aSize.minSize)
	    {
		// cout << "Go forward" << endl;
		move.forwardx(0.15);
	    }
	    else if (c[2] > aSize.maxSize)
	    {
		isCentered = true;
	    }

	    if (isCentered)
	    {
		// cout << "Going through the circle" << endl;
		// message = "DEF"
		// move.hover();
		move.goThrough(1.5);
	    }
	    else
	    {
		move.hover();
	    }

	    isCentered = false;

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
	}

	else if (bFindRect)
	{
	    // if (rectWidth + 20 < rectHeight)
	    // {
	    turnCounter = 0;
	    //------------------------------------------------

	    // Midten af firkantens placering i framen
	    H = rectPosX - 320;
	    V = rectPosY - 180;
	    // Makes sure that even though V or H is negative, it will be a possitive number, so time wont be negative.
	    Htime = std::abs(H);
	    Vtime = std::abs(V);
	    // Circle end

	    if (H < -40 || H > 40)
	    {
		if (H > 0)
		{
		    // cout << "Turn clock: H=" << H << " time=" << baseTime * Htime << endl;
		    move.turnAroundClockwise(0.05, 0.1);
		    // cout << " - X= " << rectPosX <<  " bredden =" << rectPosX + rectWidth/2  <<  endl;
		    continue;
		}
		else if (H < 0)
		{
		    // cout << "turn Counter clock: H=" << H << " time=" << baseTime * Htime << endl;
		    move.turnAroundCounterClockwise(0.05, 0.1);
		    // cout << " - X= " << rectPosX << " bredden =" << rectPosX - rectWidth/2 <<  endl;

		    continue;
		}
	    }
	    else if (V < -20 || V > 20)
	    {
		if (V < 0)
		{
		    // cout << "rect Go up: V=" << V << " time=" << baseTime * Vtime << endl;
		    move.goUp(0.1);
		    continue;
		}
		if (V > 0)
		{
		    // cout << "rect Go down: V=" << V << " time=" << baseTime * Vtime << endl;
		    move.goDown(0.1);
		    continue;
		}
	    }

	    //-------------------------------------------------------------------------
	    int picCounter = 0;
	    int arrayRectWidth[5];
	    int arrayRectHeight[5];
	    while (picCounter < 6)
	    {
		ros::spinOnce();
		minBoundingBoxes(redFilter(noBlurRGB), frame, &rectWidth, &rectHeight, &rectPosX, &rectPosY, &houghPosX, &houghPosY, &houghSize, &rectFoundCircle, &houghFoundCircle);
		arrayRectWidth[picCounter] = rectWidth;
		cout << "while rectwidth =" << arrayRectWidth[picCounter];
	    }

	    cout << "rectComparison=" << rectComparison << " rectLASTCOM=" << rectLastComparison << endl;
	    if (rectComparison > rectLastComparison)
	    // if (< rectLastWidth)
	    {
		bGoLeft = !bGoLeft;
	    }

	    if (bGoLeft)
	    {
		// cout << "rect go left" << endl;
		move.goLeft(0.1);
	    }
	    else
	    {
		// cout << "rect go right" << endl;
		move.goRight(0.1);
	    }

	    rectLastComparison = rectComparison;
	    // }
	}
	else
	{
	    // cout << "Hover" << endl;
	    move.hover();
	    //  cout << "turn around... " << endl;
	    // move.turnAroundClockwise(0.1, 0.2);
	    // resets the rectLastWidth
	    // rectLastWidth = 0;

	    // if (turnCounter >= 12 && turnCounter < 25)
	    // {
	    // cout << "nothing found - turning clockwise" << endl;
	    // move.turnAroundClockwise(0.1, 0.2);
	    // if (turnCounter == 14)
	    // {
	    //     turnCounter = 0;
	    // }
	    // }
	    // if (turnCounter < 12)
	    // {
	    // cout << "nothing found - turning counter clockwise" << endl;
	    // move.turnAroundCounterClockwise(0.1, 0.2);
	    // }
	    // turnCounter++;
	}

	// if (waitKey(10) == 27)
	//   break; // stop capturing by pressing ESC
    }
    // the camera will be closed automatically upon exit
    //cap.close();
    return 0;
}

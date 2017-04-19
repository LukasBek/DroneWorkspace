#include <cstdlib>
#include <stdio.h>
#include <std_msgs/String.h>
#include <sstream>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

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
//test
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

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test1");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);

    pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
    pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1);
    pub_cmd_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(node);
    image_transport::Subscriber sub = it.subscribe("/ardrone/front/image_raw", 1, imageCallback);

    int count = 0;
    start_time = (double)ros::Time::now().toSec();

    // takeoff();

    while (ros::ok())
    {
    ros::spinOnce();

    while ((double)ros::Time::now().toSec() < start_time + takeoff_time)
    { //takeoff
        ros::spinOnce();
        takeoff();
        ROS_INFO("Drone taking off - Nicki");
    }

    while ((double)ros::Time::now().toSec() < start_time + takeoff_time + 5)
    {
        ROS_INFO("tid til at sove");
        // ros::Duration(5).sleep();
        ROS_INFO("færdig med at sove");
        ros::spinOnce();
        
    }
    ROS_INFO("testing %d", count);

    land();
    count++;
    loop_rate.sleep();
    }
    cv::destroyWindow("view");
    return 0;
}

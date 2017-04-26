#include <cstdlib>
#include <stdio.h>
#include <std_msgs/String.h>
#include <sstream>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>



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

std_msgs::Empty emp_msg;
geometry_msgs::Twist twist_msg;
geometry_msgs::Twist twist_msg_neg;
geometry_msgs::Twist twist_msg_hover;
geometry_msgs::Twist twist_msg_up;

ros::Publisher pub_empty_takeoff;
ros::Publisher pub_empty_land;
ros::Publisher pub_cmd_vel;



//command message
double start_time = 2;
float takeoff_time = 5.0;
float fly_time = 7.0;
float land_time = 3.0;
float kill_time = 2.0;
float sleepD = 0.1;

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
    ros::Duration(sleepD).sleep();
}

void backwardx(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(-1, 0, 0, 0);
    pub_cmd_vel.publish(msg_vel);
    ros::Duration(sleepD).sleep();
}

void goLeft(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 1, 0, 0);
    pub_cmd_vel.publish(msg_vel);
    ros::Duration(sleepD).sleep();
}

void goRight(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, -1, 0, 0);
    pub_cmd_vel.publish(msg_vel);
    ros::Duration(sleepD).sleep();
}

void goUp(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 0, 1, 0);
    pub_cmd_vel.publish(msg_vel);
    ros::Duration(sleepD).sleep();
}

void goDown(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 0, -1, 0);
    pub_cmd_vel.publish(msg_vel);
    ros::Duration(sleepD).sleep();
}

void turnAround(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 0, 0, 1);
    pub_cmd_vel.publish(msg_vel);
}

void goThrough(void)
{
  geometry_msgs::Twist msg_vel;
  msg_vel = changeTwist(1, 0, 0, 0);
  pub_cmd_vel.publish(msg_vel);
  ros::Duration(1).sleep();
}

void hover(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 0, 0, 0);
    pub_cmd_vel.publish(msg_vel);
}
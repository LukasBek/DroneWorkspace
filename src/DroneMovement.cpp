#include <cstdlib>
#include <stdio.h>
#include <std_msgs/String.h>
#include <sstream>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

using namespace std;

//DroneMovements
    
std_msgs::Empty emp_msg;
geometry_msgs::Twist twist_msg;
geometry_msgs::Twist twist_msg_neg;
geometry_msgs::Twist twist_msg_hover;
geometry_msgs::Twist twist_msg_up;

ros::Publisher pub_empty_takeoff;
ros::Publisher pub_empty_land;
ros::Publisher pub_cmd_vel;

class DroneMovement{    
public:
    //command message
    double start_time;
    float takeoff_time;
    float fly_time;
    float land_time;
    float kill_time;
    float sleepD;
    int circleFound;
    void init(ros::NodeHandle node);
    void takeoff(void);
    void land(void);
    void forwardx(void);
    void backwardx(void);
    void goLeft(void);
    void goRight(void);
    void goUp(void);
    void goDown(void);
    void turnAround(double turnTime);
    void goThrough(void);
    void hover(void);
    void findCircle();
};

double start_time = 2;
float takeoff_time = 5.0;
float fly_time = 7.0;
float land_time = 3.0;
float kill_time = 2.0;
float sleepD = 0.1;
int circleFound;

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

void DroneMovement::init(ros::NodeHandle node){
    pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
    pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1);
    pub_cmd_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

 void DroneMovement::takeoff(void)
{
      std_msgs::Empty empty;
    geometry_msgs::Twist msg_vel;
    pub_empty_takeoff.publish(empty);
    ROS_INFO("Starter");
    msg_vel = changeTwist(0, 0, 0, 0);
    pub_cmd_vel.publish(msg_vel);
    ROS_INFO("Efter Starter");
}

void DroneMovement::land(void)
{
    std_msgs::Empty empty;
    pub_empty_land.publish(empty);
    ros::Duration(2).sleep();
}

void DroneMovement::forwardx(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(1, 0, 0, 0);
    pub_cmd_vel.publish(msg_vel);
    ros::Duration(sleepD).sleep();
}

void DroneMovement::backwardx(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(-1, 0, 0, 0);
    pub_cmd_vel.publish(msg_vel);
    ros::Duration(sleepD).sleep();
}

void DroneMovement::goLeft(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 1, 0, 0);
    pub_cmd_vel.publish(msg_vel);
    ros::Duration(sleepD).sleep();
}

void DroneMovement::goRight(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, -1, 0, 0);
    pub_cmd_vel.publish(msg_vel);
    ros::Duration(sleepD).sleep();
}

void DroneMovement::goUp(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 0, 1, 0);
    pub_cmd_vel.publish(msg_vel);
    ros::Duration(sleepD).sleep();
}

void DroneMovement::goDown(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 0, -1, 0);
    pub_cmd_vel.publish(msg_vel);
    ros::Duration(sleepD).sleep();
}

void DroneMovement::turnAround(double turnTime)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 0, 0, 0.2);
    pub_cmd_vel.publish(msg_vel);
    ros::Duration(turnTime).sleep();
}

void DroneMovement::goThrough(void)
{
  geometry_msgs::Twist msg_vel;
  msg_vel = changeTwist(1, 0, 0, 0);
  pub_cmd_vel.publish(msg_vel);
  ros::Duration(1).sleep();
}

void DroneMovement::hover(void)
{
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0, 0, 0, 0);
    pub_cmd_vel.publish(msg_vel);
}

 void DroneMovement::findCircle(){
    
    
    turnAround(0.1);
  }
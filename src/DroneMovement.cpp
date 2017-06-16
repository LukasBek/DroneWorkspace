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

class DroneMovement
{
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
void forwardx(double time);
void backwardx(double time);
void goLeft(double time);
void goRight(double time);
void goUp(double time);

void turnAroundCounterClockwise(double turnTime, double rotationSpeed);
void turnAroundClockwise(double turnTime, double rotationSpeed);
void goDown(double time);

void goThrough(double time);
void hover(void);
void findCircle();
};

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

void DroneMovement::init(ros::NodeHandle node)
{
        pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
        pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1);
        pub_cmd_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        start_time = 2;
        takeoff_time = 5.0;
        fly_time = 7.0;
        land_time = 3.0;
        kill_time = 2.0;
        sleepD = 0.25;
        circleFound;
}

void DroneMovement::takeoff(void)
{
        // cout << "D takeoff" << endl;
        std_msgs::Empty empty;
        geometry_msgs::Twist msg_vel;
        pub_empty_takeoff.publish(empty);
        // ROS_INFO("Starter");
        msg_vel = changeTwist(0, 0, 0, 0);
        pub_cmd_vel.publish(msg_vel);
        // ROS_INFO("Efter Starter");
}

void DroneMovement::land(void)
{
        cout << "D land" << endl;
        std_msgs::Empty empty;
        pub_empty_land.publish(empty);
        ros::Duration(2).sleep();
}

void DroneMovement::forwardx(double time)
{
        cout << "D forward" << endl;
        geometry_msgs::Twist msg_vel;
        msg_vel = changeTwist(0.5, 0, 0, 0);
        pub_cmd_vel.publish(msg_vel);
        ros::Duration(time).sleep();
        hover();
}

void DroneMovement::backwardx(double time)
{
        cout << "D backward" << endl;
        geometry_msgs::Twist msg_vel;
        msg_vel = changeTwist(-0.5, 0, 0, 0);
        pub_cmd_vel.publish(msg_vel);
        ros::Duration(time).sleep();
        hover();
}

void DroneMovement::goLeft(double time)
{
        cout << "D goLeft=" << time << endl;
        geometry_msgs::Twist msg_vel;
        msg_vel = changeTwist(0, 0.2, 0, 0);
        pub_cmd_vel.publish(msg_vel);
        ros::Duration(time).sleep();
        hover();
}

void DroneMovement::goRight(double time)
{
        cout << "D goRight=" << time << endl;
        geometry_msgs::Twist msg_vel;
        msg_vel = changeTwist(0, -0.2, 0, 0);
        pub_cmd_vel.publish(msg_vel);
        ros::Duration(time).sleep();
        hover();
}

void DroneMovement::goUp(double time)
{
        cout << "D goUp" << endl;
        geometry_msgs::Twist msg_vel;
        msg_vel = changeTwist(0, 0, 0.5, 0);
        pub_cmd_vel.publish(msg_vel);
        ros::Duration(time).sleep();
        hover();
}

void DroneMovement::goDown(double time)
{
        cout << "D goDown" << endl;
        geometry_msgs::Twist msg_vel;
        msg_vel = changeTwist(0, 0, -1, 0);
        pub_cmd_vel.publish(msg_vel);
        ros::Duration(time).sleep();
        hover();
}

void DroneMovement::turnAroundCounterClockwise(double turnTime, double rotationSpeed)
{
        cout << "D turnCounterClockwise" << endl;
        geometry_msgs::Twist msg_vel;
        msg_vel = changeTwist(0, 0, 0, rotationSpeed);
        pub_cmd_vel.publish(msg_vel);
        ros::Duration(turnTime).sleep();
        hover();
}

void DroneMovement::turnAroundClockwise(double turnTime, double rotationSpeed)
{
        cout << "D turnClockwise" << endl;
        geometry_msgs::Twist msg_vel;
        msg_vel = changeTwist(0, 0, 0, -rotationSpeed);
        pub_cmd_vel.publish(msg_vel);
        ros::Duration(turnTime).sleep();
        hover();
}

void DroneMovement::goThrough(double time)
{
        cout << "D goTrough" << endl;
        geometry_msgs::Twist msg_vel;
        msg_vel = changeTwist(0.5, 0, 0, 0);
        pub_cmd_vel.publish(msg_vel);
        ros::Duration(time).sleep();
        hover();
}

void DroneMovement::hover(void)
{
        // cout << "D hover" << endl;
        geometry_msgs::Twist msg_vel;
        msg_vel = changeTwist(0, 0, 0, 0);
        pub_cmd_vel.publish(msg_vel);
}

void DroneMovement::findCircle()
{
        cout << "D findCircle....den er ikke ordentlig denne metode. " << endl;
        turnAroundCounterClockwise(0.5, 0.2);

        turnAroundClockwise(0.1, 0.2);
}

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
			float takeoff_time=5.0;
			float fly_time=7.0;
			float land_time=3.0;
			float kill_time =2.0;

	geometry_msgs::Twist changeTwist(float x, float y, float z, float turn){
	geometry_msgs::Twist msg_vel;
	msg_vel.angular.x = 0;
		msg_vel.angular.y = 0;
		msg_vel.angular.z = turn;
		msg_vel.linear.x = x;
		msg_vel.linear.y = y;
		msg_vel.linear.z = z;
		return(msg_vel); 
	}



void takeoff (void) {
		std_msgs::Empty empty;
		geometry_msgs::Twist msg_vel;
		pub_empty_takeoff.publish(empty);
		printf("Starter");
		usleep(250000);
		printf("Den er lettet");
		msg_vel = changeTwist(0,0,0,0);
		pub_cmd_vel.publish(msg_vel);
	}

void land (void) {
	std_msgs::Empty empty;
	pub_empty_land.publish(empty);
}

void forwardx (void) {
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(1,0,0,0);
    pub_cmd_vel.publish(msg_vel);
}

void turnAround (void) {
    geometry_msgs::Twist msg_vel;
    msg_vel = changeTwist(0,0,0,1);
    pub_cmd_vel.publish(msg_vel);
}

int main(int argc, char **argv)
{
ros::init(argc, argv, "test1");
ros::NodeHandle node;
ros::Rate loop_rate(10);

pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
pub_empty_land    = node.advertise<std_msgs::Empty>("/ardrone/land",1);
pub_cmd_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel",1);

int count = 0;
start_time =(double)ros::Time::now().toSec();

while (ros::ok()) {
 ros::spinOnce();
 
while ((double)ros::Time::now().toSec()< start_time+takeoff_time + 10){ //takeoff
takeoff();
ROS_INFO("Drone taking off - Nicki");
}


while ((double)ros::Time::now().toSec()< start_time+takeoff_time + 2 ){ //takeoff
turnAround();
}



land();

    // forwardx();
	


//  loop_rate.sleep();



ROS_INFO("testing %d", count);
// printf("Testing %d", count);
count++;
} 

    return 0;
}



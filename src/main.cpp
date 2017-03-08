#include <cstdlib>
#include <stdio.h>
#include <std_msgs/String.h>
#include <sstream>

#include <ros/ros.h>

int main(int argc, char **argv)
{

//    int rate = 10;

    ros::init(argc, argv, "test1");
    ros::NodeHandle node;
//    ros::Publisher chatter_pub = node.advertise<std_msgs::String>("test2", 100);
//    ros::NodeHandle node;
//    ros::Rate loop_rate(50);

   // ros::Rate r(rate);
	printf("Testing 1,2,3");

//ros::Publisher pub_info;
//    ros::Publisher pub_takeoff;
//    ros::Publisher pub_land;

// ros::Publisher pub_land = node.advertise<std_msgs::String>("topic_name", 5);


    return 0;
}


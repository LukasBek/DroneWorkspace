#ifndef DRONEMOVEMENT_H_INCLUDED
#define DRONEMOVEMENT_H_INCLUDED

class DroneMovement{
public:
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

#endif
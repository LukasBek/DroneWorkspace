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

	void forwardx(double time);

	void backwardx(double time);

	void goLeft(double time);

	void goRight(double time);

	void goUp(double time);

	void goDown(double time);

	void turnAroundCounterClockwise(double turnTime, double rotationSpeed);
    
	void turnAroundClockwise(double turnTime, double rotationSpeed);

	void goThrough(double time);

	void hover(void);

	void findCircle();	
};

#endif
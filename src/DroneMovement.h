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

	void forwardx();

	void backwardx(void);

	void goLeft(void);

	void goRight(void);

	void goUp(double time);

	void goDown(void);

	void turnAroundCounterClockwise(double turnTime, double rotationSpeed);
    
	void turnAroundClockwise(double turnTime, double rotationSpeed);

	void goThrough(double time);

	void hover(void);

	void findCircle();	
};

#endif
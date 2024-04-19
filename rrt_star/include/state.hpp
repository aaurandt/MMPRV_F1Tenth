#ifndef STATE_H
#define STATE_H

class State{
public:
	State();
	State(double x_, double y_, double yaw_);
	double getX();
	double getY();
	double getYaw();
	void setX(double x_);
	void setY(double y_);
	void setYaw(double yaw_);
private:
	double x;
	double y;
	double yaw;
};

#endif

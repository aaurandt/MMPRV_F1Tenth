#ifndef INPUT_H
#define INPUT_H

#include <Eigen/Core>

class Input{
public:
	Input();
	Input(double velocity_, double steering_angle_);
	double getVelocity();
	double getSteeringAngle();
	Eigen::VectorXd getVector();
	void setVelocity(double velocity_);
	void setSteeringAngle(double steering_angle_);
private:
	double velocity;
	double steering_angle;
};

#endif

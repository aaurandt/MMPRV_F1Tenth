#include "input.hpp"

Input::Input(){
	velocity = 0.0;
	steering_angle = 0.0;
}

Input::Input(double velocity_, double steering_angle_){
	velocity = velocity_;
	steering_angle = steering_angle_;
}

double Input::getVelocity(){
	return velocity;
}

double Input::getSteeringAngle(){
	return steering_angle;
}

Eigen::VectorXd Input::getVector(){
	Eigen::VectorXd v(2,1);
	v << velocity, steering_angle;
	return v;
}

void Input::setVelocity(double velocity_){
	velocity = velocity_;
}

void Input::setSteeringAngle(double steering_angle_){
	steering_angle = steering_angle_;
}

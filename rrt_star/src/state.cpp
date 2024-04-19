#include "state.hpp"

State::State(){
	x = 0.0;
	y = 0.0;
	yaw = 0.0;
}

State::State(double x_, double y_, double yaw_){
	x = x_;
	y = y_;
	yaw = yaw_;
}

double State::getX(){
	return x;
}

double State::getY(){
	return y;
}

double State::getYaw(){
	return yaw;
}

void State::setX(double x_){
	x = x_;
}

void State::setY(double y_){
	y = y_;
}

void State::setYaw(double yaw_){
	yaw = yaw_;
}

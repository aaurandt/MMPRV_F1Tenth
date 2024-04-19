#include "position.hpp"

Position::Position(){
	x = 0.0;
	y = 0.0;
}

Position::Position(double x_, double y_){
	x = x_;
	y = y_;
}

double Position::getX(){
	return x;
}

double Position::getY(){
	return y;
}

void Position::setX(double x_){
	x = x_;
}

void Position::setY(double y_){
	y = y_;
}

#include "constraints.hpp"
#include <OsqpEigen/OsqpEigen.h>
#include <iostream>

Constraints::Constraints(){
	u_min.setVelocity(0.5);
	u_min.setSteeringAngle(-.436332);
	u_max.setVelocity(5.0);
	u_max.setSteeringAngle(.436332);
}

Input Constraints::getUMin(){
	return u_min;
}

Input Constraints::getUMax(){
	return u_max;
}
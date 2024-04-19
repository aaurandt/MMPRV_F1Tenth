#ifndef POSITION_H
#define POSITION_H

class Position{
public:
	Position();
	Position(double x_, double y_);
	double getX();
	double getY();
	void setX(double x_);
	void setY(double y_);
private:
	double x;
	double y;
};

#endif

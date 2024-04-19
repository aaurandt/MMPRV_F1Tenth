#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include "state.hpp"
#include "input.hpp"

class Constraints{
public:
	Constraints();
	Input getUMin();
	Input getUMax();
private:
	Input u_min;
	Input u_max;
};

#endif

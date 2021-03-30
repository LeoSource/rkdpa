#pragma once

#include "RobotMath.h"

using namespace std;
using namespace Eigen;

enum LinkType
{
	e_rotation = 0,
	e_prismatic = 1
};

class MDHLink
{
public:
	int _type;

	RobotTools::Pose _pose;

private:
	double _theta;
	double _d;
	double _a;
	double _alpha;
	double _offset;

public:
	MDHLink() {}

	MDHLink(double theta, double d, double a, double alpha, int type, double offset);

	void Transform(double q);

	~MDHLink();
};


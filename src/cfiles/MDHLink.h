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

	MDHLink(double theta, double d, double a, double alpha, int type, double offset):
		_theta(theta), _d(d), _a(a), _alpha(alpha), _type(type), _offset(offset)
	{}

	void Transform(double q)
	{
		if (_type==e_rotation)
			_theta = q+_offset;
		else
			_d = q+_offset;

		double st = sin(_theta); double sa = sin(_alpha);
		double ct = cos(_theta); double ca = cos(_alpha);
		_pose.rot(0, 0) = ct;
		_pose.rot(0, 1) = -st;
		_pose.rot(0, 2) = 0;

		_pose.rot(1, 0) = st*ca;
		_pose.rot(1, 1) = ct*ca;
		_pose.rot(1, 2) = -sa;

		_pose.rot(2, 0) = st*sa;
		_pose.rot(2, 1) = ct*sa;
		_pose.rot(2, 2) = ca;

		_pose.pos<<_a, -sa*_d, ca*_d;
	}

	~MDHLink() {}
};


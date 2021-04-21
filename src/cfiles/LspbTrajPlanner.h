#pragma once

#include "RobotMath.h"

using namespace std;
using namespace Eigen;

class LspbTrajPlanner
{
public:
	double _max_vel;
	double _max_acc;
	double _max_jerk;
	double _uniform_vel;

	int _np;
	double _tf;
	double _tc;
	VectorXd _pos;

public:
	LspbTrajPlanner() {}

	LspbTrajPlanner(VectorXd pos, double tf, double max_vel, double max_acc, char* option);

	void InitPlanner(VectorXd pos, double tf, double max_vel, double max_acc, char* option);

	~LspbTrajPlanner();

	RobotTools::JAVP GenerateMotion(double t);
};


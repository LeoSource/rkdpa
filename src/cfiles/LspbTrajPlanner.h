#pragma once

#include "RobotMath.h"

using namespace std;
using namespace Eigen;

class LspbTrajPlanner
{
public:
	double _vmax;
	double _amax;
	bool _maxvel_reached;
private:
	int _np;
	double _t0;
	double _tf;
	double _v0;
	double _vf;
	double _q0;
	double _qf;
	double _ta;
	double _td;
	int _dir;

public:
	LspbTrajPlanner() {}

	LspbTrajPlanner(Vector2d pos, double max_vel, double max_acc);

	LspbTrajPlanner(Vector2d pos, double max_vel, double max_acc, double tf);

	LspbTrajPlanner(Vector2d pos, double max_vel, double max_acc, Vector2d duration);

	LspbTrajPlanner(Vector2d pos, double max_vel, double max_acc, Vector2d duration, Vector2d vel_con);

	void InitPlanner(Vector2d pos, double max_vel, double max_acc);

	void InitPlanner(Vector2d pos, double max_vel, double max_acc, double tf);

	void InitPlanner(Vector2d pos, double max_vel, double max_acc, Vector2d duration);

	void InitPlanner(Vector2d pos, double max_vel, double max_acc, Vector2d duration, Vector2d vel_con);

	~LspbTrajPlanner() {}

	RobotTools::JAVP GenerateMotion(double t);

	double GetFinalTime();

	double GetDuratoin();

private:
	void TransformPVA(Vector2d pos, Vector2d vel, double max_vel, double max_acc);

	void SetNoTimeLimit(double h);

	void SetTimeLimit(double h, Vector2d duration);

	void SetVelConstraint(double h);
};


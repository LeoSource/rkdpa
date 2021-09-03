#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "RobotMath.h"
#include "GlobalParams.h"
#include "JointPlanner.h"

using namespace std;

class RobotDynIdenTest
{
private:
	vector<vector<double>> _traj_order_params;
	double _w0;
	int _order;
	VectorXd _q_b, _qd_b, _qdd_b;
	double _t;
	JointPlanner _jtraj_planner;
	Matrix<int, 5, 1> _traj_id;
	vector<Vector6d> _q0;
	int _traj_idx;
	Vector6d _q_cur;
private:
	ofstream _ofile;

public:
	bool _traj_completed;

public:
	RobotDynIdenTest(int order, Vector6d q_cur, string filepath);

	Vector6d GenerateTraj();

	~RobotDynIdenTest() {}

private:
	void LoadTrajParams(string filepath);

	RobotTools::JAVP GenerateJntMotion(double t, int jnt_idx, int traj_idx);

	Vector6d GenerateMotion(double t, int traj_idx);

};


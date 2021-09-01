/**
* @file		BaseTrajPlanner.h
* @brief	Trajectory plan interface for all kinds of trajectory
* @version	1.0.0
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2021/7/29
**/
#pragma once

#include "RobotMath.h"

using namespace std;
using namespace Eigen;

class BaseTrajPlanner
{
public:
	bool _plan_completed;
protected:
	double _t;

public:
	BaseTrajPlanner() { _t = 0; _plan_completed = false; }

	virtual void AddViaPos(MatrixXd* via_pos) = 0;

	virtual void Reset(Vector6d pos, bool option) = 0;

	virtual void GeneratePath(Vector6d& pos) = 0;

	virtual void GenerateMotion(Vector6d& pos, Vector6d& vel, Vector6d& acc) = 0;

	virtual ~BaseTrajPlanner() {}
};


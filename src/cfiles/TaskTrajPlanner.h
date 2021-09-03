/**
* @file		TaskTrajPlanner.h
* @brief	Trajectory plan for all tasks
* @version	1.0.0
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2021/7/28
**/
#pragma once

#include "UrRobot.h"
#include "CartesianPlanner.h"
#include "JointPlanner.h"
#include "CubicBSplinePlanner.h"

class TaskTrajPlanner
{
public:
	bool _task_completed;
private:
	int _ntraj;
	int _traj_idx;
	vector<TrajType> _traj_type;
	vector<BaseTrajPlanner*> _segplanner;
	UrRobot* _robot;
	Vector6d _pre_q;
	Pose _pre_trajpose;
	Vector6d _pre_trajq;

public:
	TaskTrajPlanner() {}

	TaskTrajPlanner(UrRobot* robot_model, Vector6d q0);

	void AddTraj(MatrixXd* via_pos, TrajType traj_type, bool traj_opt);

	void GenerateJPath(Vector6d& jpos);

	void GenerateCPath(Vector6d& cpos);

	void GenerateBothPath(Vector6d& jpos, Vector6d& cpos);

	void Reset(Vector6d q0);

	~TaskTrajPlanner() {}
private:
	double CalcBSplineTime(MatrixXd* via_pos);

	void CheckCartWorkspace(Vector3d cpos);
};


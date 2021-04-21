#pragma once

#include "CleanRobot.h"

class BaseTask
{
public:
	bool _task_completed;
protected:
	CleanRobot* _rbt;
public:
	//virtual void InitialTask() = 0;

	virtual void SetTrajPos(MatrixXd traj_pos, VectorXd q_fdb) = 0;

	virtual VectorXd RunTask(VectorXd q_fdb) = 0;
	BaseTask();
	~BaseTask();
};


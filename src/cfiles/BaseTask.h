#pragma once

#include "CleanRobot.h"

class BaseTask
{
public:
	bool _task_completed;
protected:
	CleanRobot* _rbt;

public:
	virtual int RunLogicOperation(int state, int pre_state, char* operation) = 0;

	virtual void SetTrajPos(MatrixXd traj_pos, VectorXd q_fdb) = 0;

	virtual VectorXd RunTask(VectorXd q_fdb) = 0;

	void RestTask() { _task_completed = false; }

	BaseTask() {}

	BaseTask(CleanRobot* rbt) { _rbt = rbt; }

	~BaseTask() {}
};


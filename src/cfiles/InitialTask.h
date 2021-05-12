#pragma once
#include "BaseTask.h"

class InitialTask : public BaseTask
{
public:
	InitialTask() {}

	InitialTask(CleanRobot* rbt):BaseTask(rbt) {}

	int RunLogicOperation(int state, int pre_state, char* operation) override
	{
		if (_task_completed)
			return 1;
		else
			return 0;
	}

	VectorXd RunTask(VectorXd q_fdb) override
	{
		_rbt->UpdateJntHoldPos(q_fdb);
		_task_completed = true;
		return q_fdb;
	}

	void SetTrajPos(MatrixXd traj_pos, int num_section, VectorXd q_fdb) override {}

	~InitialTask() {}
};


#pragma once
#include "BaseTask.h"

class HoldTask : public BaseTask
{
public:
	HoldTask() {}

	HoldTask(CleanRobot* rbt):BaseTask(rbt) {}

	int RunLogicOperation(int state, int pre_state, char* operation) override
	{
		if (strcmp(operation, "mirror")==0)
		{
			_task_completed = true;
			return 2;
		}
		else if (strcmp(operation, "washbasin")==0)
		{
			_task_completed = true;
			return 3;
		}
		else
			return 1;
	}

	void SetTrajPos(MatrixXd traj_pos, int num_section, VectorXd q_fdb) override {}

	VectorXd RunTask(VectorXd q_fdb) override
	{
		return _rbt->HoldJntPos();
	}

	~HoldTask() {}
};


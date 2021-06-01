#pragma once

#include "CleanRobot.h"

class BaseTask
{
public:
	bool _task_completed;
	bool _out_boundary;
protected:
	CleanRobot* _rbt;

public:
	BaseTask() {}

	BaseTask(CleanRobot* rbt) 
	{ 
		_rbt = rbt;
		_task_completed = false;
		_out_boundary = false;
	}

	void CheckWorkspace(Vector3d cmd_pos, VectorXd q_fdb)
	{
		bool jlimit = false, climit = false;
		for (int idx = 0; idx<_rbt->_nlinks; idx++)
		{
			if ((fabs(q_fdb(idx)-_rbt->_qlimit(idx, 0))<EPS)||
				(fabs(q_fdb(idx)-_rbt->_qlimit(idx, 1))<EPS))
			{
				jlimit = true;
				break;
			}
		}
		Pose pose = _rbt->FKSolve(q_fdb);
		if (fabs(MathTools::Norm(pose.pos-cmd_pos))>0.1)
			climit = true;
		
		_out_boundary = jlimit&&climit;
	}

	virtual int RunLogicOperation(int state, int pre_state, char* operation) = 0;

	virtual void SetTrajPos(MatrixXd* traj_pos,int num_section, VectorXd q_fdb) = 0;

	virtual VectorXd RunTask(VectorXd q_fdb) = 0;

	void RestTask() { _task_completed = false; }

	~BaseTask() {}
};


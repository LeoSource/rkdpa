#pragma once

#include "BaseTask.h"
#include "CubicBSplinePlanner.h"
#include "LspbTrajPlanner.h"
#include "GlobalParams.h"

using namespace std;
using namespace Eigen;

class SurfaceTask : public BaseTask
{
protected:
	LspbTrajPlanner _jplanner, _pre_uplanner, _pre_alphplanner;
	LspbTrajPlanner _uplanner, _alphplanner, _post_uplanner;
	CubicBSplinePlanner _cpath;

	double _tf_clean, _tf_pre, _tf_post;
	double _t;
	int _task_state;
	Vector5d _q_preclean, _q_postclean;
	Vector3d _pos_initial, _pos_postclean, _pos_stow, _dir_initial, _dir_stow;
	double _alph_initial, _alph_end;
	bool _j3_return;
	Vector5d _jpos_initial;

public:
	SurfaceTask() {}

	SurfaceTask(CleanRobot* rbt):BaseTask(rbt) {}

	int RunLogicOperation(int state, int pre_state, char* operation) override;

	void SetTrajPos(MatrixXd* traj_pos, int num_section, VectorXd q_fdb) override;

	VectorXd RunTask(VectorXd q_fdb) override;

	~SurfaceTask() {}

private:
	VectorXd RunPreCleanAction(VectorXd q_fdb);

	VectorXd RunCleanAction(VectorXd q_fdb);

	VectorXd RunPostCleanAction(VectorXd q_fdb);

	void CompletePreCleanAction(VectorXd q_fdb);

	void CompleteCleanAction(VectorXd q_fdb);

	void CompletePostCleanAction(VectorXd q_fdb);
};


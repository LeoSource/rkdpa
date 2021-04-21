#pragma once

#include "BaseTask.h"
#include "ArcTransPathPlanner.h"
#include "LspbTrajPlanner.h"
#include "GlobalParams.h"

using namespace std;
using namespace Eigen;


class MirrorTask :public BaseTask
{
protected:
	LspbTrajPlanner _jplanner, _pre_jplanner[5], _post_jplanner[5];
	ArcTransPathPlanner _cpath;
	double _tf_clean, _tf_pre, _tf_post;
	double _radius;
	double _t;
	int _task_state;
	VectorXd _q_preclean, _q_postclean;

public:
	MirrorTask() {}

	MirrorTask(double tf, double radius);

	void InitTask(CleanRobot* rbt, Vector3d pos1, Vector3d pos2, Vector3d pos3, Vector3d pos4, VectorXd q_fdb);

	void SetTrajPos(MatrixXd traj_pos, VectorXd q_fdb) override;

	VectorXd RunTask(VectorXd q_fdb) override;

	~MirrorTask() {}

private:
	VectorXd RunPreCleanAction();

	VectorXd RunCleanAction();

	VectorXd RunPostCleanAction();

	void CompletePreCleanAction(VectorXd q_fdb);

	void CompleteCleanAction(VectorXd q_fdb);

	void CompletePostCleanAction(VectorXd q_fdb);
};


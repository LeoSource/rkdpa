#pragma once

#include "BaseTask.h"
#include "ArcTransPathPlanner.h"
#include "LspbTrajPlanner.h"
#include "GlobalParams.h"

using namespace std;
using namespace Eigen;


class MirrorTask :public BaseTask
{
public:
	char* _mirror_style;
protected:
	LspbTrajPlanner _splanner, _pre_uplanner, _post_uplanner, _alphplanner;
	ArcTransPathPlanner _cpath;
	double _tf_clean, _tf_pre, _tf_post;
	double _radius;
	double _t;
	int _task_state;
	Vector5d _q_preclean, _q_postclean;
	Vector3d _pos_initial, _pos_postclean, _pos_stow, _dir_initial, _dir_stow;
	double _alph_initial;
	int _interval_idx;
	double _varc;
	double _interval;
	Vector3d _center;
	bool _j3_return;
	Vector5d _jpos_initial;
	LspbTrajPlanner _jplanner;
	Matrix3d _circel_rot;

public:
	MirrorTask() {}

	MirrorTask(CleanRobot* rbt):BaseTask(rbt) {}

	MirrorTask(UrRobot* urrbt) :BaseTask(urrbt) {}

	MirrorTask(CleanRobot* rbt, char* mirror_style);

	int RunLogicOperation(int state, int pre_state, char* operation) override;

	void SetTrajPos(MatrixXd* traj_pos, int num_section, VectorXd q_fdb) override;

	VectorXd RunTask(VectorXd q_fdb) override;

	~MirrorTask() {}

private:
	void SetRectMirror(MatrixXd* corner_pos, VectorXd q_fdb);

	void SetCircleMirror(Vector3d center,Vector3d norm_vec, double radius, double interval, VectorXd q_fdb);

	VectorXd RunPreCleanAction(VectorXd q_fdb);

	VectorXd RunRectCleanAction(VectorXd q_fdb);

	VectorXd RunCircleCleanAction(VectorXd q_fdb);

	VectorXd RunPostCleanAction(VectorXd q_fdb);

	void CompletePreCleanAction(VectorXd q_fdb);

	void CompleteCleanAction(VectorXd q_fdb);

	void CompletePostCleanAction(VectorXd q_fdb);

};


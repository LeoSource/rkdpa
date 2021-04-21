#include "stdafx.h"
#include "MirrorTask.h"


MirrorTask::MirrorTask(double tf, double radius)
{
	_tf_clean = tf;
	_radius = radius;
}

void MirrorTask::InitTask(CleanRobot* rbt, Vector3d pos1, Vector3d pos2, Vector3d pos3, Vector3d pos4, VectorXd q_fdb)
{
	_rbt = rbt;
	_q_preclean = _rbt->IKSolve(pos1, "q2first", 0);
	_q_postclean = _rbt->IKSolve(pos3, "q2first", 0);
	_tf_pre = 10;
	_tf_post = 10;
	for (int idx = 0; idx<5; idx++)
	{
		Vector2d via_points(q_fdb(idx), _q_preclean(idx));
		_pre_jplanner[idx].InitPlanner(via_points, 10, 1, 2, "limitvel");
		via_points<<_q_postclean(idx), g_stowed_pos[idx];
		_post_jplanner[idx].InitPlanner(via_points, 10, 1, 2, "limitvel");
	}

    Matrix<double,3,4> corner_pos;
    corner_pos<<pos1, pos2, pos3, pos4;
	MatrixXd via_pos = RobotTools::CalcRectanglePath(corner_pos, 15, "s");
	_cpath.InitPlanner(via_pos, _radius);
	Vector2d via_path(0, _cpath._distance);
	_jplanner.InitPlanner(via_path, _tf_clean, 0.5, 2, "limitvel");

	_t = 0;
	_task_state = 0;
	_task_completed = false;
}

void MirrorTask::SetTrajPos(MatrixXd traj_pos, VectorXd q_fdb)
{
	_q_preclean = _rbt->IKSolve(traj_pos.col(0), "q2first", 0);
	_q_postclean = _rbt->IKSolve(traj_pos.col(2), "q2first", 0);
	_tf_pre = 10;
	_tf_post = 10;
	for (int idx = 0; idx<5; idx++)
	{
		Vector2d via_points(q_fdb(idx), _q_preclean(idx));
		_pre_jplanner[idx].InitPlanner(via_points, 10, 1, 2, "limitvel");
		via_points<<_q_postclean(idx), g_stowed_pos[idx];
		_post_jplanner[idx].InitPlanner(via_points, 10, 1, 2, "limitvel");
	}

	MatrixXd via_pos = RobotTools::CalcRectanglePath(traj_pos, 15, "s");
	_cpath.InitPlanner(via_pos, _radius);
	Vector2d via_path(0, _cpath._distance);
	_jplanner.InitPlanner(via_path, _tf_clean, 0.5, 2, "limitvel");

	_t = 0;
	_task_state = 0;
	_task_completed = false;
}
 
VectorXd MirrorTask::RunTask(VectorXd q_fdb)
{
	Matrix<double, 5, 1> q_traj;
	switch (_task_state)
	{
	case 0:
		q_traj = RunPreCleanAction();
		CompletePreCleanAction(q_fdb);
		break;
	case 1:
		q_traj = RunCleanAction();
		CompleteCleanAction(q_fdb);
		break;
	case 2:
		q_traj = RunPostCleanAction();
		CompletePostCleanAction(q_fdb);
		break;
	case 3:
		_task_completed = true;
		break;
	}

	return q_traj;
}

VectorXd MirrorTask::RunPreCleanAction()
{
	Matrix<double, 5, 1> q_traj;
	for (int idx = 0; idx<5; idx++)
	{
		JAVP javp = _pre_jplanner[idx].GenerateMotion(_t);
		q_traj(idx) = javp.pos;
	}
	_t += g_cycle_time;
	MathTools::LimitMax(_tf_pre, _t);

	return q_traj;
}

VectorXd MirrorTask::RunCleanAction()
{
	Matrix<double, 5, 1> q_traj;
	JAVP javp = _jplanner.GenerateMotion(_t);
	CLineAVP avp = _cpath.GenerateMotion(javp.pos, javp.vel, javp.acc);
	q_traj = _rbt->IKSolve(avp.pos, "q2first", 0);
	_t += g_cycle_time;
	MathTools::LimitMax(_tf_clean, _t);

	return q_traj;
}

VectorXd MirrorTask::RunPostCleanAction()
{
	Matrix<double, 5, 1> q_traj;
	for (int idx = 0; idx<5; idx++)
	{
		JAVP javp = _post_jplanner[idx].GenerateMotion(_t);
		q_traj(idx) = javp.pos;
	}
	_t += g_cycle_time;
	MathTools::LimitMax(_tf_post, _t);

	return q_traj;
}

void MirrorTask::CompletePreCleanAction(VectorXd q_fdb)
{
	bool res = true;
	for (int idx = 0; idx<5; idx++)
	{
		res = res & fabs(_q_preclean(idx)-q_fdb(idx))<g_return_err[idx];
	}
	if (res)
	{
		_t = 0;
		_task_state = 1;
	}
}

void MirrorTask::CompleteCleanAction(VectorXd q_fdb)
{
	bool res = true;
	for (int idx = 0; idx<5; idx++)
	{
		res = res & fabs(_q_postclean(idx)-q_fdb(idx))<g_return_err[idx];
	}
	if (res)
	{
		_t = 0;
		_task_state = 2;
	}
}


void MirrorTask::CompletePostCleanAction(VectorXd q_fdb)
{
	bool res = true;
	for (int idx = 0; idx<5; idx++)
	{
		res = res & fabs(g_stowed_pos[idx]-q_fdb(idx))<g_return_err[idx];
	}
	if (res)
	{
		_t = 0;
		_task_state = 3;
	}
}
#include "stdafx.h"
#include "SurfaceTask.h"
#include <iostream>

void SurfaceTask::SetTrajPos(MatrixXd traj_pos, int num_section, VectorXd q_fdb)
{
	if (_task_completed)
	{
		_task_state = 0;
		_t = 0;
	}
	_task_completed = false;
	if (_task_state==0)
	{
		_j3_return = false;
		_jpos_initial = q_fdb;
		Vector2d vjpos(q_fdb(2), -pi/6);
		_jplanner.InitPlanner(vjpos, g_jvmax[2], g_jamax[2]);
		_q_preclean = _rbt->IKSolve(traj_pos.col(0), "q3firstn", 0);
		Vector5d q0 = q_fdb;
		q0(2) = -pi/6;
		_pos_initial = _rbt->FKSolve(q0).pos;
		double line_len = MathTools::Norm(traj_pos.col(0)-_pos_initial);
		_dir_initial = (traj_pos.col(0)-_pos_initial)/line_len;
		Vector2d vp(0, line_len);
		_pre_uplanner.InitPlanner(vp, g_cvmax, g_camax);
		_alph_initial = q_fdb(0)+q_fdb(q_fdb.size()-1);
		_alph_end = 2*pi*num_section;
		Vector2d alph(_alph_initial, 0);
		_pre_alphplanner.InitPlanner(alph, g_jvmax[0], g_jamax[0]);
		Vector2d tf(_pre_alphplanner.GetFinalTime(), _pre_uplanner.GetFinalTime());
		_tf_pre = tf.maxCoeff();
		_pre_uplanner.InitPlanner(vp, g_cvmax, g_camax, _tf_pre);
		_pre_alphplanner.InitPlanner(alph, g_jvmax[0], g_jamax[0], _tf_pre);

		_pos_postclean = traj_pos.col(traj_pos.cols()-1);
		_pos_stow = Vector3d(g_stowed_cpos);
		_q_postclean = _rbt->IKSolve(_pos_postclean, "q3firstn", _alph_end);
		line_len = MathTools::Norm(_pos_stow-_pos_postclean);
		_dir_stow = (_pos_stow-_pos_postclean)/line_len;
		vp<<0, line_len;
		_post_uplanner.InitPlanner(vp, g_cvmax, g_camax);
		_tf_post = _post_uplanner.GetFinalTime();

		_t = 0;
	}

	_tf_clean = 60;
	_cpath = CubicBSplinePlanner(traj_pos, "approximation", _tf_clean);
	Vector2d alph(0, _alph_end);
	_alphplanner.InitPlanner(alph, 1, 2, _tf_clean);
	Vector2d vp(_cpath._uknot_vec(0), _cpath._uknot_vec(_cpath._uknot_vec.size()-1));
	_uplanner.InitPlanner(vp, 2, 1, _cpath._uknot_vec(_cpath._uknot_vec.size()-1));

}

VectorXd SurfaceTask::RunTask(VectorXd q_fdb)
{
	Vector5d q_traj;
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
		q_traj = _rbt->HoldJntPos();
		break;
	}
	_rbt->UpdateJntHoldPos(q_traj);

	return q_traj;
}

VectorXd SurfaceTask::RunPreCleanAction()
{
	Vector5d q_traj;
	if (!_j3_return)
	{
		JAVP j3avp = _jplanner.GenerateMotion(_t);
		q_traj = _jpos_initial;
		q_traj(2) = j3avp.pos;
		_t += g_cycle_time;
		MathTools::LimitMax(_jplanner.GetFinalTime(), _t);
		if (fabs(_t-_jplanner.GetFinalTime())<EPS)
		{
			_t = 0;
			_j3_return = true;
		}
	}
	else
	{
		JAVP uavp = _pre_uplanner.GenerateMotion(_t);
		Vector3d cpos = _pos_initial+uavp.pos*_dir_initial;
		JAVP alphavp = _pre_alphplanner.GenerateMotion(_t);
		q_traj = _rbt->IKSolve(cpos, "q3firstn", alphavp.pos);
		CheckWorkspace(cpos, q_traj);
		_t += g_cycle_time;
		MathTools::LimitMax(_tf_pre, _t);
	}

	return q_traj;
}

VectorXd SurfaceTask::RunCleanAction()
{
	Vector5d q_traj;
	JAVP uavp = _uplanner.GenerateMotion(_t);
	CLineAVP cavp = _cpath.GenerateMotion(uavp.pos, uavp.vel, uavp.acc);
	JAVP alphavp = _alphplanner.GenerateMotion(_t);
	q_traj = _rbt->IKSolve(cavp.pos, "q3firstn", alphavp.pos);
	CheckWorkspace(cavp.pos, q_traj);
	_t += g_cycle_time;
	MathTools::LimitMax(_tf_clean, _t);

	return q_traj;
}

VectorXd SurfaceTask::RunPostCleanAction()
{
	Vector5d q_traj;
	JAVP uavp = _post_uplanner.GenerateMotion(_t);
	Vector3d cpos = _pos_postclean+uavp.pos*_dir_stow;
	q_traj = _rbt->IKSolve(cpos, "q3firstn", _alph_end);
	CheckWorkspace(cpos, q_traj);
	_t += g_cycle_time;
	MathTools::LimitMax(_tf_post, _t);

	return q_traj;
}

void SurfaceTask::CompletePreCleanAction(VectorXd q_fdb)
{
	bool res = true;
	for (int idx = 0; idx<5; idx++)
	{
		res = res & (fabs(_q_preclean(idx)-q_fdb(idx))<g_return_err[idx]);
	}
	res = res & (fabs(_t-_tf_pre)<EPS);
	if (res)
	{
		_t = 0;
		_task_state = 1;
	}
}

void SurfaceTask::CompleteCleanAction(VectorXd q_fdb)
{
	bool res = true;
	for (int idx = 0; idx<5; idx++)
	{
		res = res & (fabs(_q_postclean(idx)-q_fdb(idx))<g_return_err[idx]);
	}
	res = res & (fabs(_t-_tf_clean)<EPS);
	if (res)
	{
		_t = 0;
		_task_state = 2;
	}
}

void SurfaceTask::CompletePostCleanAction(VectorXd q_fdb)
{
	bool res = true;
	Vector5d jpos_stow = _rbt->IKSolve(_pos_stow, "q3firstn", _alph_end);
	for (int idx = 0; idx<5; idx++)
	{
		res = res & (fabs(jpos_stow(idx)-q_fdb(idx))<g_return_err[idx]);
	}
	res = res & (fabs(_t-_tf_post)<EPS);
	if (res)
	{
		_t = 0;
		_task_state = 3;
	}
}

int SurfaceTask::RunLogicOperation(int state, int pre_state, char* operation)
{
	int res = 3;
	if (_task_completed)
	{
		res = 1;
		cout<<"succeed to complete surface task"<<endl;
	}
	else
	{
		if (strcmp(operation, "pause")==0)
		{
			res = 1;
		}
		else if (strcmp(operation, "stop")==0)
		{
			res = 1;
			_t = 0;
			_task_state = 0;
		}
	}
	return res;
}
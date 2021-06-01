#include "ShowerRoomTask.h"
#include <iostream>

void ShowerRoomTask::SetTrajPos(MatrixXd* pos_info, int num_section, VectorXd q_fdb)
{
	if (_task_completed)
	{
		_task_state = 0;
		_t = 0;
		_interval_idx = 0;
	}
	_task_completed = false;
	_interval = 0.08;
	SetRectTable(pos_info, q_fdb);

}

void ShowerRoomTask::SetRectTable(MatrixXd* corner_pos, VectorXd q_fdb)
{
	MatrixXd via_pos = RobotTools::CalcRectanglePath(corner_pos, "s",_interval);
	if (_task_state == 0)
	{
		_j3_return = false;
		_jpos_initial = q_fdb;
		Vector2d vjpos(q_fdb(2), 0);
		_jplanner.InitPlanner(vjpos, g_jvmax[2], g_jamax[2]);
		Vector5d q0 = q_fdb;
		q0(2) = 0;

		_q_preclean = _rbt->IKSolve(via_pos.col(0), "q2first", 0, q_fdb);
		_pos_initial = _rbt->FKSolveTool(q0).pos;
		double line_len = MathTools::Norm(via_pos.col(0) - _pos_initial);
		_dir_initial = (via_pos.col(0) - _pos_initial) / line_len;
		Vector2d vp(0, line_len);
		_pre_uplanner.InitPlanner(vp, g_cvmax, g_camax);
		_alph_initial = q0(0) + q0(q0.size() - 1);
		Vector2d valph(_alph_initial, 0);
		_alphplanner.InitPlanner(valph, g_jvmax[0], g_jamax[0]);
		Vector2d tf(_pre_uplanner.GetFinalTime(), _alphplanner.GetFinalTime());
		_tf_pre = tf.maxCoeff();
		_pre_uplanner.InitPlanner(vp, g_cvmax, g_camax, _tf_pre);
		_alphplanner.InitPlanner(valph, g_jvmax[0], g_jamax[0], _tf_pre);

		Vector5d stowed_jpos(g_stowed_jpos);
		_pos_stow = _rbt->FKSolveTool(stowed_jpos).pos;
		_pos_postclean = via_pos.col(via_pos.cols() - 1);
		_q_postclean = _rbt->IKSolve(_pos_postclean, "q2first", 0, q_fdb);
		line_len = MathTools::Norm(_pos_stow - _pos_postclean);
		_dir_stow = (_pos_stow - _pos_postclean) / line_len;
		vp << 0, line_len;
		_post_uplanner.InitPlanner(vp, g_cvmax, g_camax);
		_tf_post = _post_uplanner.GetFinalTime();
		_t = 0;
	}

	_cpath.InitPlanner(via_pos, 0);
	_radius = _cpath._radius;
	_varc = sqrt(g_camax*_cpath._radius);
}

VectorXd ShowerRoomTask::RunTask(VectorXd q_fdb)
{
	Matrix<double, 5, 1> q_traj;
	switch (_task_state)
	{
	case 0:
		q_traj = RunPreCleanAction(q_fdb);
		CompletePreCleanAction(q_fdb);
		break;
	case 1:
		q_traj = RunRectCleanAction(q_fdb);
		CompleteCleanAction(q_fdb);
		break;
	case 2:
		q_traj = RunPostCleanAction(q_fdb);
		CompletePostCleanAction(q_fdb);
		break;
	case 3:
		_task_completed = true;
		_task_state = 0;
		q_traj = _rbt->HoldJntPos();
		break;
	}
	_rbt->UpdateJntHoldPos(q_traj);

	return q_traj;
}

VectorXd ShowerRoomTask::RunPreCleanAction(VectorXd q_fdb)
{
	Vector5d q_traj;
	if (!_j3_return)
	{
		JAVP j3avp = _jplanner.GenerateMotion(_t);
		q_traj = _jpos_initial;
		q_traj(2) = j3avp.pos;
		_t += g_cycle_time;
		MathTools::LimitMax(_jplanner.GetFinalTime(), _t);
		if (fabs(_t - _jplanner.GetFinalTime())<EPS)
		{
			_t = 0;
			_j3_return = true;
		}
	}
	else
	{
		JAVP uavp = _pre_uplanner.GenerateMotion(_t);
		Vector3d cpos = _pos_initial + uavp.pos*_dir_initial;
		JAVP alphavp = _alphplanner.GenerateMotion(_t);
		q_traj = _rbt->IKSolve(cpos, "q2first", alphavp.pos, q_fdb);
		CheckWorkspace(cpos, q_traj);
		_t += g_cycle_time;
		MathTools::LimitMax(_tf_pre, _t);
	}
	return q_traj;
}

VectorXd ShowerRoomTask::RunRectCleanAction(VectorXd q_fdb)
{
	Vector2d vel_con;
	JAVP savp;
	if (_interval_idx % 2 == 0)
	{
		// line segment
		if (_interval_idx == 0)
			vel_con << 0, _varc;
		else if (_interval_idx == _cpath._dis_interval.size() - 2)
			vel_con << _varc, 0;
		else
			vel_con << _varc, _varc;
		Vector2d q(_cpath._dis_interval(_interval_idx), _cpath._dis_interval(_interval_idx + 1));
		Vector2d duration(0, 0);
		_splanner.InitPlanner(q, g_cvmax, g_camax, duration, vel_con);

		savp = _splanner.GenerateMotion(_t);
		if (fabs(_t - _splanner.GetFinalTime())<g_cycle_time)
		{
			_interval_idx += 1;
			_t = -g_cycle_time;
		}
		MathTools::LimitMax(_splanner.GetFinalTime(), _t);
	}
	else
	{
		//semicircle segment
		double tlen = (_cpath._dis_interval(_interval_idx + 1) - _cpath._dis_interval(_interval_idx)) / _varc;
		savp.acc = 0;
		savp.vel = _varc;
		savp.pos = _cpath._dis_interval(_interval_idx) + _varc*_t;
		if (fabs(_t - tlen)<g_cycle_time)
		{
			_interval_idx += 1;
			_t = -g_cycle_time;
		}
		MathTools::LimitMax(tlen, _t);
	}
	CLineAVP avp = _cpath.GenerateMotion(savp.pos, savp.vel, savp.acc);
	//UpdateArcTransParams(savp.pos, savp.vel, savp.acc);

	Matrix<double, 5, 1> q_traj;
	q_traj = _rbt->IKSolve(avp.pos, "q2first", 0, q_fdb);
	CheckWorkspace(avp.pos, q_traj);
	_t += g_cycle_time;

	return q_traj;
}

VectorXd ShowerRoomTask::RunPostCleanAction(VectorXd q_fdb)
{
	Matrix<double, 5, 1> q_traj;
	JAVP uavp = _post_uplanner.GenerateMotion(_t);
	Vector3d cpos = _pos_postclean + uavp.pos*_dir_stow;
	q_traj = _rbt->IKSolve(cpos, "q2first", 0, q_fdb);
	CheckWorkspace(cpos, q_traj);
	_t += g_cycle_time;
	MathTools::LimitMax(_tf_post, _t);

	return q_traj;
}

void ShowerRoomTask::CompletePreCleanAction(VectorXd q_fdb)
{
	bool res = true;
	for (int idx = 0; idx<5; idx++)
	{
		res = res & (fabs(_q_preclean(idx) - q_fdb(idx))<g_return_err[idx]);
	}
	res = res & (fabs(_t - _tf_pre)<EPS);
	if (res)
	{
		_t = 0;
		_task_state = 1;
	}
}

void ShowerRoomTask::CompleteCleanAction(VectorXd q_fdb)
{
	bool res = true;
	for (int idx = 0; idx<5; idx++)
	{
		res = res & (fabs(_q_postclean(idx) - q_fdb(idx))<g_return_err[idx]);
	}
		res = res && (_interval_idx == _cpath._dis_interval.size() - 1);
	if (res)
	{
		_t = 0;
		_task_state = 2;
	}
}


void ShowerRoomTask::CompletePostCleanAction(VectorXd q_fdb)
{
	bool res = true;
	for (int idx = 0; idx<5; idx++)
	{
		res = res & (fabs(g_stowed_jpos[idx] - q_fdb(idx))<g_return_err[idx]);
	}
	res = res & (fabs(_t - _tf_post)<EPS);
	if (res)
	{
		_t = 0;
		_task_state = 3;
	}
}


int ShowerRoomTask::RunLogicOperation(int state, int pre_state, char* operation)
{
	int res = 5;
	if (_task_completed)
	{
		res = 1;
		cout << "succeed to complete showerroom task" << endl;
	}
	else
	{
		if (strcmp(operation, "pause") == 0)
		{
			res = 1;
		}
		else if (strcmp(operation, "stop") == 0)
		{
			res = 1;
			_t = 0;
			_task_state = 0;
			_interval_idx = 0;
		}
	}
	return res;
}


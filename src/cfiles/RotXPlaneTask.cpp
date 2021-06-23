#include "RotXPlaneTask.h"
#include <iostream>


RotXPlaneTask::RotXPlaneTask(CleanRobot* rbt, char* clean_type):BaseTask(rbt)
{
	_clean_type = clean_type;
	_t = 0;
}


void RotXPlaneTask::SetTrajPos(MatrixXd* pos_info, int num_section, VectorXd q_fdb)
{
	if (_task_completed)
	{
		_task_state = 0;
		_t = 0;
	}
	_task_completed = false;

	if (_task_state==0)
	{
		/*
		double pitch_x = (*pos_info)(0, 2);
		double inc_angle[] = { 20*pi/180, 50*pi/180 };
		double pitch_high = pitch_x-inc_angle[0];
		double pitch_low = pitch_x-inc_angle[1];
		_rbt->SetPitchRange(pitch_high, pitch_low);
		//pre-clean:joint plan for joint1 and joint5
		_pitch_return = false;
		_jpos_initial = q_fdb;
		Vector2d vj1pos(q_fdb(0), 0), vj5pos(q_fdb(4), 0);
		_pre_j1planner.InitPlanner(vj1pos, g_jvmax[0], g_jamax[0]);
		_pre_j5planner.InitPlanner(vj5pos, g_jvmax[4], g_jamax[4]);
		Vector2d tf(_pre_j1planner.GetFinalTime(), _pre_j5planner.GetFinalTime());
		_tf1_pre = tf.maxCoeff();
		_pre_j1planner.InitPlanner(vj1pos, g_jvmax[0], g_jamax[0], _tf1_pre);
		_pre_j5planner.InitPlanner(vj5pos, g_jvmax[4], g_jamax[4], _tf1_pre);
		//pre-clean:cartesian plan for pitch and line
		Vector5d jpos = q_fdb;
		jpos(0) = jpos(4) = 0;
		_pos_initial = _rbt->FKSolveTool(jpos).pos;
		double line_len = MathTools::Norm(pos_info->col(0)-_pos_initial);
		_dir_initial = (pos_info->col(0)-_pos_initial)/line_len;
		Vector2d vpos(0, line_len);
		_pre_uplanner.InitPlanner(vpos, g_cvmax, g_camax);
		double pitch0 = q_fdb(2)+_rbt->_tool_pitch;
		Vector2d vpitch(pitch0, _rbt->_pitch_high);
		_pre_pitchplanner.InitPlanner(vpitch, g_jvmax[2], g_jamax[2]);
		tf<<_pre_uplanner.GetFinalTime(), _pre_pitchplanner.GetFinalTime();
		_tf2_pre = tf.maxCoeff();
		_pre_uplanner.InitPlanner(vpos, g_cvmax, g_camax,_tf2_pre);
		_pre_pitchplanner.InitPlanner(vpitch, g_jvmax[2], g_jamax[2],_tf2_pre);
		_q_preclean = _rbt->IKSolvePitch(pos_info->col(0), _rbt->_pitch_high);
		//clean plane action: cartesian line plan
		_pos_preclean = pos_info->col(0);
		vpitch<<_rbt->_pitch_high, _rbt->_pitch_low;
		_pitchplanner.InitPlanner(vpitch, g_jvmax[2], g_jamax[2]);
		line_len = MathTools::Norm(pos_info->col(1)-pos_info->col(0));
		vpos<<0, line_len;
		_uplanner.InitPlanner(vpos, g_cvmax, g_camax);
		tf<<_pitchplanner.GetFinalTime(), _uplanner.GetFinalTime();
		_tf_clean = tf.maxCoeff();
		_pitchplanner.InitPlanner(vpitch, g_jvmax[2], g_jamax[2],_tf_clean);
		_uplanner.InitPlanner(vpos, g_cvmax, g_camax,_tf_clean);
		_dir_clean = (pos_info->col(1)-pos_info->col(0))/line_len;
		_q_postclean = _rbt->IKSolvePitch(pos_info->col(1), _rbt->_pitch_low);
		//post-clean: cartesian line plan
		_pos_postclean = pos_info->col(1);
		Vector3d tmp_pos(0, -0.1, 0);
		_pos_stow = pos_info->col(1)+tmp_pos;
		_q_stow = _rbt->IKSolvePitch(_pos_stow, _rbt->_pitch_low);
		line_len = MathTools::Norm(_pos_stow-pos_info->col(1));
		vpos<<0, line_len;
		_post_uplanner.InitPlanner(vpos, g_cvmax, g_camax);
		_dir_stow = (_pos_stow-pos_info->col(1))/line_len;
		_tf_post = _post_uplanner.GetFinalTime();
		*/
		double pitch_x = (*pos_info)(0, 2);
		double inc_angle[] = { 20*pi/180, 50*pi/180 };
		double pitch_high = pitch_x-inc_angle[0];
		double pitch_low = pitch_x-inc_angle[1];
		_rbt->SetPitchRange(pitch_high, pitch_low);

		Vector3d pos0 = _rbt->FKSolveTool(q_fdb).pos;
		double pitch0 = q_fdb(2)+_rbt->_tool_pitch;
		double yaw0 = q_fdb(0)+q_fdb(4);
		Vector3d rpy0(0, pitch0, yaw0);
		Vector6d pos_rpy0, pos_rpyn;
		pos_rpy0<<pos0, rpy0;
		pos_rpyn<<pos_info->col(0), Vector3d(0, _rbt->_pitch_high, 0);
		double vmax[] = { g_cvmax,0.15 };
		double amax[] = { g_camax,0.3 };
		double vel_cons[] = { 0,0,0,0 };
		_line_traj[0] = LineTrajPlanner(pos_rpy0, pos_rpyn, vmax, amax, vel_cons, "both");


		pos_rpy0<<pos_info->col(0), pos_rpyn.tail(3);
		pos_rpyn<<pos_info->col(1), Vector3d(0, _rbt->_pitch_low, 0);
		_line_traj[1] = LineTrajPlanner(pos_rpy0, pos_rpyn, vmax, amax, vel_cons, "both");

		pos_rpy0 = pos_rpyn;
		Vector3d pos_tmp(0, -0.1, 0);
		Vector3d posn = pos_rpyn.head(3)+pos_tmp;
		pos_rpyn<<posn, pos_rpy0.tail(3);
		_line_traj[2] = LineTrajPlanner(pos_rpy0, pos_rpyn, vmax[0], amax[0], vel_cons, "pos");

		_t = 0;
	}
}

VectorXd RotXPlaneTask::RunTask(VectorXd q_fdb)
{
	Matrix<double, 5, 1> q_traj;
	switch (_task_state)
	{
	case 0:
		q_traj = RunPreCleanAction(q_fdb);
		CompletePreCleanAction(q_fdb);
		break;
	case 1:
		q_traj = RunScrapeAction(q_fdb);
		CompleteCleanAction(q_fdb);
		break;
	case 2:
		q_traj = RunPostCleanAction(q_traj);
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

VectorXd RotXPlaneTask::RunPreCleanAction(VectorXd q_fdb)
{
	Vector5d q_traj;
	Vector6d pos_rpy = _line_traj[0].GeneratePoint(_t);
	q_traj = _rbt->IKSolvePitchYaw(pos_rpy.head(3), pos_rpy(4), pos_rpy(5), q_fdb);
	_t += g_cycle_time;
	MathTools::LimitMax(_line_traj[0].GetFinalTime(), _t);
	/*
	if (!_pitch_return)
	{
		JAVP j1avp = _pre_j1planner.GenerateMotion(_t);
		JAVP j5avp = _pre_j5planner.GenerateMotion(_t);
		q_traj = _jpos_initial;
		q_traj(0) = j1avp.pos;
		q_traj(4) = j5avp.pos;
		_t += g_cycle_time;
		MathTools::LimitMax(_tf1_pre, _t);
		if (fabs(_t-_pre_j1planner.GetFinalTime())<g_cycle_time)
		{
			_t = 0;
			_pitch_return = true;
		}
	}
	else
	{
		JAVP uavp = _pre_uplanner.GenerateMotion(_t);
		Vector3d cpos = _pos_initial+uavp.pos*_dir_initial;
		JAVP pitchavp = _pre_pitchplanner.GenerateMotion(_t);
		q_traj = _rbt->IKSolvePitch(cpos, pitchavp.pos);
		CheckWorkspace(cpos, q_traj);
		_t += g_cycle_time;
		MathTools::LimitMax(_tf2_pre, _t);
	}
	*/
	return q_traj;
}

VectorXd RotXPlaneTask::RunScrapeAction(VectorXd q_fdb)
{
	Vector5d q_traj;
	Vector6d pos_rpy = _line_traj[1].GeneratePoint(_t);
	q_traj = _rbt->IKSolvePitchYaw(pos_rpy.head(3), pos_rpy(4), pos_rpy(5), q_fdb);
	_t += g_cycle_time;
	MathTools::LimitMax(_line_traj[1].GetFinalTime(), _t);
	/*
	JAVP uavp = _uplanner.GenerateMotion(_t);
	JAVP pitchavp = _pitchplanner.GenerateMotion(_t);
	Vector3d cpos = _pos_preclean+uavp.pos*_dir_clean;
	q_traj = _rbt->IKSolvePitch(cpos, pitchavp.pos);
	CheckWorkspace(cpos, q_traj);
	_t += g_cycle_time;
	MathTools::LimitMax(_tf_clean, _t);
	*/
	return q_traj;
}

VectorXd RotXPlaneTask::RunPostCleanAction(VectorXd q_fdb)
{
	Vector5d q_traj;
	Vector6d pos_rpy = _line_traj[2].GeneratePoint(_t);
	q_traj = _rbt->IKSolvePitchYaw(pos_rpy.head(3), pos_rpy(4), pos_rpy(5), q_fdb);
	_t += g_cycle_time;
	MathTools::LimitMax(_line_traj[2].GetFinalTime(), _t);
	/*
	JAVP uavp = _post_uplanner.GenerateMotion(_t);
	Vector3d cpos = _pos_postclean+uavp.pos*_dir_stow;
	q_traj = _rbt->IKSolvePitch(cpos, _rbt->_pitch_low);
	CheckWorkspace(cpos, q_traj);
	_t += g_cycle_time;
	MathTools::LimitMax(_tf_post, _t);
	*/

	return q_traj;
}

void RotXPlaneTask::CompletePreCleanAction(VectorXd q_fdb)
{
	bool res = true;
	//for (int idx = 0; idx<5; idx++)
	//{
	//	res = res && (fabs(_q_preclean(idx)-q_fdb(idx))<g_return_err[idx]);
	//}
	res = res && (fabs(_t-_line_traj[0].GetFinalTime())<g_cycle_time);
	if (res)
	{
		_t = 0;
		_task_state = 1;
	}
}

void RotXPlaneTask::CompleteCleanAction(VectorXd q_fdb)
{
	bool res = true;
	//for (int idx = 0; idx<5; idx++)
	//{
	//	res = res & (fabs(_q_postclean(idx)-q_fdb(idx))<g_return_err[idx]);
	//}
	res = res && (fabs(_t-_line_traj[1].GetFinalTime())<g_cycle_time);

	if (res)
	{
		_t = 0;
		_task_state = 2;
	}
}

void RotXPlaneTask::CompletePostCleanAction(VectorXd q_fdb)
{
	bool res = true;
	//for (int idx = 0; idx<5; idx++)
	//{
	//	res = res && (fabs(_q_stow(idx)-q_fdb(idx))<g_return_err[idx]);
	//}
	res = res && (fabs(_t-_line_traj[2].GetFinalTime())<g_cycle_time);
	if (res)
	{
		_t = 0;
		_task_state = 3;
	}
}

int RotXPlaneTask::RunLogicOperation(int state, int pre_state, char* operation)
{
	int res = 2;
	if (_task_completed)
	{
		res = 1;
		cout<<"succeed to complete mirror task"<<endl;
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

MatrixXd RotXPlaneTask::CalTrajViaPos(MatrixXd* pos_info, VectorXd q_fdb)
{
    MatrixXd via_pos;
    via_pos.setZero(3, (1 + 4)*3); //startpos + posnum

    double pitch_x = 90*pi/180;
    double inc_angle[] = { 20*pi/180, 50*pi/180 };
    double pitch_high = pitch_x-inc_angle[0];
    double pitch_low = pitch_x-inc_angle[1];
    _rbt->SetPitchRange(pitch_high, pitch_low);

    Vector3d pos0 = _rbt->FKSolveTool(q_fdb).pos;
    double pitch0 = q_fdb(2)+_rbt->_tool_pitch;
    double yaw0 = q_fdb(0)+q_fdb(4);
    Vector3d rpy0(0, pitch0, yaw0);
	
    via_pos.col(0) = pos0;
    via_pos.col(1) = rpy0; //startpos.rpy
    via_pos.col(2) = Vector3d(1,1,1); //

    via_pos.col(3) = pos_info->col(0);
    via_pos.col(4) = Vector3d(0, _rbt->_pitch_high, 0); //rpy
    via_pos.col(5) = Vector3d(0, 0, 1); //

    via_pos.col(6) = pos_info->col(1);
    via_pos.col(7) = Vector3d(0, _rbt->_pitch_high, 0);  //rpy
    via_pos.col(8) = Vector3d(1, 0, 0); //

    via_pos.col(9) = pos_info->col(2);
    via_pos.col(10) = Vector3d(0, _rbt->_pitch_low, 0);  //rpy
    via_pos.col(11) = Vector3d(0, 0, 1); //

    via_pos.col(12) = pos_info->col(3);
    via_pos.col(13) = Vector3d(0, _rbt->_pitch_low, 0);  //rpy
    via_pos.col(14) = Vector3d(1, 0, 0); //

    for(int i = 0; i < pos_info->cols(); ++i)
    {
        double line1_len = MathTools::Norm(via_pos.col((i+1)*3) - via_pos.col(i*3));
        if(line1_len < 0.2)
        {
            via_pos.col(2) = Vector3d(0, 0 ,0); //path smooth flag
            break;
        }

    }

    return via_pos;
}

#include "TaskTrajPlanner.h"


TaskTrajPlanner::TaskTrajPlanner(UrRobot* robot_model, Vector6d q0)
	:_robot(robot_model),_pre_q(q0),_pre_trajq(q0)
{
	_task_completed = false;
	_ntraj = 0;
	_traj_idx = 0;
	_pre_trajpose = _robot->FKSolveTool(q0);
}

void TaskTrajPlanner::AddTraj(MatrixXd* via_pos, TrajType traj_type, bool traj_opt)
{
	switch (traj_type)
	{
	case RobotTools::eJointSpace:
	{
		BaseTrajPlanner* jplanner = new JointPlanner(_pre_trajq, traj_opt);
		jplanner->AddViaPos(via_pos);
		_segplanner.push_back(jplanner);
		_ntraj += 1;
		_traj_type.push_back(eJointSpace);
		_pre_trajq = via_pos->rightCols(1);
		_pre_trajpose = _robot->FKSolveTool(_pre_trajq);
		break;
	}
	case RobotTools::eCartesianSpace:
	{
		int np = static_cast<int>(via_pos->cols());
		for (int idx = 0; idx < np; idx++)
		{
			Vector6d pos_rpy = via_pos->col(idx);
			CheckCartWorkspace(pos_rpy.head(3));
		}
		Vector3d pos0 = _pre_trajpose.pos;
		Vector3d rpy0 = RobotTools::Tr2FixedZYX(_pre_trajpose.rot);
		Vector6d pos_rpy0;
		pos_rpy0<<pos0, rpy0;
		BaseTrajPlanner* cplanner = new CartesianPlanner(pos_rpy0, traj_opt);
		cplanner->AddViaPos(via_pos);
		_segplanner.push_back(cplanner);
		_ntraj += 1;
		_traj_type.push_back(eCartesianSpace);
		Vector6d pos_rpyn = via_pos->rightCols(1);
		_pre_trajpose.pos = pos_rpyn.head(3);
		_pre_trajpose.rot = RobotTools::FixedZYX2Tr(pos_rpyn.tail(3));
		_robot->IKSolve(&_pre_trajq, &_pre_trajpose, &_pre_trajq);
	}
		break;
	case RobotTools::eCartesianArc:
	{
		//add line trajectory befor arc for transition
		Vector3d pos0 = _pre_trajpose.pos;
		Vector3d rpy0 = RobotTools::Tr2FixedZYX(_pre_trajpose.rot);
		Vector6d pos_rpy0;
		pos_rpy0<<pos0, rpy0;
		BaseTrajPlanner* lineplanner = new CartesianPlanner(pos_rpy0, false);
		MatrixXd pos_trans = via_pos->col(0);
		lineplanner->AddViaPos(&pos_trans);
		_segplanner.push_back(lineplanner);
		_ntraj += 1;
		_traj_type.push_back(eCartesianSpace);
		//add arc trajectory
		Vector6d pos_rpy1 = via_pos->col(0);
		Vector6d pos_rpy3 = via_pos->col(2);
		Vector3d pos2 = via_pos->col(1).head(3);
		BaseTrajPlanner* arcplanner = new CartesianPlanner(&pos_rpy1, &pos2, &pos_rpy3);
		_segplanner.push_back(arcplanner);
		_ntraj += 1;
		_traj_type.push_back(eCartesianArc);
		Vector6d pos_rpyn = via_pos->col(2);
		_pre_trajpose.pos = pos_rpyn.head(3);
		_pre_trajpose.rot = RobotTools::FixedZYX2Tr(pos_rpyn.tail(3));
		_robot->IKSolve(&_pre_trajq, &_pre_trajpose, &_pre_trajq);
	}
		break;
	case RobotTools::eBSpline:
	{
		int np = static_cast<int>(via_pos->cols());
		for (int idx = 0; idx < np; idx++)
		{
			Vector6d pos_rpy = via_pos->col(idx);
			CheckCartWorkspace(pos_rpy.head(3));
		}
		Vector3d pos0 = _pre_trajpose.pos;
		Vector3d rpy0 = RobotTools::Tr2FixedZYX(_pre_trajpose.rot);
		Vector6d pos_rpy0;
		pos_rpy0<<pos0, rpy0;
		MatrixXd viapos_new;
		viapos_new.setZero(6, via_pos->cols()+1);
		viapos_new<<pos_rpy0, *via_pos;
		double tf_uk = CalcBSplineTime(&viapos_new);
		BaseTrajPlanner* bplanner = new CubicBSplinePlanner(&viapos_new, traj_opt, tf_uk);
		_segplanner.push_back(bplanner);
		_ntraj += 1;
		_traj_type.push_back(eBSpline);
		Vector6d pos_rpyn = via_pos->rightCols(1);
		_pre_trajpose.pos = pos_rpyn.head(3);
		_pre_trajpose.rot = RobotTools::FixedZYX2Tr(pos_rpyn.tail(3));
		_robot->IKSolve(&_pre_trajq, &_pre_trajpose, &_pre_trajq);
	}
		break;
	default:
		break;
	}
}


void TaskTrajPlanner::GenerateJPath(Vector6d& jpos)
{
	switch (_traj_type[_traj_idx])
	{
	case eJointSpace:
	{
		_segplanner[_traj_idx]->GeneratePath(jpos);
		_pre_q = jpos;
	}		
	break;
	case eCartesianSpace:
	{
		Vector6d cpos;
		_segplanner[_traj_idx]->GeneratePath(cpos);
		Pose cmd_pose;
		cmd_pose.pos = cpos.head(3);
		cmd_pose.rot = RobotTools::FixedZYX2Tr(cpos.tail(3));
		_robot->IKSolve(&jpos, &cmd_pose, &_pre_q);
		_pre_q = jpos;
	}
	break;
	case eBSpline:
	{
		Vector6d cpos;
		_segplanner[_traj_idx]->GeneratePath(cpos);
		Pose cmd_pose;
		cmd_pose.pos = cpos.head(3);
		cmd_pose.rot = RobotTools::FixedZYX2Tr(cpos.tail(3));
		_robot->IKSolve(&jpos, &cmd_pose, &_pre_q);
		_pre_q = jpos;
	}
	break;
	case eCartesianArc:
	{
		Vector6d cpos;
		_segplanner[_traj_idx]->GeneratePath(cpos);
		Pose cmd_pose;
		cmd_pose.pos = cpos.head(3);
		cmd_pose.rot = RobotTools::FixedZYX2Tr(cpos.tail(3));
		_robot->IKSolve(&jpos, &cmd_pose, &_pre_q);
		_pre_q = jpos;
	}
	break;
	default:
		break;
	}
	if (_segplanner[_traj_idx]->_plan_completed)
	{
		if (_traj_idx==_ntraj-1)
			_task_completed = true;
		else
			_traj_idx += 1;
	}
}

void TaskTrajPlanner::GenerateCPath(Vector6d& cpos)
{
	switch (_traj_type[_traj_idx])
	{
	case eJointSpace:
	{
		Vector6d jpos;
		_segplanner[_traj_idx]->GeneratePath(jpos);
		Pose cpose = _robot->FKSolveTool(jpos);
		cpos.head(3) = cpose.pos;
		cpos.tail(3) = RobotTools::Tr2FixedZYX(cpose.rot);
		break;
	}
	case eCartesianSpace:
	{
		_segplanner[_traj_idx]->GeneratePath(cpos);
		break;
	}
	case eBSpline:
	{
		_segplanner[_traj_idx]->GeneratePath(cpos);
		break;
	}
	case eCartesianArc:
	{
		_segplanner[_traj_idx]->GeneratePath(cpos);
		break;
	}
	default:
		break;
	}
	if (_segplanner[_traj_idx]->_plan_completed)
	{
		if (_traj_idx==_ntraj-1)
			_task_completed = true;
		else
			_traj_idx += 1;
	}
}

void TaskTrajPlanner::GenerateBothPath(Vector6d& jpos, Vector6d& cpos)
{
	switch (_traj_type[_traj_idx])
	{
	case eJointSpace:
	{
		_segplanner[_traj_idx]->GeneratePath(jpos);
		Pose cpose = _robot->FKSolveTool(jpos);
		cpos.head(3) = cpose.pos;
		cpos.tail(3) = RobotTools::Tr2FixedZYX(cpose.rot);
		_pre_q = jpos;
		break;
	}
	case eCartesianSpace:
	{
		_segplanner[_traj_idx]->GeneratePath(cpos);
		Pose cmd_pose;
		cmd_pose.pos = cpos.head(3);
		cmd_pose.rot = RobotTools::FixedZYX2Tr(cpos.tail(3));
		_robot->IKSolve(&jpos, &cmd_pose, &_pre_q);
		_pre_q = jpos;
		break;
	}
	case eBSpline:
	{
		_segplanner[_traj_idx]->GeneratePath(cpos);
		Pose cmd_pose;
		cmd_pose.pos = cpos.head(3);
		cmd_pose.rot = RobotTools::FixedZYX2Tr(cpos.tail(3));
		_robot->IKSolve(&jpos, &cmd_pose, &_pre_q);
		_pre_q = jpos;
		break;
	}
	case eCartesianArc:
	{
		_segplanner[_traj_idx]->GeneratePath(cpos);
		Pose cmd_pose;
		cmd_pose.pos = cpos.head(3);
		cmd_pose.rot = RobotTools::FixedZYX2Tr(cpos.tail(3));
		_robot->IKSolve(&jpos, &cmd_pose, &_pre_q);
		_pre_q = jpos;
		break;
	}
	default:
		break;
	}
	if (_segplanner[_traj_idx]->_plan_completed)
	{
		if (_traj_idx==_ntraj-1)
			_task_completed = true;
		else
			_traj_idx += 1;
	}
}

void TaskTrajPlanner::Reset(Vector6d q0)
{
	_pre_q = q0;
	_pre_trajq = q0;
	_pre_trajpose = _robot->FKSolveTool(q0);
	for (int idx = 0; idx<_ntraj; idx++)
	{
		_segplanner[idx]->Reset(q0, false);
	}
	for (auto it = _segplanner.begin(); it!=_segplanner.end(); it++)
	{
		if (*it!=nullptr)
		{
			delete *it;
			*it = nullptr;
		}
	}
	_segplanner.clear();
	_traj_type.clear();
	_traj_idx = 0;
	_ntraj = 0;
	_task_completed = false;
}


double TaskTrajPlanner::CalcBSplineTime(MatrixXd* via_pos)
{
	double pos_len = 0;
	double rot_len = 0;
	int np = static_cast<int>(via_pos->cols());
	MatrixXd pos_bspline = via_pos->topRows(3);
	MatrixXd rpy_bspline = via_pos->bottomRows(3);
	for (int idx = 0; idx<np-1; idx++)
	{
		pos_len += MathTools::Norm(pos_bspline.col(idx+1)-pos_bspline.col(idx));
		rot_len += RobotTools::CalcAngleAxis(rpy_bspline.col(idx), rpy_bspline.col(idx+1))(3);
	}
	double coef = 1.5;
	double tf_pos = pos_len/g_cvmax[0]*coef;
	double tf_rot = rot_len/g_cvmax[1]*coef;
	Vector2d tf(tf_pos, tf_rot);
	
	return tf.maxCoeff();
}

void TaskTrajPlanner::CheckCartWorkspace(Vector3d cpos)
{
	double radius_ws = 1.1;
	double ratio_ws = 0.9;//0.8
	double zmin = -0.6;
	double len_tool = _robot->GetToolLength();
	Vector3d center(0, 0, 0);
	if ((MathTools::Norm(cpos - center) > radius_ws*ratio_ws + len_tool)
		|| (cpos(2)<zmin - len_tool))
		throw eErrJointOutOfRange;
}

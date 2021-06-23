#include "TaskTrajPlanner.h"

TaskTrajPlanner::TaskTrajPlanner(Vector3d* pos0, Vector3d* rpy0, bool conti_type)
{
	_pos_corner.push_back(pos0);
	_rpy_corner.push_back(rpy0);
	_ntraj = 0;
	_t = 0;
	_traj_idx = 0;
	_task_completed = false;
	_continuity = conti_type;
}

void TaskTrajPlanner::AddPosRPY(Vector3d* pos, Vector3d* rpy)
{		
	if (_continuity)
		AddContiPosRPY(pos, rpy);
	else
		AddDiscontiPosRPY(pos, rpy);
}

void TaskTrajPlanner::AddContiPosRPY(Vector3d* pos, Vector3d* rpy)
{
	_pos_corner.push_back(pos);
	_rpy_corner.push_back(rpy);
	int np = _pos_corner.size();
	if (np==2)
	{
		Vector3d pos0 = *_pos_corner[0];
		Vector3d posn = *_pos_corner[1];
		Vector3d rpy0 = *_rpy_corner[0];
		Vector3d rpyn = *_rpy_corner[1];
		Vector6d pos_rpy0, pos_rpyn;
		pos_rpy0<<pos0, rpy0;
		pos_rpyn<<posn, rpyn;
		double vmax[] = { g_cvmax,0.15 };
		double amax[] = { g_camax,0.3 };
		double vel_cons[] = { 0,0,0,0 };
		char* opt = "none";
		CalcTrajOption(opt, pos_rpy0, pos_rpyn);
		BaseCTrajPlanner* traj_planner = new LineTrajPlanner(pos_rpy0, pos_rpyn, vmax, amax, vel_cons, opt);
		_segtraj_planner.push_back(traj_planner);
		_ntraj = 1;
	}
	else if (np==3)
	{
		MatrixXd pos_tmp = RobotTools::CalcSplineTransPos(*_pos_corner[0], *_pos_corner[1], *_pos_corner[2], 0.05, "arc");
		Vector3d* p1 = new Vector3d(pos_tmp.col(0));
		Vector3d* p2 = new Vector3d(pos_tmp.col(1));
		_pos_seg.push_back(p1);
		_pos_seg.push_back(p2);
		double radius = RobotTools::CalcArcRadius(*_pos_seg[0], *_pos_corner[1], *_pos_seg[1]);
		_varc.push_back(sqrt(g_camax*radius));
		Vector3d pos0 = *_pos_corner[0];
		Vector3d posn = *_pos_seg[0];
		Vector3d rpy0 = *_rpy_corner[0];
		Vector3d rpyn = *_rpy_corner[1];
		Vector6d pos_rpy0, pos_rpyn;
		pos_rpy0<<pos0, rpy0;
		pos_rpyn<<posn, rpyn;
		double vmax[] = { g_cvmax,0.15 };
		double amax[] = { g_camax,0.3 };
		double vel_cons[] = { 0,_varc[0],0,0 };
		delete _segtraj_planner[0];
		_segtraj_planner.clear();
		char* opt = "none";
		CalcTrajOption(opt, pos_rpy0, pos_rpyn);
		BaseCTrajPlanner* traj_planner = new LineTrajPlanner(pos_rpy0, pos_rpyn, vmax, amax, vel_cons, opt);
		_segtraj_planner.push_back(traj_planner);
		_ntraj = 3;
	}
	else
	{
		MatrixXd pos_tmp = RobotTools::CalcSplineTransPos(*_pos_corner[np-3], *_pos_corner[np-2], *_pos_corner[np-1], 0.05, "arc");
		Vector3d* p1 = new Vector3d(pos_tmp.col(0));
		Vector3d* p2 = new Vector3d(pos_tmp.col(1));
		_pos_seg.push_back(p1);
		_pos_seg.push_back(p2);
		int nseg = _pos_seg.size();
		double radius = RobotTools::CalcArcRadius(*_pos_seg[nseg-2], *_pos_corner[np-2], *_pos_seg[nseg-1]);
		_varc.push_back(sqrt(g_camax*radius));
		_ntraj += 2;
	}
}

void TaskTrajPlanner::AddDiscontiPosRPY(Vector3d* pos, Vector3d* rpy)
{
	_pos_corner.push_back(pos);
	_rpy_corner.push_back(rpy);
	int np = _pos_corner.size();
	_ntraj += 1;
	Vector3d pos0 = *_pos_corner[np-2];
	Vector3d posn = *_pos_corner[np-1];
	Vector3d rpy0 = *_rpy_corner[np-2];
	Vector3d rpyn = *_rpy_corner[np-1];
	Vector6d pos_rpy0, pos_rpyn;
	pos_rpy0<<pos0, rpy0;
	pos_rpyn<<posn, rpyn;
	double vmax[] = { g_cvmax,0.15 };
	double amax[] = { g_camax,0.3 };
	double vel_cons[] = { 0,0,0,0 };
	char* opt = "none";
	CalcTrajOption(opt, pos_rpy0, pos_rpyn);
	BaseCTrajPlanner* traj_planner = new LineTrajPlanner(pos_rpy0, pos_rpyn, vmax, amax, vel_cons, opt);
	_segtraj_planner.push_back(traj_planner);
}

RobotTools::CAVP TaskTrajPlanner::GenerateMotion()
{
	if (_continuity)
		return GenerateContiMotion();
	else
		return GenerateDiscontiMotion();
}


RobotTools::CAVP TaskTrajPlanner::GenerateContiMotion()
{
	RobotTools::CAVP cavp;
	cavp = _segtraj_planner[_traj_idx]->GenerateMotion(_t);
	if ((_traj_idx==_ntraj-1)&&(fabs(_segtraj_planner[_traj_idx]->GetFinalTime()-_t)<EPS))
	{
		_task_completed = true;
	}
	else
	{
		//make transition between line and arc smooth
		double tf = _segtraj_planner[_traj_idx]->GetFinalTime();
		double tf_int = floor(tf/g_cycle_time)*g_cycle_time;
		if ((_traj_idx%2==0)&&(fabs(_t-tf_int+g_cycle_time)<EPS)&&(_traj_idx!=_ntraj-1))
		{
			//plan the next trajectory: arc segment
			RobotTools::CAVP cavpn = _segtraj_planner[_traj_idx]->GenerateMotion(tf_int);
			Vector3d pos1 = cavpn.pos.head(3);
			Vector3d pos2 = *_pos_corner[_traj_idx/2+1];
			Vector3d pos3 = UpdateSegPos(pos1, pos2, *_pos_corner[_traj_idx/2+2]);
			double vcons = cavpn.vel.head(3).norm();
			Vector3d rpy1 = cavp.pos.tail(3);
			Vector6d pos_rpy1, pos_rpy3;
			pos_rpy1<<pos1, rpy1;
			pos_rpy3<<pos3, rpy1;
			double vmax[] = { vcons,0.15 };
			double amax[] = { g_camax,0.3 };
			double vel_cons[] = { vcons,vcons,0,0 };
			BaseCTrajPlanner* traj_planner = new ArcTrajPlanner(pos_rpy1, pos2, pos_rpy3, vmax, amax, vel_cons, "pos");
			_segtraj_planner.push_back(traj_planner);
			_traj_idx += 1;
			_t = -g_cycle_time;
		}
		else if ((_traj_idx%2==1)&&(fabs(_t-tf_int+g_cycle_time)<EPS))
		{
			//plan the next trajectory: line segment
			RobotTools::CAVP cavpn = _segtraj_planner[_traj_idx]->GenerateMotion(tf_int);
			Vector3d pos0 = cavpn.pos.head(3);
			Vector3d rpy0 = cavpn.pos.tail(3);
			Vector3d rpyn = *_rpy_corner[(_traj_idx+1)/2+1];
			double v0 = cavpn.vel.head(3).norm();
			Vector3d posn;
			double vf;
			if (_traj_idx==_ntraj-2)
			{
				posn = *_pos_corner[_pos_corner.size()-1];
				vf = 0;
			}
			else
			{
				posn = *_pos_seg[_traj_idx+1];
				vf = _varc[_traj_idx/2+1];
			}
			Vector6d pos_rpy0, pos_rpyn;
			pos_rpy0<<pos0, rpy0;
			pos_rpyn<<posn, rpyn;
			double vmax[] = { g_cvmax,0.15 };
			double amax[] = { g_camax,0.3 };
			double vel_cons[] = { v0,vf,0,0 };
			char* opt = "none";
			CalcTrajOption(opt, pos_rpy0, pos_rpyn);
			BaseCTrajPlanner* traj_planner = new LineTrajPlanner(pos_rpy0, pos_rpyn, vmax, amax, vel_cons, opt);
			_segtraj_planner.push_back(traj_planner);
			_traj_idx += 1;
			_t = -g_cycle_time;
		}
		_t += g_cycle_time;
		MathTools::LimitMax(_segtraj_planner[_traj_idx]->GetFinalTime(), _t);
	}

	return cavp;
}

RobotTools::CAVP TaskTrajPlanner::GenerateDiscontiMotion()
{
	RobotTools::CAVP cavp;
	cavp = _segtraj_planner[_traj_idx]->GenerateMotion(_t);
	_t += g_cycle_time;
	MathTools::LimitMax(_segtraj_planner[_traj_idx]->GetFinalTime(), _t);
	if ((_traj_idx==_ntraj-1)&&(fabs(_segtraj_planner[_traj_idx]->GetFinalTime()-_t)<EPS))
	{
		_task_completed = true;
	}
	else
	{
		if (fabs(_t-_segtraj_planner[_traj_idx]->GetFinalTime())<EPS)
		{
			_traj_idx += 1;
			_t = 0;
		}
	}

	return cavp;
}

void TaskTrajPlanner::Reset(bool conti_type)
{
	_ntraj = 0;
	_t = 0;
	_traj_idx = 0;
	_task_completed = false;
	_continuity = conti_type;
	ClearTemp();
}

void TaskTrajPlanner::Reset(Vector3d* pos0, Vector3d* rpy0, bool conti_type)
{
	_ntraj = 0;
	_t = 0;
	_traj_idx = 0;
	_task_completed = false;
	ClearTemp();
	_pos_corner.push_back(pos0);
	_rpy_corner.push_back(rpy0);
	_continuity = conti_type;
}


void TaskTrajPlanner::AddPosRPY(Vector3d *pos, Vector3d *rpy, Vector3d &seg_opt)
{
	char* opt = (char*)"both";
	if (seg_opt[0] > EPS)
		opt = (char*)"pos";
	else if (seg_opt[1] > EPS)
		opt = (char*)"rot";
	else
		opt = (char*)"both";

	AddPosRPY(pos, rpy);
}

Vector6d TaskTrajPlanner::GeneratePoint()
{
	Vector6d pos_rpy;
	if (_traj_idx!=_ntraj)
	{
		pos_rpy = _segtraj_planner[_traj_idx]->GeneratePoint(_t);
		_t += g_cycle_time;
		MathTools::LimitMax(_segtraj_planner[_traj_idx]->GetFinalTime(), _t);
		if (fabs(_t-_segtraj_planner[_traj_idx]->GetFinalTime())<g_cycle_time)
		{			
			_traj_idx += 1;
			_t = 0;

			//delete _segtraj_planner;
			if (_traj_idx==_ntraj)
			{
				for (vector<BaseCTrajPlanner*>::iterator it = _segtraj_planner.begin(); it!=_segtraj_planner.end(); it++)
				{
					if (*it!=NULL)
					{
						delete *it;
						*it = NULL;
					}
				}
				_segtraj_planner.clear();
				_pos_corner.clear();
				_rpy_corner.clear();
			}

		}
	}
	else
	{
		double t = _segtraj_planner[_ntraj-1]->GetFinalTime();
		pos_rpy = _segtraj_planner[_ntraj-1]->GeneratePoint(t);
	}

	return pos_rpy;
}

void TaskTrajPlanner::ReInitSegTrajPlanner(Vector3d &pos0, Vector3d &rpy0)
{
    if(_continuity)
    {
        //temporary variable : cache segment pos
        vector<Vector3d> posn;
        vector<Vector3d> rpyn;
        int length = (int)_pos_corner.size();
        int trajindex = 0;

        if(_traj_idx%2 == 0)
            trajindex = _traj_idx/2;
        else if(_traj_idx%2 == 1)
            trajindex = (_traj_idx-1)/2;

        for(int i = 0; i < length- trajindex - 1; ++i)
        {
            posn.push_back(*_pos_corner[trajindex+1 +i]);
            rpyn.push_back(*_rpy_corner[trajindex+1 +i]);
        }

        Reset(&pos0, &rpy0, false);
        for(int i = 0; i < (int)posn.size(); ++i)
        {
            AddPosRPY(&posn[i], &rpyn[i]);
        }

        posn.clear();
        rpyn.clear();
        return;
    }
    else
    {
        Vector3d posn = *_pos_corner[_traj_idx+1];
        Vector3d rpyn = *_rpy_corner[_traj_idx+1];

        Vector6d pos_rpy0, pos_rpyn;
        pos_rpy0<<pos0, rpy0;
        pos_rpyn<<posn, rpyn;

        double vmax[] = { g_cvmax,0.15 };
        double amax[] = { g_camax,0.3 };
        double vel_cons[] = { 0,0,0,0 };


        vector<BaseCTrajPlanner*>::iterator iter_segtraj = _segtraj_planner.begin()+_traj_idx;
        _segtraj_planner.erase(iter_segtraj);

        char* opt = "none";
        CalcTrajOption(opt, pos_rpy0, pos_rpyn);

        BaseCTrajPlanner* traj_planner = new LineTrajPlanner(pos_rpy0, pos_rpyn, vmax, amax, vel_cons, opt);
        _segtraj_planner.insert(_segtraj_planner.begin()+_traj_idx, traj_planner);

        _t = 0;
    }
}

bool TaskTrajPlanner::GetSegTrajPlannerDone()
{
    return _task_completed;//_traj_idx==_ntraj;
}

Vector3d TaskTrajPlanner::UpdateSegPos(Vector3d p1, Vector3d p2, Vector3d p3_corner)
{
	Vector3d pos3;
	double line_len = MathTools::Norm(p2-p1);
	Vector3d dir_p2p3 = (p3_corner-p2)/MathTools::Norm(p3_corner-p2);
	pos3 = p2+line_len*dir_p2p3;

	return pos3;
}

void TaskTrajPlanner::ClearTemp()
{
	for (vector<BaseCTrajPlanner*>::iterator it = _segtraj_planner.begin(); it!=_segtraj_planner.end(); it++)
	{
		if (*it!=nullptr)
		{
			delete *it;
			*it = nullptr;
		}
	}
	for (vector<Vector3d*>::iterator it = _pos_seg.begin(); it!=_pos_seg.end(); it++)
	{
		if (*it!=nullptr)
		{
			delete *it;
			*it = nullptr;
		}
	}
	_segtraj_planner.clear();
	_pos_seg.clear();
	_pos_corner.clear();
	_rpy_corner.clear();
	_rpy_seg.clear();
	_varc.clear();
}

void TaskTrajPlanner::CalcTrajOption(char* &opt, Vector6d pos_rpy1, Vector6d pos_rpy2)
{
	double pos_len = MathTools::Norm(pos_rpy1.head(3) - pos_rpy2.head(3));
	Vector4d angleaxis = RobotTools::CalcAngleAxis(pos_rpy1.tail(3), pos_rpy2.tail(3));
	double rpy_len = angleaxis(3);
	//char* opt;
	if ((pos_len>EPS) && (rpy_len>EPS))
		opt = "both";
	else if ((pos_len>EPS) && (rpy_len <= EPS))
		opt = "pos";
	else if ((pos_len <= EPS) && (rpy_len>EPS))
		opt = "rot";
	else
		opt = "none";
}


#include "CartesianPlanner.h"

CartesianPlanner::CartesianPlanner(Vector6d pos_rpy0, bool conti_type)
	:_continuity(conti_type)
{
	_pos_corner.push_back(pos_rpy0.head(3));
	_rpy_corner.push_back(pos_rpy0.tail(3));
	_ntraj = 0;
	_traj_idx = 0;
	_arc_type = false;
}

CartesianPlanner::CartesianPlanner(Vector6d* pos_rpy1, Vector3d* pos2, Vector6d* pos_rpy3)
{
	_ntraj = 1;
	_traj_idx = 0;
	_arc_type = true;
	double vel_cons[] = { 0,0,0,0 };
	BaseCartesianPlanner* traj_planner = new 
		ArcPlanner(*pos_rpy1, *pos2, *pos_rpy3, g_cvmax, g_camax, vel_cons,"arc");
	_segtraj_planner.push_back(traj_planner);
}

void CartesianPlanner::AddViaPos(MatrixXd* via_pos)
{
	int np = static_cast<int>(via_pos->cols());
	for (int idx = 0; idx<np; idx++)
	{
		Vector6d pos_rpy = via_pos->col(idx);
		if (_continuity)
			AddContiPosRPY(pos_rpy.head(3), pos_rpy.tail(3));
		else
			AddDiscontiPosRPY(pos_rpy.head(3), pos_rpy.tail(3));
	}
}

void CartesianPlanner::GenerateMotion(Vector6d& cpos, Vector6d& cvel, Vector6d& cacc)
{
	RobotTools::CAVP cavp;
	if (_arc_type)
		GenerateArcMotion(cavp);
	else
	{
		if (_continuity)
			GenerateContiMotion(cavp);
		else
			GenerateDiscontiMotion(cavp);
	}
	cpos = cavp.pos;
	cvel = cavp.vel;
	cacc = cavp.acc;
}

void CartesianPlanner::GeneratePath(Vector6d& cpos)
{
	RobotTools::CAVP cavp;
	if (_arc_type)
		GenerateArcMotion(cavp);
	else
	{
		if (_continuity)
			GenerateContiMotion(cavp);
		else
			GenerateDiscontiMotion(cavp);
	}
	cpos = cavp.pos;
}

void CartesianPlanner::AddContiPosRPY(Vector3d pos, Vector3d rpy)
{
	double radius_trans = 0.05;
	if (MathTools::Norm(pos-_pos_corner.back())<=radius_trans)
		throw eErrArcPlanner;

	_pos_corner.push_back(pos);
	_rpy_corner.push_back(rpy);
	int np = static_cast<int>(_pos_corner.size());
	if (np==2)
	{
		Vector3d pos0 = _pos_corner[0];
		Vector3d posn = _pos_corner[1];
		Vector3d rpy0 = _rpy_corner[0];
		Vector3d rpyn = _rpy_corner[1];
		Vector6d pos_rpy0, pos_rpyn;
		pos_rpy0<<pos0, rpy0;
		pos_rpyn<<posn, rpyn;
		double vel_cons[] = { 0,0,0,0 };
		BaseCartesianPlanner* traj_planner = new 
			LinePlanner(pos_rpy0, pos_rpyn, g_cvmax, g_camax, vel_cons);
		_segtraj_planner.push_back(traj_planner);
		_ntraj = 1;
	}
	else if (np==3)
	{
		MatrixXd pos_tmp = PlanTools::CalcSplineTransPos
			(_pos_corner[0], _pos_corner[1], _pos_corner[2], radius_trans, "arc");
		Vector3d p1 = Vector3d(pos_tmp.col(0));
		Vector3d p2 = Vector3d(pos_tmp.col(1));
		_pos_seg.push_back(p1);
		_pos_seg.push_back(p2);
		double radius = PlanTools::CalcArcRadius
			(_pos_seg[0], _pos_corner[1], _pos_seg[1]);
        _varc.push_back(sqrt(g_camax[0]*radius));
		Vector3d pos0 = _pos_corner[0];
		Vector3d posn = _pos_seg[0];
		Vector3d rpy0 = _rpy_corner[0];
		Vector3d rpyn = _rpy_corner[1];
		Vector6d pos_rpy0, pos_rpyn;
		pos_rpy0<<pos0, rpy0;
		pos_rpyn<<posn, rpyn;
		double vel_cons[] = { 0,_varc[0],0,0 };
		delete _segtraj_planner[0];
		_segtraj_planner.clear();
		BaseCartesianPlanner* traj_planner = new 
			LinePlanner(pos_rpy0, pos_rpyn, g_cvmax, g_camax, vel_cons);
		_segtraj_planner.push_back(traj_planner);
		_ntraj = 3;
	}
	else
	{
		MatrixXd pos_tmp = PlanTools::CalcSplineTransPos
			(_pos_corner[np-3], _pos_corner[np-2], _pos_corner[np-1], radius_trans, "arc");
		Vector3d p1 = Vector3d(pos_tmp.col(0));
		Vector3d p2 = Vector3d(pos_tmp.col(1));
		_pos_seg.push_back(p1);
		_pos_seg.push_back(p2);
		int nseg = static_cast<int>(_pos_seg.size());
		double radius = PlanTools::CalcArcRadius
			(_pos_seg[nseg-2], _pos_corner[np-2], _pos_seg[nseg-1]);
        _varc.push_back(sqrt(g_camax[0]*radius));
		_ntraj += 2;
	}

}

void CartesianPlanner::AddDiscontiPosRPY(Vector3d pos, Vector3d rpy)
{
	_pos_corner.push_back(pos);
	_rpy_corner.push_back(rpy);
	int np = static_cast<int>(_pos_corner.size());
	_ntraj += 1;
	Vector3d pos0 = _pos_corner[np-2];
	Vector3d posn = _pos_corner[np-1];
	Vector3d rpy0 = _rpy_corner[np-2];
	Vector3d rpyn = _rpy_corner[np-1];
	Vector6d pos_rpy0, pos_rpyn;
	pos_rpy0<<pos0, rpy0;
	pos_rpyn<<posn, rpyn;
	double vel_cons[] = { 0,0,0,0 };
	BaseCartesianPlanner* traj_planner = new 
		LinePlanner(pos_rpy0, pos_rpyn, g_cvmax, g_camax, vel_cons);
	_segtraj_planner.push_back(traj_planner);
}

void CartesianPlanner::GenerateContiMotion(RobotTools::CAVP &cavp)
{
	cavp = _segtraj_planner[_traj_idx]->GenerateMotion(_t);
	if ((_traj_idx==_ntraj-1)&&(fabs(_segtraj_planner[_traj_idx]->GetFinalTime()-_t)<EPS))
	{
		_plan_completed = true;
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
			Vector3d pos2 = _pos_corner[_traj_idx/2+1];
			Vector3d pos3 = UpdateSegPos(pos1, pos2, _pos_corner[_traj_idx/2+2]);
			double vcons = cavpn.vel.head(3).norm();
			Vector3d rpy1 = cavp.pos.tail(3);
			Vector6d pos_rpy1, pos_rpy3;
			pos_rpy1<<pos1, rpy1;
			pos_rpy3<<pos3, rpy1;
			double vmax[] = { vcons, g_cvmax[1]};
			double vel_cons[] = { vcons,vcons,0,0 };
			BaseCartesianPlanner* traj_planner = new 
				ArcPlanner(pos_rpy1, pos2, pos_rpy3, vmax, g_camax, vel_cons, "arctrans");
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
			Vector3d rpyn = _rpy_corner[(_traj_idx+1)/2+1];
			double v0 = cavpn.vel.head(3).norm();
			Vector3d posn;
			double vf;
			if (_traj_idx==_ntraj-2)
			{
				posn = _pos_corner[_pos_corner.size()-1];
				vf = 0;
			}
			else
			{
				posn = _pos_seg[_traj_idx+1];
				vf = _varc[_traj_idx/2+1];
			}
			Vector6d pos_rpy0, pos_rpyn;
			pos_rpy0<<pos0, rpy0;
			pos_rpyn<<posn, rpyn;
			double vel_cons[] = { v0,vf,0,0 };
			BaseCartesianPlanner* traj_planner = new 
				LinePlanner(pos_rpy0, pos_rpyn, g_cvmax, g_camax, vel_cons);
			_segtraj_planner.push_back(traj_planner);
			_traj_idx += 1;
			_t = -g_cycle_time;
		}
		_t += g_cycle_time;
		MathTools::LimitMax(_segtraj_planner[_traj_idx]->GetFinalTime(), _t);
	}

}

void CartesianPlanner::GenerateDiscontiMotion(RobotTools::CAVP &cavp)
{
	cavp = _segtraj_planner[_traj_idx]->GenerateMotion(_t);
	_t += g_cycle_time;
	MathTools::LimitMax(_segtraj_planner[_traj_idx]->GetFinalTime(), _t);
	if ((_traj_idx==_ntraj-1)&&(fabs(_segtraj_planner[_traj_idx]->GetFinalTime()-_t)<EPS))
	{
		_plan_completed = true;
	}
	else
	{
		if (fabs(_t-_segtraj_planner[_traj_idx]->GetFinalTime())<EPS)
		{
			_traj_idx += 1;
			_t = 0;
		}
	}

}

void CartesianPlanner::GenerateArcMotion(RobotTools::CAVP &cavp)
{
	cavp = _segtraj_planner[0]->GenerateMotion(_t);
	_t += g_cycle_time;
	double tf = _segtraj_planner[0]->GetFinalTime();
	MathTools::LimitMax(tf, _t);
	if (fabs(tf-_t)<EPS3)
		_plan_completed = true;
}

void CartesianPlanner::Reset(Vector6d pos_rpy0, bool conti_type)
{
	_ntraj = 0;
	_t = 0;
	_traj_idx = 0;
	_plan_completed = false;
	ClearTemp();
	_pos_corner.push_back(pos_rpy0.head(3));
	_rpy_corner.push_back(pos_rpy0.tail(3));
	_continuity = conti_type;
	_arc_type = false;
}

void CartesianPlanner::Reset(Vector6d* pos_rpy1, Vector3d* pos2, Vector6d* pos_rpy3)
{
	_ntraj = 1;
	_t = 0;
	_traj_idx = 0;
	_plan_completed = false;
	ClearTemp();
	_arc_type = true;
	double vel_cons[] = { 0,0,0,0 };
	BaseCartesianPlanner* traj_planner = new 
		ArcPlanner(*pos_rpy1, *pos2, *pos_rpy3, g_cvmax, g_camax, vel_cons, "arc");
	_segtraj_planner.push_back(traj_planner);
}

void CartesianPlanner::ReInitSegTrajPlanner(Vector3d &pos0, Vector3d &rpy0, Vector3d *cache_pos_addr, Vector3d *cache_rpy_addr)
{
    if(_continuity)
    {
        //addr variable : need to cache segment pos :cache_pos_addr + cache_rpy_addr
        int length = (int)_pos_corner.size();
        int trajindex = 0;

        if(_traj_idx%2 == 0)
            trajindex = _traj_idx/2;
        else if(_traj_idx%2 == 1)
            trajindex = (_traj_idx-1)/2;

        for(int i = 0; i < length- trajindex - 1; ++i)
        {
            cache_pos_addr[i] = (_pos_corner[trajindex+1 +i]);
            cache_rpy_addr[i] = (_rpy_corner[trajindex+1 +i]);
        }
		Vector6d pos_rpy0;
		pos_rpy0<<pos0, rpy0;
        Reset(pos_rpy0, false);
		MatrixXd cache_pos_rpy;
		cache_pos_rpy.setZero(6, length-trajindex-1);
        for(int i = 0; i < length- trajindex - 1; ++i)
        {
			cache_pos_rpy.col(i).head(3) = cache_pos_addr[i];
			cache_pos_rpy.col(i).tail(3) = cache_rpy_addr[i];
            //AddPosRPY(&cache_pos_addr[i], &cache_rpy_addr[i]);
        }
		AddViaPos(&cache_pos_rpy);

        return;
    }
    else
    {
        Vector3d posn = _pos_corner[_traj_idx+1];
        Vector3d rpyn = _rpy_corner[_traj_idx+1];

        Vector6d pos_rpy0, pos_rpyn;
        pos_rpy0<<pos0, rpy0;
        pos_rpyn<<posn, rpyn;
        double vel_cons[] = { 0,0,0,0 };

        vector<BaseCartesianPlanner*>::iterator iter_segtraj = _segtraj_planner.begin()+_traj_idx;
        _segtraj_planner.erase(iter_segtraj);

        BaseCartesianPlanner* traj_planner = new LinePlanner(pos_rpy0, pos_rpyn, g_cvmax, g_camax, vel_cons);
        _segtraj_planner.insert(_segtraj_planner.begin()+_traj_idx, traj_planner);

        _t = 0;
    }
}

void CartesianPlanner::ReInitSegTrajPlanner(Vector6d &q_cur, Vector6d *cache_jntpos_addr)
{
        //addr : need to cache segment pos : cache_jntpos_addr

        //int length = (int)_q_save.size();
        //int trajindex = _traj_idx;
		//
        //for(int i = 0; i < length- trajindex - 1; ++i)
        //{
        //    cache_jntpos_addr[i] = (*_q_save[trajindex+1 +i]);
        //}
		//
        //Reset(&q_cur, true);
        //for(int i = 0; i < length- trajindex - 1; ++i)
        //{
        //    AddJntPos(&cache_jntpos_addr[i]);
        //}
}

Vector3d CartesianPlanner::UpdateSegPos(Vector3d p1, Vector3d p2, Vector3d p3_corner)
{
	Vector3d pos3;
	double line_len = MathTools::Norm(p2-p1);
	Vector3d dir_p2p3 = (p3_corner-p2)/MathTools::Norm(p3_corner-p2);
	pos3 = p2+line_len*dir_p2p3;

	return pos3;
}

void CartesianPlanner::ClearTemp()
{
	if (_arc_type)
	{
		for (auto it = _segtraj_planner.begin(); it!=_segtraj_planner.end(); it++)
		{
			if (*it!=nullptr)
			{
				delete *it;
				*it = nullptr;
			}
		}
		_segtraj_planner.clear();
	}
	else
	{
		for (auto it = _segtraj_planner.begin(); it!=_segtraj_planner.end(); it++)
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

}




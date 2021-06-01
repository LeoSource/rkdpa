#pragma once

#include "LineTrajPlanner.h"
#include "GlobalParams.h"
#include <vector>

class TaskTrajPlanner
{
private:
	vector<BaseCTrajPlanner*> _segtraj_planner;
	int _ntraj;
	vector<Vector3d*> _pos_corner;
	vector<Vector3d*> _rpy_corner;
	vector<Vector3d*> _pos_seg;
	vector<Vector3d*> _rpy_seg;
	double _t;
	int _traj_idx;

public:
	TaskTrajPlanner() {}

	TaskTrajPlanner(Vector3d pos0, Vector3d rpy0)
	{
		_pos_corner.push_back(&pos0);
		_rpy_corner.push_back(&rpy0);
		_ntraj = 0;
		_t = 0;
		_traj_idx = 0;
	}

	void AddPosRPY(Vector3d pos, Vector3d rpy, char* opt)
	{
		_pos_corner.push_back(&pos);
		_rpy_corner.push_back(&rpy);
		int pos_idx = _pos_corner.size();
		_ntraj += 1;
		Vector3d pos0 = *_pos_corner[pos_idx-2];
		Vector3d posn = *_pos_corner[pos_idx-1];
		Vector3d rpy0 = *_rpy_corner[pos_idx-2];
		Vector3d rpyn = *_rpy_corner[pos_idx-1];
		Vector6d pos_rpy0, pos_rpyn;
		pos_rpy0<<pos0, rpy0;
		pos_rpyn<<posn, rpyn;
		double vmax[] = { g_cvmax,0.15 };
		double amax[] = { g_camax,0.3 };
		double duration[] = { -1,-1 };
		double vel_cons[] = { 0,0,0,0 };
		BaseCTrajPlanner* traj_planner = new LineTrajPlanner(pos_rpy0, pos_rpyn, vmax, amax, duration, vel_cons, opt);
		_segtraj_planner.push_back(traj_planner);
		//delete traj_planner;
	}

	Vector6d GeneratePoint()
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
			}
		}
		else
		{
			double t = _segtraj_planner[_ntraj-1]->GetFinalTime();
			pos_rpy = _segtraj_planner[_ntraj-1]->GeneratePoint(t);
		}

		return pos_rpy;
	}


	~TaskTrajPlanner() {}
};


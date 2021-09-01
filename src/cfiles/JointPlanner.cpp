#include "JointPlanner.h"

JointPlanner::JointPlanner(Vector6d q0, bool sync)
	:_sync(sync)
{
	_jpos.push_back(q0);
	_t = 0;
	_plan_completed = false;
	_vmax = g_jvmax;
	_amax = g_jamax;
	_ntraj = 0;
	_traj_idx = 0;
}

void JointPlanner::SetKineConstraint(double* vmax, double* amax)
{
	_vmax = vmax;
	_amax = amax;
}

void JointPlanner::AddViaPos(MatrixXd* jpos)
{
	_ntraj = (int)jpos->cols();
	for (int traj_idx = 0; traj_idx<_ntraj; traj_idx++)
	{
		_jpos.push_back(jpos->col(traj_idx));
		Vector6d q0, qf;
		q0 = _jpos[traj_idx];
		qf = _jpos[traj_idx+1];
		vector<int> planidx, unplanidx;
		for (int jidx = 0; jidx<6; jidx++)
		{
			if (fabs(q0(jidx)-qf(jidx))>EPS3)
				planidx.push_back(jidx);
			else
				unplanidx.push_back(jidx);
		}
		_jplanned.push_back(planidx);
		_junplanned.push_back(unplanidx);
		vector<double> tf_jnt;
		vector<LspbPlanner*> vec_uplanner;
		vec_uplanner.resize(6);
		for (int jidx : _jplanned[traj_idx])
		{
			if ((qf(jidx)-g_qlimit_ur[2*jidx+1]>0)||(qf(jidx)-g_qlimit_ur[2*jidx]<0))
				throw eErrJointOutOfRange;

			LspbPlanner* uplanner = new LspbPlanner(Vector2d(q0(jidx), qf(jidx)),
				_vmax[jidx], _amax[jidx]);
			vec_uplanner[jidx] = uplanner;
			tf_jnt.push_back(uplanner->GetFinalTime());
		}
		if (tf_jnt.size()>0)
			_tf.push_back(*max_element(tf_jnt.begin(), tf_jnt.end()));
		else
			_tf.push_back(0);
		if (_sync)
		{
			for (int jidx : _jplanned[traj_idx])
			{
				LspbPlanner* uplanner = new LspbPlanner(Vector2d(q0(jidx), qf(jidx)),
					_vmax[jidx], _amax[jidx], _tf[traj_idx]);
				vec_uplanner[jidx] = uplanner;
			}
		}
		_segplanner.push_back(vec_uplanner);
	}
}						   

void JointPlanner::GeneratePath(Vector6d& jpos)
{
	RobotTools::JAVP javp;
	//generate joint position
	if (_jplanned[_traj_idx].size()>0)
	{
		for (int jidx:_jplanned[_traj_idx])
		{
			javp = _segplanner[_traj_idx][jidx]->GenerateMotion(_t);
			jpos(jidx) = javp.pos;
		}
		_t += g_cycle_time;
		MathTools::LimitMax(_tf[_traj_idx], _t);
	}
	if (_junplanned[_traj_idx].size()>0)
	{
		for (int jidx : _junplanned[_traj_idx])
			jpos(jidx) = _jpos[_traj_idx](jidx);
	}
	//logical judgment about trajectory index and plan status
	if ((_traj_idx==_ntraj-1)&&(fabs(_t-_tf[_traj_idx])<EPS3))
		_plan_completed = true;
	else
	{
		if (fabs(_t-_tf[_traj_idx])<EPS3)
		{
			_traj_idx += 1;
			_t = 0;
		}
	}
}

void JointPlanner::GenerateMotion(Vector6d& jpos, Vector6d& jvel, Vector6d& jacc)
{
	RobotTools::JAVP javp;
	//generate joint position
	if (_jplanned[_traj_idx].size()>0)
	{
		for (int jidx : _jplanned[_traj_idx])
		{
			javp = _segplanner[_traj_idx][jidx]->GenerateMotion(_t);
			jpos(jidx) = javp.pos;
			jvel(jidx) = javp.vel;
			jacc(jidx) = javp.acc;
			_t += g_cycle_time;
			MathTools::LimitMax(_tf[_traj_idx], _t);
		}
	}
	if (_junplanned[_traj_idx].size()>0)
	{
		for (int jidx : _junplanned[_traj_idx])
		{
			jpos(jidx) = _jpos[_traj_idx](jidx);
			jvel(jidx) = 0;
			jacc(jidx) = 0;
		}
	}
	//logical judgment about trajectory index and plan status
	if ((_traj_idx==_ntraj-1)&&(fabs(_t-_tf[_traj_idx])<EPS3))
		_plan_completed = true;
	else
	{
		if (fabs(_t-_tf[_traj_idx])<EPS3)
		{
			_traj_idx += 1;
			_t = 0;
		}
	}

}

void JointPlanner::Reset(Vector6d q0, bool sync)
{
	ClearTmp();
	_jpos.clear();
	_jpos.push_back(q0);
	_tf.clear();
	_jplanned.clear();
	_junplanned.clear();

	_plan_completed = false;
	_t = 0;
	_sync = sync;
	_ntraj = 0;
	_traj_idx = 0;
}

void JointPlanner::ClearTmp()
{
	for (int traj_idx = 0; traj_idx<_ntraj; traj_idx++)
	{
		for (auto it = _segplanner[traj_idx].begin(); it!=_segplanner[traj_idx].end(); it++)
		{
			if (*it!=nullptr)
			{
				delete *it;
				*it = nullptr;
			}
		}
		_segplanner[traj_idx].clear();
	}
	_segplanner.clear();
}


#pragma once

#include "BaseCTrajPlanner.h"

class LineTrajPlanner : public BaseCTrajPlanner
{
public:
	LineTrajPlanner() {}

	LineTrajPlanner(Vector6d pos0, Vector6d posn, double* vmax, double* amax, double* duration, double* vel_cons, char* opt)
		:BaseCTrajPlanner(pos0, posn, vmax, amax, duration, vel_cons, opt)
	{
		if (strcmp(_option, "both")==0)
		{
			InitPosPlanner(pos0.head(3), posn.head(3), vmax[0], amax[0], duration[0], vel_cons);
			InitRotPlanner(pos0.tail(3), posn.tail(3), vmax[1], amax[1], duration[1], &(vel_cons[2]));
			_tf_pos = _pos_uplanner.GetFinalTime();
			_tf_rot = _rot_uplanner.GetFinalTime();
		}
		else if (strcmp(_option, "pos")==0)
		{
			InitPosPlanner(pos0.head(3), posn.head(3), vmax[0], amax[0], duration[0], vel_cons);
			_tf_pos = _pos_uplanner.GetFinalTime();
			_tf_rot = 0;
		}
		else if (strcmp(_option, "rot")==0)
		{
			InitRotPlanner(pos0.tail(3), posn.tail(3), vmax[1], amax[1], duration[1], &(vel_cons[2]));
			_tf_rot = _rot_uplanner.GetFinalTime();
			_tf_pos = 0;
		}
	}

	LineTrajPlanner(Vector6d pos0, Vector6d posn, double vmax, double amax, double duration, double* vel_cons, char* opt)
		:BaseCTrajPlanner(pos0, posn, vmax, amax, duration, vel_cons, opt)
	{
		if (strcmp(_option, "pos")==0)
		{
			InitPosPlanner(pos0.head(3), posn.head(3), vmax, amax, duration, vel_cons);
			_tf_pos = _pos_uplanner.GetFinalTime();
			_tf_rot = 0;
		}
		else if (strcmp(_option, "rot")==0)
		{
			InitRotPlanner(pos0.tail(3), posn.tail(3), vmax, amax, duration, vel_cons);
			_tf_rot = _rot_uplanner.GetFinalTime();
			_tf_pos = 0;
		}
	}

	void InitPosPlanner(Vector3d pos0, Vector3d posn, double line_vmax, double line_amax, double tf, double* vel_cons) override
	{
		_pos_len = MathTools::Norm(posn-pos0);
		_pos_dir = (posn-pos0)/_pos_len;
		if (tf<0)
			_pos_uplanner.InitPlanner(Vector2d(0, _pos_len), line_vmax, line_amax, Vector2d(0, tf), Vector2d(vel_cons[0], vel_cons[1]));
		else
			_pos_uplanner.InitPlanner(Vector2d(0, _pos_len), line_vmax, line_amax, tf);
	}

	~LineTrajPlanner() {}
};


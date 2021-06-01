#pragma once

#include "LspbTrajPlanner.h"

using namespace std;
using namespace Eigen;

class BaseCTrajPlanner
{
protected:
	Vector3d _pos_initial, _rpy_initial;
	Vector3d _pos_dir, _rot_dir;
	double _pos_len, _rot_len;
	double _tf_pos, _tf_rot;
	LspbTrajPlanner _pos_uplanner, _rot_uplanner;
	char* _option;

public:
	BaseCTrajPlanner() {}

	BaseCTrajPlanner(Vector6d pos0, Vector6d posn, double* vmax, double* amax, double* duration, double* vel_cons, char* opt)
	{
		_option = opt;
		_pos_initial = pos0.head(3);
		_rpy_initial = pos0.tail(3);
		_tf_pos = _tf_rot = 0;
		_pos_len = _rot_len = 0;
	}

	BaseCTrajPlanner(Vector6d pos0, Vector6d posn, double vmax, double amax, double duration, double* vel_cons, char* opt)
	{
		_option = opt;
		_pos_initial = pos0.head(3);
		_rpy_initial = pos0.tail(3);
		_tf_pos = _tf_rot = 0;
		_pos_len = _rot_len = 0;
	}

	Vector6d GeneratePoint(double t)
	{
		Vector6d pos_rpy;
		if (strcmp(_option, "pos")==0)
		{
			pos_rpy.head(3) = GeneratePos(t);
			pos_rpy.tail(3) = _rpy_initial;
		}
		else if (strcmp(_option, "rot")==0)
		{
			pos_rpy.head(3) = _pos_initial;
			pos_rpy.tail(3) = GenerateRot(t);
		}
		else if (strcmp(_option, "both")==0)
		{
			pos_rpy.head(3) = GeneratePos(t);
			pos_rpy.tail(3) = GenerateRot(t);
		}

		return pos_rpy;
	}

	double GetFinalTime()
	{
		Vector2d tf(_tf_pos, _tf_rot);
		return tf.maxCoeff();
	}

	~BaseCTrajPlanner() {}

protected:
	virtual void InitPosPlanner(Vector3d pos0, Vector3d posn, double line_vmax, double line_amax, double tf, double* vel_cons) = 0;

	void InitRotPlanner(Vector3d rpy0, Vector3d rpyn, double ang_vmax, double ang_amax, double tf, double* vel_cons)
	{
		_rot_len = MathTools::Norm(rpyn-rpy0);
		_rot_dir = (rpyn-rpy0)/_rot_len;
		Vector2d vp(0, _rot_len), velcons(vel_cons[0], vel_cons[1]);
		if (tf<0)
			_rot_uplanner.InitPlanner(vp, ang_vmax, ang_amax, Vector2d(0, tf), velcons);
		else
			_rot_uplanner.InitPlanner(vp, ang_vmax, ang_amax, tf);
	}

	Vector3d GeneratePos(double t)
	{
		Vector3d pos;
		double up;
		if (t>_tf_pos)
			up = _pos_len;
		else
		{
			RobotTools::JAVP uavp = _pos_uplanner.GenerateMotion(t);
			up = uavp.pos;
		}
		pos = _pos_initial+up*_pos_dir;

		return pos;
	}

	Vector3d GenerateRot(double t)
	{
		Vector3d rpy;
		double up;
		if (t>_tf_rot)
			up = _rot_len;
		else
		{
			RobotTools::JAVP uavp = _rot_uplanner.GenerateMotion(t);
			up = uavp.pos;
		}
		rpy = _rpy_initial+up*_rot_dir;

		return rpy;
	}

};


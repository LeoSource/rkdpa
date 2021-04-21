#include "stdafx.h"
#include "LspbTrajPlanner.h"


LspbTrajPlanner::LspbTrajPlanner(VectorXd pos, double tf, double max_vel, double max_acc, char* option)
{
	InitPlanner(pos, tf, max_vel, max_acc, option);
}

void LspbTrajPlanner::InitPlanner(VectorXd pos, double tf, double max_vel, double max_acc, char* option)
{
	_tf = tf;
	_pos = pos;
	_np = pos.size();
	_max_vel = fabs(max_vel)*MathTools::Sign(pos(1)-pos(0));
	_max_acc = fabs(max_acc)*MathTools::Sign(pos(1)-pos(0));
	_max_jerk = 100;
	if (strcmp(option, "limitacc")==0)
	{
		double tmp_acc = 4*fabs(pos(1)-pos(0))/pow(tf, 2);

		assert(fabs(max_acc)>=tmp_acc);

		double tc_tmp = (pow(tf, 2)*_max_acc-4*(pos(1)-pos(0)))/_max_acc;
		MathTools::LimitMin(0, tc_tmp);
		_tc = tf*0.5-0.5*sqrt(tc_tmp);
	}
	else if (strcmp(option, "limitvel")==0)
	{
		double low_value = fabs(pos(1)-pos(0))/tf;
		double up_value = 2*low_value;

		assert(fabs(max_vel) > low_value);

		if ((fabs(max_vel) > low_value)&&(fabs(max_vel) < up_value))
		{
			_uniform_vel = true;
			_tc = (pos(0)-pos(1)+_max_vel*tf)/_max_vel;
			_max_acc = pow(_max_vel, 2)/(pos(0)-pos(1)+_max_vel*tf);
		}
		else if (fabs(max_vel)>=up_value)
		{
			_uniform_vel = false;
			_tc = 0.5*tf;
			_max_vel = 2*(pos(1)-pos(0))/tf;
			_max_acc = _max_vel/_tc;
		}
	}
}

RobotTools::JAVP LspbTrajPlanner::GenerateMotion(double t)
{
	RobotTools::JAVP avp;
	if (_uniform_vel)
	{
		if ((t>=0) && (t<=_tc))
		{
			avp.pos = _pos(0) + 0.5*_max_acc*pow(t, 2);
			avp.vel = _max_acc*t;
			avp.acc = _max_acc;
		}
		else if ((t>_tc) && (t<=(_tf-_tc)))
		{
			avp.pos = _pos(0) + _max_acc*_tc*(t - 0.5*_tc);
			avp.vel = _max_acc*_tc;
			avp.acc = 0;
		}
		else if ((t>(_tf-_tc)) && (t<=_tf))
		{
			avp.pos = _pos(1) - 0.5*_max_acc*pow((_tf - t), 2);
			avp.vel = _max_acc*(_tf - t);
			avp.acc = -_max_acc;
		}
	}
	else
	{
		if ((t>=0) && (t<=_tc))
		{
			avp.pos = _pos(0) + 0.5*_max_acc*pow(t, 2);
			avp.vel = _max_acc*t;
			avp.acc = _max_acc;
		}
		else if ((t>_tc) && (t<=_tf))
		{
			avp.pos = _pos(1) - 0.5*_max_vel*pow((_tf - t), 2) / (_tf - _tc);
			avp.vel = -_max_vel / (_tf - _tc)*(t - _tf);
			avp.acc = -_max_acc;
		}
	}

	return avp;
}


LspbTrajPlanner::~LspbTrajPlanner()
{
}

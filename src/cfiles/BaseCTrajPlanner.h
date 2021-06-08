/**
* @file		BaseCTrajPlanner.h
* @brief	Cartesian trajectory plan interface
* @version	1.0.0
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2021/6/1
**/
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
	double _tf_pos, _tf_rot, _tf;
	LspbTrajPlanner _pos_uplanner, _rot_uplanner;
	char* _option;

public:
	BaseCTrajPlanner() {}

	/**
	* @brief	constructor
	* @author	zxliao
	* @date		2021/6/1
	* @param	pos0		initial position and rpy
	* @param	posn		final position and rpy
	* @param	vmax		spatial velocity constraint
	* @param	amax		spatial acceleration constraint
	* @param	vel_cons	initial and final spatial velocity
	* @param	opt			option of planner
	**/
	BaseCTrajPlanner(Vector6d pos0, Vector6d posn, double* vmax, double* amax, double* vel_cons, char* opt)
	{
		_option = opt;
		_pos_initial = pos0.head(3);
		_rpy_initial = pos0.tail(3);
		_tf_pos = _tf_rot = _tf = 0;
		_pos_len = _rot_len = 0;
	}

	/**
	* @brief	constructor
	* @author	zxliao
	* @date		2021/6/1
	* @param	pos0		initial position and rpy
	* @param	posn		final position and rpy
	* @param	vmax		line or angular velocity constraint
	* @param	amax		line or angular acceleration constraint
	* @param	vel_cons	initial and final line or angular velocity
	* @param	opt			option of planner
	**/
	BaseCTrajPlanner(Vector6d pos0, Vector6d posn, double vmax, double amax, double* vel_cons, char* opt)
	{
		_option = opt;
		_pos_initial = pos0.head(3);
		_rpy_initial = pos0.tail(3);
		_tf_pos = _tf_rot = _tf = 0;
		_pos_len = _rot_len = 0;
	}

	/**
	* @brief	generate Cartesian motion data
	* @author	zxliao
	* @date		2021/6/3
	* @param	t		time
	* @return	pos_rpy	position, rpy, spatial velocity and acceleration
	**/
	RobotTools::CAVP GenerateMotion(double t)
	{
		RobotTools::CLineAVP line_avp, rpy_avp;
		if (strcmp(_option, "pos")==0)
		{
			line_avp = GeneratePosMotion(t);
			rpy_avp.pos = _rpy_initial;
			rpy_avp.vel = Vector3d::Zero();
			rpy_avp.acc = Vector3d::Zero();
		}
		else if (strcmp(_option, "rot")==0)
		{
			rpy_avp = GenerateRotMotion(t);
			line_avp.pos = _pos_initial;
			line_avp.vel = Vector3d::Zero();
			line_avp.acc = Vector3d::Zero();
		}
		else if (strcmp(_option, "both")==0)
		{
			line_avp = GeneratePosMotion(t);
			rpy_avp = GenerateRotMotion(t);
		}

		return toSpatial(&line_avp, &rpy_avp);
	}

	/**
	* @brief	generate position
	* @author	zxliao
	* @date		2021/6/1
	* @param	t		time
	* @return	pos_rpy	position and rpy
	**/
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

	/**
	* @brief	get final time
	* @author	zxliao
	* @date		2021/6/1
	* @return	tf		final time of planner
	**/
	double GetFinalTime()
	{
		return _tf;
	}

	~BaseCTrajPlanner() {}

protected:
	virtual void InitPosPlanner(Vector3d pos0, Vector3d posn, double line_vmax, double line_amax, double tf, double* vel_cons) = 0;

	virtual void InitPosPlanner(Vector3d pos1, Vector3d pos2, Vector3d pos3, double vmax, double amax, double tf, double* vel_cons) = 0;

	virtual RobotTools::CLineAVP GeneratePosMotion(double t) = 0;

	/**
	* @brief	initialize rotation planner
	* @author	zxliao
	* @date		2021/6/1
	* @param	pos0		initial rpy
	* @param	posn		final rpy
	* @param	line_vmax	angular velocity constraint
	* @param	line_amax	angular acceleration constraint
	* @param	tf			final time
	* @param	vel_cons	initial and final angular velocity
	**/
	void InitRotPlanner(Vector3d rpy0, Vector3d rpyn, double ang_vmax, double ang_amax, double tf, double* vel_cons)
	{
		_rot_len = MathTools::Norm(rpyn-rpy0);
		_rot_dir = (rpyn-rpy0)/_rot_len;
		Vector2d vp(0, _rot_len), velcons(vel_cons[0], vel_cons[1]);
		if (tf<0)
			_rot_uplanner.InitPlanner(vp, ang_vmax, ang_amax, Vector2d(0, -1), velcons);
		else
			_rot_uplanner.InitPlanner(vp, ang_vmax, ang_amax, Vector2d(0, tf), velcons);
	}

	/**
	* @brief	generate positin
	* @author	zxliao
	* @date		2021/6/1
	* @param	t	time
	* @return	pos	Cartesian position
	**/
	Vector3d GeneratePos(double t)
	{
		Vector3d pos;
		double up;
		if (t>_tf)
			up = _pos_len;
		else
		{
			RobotTools::JAVP uavp = _pos_uplanner.GenerateMotion(t);
			up = uavp.pos;
		}
		pos = _pos_initial+up*_pos_dir;

		return pos;
	}

	/**
	* @brief	generate rotation
	* @author	zxliao
	* @date		2021/6/1
	* @param	t	time
	* @return	rpy	roll, pitch, yaw angle
	**/
	Vector3d GenerateRot(double t)
	{
		Vector3d rpy;
		double up;
		if (t>_tf)
			up = _rot_len;
		else
		{
			RobotTools::JAVP uavp = _rot_uplanner.GenerateMotion(t);
			up = uavp.pos;
		}
		rpy = _rpy_initial+up*_rot_dir;

		return rpy;
	}

	/**
	* @brief	generate rpy motion data
	* @author	zxliao
	* @date		2021/6/3
	* @param	t		time
	* @return	rpy_avp	rpy, angular velocity and acceleration
	**/
	RobotTools::CLineAVP GenerateRotMotion(double t)
	{
		RobotTools::CLineAVP rpy_avp;
		RobotTools::JAVP uavp;
		if (t>_tf)
		{
			uavp.pos = _rot_len;
			uavp.vel = 0;
			uavp.acc = 0;
		}
		else
		{
			uavp = _rot_uplanner.GenerateMotion(t);
		}
		rpy_avp.pos = _rpy_initial+uavp.pos*_rot_dir;
		rpy_avp.vel = uavp.vel*_rot_dir;
		rpy_avp.acc = uavp.acc*_rot_dir;

		return rpy_avp;
	}



};


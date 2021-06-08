/**
* @file		ArcTrajPlanner.h
* @brief	Cartesian arc trajectory plan
* @version	1.0.0
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2021/6/3
**/
#pragma once

#include "BaseCTrajPlanner.h"

class ArcTrajPlanner : public BaseCTrajPlanner
{
private:
	Vector3d _center;
	double _radius;
	double _theta;
	Matrix3d _rot;

public:
	ArcTrajPlanner() {}

	/**
	* @brief	constructor
	* @author	zxliao
	* @date		2021/6/3
	* @param	pos0		initial position and rpy
	* @param	posn		final position and rpy
	* @param	vmax		spatial velocity constraint
	* @param	amax		spatial acceleration constraint
	* @param	duration	initial and final time
	* @param	vel_cons	initial and final spatial velocity
	* @param	opt			option of planner
	**/
	ArcTrajPlanner(Vector6d pos1, Vector3d pos2, Vector6d pos3, double* vmax, double* amax, double* vel_cons, char* opt)
		:BaseCTrajPlanner(pos1, pos3, vmax, amax, vel_cons, opt)
	{
		if (strcmp(_option, "both")==0)
		{
			InitPosPlanner(pos1.head(3), pos2,pos3.head(3), vmax[0], amax[0], -1, vel_cons);
			InitRotPlanner(pos1.tail(3), pos3.tail(3), vmax[1], amax[1], -1, &(vel_cons[2]));
			_tf_pos = _pos_uplanner.GetFinalTime();
			_tf_rot = _rot_uplanner.GetFinalTime();
			Vector2d tf(_tf_pos, _tf_rot);
			_tf = tf.maxCoeff();
			if (_tf_pos>_tf_rot)
				InitRotPlanner(pos1.tail(3), pos3.tail(3), vmax[1], amax[1], _tf, &(vel_cons[2]));
			else
				InitPosPlanner(pos1.head(3), pos2,pos3.head(3), vmax[0], amax[0], _tf, vel_cons);
		}
		else if (strcmp(_option, "pos")==0)
		{
			InitPosPlanner(pos1.head(3), pos2,pos3.head(3), vmax[0], amax[0], -1, vel_cons);
			_tf_pos = _pos_uplanner.GetFinalTime();
			_tf = _tf_pos;
			_tf_rot = 0;
		}
		else if (strcmp(_option, "rot")==0)
		{
			InitRotPlanner(pos1.tail(3), pos3.tail(3), vmax[1], amax[1], -1, &(vel_cons[2]));
			_tf_rot = _rot_uplanner.GetFinalTime();
			_tf = _tf_rot;
			_tf_pos = 0;
		}
	}

	/**
	* @brief	constructor
	* @author	zxliao
	* @date		2021/6/1
	* @param	pos0		initial position and rpy
	* @param	posn		final position and rpy
	* @param	vmax		line or angular velocity constraint
	* @param	amax		line or angular acceleration constraint
	* @param	duration	initial and final time
	* @param	vel_cons	initial and final line or angular velocity
	* @param	opt			option of planner
	**/
	ArcTrajPlanner(Vector6d pos1,Vector3d pos2, Vector6d pos3, double vmax, double amax, double* vel_cons, char* opt)
		:BaseCTrajPlanner(pos1, pos3, vmax, amax, vel_cons, opt)
	{
		if (strcmp(_option, "pos")==0)
		{
			InitPosPlanner(pos1.head(3), pos2,pos3.head(3), vmax, amax, -1, vel_cons);
			_tf_pos = _pos_uplanner.GetFinalTime();
			_tf = _tf_pos;
			_tf_rot = 0;
		}
		else if (strcmp(_option, "rot")==0)
		{
			InitRotPlanner(pos1.tail(3), pos3.tail(3), vmax, amax, -1, vel_cons);
			_tf_rot = _rot_uplanner.GetFinalTime();
			_tf = _tf_rot;
			_tf_pos = 0;
		}
	}


	~ArcTrajPlanner() {}

private:
	/**
	* @brief	initialize position planner
	* @author	zxliao
	* @date		2021/6/1
	* @param	pos0		initial position
	* @param	posn		final position
	* @param	line_vmax	line velocity constraint
	* @param	line_amax	line acceleration constraint
	* @param	tf			final time
	* @param	vel_cons	initial and final line velocity
	**/
	void InitPosPlanner(Vector3d pos1, Vector3d pos2, Vector3d pos3, double line_vmax, double line_amax, double tf, double* vel_cons) override
	{
		Vector3d p2p1 = pos1-pos2;
		Vector3d p2p3 = pos3-pos2;
		double inc_angle = acos(p2p1.dot(p2p3)/p2p1.norm()/p2p3.norm());
		_theta = pi-inc_angle;
		_radius = p2p1.norm()*tan(0.5*inc_angle);
		Vector3d pc = 0.5*(pos1+pos3);
		Vector3d p2pc = pc-pos2;
		double scale = _radius/sin(0.5*inc_angle)/p2pc.norm();
		Vector3d p2center = scale*p2pc;
		_center = pos2+p2center;
		_pos_len = _radius*_theta;

		if (tf<0)
			_pos_uplanner.InitPlanner(Vector2d(0, _pos_len), line_vmax, line_amax, Vector2d(0, -1), Vector2d(vel_cons[0], vel_cons[1]));
		else
			_pos_uplanner.InitPlanner(Vector2d(0, _pos_len), line_vmax, line_amax, Vector2d(0, tf), Vector2d(vel_cons[0], vel_cons[1]));

		Vector3d n = (pos1-_center)/MathTools::Norm(pos1-_center);
		Vector4d tmp_a = PointsCoplane(pos1, pos2, pos3);
		Vector3d a = tmp_a.head(3);
		a.normalize();
		Vector3d o = a.cross(n);
		_rot<<n, o, a;

	}

	Vector4d PointsCoplane(Vector3d pos1, Vector3d pos2, Vector3d pos3)
	{
		double x1 = pos1(0), y1 = pos1(1), z1 = pos1(2);
		double x2 = pos2(0), y2 = pos2(1), z2 = pos2(2);
		double x3 = pos3(0), y3 = pos3(1), z3 = pos3(2);
		double a = y1*z2-y2*z1-y1*z3+y3*z1+y2*z3-y3*z2;
		double b = -(x1*z2-x2*z1-x1*z3+x3*z1+x2*z3-x3*z2);
		double c = x1*y2-x2*y1-x1*y3+x3*y1+x2*y3-x3*y2;
		double d = -(x1*y2*z3-x1*y3*z2-x2*y1*z3+x2*y3*z1+x3*y1*z2-x3*y2*z1);
		
		return Vector4d(a, b, c, d);
	}

	void InitPosPlanner(Vector3d pos0, Vector3d posn, double line_vmax, double line_amax, double tf, double* vel_cons) override
	{}

	/**
	* @brief	generate position motion data
	* @author	zxliao
	* @date		2021/6/3
	* @param	t		time
	* @return	line_avp	position, line velocity and acceleration
	**/
	RobotTools::CLineAVP GeneratePosMotion(double t) override
	{
		RobotTools::CLineAVP line_avp;
		RobotTools::JAVP uavp;
		if (t>_tf)
		{
			uavp.pos = _pos_len;
			uavp.vel = 0;
			uavp.acc = 0;
		}
		else
		{
			uavp = _pos_uplanner.GenerateMotion(t);
		}
		double r = _radius;
		double th = uavp.pos/r;
		Vector3d parc = Vector3d::Zero();
		parc(0) = r*cos(th);
		parc(1) = r*sin(th);
		line_avp.pos = _center+_rot*parc;
		Vector3d vc = Vector3d::Zero();
		Vector3d ac = Vector3d::Zero();
		vc(0) = -uavp.vel*sin(th);
		vc(1) = uavp.vel*cos(th);
		line_avp.vel = _rot*vc;
		ac(0) = -pow(uavp.vel, 2)*cos(th)/r-uavp.acc*sin(th);
		ac(1) = -pow(uavp.vel, 2)*sin(th)/r+uavp.acc*cos(th);
		line_avp.acc = _rot*ac;

		return line_avp;
	}


};



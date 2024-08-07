#include "CleanRobot.h"


CleanRobot::CleanRobot(MatrixXd mdh_table, Matrix<int, Dynamic, 1> type, VectorXd offset):
	_mdh_table(mdh_table),_type(type),_offset(offset)
{
	_nlinks = static_cast<int>(type.size());
	_tool.pos = Vector3d::Zero();
	_tool.rot = Matrix3d::Identity();
	for (int idx = 0; idx<_nlinks; idx++)
	{
		double theta = mdh_table(idx, 0);
		double d = mdh_table(idx, 1);
		double a = mdh_table(idx, 2);
		double alpha = mdh_table(idx, 3);
		MDHLink tmp_link = MDHLink(theta, d, a, alpha, type(idx), offset(idx));
		_links.push_back(tmp_link);

		_qlimit.setZero(_nlinks, 2);
		if (_links[idx]._type==e_rotation)
			_qlimit.row(idx)<<-2*pi, 2*pi;
		else
			_qlimit.row(idx)<<-2, 2;
	}
	_hold_jpos = MatrixXd::Zero(5, 1);
}

CleanRobot::CleanRobot(MatrixXd mdh_table, Matrix<int, Dynamic, 1> type, VectorXd offset, Pose tool):
	_mdh_table(mdh_table),_type(type),_offset(offset),_tool(tool)
{
	_nlinks = static_cast<int>(type.size());
	for (int idx = 0; idx<_nlinks; idx++)
	{
		double theta = mdh_table(idx, 0);
		double d = mdh_table(idx, 1);
		double a = mdh_table(idx, 2);
		double alpha = mdh_table(idx, 3);
		MDHLink tmp_link = MDHLink(theta, d, a, alpha, type(idx), offset(idx));
		_links.push_back(tmp_link);

		_qlimit.setZero(_nlinks, 2);
		if (_links[idx]._type==e_rotation)
			_qlimit.row(idx)<<-2*pi, 2*pi;
		else
			_qlimit.row(idx)<<-2, 2;
	}
	_hold_jpos = MatrixXd::Zero(5, 1);
}

CleanRobot::CleanRobot(MatrixXd mdh_table, Matrix<int, Dynamic, 1> type, VectorXd offset, double tool_pitch, Vector3d tool_pos):
	_mdh_table(mdh_table),_type(type),_offset(offset),_tool_pitch(tool_pitch)
{
	_nlinks = static_cast<int>(type.size());
	_tool.pos = tool_pos;
	_tool.rot = RotX(tool_pitch);
	for (int idx = 0; idx<_nlinks; idx++)
	{
		double theta = mdh_table(idx, 0);
		double d = mdh_table(idx, 1);
		double a = mdh_table(idx, 2);
		double alpha = mdh_table(idx, 3);
		MDHLink tmp_link = MDHLink(theta, d, a, alpha, type(idx), offset(idx));
		_links.push_back(tmp_link);

		_qlimit.setZero(_nlinks, 2);
		if (_links[idx]._type==e_rotation)
			_qlimit.row(idx)<<-2*pi, 2*pi;
		else
			_qlimit.row(idx)<<-2, 2;
	}
	_hold_jpos = MatrixXd::Zero(5, 1);
}

void CleanRobot::UpdateTool(double tool_pitch, Vector3d tool_pos)
{
	_tool_pitch = tool_pitch;
	
	_tool.pos = tool_pos;
	_tool.rot = RotX(tool_pitch);
}

void CleanRobot::SetJntLimit(MatrixXd qlimit)
{
	_qlimit = qlimit;
}

void CleanRobot::SetPitchRange(double pitch_high, double pitch_low)
{
	_pitch_high = pitch_high;
	_pitch_low = pitch_low;
}

Pose CleanRobot::Transform(VectorXd q, int s_idx, int e_idx)
{
	Pose pose;
	pose.pos.setZero();
	pose.rot.setIdentity();
	for (int idx = s_idx; idx<e_idx; idx++)
	{
		_links[idx].Transform(q(idx));
		pose = RobotTools::PoseProduct(pose, _links[idx]._pose);
	}

	return pose;
}

Pose CleanRobot::FKSolve(VectorXd q)
{
	Pose res;
	res = Transform(q, 0, _nlinks);

	return res;
}

Pose CleanRobot::FKSolveTool(VectorXd q)
{
	Pose res;
	res = Transform(q, 0, _nlinks);
	res = PoseProduct(res, _tool);

	return res;
}

VectorXd CleanRobot::IKSolvePitch(Vector3d pos, double pitch)
{
	VectorXd q(_nlinks);
	q.setZero();
	q(2) = pitch-_tool_pitch;
	double ty = _tool.pos(1), tz = _tool.pos(2);
	double l4 = -_mdh_table(4, 1);
	double s1 = sin(q(0)), c1 = cos(q(0));
	double s3 = sin(q(2)), c3 = cos(q(2));
	double s5 = sin(q(4)), c5 = cos(q(4));
	double h = _offset(1), w = _offset(3);
	double lx = _mdh_table(2, 1)+_mdh_table(4, 2), ly = _mdh_table(2, 2);
	double tmp_value = pos(1)-ty*(-s1*s5+c1*c3*c5)+c1*s3*tz-ly*c1-lx*s1-c1*s3*l4;
	q(3) = tmp_value/(c1*c3)-w;
	q(1) = pos(2)-s3*c5*ty-c3*tz-s3*(q(3)+w)-h+c3*l4;
	MathTools::LimitVector(_qlimit.col(0), &q, _qlimit.col(1));

	return q;
}

VectorXd CleanRobot::IKSolvePitchYaw(Vector3d pos, double pitch, double yaw, VectorXd q_in)
{
	VectorXd q(_nlinks);
	q.setZero();
	q(2) = pitch-_tool_pitch;
	double ty = _tool.pos(1), tz = _tool.pos(2);
	double l4 = -_mdh_table(4, 1);
	double a = pos(0)+sin(yaw)*ty;
	double b = pos(1)-cos(yaw)*ty;
	double c = _mdh_table(2, 1)+_mdh_table(4, 2);
	q(0) = MathTools::CalcTransEqua(a, b, c, q_in(0));
	q(4) = yaw-q(0);

	double s1 = sin(q(0)), c1 = cos(q(0));
	double s3 = sin(q(2)), c3 = cos(q(2));
	double s5 = sin(q(4)), c5 = cos(q(4));
	double h = _offset(1), w = _offset(3);
	double lx = _mdh_table(2, 1)+_mdh_table(4, 2), ly = _mdh_table(2, 2);
	double tmp_value = pos(1)-ty*(-s1*s5+c1*c3*c5)+c1*s3*tz-ly*c1-lx*s1-c1*s3*l4;
	q(3) = tmp_value/(c1*c3)-w;
	q(1) = pos(2)-s3*c5*ty-c3*tz-s3*(q(3)+w)-h+c3*l4;
	MathTools::LimitVector(_qlimit.col(0), &q, _qlimit.col(1));

	return q;
}

VectorXd CleanRobot::IKSolve(Vector3d pos, char* option, double alpha, VectorXd q_in)
{
	VectorXd q(_nlinks);
	if (strcmp(option, "q3first0")==0)
		q = IKJnt3(pos, alpha, 0, q_in);
	else if (strcmp(option, "q3firstn")==0)
		q = IKJnt3(pos, alpha, -pi/6, q_in);
	else if (strcmp(option, "q2first")==0)
		q = IKJnt2(pos, alpha, q_in);

	MathTools::LimitVector(_qlimit.col(0), &q, _qlimit.col(1));
	return q;
}


VectorXd CleanRobot::IKJnt3(Vector3d pos, double alpha, double q3, VectorXd q_in)
{
	VectorXd q(_nlinks);
	double ty = _tool.pos(1), tz = _tool.pos(2);
	double l4 = -_mdh_table(4, 1);
	q(2) = q3;
	double a = pos(0)+sin(alpha)*ty;
	double b = pos(1)-cos(alpha)*ty;
	double c = _mdh_table(2, 1)+_mdh_table(4, 2);
	q(0) = MathTools::CalcTransEqua(a, b, c, q_in(0));
	q(4) = alpha-q(0);

	double s1 = sin(q(0)), c1 = cos(q(0));
	double s3 = sin(q(2)), c3 = cos(q(2));
	double s5 = sin(q(4)), c5 = cos(q(4));
	double h = _offset(1), w = _offset(3);
	double lx = _mdh_table(2, 1)+_mdh_table(4, 2), ly = _mdh_table(2, 2);
	double tmp_value = pos(1)-ty*(-s1*s5+c1*c3*c5)+c1*s3*tz-ly*c1-lx*s1-c1*s3*l4;
	q(3) = tmp_value/(c1*c3)-w;
	q(1) = pos(2)-s3*c5*ty-c3*tz-s3*(q(3)+w)-h+c3*l4;

	return q;
}

VectorXd CleanRobot::IKJnt2(Vector3d pos, double alpha, VectorXd q_in)
{
	VectorXd q(_nlinks);
	double ty = _tool.pos(1), tz = _tool.pos(2);
	double h = _offset(1), w = _offset(3), l4 = -_mdh_table(4, 1);
	Vector2d height_limit(_qlimit(1, 0)+tz+h-l4, _qlimit(1, 1)+tz+h-l4);
	if (pos(2)>height_limit(1))
		q(1) = _qlimit(1, 1);
	else if (pos(2)<height_limit(0))
		q(1) = _qlimit(1, 0);
	else
		q(1) = pos(2)+l4-tz-h;

	double a = pos(0)+sin(alpha)*ty;
	double b = pos(1)-cos(alpha)*ty;
	double c = _mdh_table(2, 1)+_mdh_table(4, 2);
	q(0) = MathTools::CalcTransEqua(a, b, c, q_in(0));
	q(4) = alpha-q(0);

	double s1 = sin(q(0)), c1 = cos(q(0));
	double s5 = sin(q(4)), c5 = cos(q(4));
	double lx = _mdh_table(2, 1)+_mdh_table(4, 2), ly = _mdh_table(2, 2);
	a = c1*(pos(2)-q(1)-h);
	b = -pos(1)-s1*s5*ty+c1*ly+s1*lx;
	c = c1*tz-c1*l4;
	q(2) = MathTools::CalcTransEqua(a, b, c, q_in(2));
	double s3 = sin(q(2)), c3 = cos(q(2));
	double tmp_value = pos(1)-ty*(-s1*s5+c1*c3*c5)+c1*s3*tz-ly*c1-lx*s1-c1*s3*l4;
	q(3) = tmp_value/(c1*c3)-w;

	return q;
}

MatrixXd CleanRobot::CalcJaco(VectorXd q)
{
	MatrixXd jaco(6, _nlinks), jv(3, _nlinks), jw(3, _nlinks);
	Vector3d z0(0, 0, 1);
	Vector3d pt = FKSolve(q).pos;
	Pose tmp_p = Transform(q, 0, 0);
	for (int idx = 0; idx<_nlinks; idx++)
	{
		Pose tmp_pose = Transform(q, 0, idx+1);
		Matrix3d rot_0_t = tmp_pose.rot;
		Vector3d pos_0_t = tmp_pose.pos;
		if (_type(idx)==e_rotation)
		{
			jw.col(idx) = rot_0_t*z0;
			jv.col(idx) = MathTools::Cross(jw.col(idx), pt-pos_0_t);
		}
		else
		{

			jw.col(idx).setZero();
			jv.col(idx) = rot_0_t*z0;
		}
		
	}
	jaco<<jv, jw;

	return jaco;
}

MatrixXd CleanRobot::CalcJv(VectorXd q)
{
	MatrixXd jaco(6, _nlinks), jv(3, _nlinks);
	jaco = CalcJaco(q);
	jv = jaco.topRows(3);

	return jv;
}

MatrixXd CleanRobot::CalcJw(VectorXd q)
{
	MatrixXd jaco(6, _nlinks), jw(3, _nlinks);
	jaco = CalcJaco(q);
	jw = jaco.bottomRows(3);

	return jw;
}


void CleanRobot::UpdateJntHoldPos(VectorXd q)
{
	_hold_jpos = q;
}

VectorXd CleanRobot::HoldJntPos()
{
	return _hold_jpos;
}

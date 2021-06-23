#include "UrRobot.h"

UrRobot::UrRobot(MatrixXd mdh_table, Matrix<int, Dynamic, 1> type, VectorXd offset, Pose tool) :
	_mdh_table(mdh_table), _type(type), _offset(offset), _tool(tool)
{
	_nlinks = type.size();
	for (int idx = 0; idx<_nlinks; idx++)
	{
		double theta = mdh_table(idx, 0);
		double d = mdh_table(idx, 1);
		double a = mdh_table(idx, 2);
		double alpha = mdh_table(idx, 3);
		MDHLink tmp_link = MDHLink(theta, d, a, alpha, type(idx), offset(idx));
		_links.push_back(tmp_link);

		_qlimit.setZero(_nlinks, 2);
		if (_links[idx]._type == e_rotation)
			_qlimit.row(idx) << -2 * pi, 2 * pi;
		else
			_qlimit.row(idx) << -2, 2;
	}
	_hold_jpos = MatrixXd::Zero(6, 1);
}

void UrRobot::SetJntLimit(MatrixXd qlimit)
{
	_qlimit = qlimit;
}

Pose UrRobot::Transform(VectorXd q, int s_idx, int e_idx)
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

Pose UrRobot::FKSolve(VectorXd q)
{
	Pose res;
	res = Transform(q, 0, _nlinks);

	return res;
}

Pose UrRobot::FKSolveTool(VectorXd q)
{
	Pose res;
	res = Transform(q, 0, _nlinks);
	res = PoseProduct(res, _tool);

	return res;
}


int UrRobot::IKSolve(Vector6d * q_out, Pose * pose, Vector6d * q_old)
{
	int error = 0;
	double s1 = 0, c1 = 0, s2 = 0, c2 = 0, s3 = 0, c3 = 0, s4 = 0, c4 = 0, s5 = 0, c5 = 0, s6 = 0,c6 = 0;
	double q1_1 = 0, q1_2 = 0, q5_1 = 0, q5_2 = 0, q6_1 = 0,q6_2 = 0, q6_3 = 0,q6_4 = 0,q3_1 = 0,q3_2= 0;
	VectorXd q(_nlinks),q_in(_nlinks);
	q.setZero();
	q_in = *q_old;
	double a3 = _mdh_table(2, 2);
	double a4 = _mdh_table(3, 2);
	double d1 = _mdh_table(0, 1);
	double d4 = _mdh_table(3, 1);
	double d5 = _mdh_table(4, 1);
	double d6 = _mdh_table(5, 1);
	Pose inv_tool;
	Pose mi;
	inv_tool.rot = _tool.rot.transpose();
	inv_tool.pos = -_tool.rot.transpose()*_tool.pos;
	//calc pose of mounting interface
	mi.pos = pose->rot*inv_tool.pos + pose->pos;
	mi.rot = pose->rot*inv_tool.rot;

	/* joint1 */
	double a1 = d6 * mi.rot(0, 2) - mi.pos(0);
	double b1 = mi.pos(1) - d6 * mi.rot(1, 2);
	double cc1 = -d4;
	int n;
	n = MathTools::CalcSinCosEqua(&q1_1, &q1_2, a1, b1, cc1);
	if (n <= 0)
	{
		error = arm_singularity;
		q(0) = q_in(0);
	}
	else
	{
		if (fabs(q1_1 - q_in(0)) < fabs(q1_2 - q_in(0)))
			q(0) = q1_1;
		else
			q(0) = q1_2;
	}
	s1 = sin(q(0)); c1 = cos(q(0));
	(*q_out)(0) = q(0);
	/* joint5 */
	double temp5 = mi.rot(1, 2)*c1 - mi.rot(0, 2)*s1;
	q5_1 = acos(temp5);
	q5_2 = -acos(temp5);
	if (fabs(q5_1 - q_in(4)) < fabs(q5_2 - q_in(4)))
		q(4) = q5_1;
	else
		q(4) = q5_2;
	s5 = sin(q(4)); c5 = cos(q(4));
	(*q_out)(4) = q(4);
	/* joint6 */
	int i;
	if (abs(s5) < EPS3)
	{
		q(5) = q_in(5);
		error = wrist_singularity;
	}
	else
	{
		double b6 = mi.rot(0, 0)*s1 - mi.rot(1, 0)*c1;
		double a6 = mi.rot(1, 1)*c1 - mi.rot(0, 1)*s1;
		n = MathTools::CalcSinCosEqua(&q6_1, &q6_2, a6, b6, -s5);
		double q6_temp[4];
		if (q6_1 > 0)
			q6_temp[2] = q6_1 - 2 * pi;
		else
			q6_temp[2] = q6_1 + 2 * pi;
		if (q6_2 > 0)
			q6_temp[3] = q6_2 - 2 * pi;
		else
			q6_temp[3] = q6_2 + 2 * pi;
		q6_temp[0] = q6_1; q6_temp[1] = q6_2;
		double temp_max = 10000;
		double temp_value = 0;
		for (i = 0; i < 4; i++)
		{
			temp_value = fabs(q6_temp[i] - q_in(5));

			if (temp_value < temp_max)
			{
				temp_max = temp_value;
				q(5) = q6_temp[i];
			}
		}
	}
	s6 = sin(q(5)); c6 = cos(q(5));
	(*q_out)(5) = q(5);
	/* joint3 */
	double m3 = mi.pos(0)*c1 + mi.pos(1)*s1 - d6*(mi.rot(1, 2)*s1 + mi.rot(0, 2)*c1) - d5*((mi.rot(0, 0)*c1 + mi.rot(1, 0)*s1)*s6 + (mi.rot(0, 1)*c1 + mi.rot(1, 1)*s1)*c6);
	double n3 = mi.pos(2) - d1 - d6*mi.rot(2, 2) - d5*(mi.rot(2, 1)*c6 + mi.rot(2, 0)*s6);
	double temp3 = (pow(a3 ,2) + pow(a4 ,2) - pow(m3 ,2) - pow(n3 , 2)) / (2 * a3*a4);
	if (fabs(temp3) < 0.99985)
	{
		q3_1 = asin(temp3);
		q3_2 = -pi - asin(temp3);
		if (fabs(q3_1 - q_in(2)) < fabs(q3_2 - q_in(2)))
			q(2) = q3_1;
		else
			q(2) = q3_2;
	}
	else
	{
		q(2) = q_in(2);
		error = outside_reach;
	}
	s3 = sin(q(2)); c3 = cos(q(2));
	(*q_out)(2) = q(2);
	/* joint2 */
	double A = a3 - a4*s3;
	double B = a4*c3;
	s2 = (A*m3 - B*n3) / (pow(A, 2) + pow(B, 2));
	c2 = (A*n3 + B*m3) / (pow(A, 2) + pow(B, 2));
	q(1) = atan2(s2, c2);
	s2 = sin(q(1)); c2 = cos(q(1));
	(*q_out)(1) = q(1);
	/* joint 4*/
	double s23 = sin(q(1) + q(2));
	double c23 = cos(q(1) + q(2));
	double c234 = mi.rot(2, 2)*s5 + mi.rot(2, 0)*c5*c6 - mi.rot(2, 1)*c5*s6;
	double s234 = mi.rot(0, 2)*c1*s5 + mi.rot(1, 2)*s1*s5 + mi.rot(0, 0)*c1*c5*c6 + mi.rot(1, 0)*c5*c6*s1 - mi.rot(0, 1)*c1*c5*s6 - mi.rot(1, 1)*c5*s1*s6;
	q(3) = atan2(s234*c23 - c234*s23, c234*c23 + s234*s23);
	(*q_out)(3) = q(3);
	return error;
}

MatrixXd UrRobot::CalcJaco(VectorXd q)
{
	MatrixXd jaco(6, _nlinks), jv(3, _nlinks), jw(3, _nlinks);
	Vector3d z0(0, 0, 1);
	Vector3d pt = FKSolve(q).pos;
	Pose tmp_p = Transform(q, 0, 0);
	for (int idx = 0; idx<_nlinks; idx++)
	{
		Pose tmp_pose = Transform(q, 0, idx + 1);
		Matrix3d rot_0_t = tmp_pose.rot;
		Vector3d pos_0_t = tmp_pose.pos;
		if (_type(idx) == e_rotation)
		{
			jw.col(idx) = rot_0_t*z0;
			jv.col(idx) = MathTools::Cross(jw.col(idx), pt - pos_0_t);
		}
		else
		{

			jw.col(idx).setZero();
			jv.col(idx) = rot_0_t*z0;
		}

	}
	jaco << jv, jw;

	return jaco;
}

MatrixXd UrRobot::CalcJv(VectorXd q)
{
	MatrixXd jaco(6, _nlinks), jv(3, _nlinks);
	jaco = CalcJaco(q);
	jv = jaco.topRows(3);

	return jv;
}

MatrixXd UrRobot::CalcJw(VectorXd q)
{
	MatrixXd jaco(6, _nlinks), jw(3, _nlinks);
	jaco = CalcJaco(q);
	jw = jaco.bottomRows(3);

	return jw;
}


void UrRobot::UpdateJntHoldPos(VectorXd q)
{
	_hold_jpos = q;
}

VectorXd UrRobot::HoldJntPos()
{
	return _hold_jpos;
}

void UrRobot::UpdateTool(Vector3d tool_pos, Vector3d tool_rpy)
{
    RobotTools::Pose pose_tool;
    pose_tool.rot.setIdentity();
    pose_tool.rot = RotZ(tool_rpy(2)) * RotY(tool_rpy(1)) * RotX(tool_rpy(0));
    pose_tool.pos = tool_pos;

    _tool = pose_tool;
}

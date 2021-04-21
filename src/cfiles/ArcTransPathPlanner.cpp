#include "stdafx.h"
#include "ArcTransPathPlanner.h"

ArcTransPathPlanner::ArcTransPathPlanner(MatrixXd pos, double radius)
{
	_radius = radius;
	_nump = pos.cols();
	_numarc = _nump-2;
	_p_initial = pos.col(0);
	_p_goal = pos.rightCols(1);
	CalcArcInfo(pos);
}

void ArcTransPathPlanner::InitPlanner(MatrixXd pos, double radius)
{
	_radius = radius;
	_nump = pos.cols();
	_numarc = _nump-2;
	_p_initial = pos.col(0);
	_p_goal = pos.rightCols(1);
	CalcArcInfo(pos);
}

void ArcTransPathPlanner::CalcArcInfo(MatrixXd pos)
{
	_center.setZero(3, _numarc);
	_theta.setZero(_numarc);
	_pt.setZero(3, 2*_numarc);
	_dr.setZero(_numarc);
	_dl.setZero(_numarc+1);
	_line_vec.setZero(3, _numarc+1);
	for (int idx = 0; idx<_numarc; idx++)
	{
		ArcData arc_data = CalcArcPoints(pos.col(idx), pos.col(idx+1), pos.col(idx+2));
		_center.col(idx) = arc_data.center;
		_theta(idx) = arc_data.theta;
		_pt.col(2*idx) = arc_data.pt1;
		_pt.col(2*idx+1) = arc_data.pt2;
		_dr(idx) = _radius*(pi-_theta(idx));
		_dl(idx) = MathTools::Norm(arc_data.pt1-pos.col(idx));
		Matrix3d tmp_rot= CalcArcRot(arc_data.center, arc_data.pt1, arc_data.pt2);
		_rot.push_back(tmp_rot);
		if (idx!=0)
			_dl(idx) = _dl(idx)-MathTools::Norm(pos.col(idx)-_pt.col(2*idx-1));
		if (idx==_numarc-1)
			_dl(idx+1) = MathTools::Norm(pos.rightCols(1)-arc_data.pt2);
		_line_vec.col(idx) = pos.col(idx+1)-pos.col(idx);
		_line_vec.col(idx).normalize();
	}
	_line_vec.col(_numarc) = pos.col(_nump-1)-pos.col(_nump-2);
	_line_vec.col(_numarc).normalize();
	_distance = _dl.sum()+_dr.sum();
}

Matrix3d ArcTransPathPlanner::CalcArcRot(Vector3d center, Vector3d p1, Vector3d p2)
{
	Matrix3d rot;
	Vector3d px = p1-center;
	Vector3d pz = px.cross(p2-center);
	Vector3d py = pz.cross(px);
	px.normalize(); py.normalize(); pz.normalize();
	rot<<px, py, pz;

	return rot;
}

ArcData ArcTransPathPlanner::CalcArcPoints(Vector3d p1, Vector3d p2, Vector3d p3)
{
	ArcData arc_data;
	Vector3d p21 = p1-p2, p23 = p3-p2;
	double tmp_cos = p21.dot(p23)/p21.norm()/p23.norm();
	MathTools::LimitNum(-1, tmp_cos, 1);
	double theta = acos(tmp_cos);

	Vector3d p2t1 = _radius/tan(0.5*theta)/p21.norm()*p21;
	Vector3d pt1 = p2+p2t1;
	Vector3d p2t2 = _radius/tan(0.5*theta)/p23.norm()*p23;
	Vector3d pt2 = p2+p2t2;

	Vector3d pt1t2 = pt2-pt1;
	Vector3d pt1m1 = 0.5*pt1t2;
	Vector3d p2m1 = p2t1+pt1m1;
	Vector3d p2c = _radius/sin(0.5*theta)/p2m1.norm()*p2m1;
	Vector3d center = p2+p2c;

	arc_data.center = center; arc_data.pt1 = pt1; arc_data.pt2 = pt2;
	arc_data.theta = theta;
	return arc_data;
}

Vector3d ArcTransPathPlanner::GeneratePath(double varp)
{
	int n = _dl.size()+_dr.size()+1;
	VectorXd dis;
	dis.setZero(n);
	double m = dis(0);
	for (int idx = 1; idx<n; idx++)
	{
		if (idx%2==1)
			m += _dl((idx-1)/2);
		else
			m += _dr((idx-2)/2);
		dis(idx) = m;
	}
	int idx = MathTools::Discretize(dis, n, varp);
	Vector3d p;
	if (idx%2==0)
	{
		if (idx==0)
			p = _p_initial+_line_vec.col(idx)*varp;
		else
			p = _pt.col(idx-1)+_line_vec.col(idx/2)*(varp-dis(idx));
	}
	else
	{
		double th = (varp-dis(idx))/_radius;
		Vector3d parc(0, 0, 0);
		parc(0) = _radius*cos(th);
		parc(1) = _radius*sin(th);
		p = _center.col((idx-1)/2)+_rot[(idx-1)/2]*parc;
	}

	return p;
}

RobotTools::CLineAVP ArcTransPathPlanner::GenerateMotion(double varp, double varv, double vara)
{
	RobotTools::CLineAVP avp;
	Vector3d p = GeneratePath(varp);
	Vector3d v(0, 0, 0), a(0, 0, 0);
	int n = _dl.size()+_dr.size()+1;
	VectorXd dis;
	dis.setZero(n);
	double m = dis(0);
	for (int idx = 1; idx<n; idx++)
	{
		if (idx%2==1)
			m += _dl((idx-1)/2);
		else
			m += _dr((idx-2)/2);
		dis(idx) = m;
	}
	int idx = MathTools::Discretize(dis, n, varp);
	if (idx%2==0)
	{
		v = varv*_line_vec.col(idx/2);
		a = vara*_line_vec.col(idx/2);
	}
	else
	{
		double s = varp-dis(idx);
		double r = _radius;
		Vector3d vc(0, 0, 0), ac(0, 0, 0);
		vc(0) = -varv*sin(s/r);
		vc(1) = varv*cos(s/r);
		v = _rot[(idx-1)/2]*vc;
		ac(0) = -pow(varv, 2)*cos(s/r)/r-vara*sin(s/r);
		ac(1) = -pow(varv, 2)*sin(s/r)/r+vara*cos(s/r);
		a = _rot[(idx-1)/2]*ac;
	}

	avp.pos = p; avp.vel = v; avp.acc = a;
	return avp;
}

ArcTransPathPlanner::~ArcTransPathPlanner()
{
}

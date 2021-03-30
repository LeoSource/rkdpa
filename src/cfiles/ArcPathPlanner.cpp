#include "stdafx.h"
#include "ArcPathPlanner.h"


ArcPathPlanner::ArcPathPlanner(Vector3d pos1, Vector3d pos2, Vector3d pos3)
{
	Vector4d params1 = PointsCoplane(pos1, pos2, pos3);
	Vector4d params2 = RadiusEqual(pos1, pos2);
	Vector4d params3 = RadiusEqual(pos1, pos3);
	double a1 = params1(0), b1 = params1(1), c1 = params1(2), d1 = params1(3);
	double a2 = params2(0), b2 = params2(1), c2 = params2(2), d2 = params2(3);
	double a3 = params3(0), b3 = params3(1), c3 = params3(2), d3 = params3(3);
	_center(0) = -(b1*c2*d3-b1*c3*d2-b2*c1*d3+b2*c3*d1+b3*c1*d2-b3*c2*d1)/
				(a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1);
	_center(1) = (a1*c2*d3-a1*c3*d2-a2*c1*d3+a2*c3*d1+a3*c1*d2-a3*c2*d1)/
				(a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1);
	_center(2) = -(a1*b2*d3-a1*b3*d2-a2*b1*d3+a2*b3*d1+a3*b1*d2-a3*b2*d1)/
				(a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1);
	Vector3d line_vec = pos3-pos1;
	double line_length = line_vec.norm();
	Vector3d xr = pos1-_center;
	_radius = xr.norm();
	double tmp_cos = (pow(_radius, 2)*2-pow(line_length, 2))/(2*_radius*_radius);
	MathTools::LimitNum(-1, tmp_cos, 1);
	_theta = acos(tmp_cos);

	Vector3d zr(a1, b1, c1);
	Vector3d yr = zr.cross(xr);
	xr.normalize(); yr.normalize(); zr.normalize();
	_rot<<xr, yr, zr;
}

ArcPathPlanner::ArcPathPlanner(Vector3d center, Vector3d n_vec, double radius)
{
	_center = center;
	_radius = radius;
	_theta = 2*pi;

	Vector3d x0(1, 0, 0), y0(0, 1, 0);
	Vector3d zr = n_vec;
	Vector3d yr = zr.cross(x0);
	if (!MathTools::Any(yr))
		yr = zr.cross(y0);
	Vector3d xr = zr.cross(yr);
	xr.normalize(); yr.normalize(); zr.normalize();
	_rot<<xr, yr, zr;
}

Vector4d ArcPathPlanner::RadiusEqual(Vector3d pos1, Vector3d pos2)
{
	Vector4d res;
	res(0) = 2*(pos2(0)-pos1(0));
	res(1) = 2*(pos2(1)-pos1(1));
	res(2) = 2*(pos2(2)-pos1(2));
	res(3) = pow(pos1(0), 2)+pow(pos1(1), 2)+pow(pos1(2), 2)
			-pow(pos2(0), 2)-pow(pos2(1), 2)-pow(pos2(2), 2);

	return res;
}

Vector4d ArcPathPlanner::PointsCoplane(Vector3d pos1, Vector3d pos2, Vector3d pos3)
{
	double x1 = pos1(0), y1 = pos1(1), z1 = pos1(2);
	double x2 = pos2(0), y2 = pos2(1), z2 = pos2(2);
	double x3 = pos3(0), y3 = pos3(1), z3 = pos3(2);
	Vector4d res;
	res(0) = y1*z2-y2*z1-y1*z3+y3*z1+y2*z3-y3*z2;
	res(1) = -(x1*z2-x2*z1-x1*z3+x3*z1+x2*z3-x3*z2);
	res(2) = x1*y2-x2*y1-x1*y3+x3*y1+x2*y3-x3*y2;
	res(3) = -(x1*y2*z3-x1*y3*z2-x2*y1*z3+x2*y3*z1+x3*y1*z2-x3*y2*z1);

	return res;
}

Vector3d ArcPathPlanner::GeneratePath(double varp)
{
	Vector3d pc(0, 0, 0), p(0, 0, 0);
	pc(0) = _radius*cos(varp);
	pc(1) = _radius*sin(varp);
	p = _center+_rot*pc;

	return p;
}

RobotTools::CLineAVP ArcPathPlanner::GenerateMotion(double varp, double varv, double vara)
{
	RobotTools::CLineAVP avp;
	Vector3d p = GeneratePath(varp);
	Vector3d vc(0, 0, 0), v(0, 0, 0);
	Vector3d ac(0, 0, 0), a(0, 0, 0);
	vc(0) = -varv*sin(varp);
	vc(1) = varv*cos(varp);
	v = _rot*vc;
	ac(0) = -pow(varv, 2)*cos(varp)-vara*sin(varp);
	ac(1) = -pow(varv, 2)*sin(varp)+vara*cos(varp);
	a = _rot*ac;
	avp.pos = p; avp.vel = v; avp.acc = a;

	return avp;
}

ArcPathPlanner::~ArcPathPlanner()
{
}

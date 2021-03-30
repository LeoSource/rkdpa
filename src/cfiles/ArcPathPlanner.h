#pragma once

#include "RobotMath.h"

using namespace std;
using namespace Eigen;

class ArcPathPlanner
{
public:
	Vector3d _center;
	double	_radius;
	double	_theta;
	Matrix3d	_rot;

public:
	ArcPathPlanner() {}

	ArcPathPlanner(Vector3d pos1, Vector3d pos2, Vector3d pos3);

	ArcPathPlanner(Vector3d center, Vector3d n_vec, double radius);

	Vector3d GeneratePath(double varp);

	RobotTools::CLineAVP GenerateMotion(double varp, double varv, double vara);

	~ArcPathPlanner();

private:
	Vector4d RadiusEqual(Vector3d pos1, Vector3d pos2);

	Vector4d PointsCoplane(Vector3d pos1, Vector3d pos2, Vector3d pos3);
};


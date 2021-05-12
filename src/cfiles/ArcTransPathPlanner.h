#pragma once

#include "RobotMath.h"
#include <vector>

using namespace std;
using namespace Eigen;

struct ArcData
{
	Vector3d center;
	double theta;
	Vector3d pt1;
	Vector3d pt2;
};

class ArcTransPathPlanner
{
public:
	MatrixXd _center;
	double _radius;
	VectorXd _theta;
	vector<Matrix3d> _rot;
	VectorXd _dr;
	VectorXd _dl;
	MatrixXd _pt;
	MatrixXd _line_vec;
	double _distance;
	VectorXd _dis_interval;

	int _nump;
	int _numarc;
	Vector3d _p_initial;
	Vector3d _p_goal;

public:
	ArcTransPathPlanner() {}

	ArcTransPathPlanner(MatrixXd pos, double radius);

	void InitPlanner(MatrixXd pos, double radius);

	Vector3d GeneratePath(double varp);

	RobotTools::CLineAVP GenerateMotion(double varp, double varv, double vara);

	~ArcTransPathPlanner() {}

private:
	void CalcArcInfo(MatrixXd pos);

	Matrix3d CalcArcRot(Vector3d center, Vector3d p1, Vector3d p2);

	ArcData CalcArcPoints(Vector3d p1, Vector3d p2, Vector3d p3);

	void CalcSemicircleInfo(MatrixXd pos);

	ArcData CalcSemicirclePoints(Vector3d p1, Vector3d p2, Vector3d p3, Vector3d p4);

	VectorXd CalcDisInterval();

	int CalcPosIdx(double varp);
};


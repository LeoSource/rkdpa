#pragma once

#include "RobotMath.h"

class CubicBSplinePlanner
{
public:
	char* option;

public:
	int _nump;
	int _num_ctrlp;
	int _pdegree;
	VectorXd _knot_vec;
	VectorXd _uknot_vec;
	MatrixXd _ctrl_pos;


public:
	CubicBSplinePlanner() {}

	CubicBSplinePlanner(MatrixXd via_pos, char* option);

	CubicBSplinePlanner(MatrixXd via_pos, char* option, double uk);

	CubicBSplinePlanner(MatrixXd via_pos, char* option, VectorXd uk);

	RobotTools::CLineAVP GenerateMotion(double u, double du, double ddu);

	Vector3d GeneratePos(double u);

	Vector3d GenerateVel(double u, double du);

	Vector3d GenerateAcc(double u, double du, double ddu);

	~CubicBSplinePlanner() {}

private:
	void CalcBSplineParams(MatrixXd via_pos, char* option);

	MatrixXd CalcCtrlPos(MatrixXd q);

	MatrixXd CalcApproCtrlPos(MatrixXd q);

	VectorXd CalcKnotVec();

	VectorXd CalcUKnot(MatrixXd q);

	VectorXd CalcApproKnotVec();

	double CalcBSplineCoeff(int p, int idx, double u);

	double DiffBSplineCoeff(int p, int jidx, double u, int k);

	double CalcDiffCoeff(int k, int idx, int jidx);

	double Divide(double num, double den);
};


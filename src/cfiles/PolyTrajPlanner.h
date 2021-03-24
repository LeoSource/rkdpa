#pragma once

#include "MathTools.h"

using namespace std;
using namespace Eigen;


class PolyTrajPlanner
{
public:
	int _nump;
	double _tf;
	VectorXd _tf_vec;
	double _dt;

	int _order;
	MatrixXd _poly_params;


public:
	PolyTrajPlanner() {}
	PolyTrajPlanner(VectorXd pos, double tf, int order);
	PolyTrajPlanner(VectorXd pos, double* tf_vec, int order);
	~PolyTrajPlanner();

public://private
	RowVectorXd PolyPos(double t);

	RowVectorXd PolyVel(double t);

	RowVectorXd PolyAcc(double t);

	MatrixXd LhsMat(double t0, double tf);

	MatrixXd CalcPolyParams(VectorXd pos);

	MatrixXd PolyAutoVel(VectorXd pos);

	MatrixXd PolyContiAcc(VectorXd pos);

	double GenerateMotion(double t);
};


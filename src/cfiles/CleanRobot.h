#pragma once

#include "MDHLink.h"
#include <vector>

using namespace Eigen;
using namespace RobotTools;

class CleanRobot
{
public:
	MatrixXd _mdh_table;
	VectorXd _offset;
	Matrix<int, Dynamic, 1> _type;
	int _nlinks;

private:
	vector<MDHLink> _links;
	Pose _tool;
	MatrixXd _qlimit;

public:
	CleanRobot(){}

	CleanRobot(MatrixXd mdh_table, Matrix<int, Dynamic, 1> type, VectorXd offset, Pose tool);

	void SetJntLimit(MatrixXd qlimit);

	Pose Transform(VectorXd q, int s_idx, int e_idx);

	Pose FKSolve(VectorXd q);

	VectorXd IKSolve(Vector3d pos, char* option, double alpha);

	MatrixXd CalcJaco(VectorXd q);

	MatrixXd CalcJv(VectorXd q);

	MatrixXd CalcJw(VectorXd q);

	~CleanRobot() {}

private:
	VectorXd IKJnt3(Vector3d pos, double alpha, double q3);

	VectorXd IKJnt2(Vector3d pos, double alpha);
};

#pragma once

#include "MDHLink.h"
#include <vector>

using namespace Eigen;
using namespace RobotTools;

class CleanRobot
{
public:
	Matrix<double, 5, 4, RowMajor> _mdh_table;
	Vector5d _offset;
	Matrix<int, 5, 1> _type;
	int _nlinks;
	MatrixXd _qlimit;
	double _tool_pitch;
	double _pitch_high, _pitch_low;

private:
	vector<MDHLink> _links;
	Pose _tool;
	VectorXd _hold_jpos;

public:
	CleanRobot(){}

	CleanRobot(MatrixXd mdh_table, Matrix<int, Dynamic, 1> type, VectorXd offset, Pose tool);

	CleanRobot(MatrixXd mdh_table, Matrix<int, Dynamic, 1> type, VectorXd offset, double tool_pitch, Vector3d tool_pos);

	void SetJntLimit(MatrixXd qlimit);

	void SetPitchRange(double pitch_high, double pitch_low);

	Pose FKSolve(VectorXd q);

	Pose FKSolveTool(VectorXd q);

	VectorXd IKSolve(Vector3d pos, char* option, double alpha, VectorXd q_in);

	VectorXd IKSolvePitch(Vector3d pos, double pitch);

	VectorXd IKSolveYaw(Vector3d pos, double pitch, double yaw, VectorXd q_in);

	//TO DO: update jacobian calculation functions
	MatrixXd CalcJaco(VectorXd q);

	MatrixXd CalcJv(VectorXd q);

	MatrixXd CalcJw(VectorXd q);

	void UpdateJntHoldPos(VectorXd q);

	VectorXd HoldJntPos();

	~CleanRobot() {}

private:
	VectorXd IKJnt3(Vector3d pos, double alpha, double q3, VectorXd q_in);

	VectorXd IKJnt2(Vector3d pos, double alpha, VectorXd q_in);

	Pose Transform(VectorXd q, int s_idx, int e_idx);
};


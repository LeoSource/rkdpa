#pragma once

#include <algorithm>
#include <math.h>
#include <Eigen/Dense>

#define	EPS	1e-8
#define pi	3.141592657

using namespace std;
using namespace Eigen;

namespace MathTools
{
	int Discretize(double* arr, int len, double value);

	int Discretize(VectorXd vec, int len, double value);

	double CalcTransEqua(double a, double b, double c, double pre_value);

	int Sign(double x);

	void LimitNum(double min_value, double& value, double max_value);

	void LimitMin(double min_value, double& value);

	void LimitMax(double max_value, double& value);

	void LimitVector(VectorXd min_vec, VectorXd* value, VectorXd max_vec);

	bool Any(VectorXd vec);

	double Norm(VectorXd vec);

	Vector3d Cross(Vector3d v1, Vector3d v2);

	int Factorial(int n);
}

using Vector5d = Matrix<double, 5, 1>;
using Vector6d = Matrix<double, 6, 1>;

namespace RobotTools
{
	struct JAVP
	{
		double pos;
		double vel;
		double acc;
	};

	struct CLineAVP
	{
		Vector3d pos;
		Vector3d vel;
		Vector3d acc;
	};

	struct CAVP
	{
		Matrix<double, 6, 1> pos;
		Matrix<double, 6, 1> vel;
		Matrix<double, 6, 1> acc;
	};

	struct Pose
	{
		Vector3d pos;
		Matrix3d rot;
	};

	MatrixXd CalcRectanglePath(MatrixXd* corner_pos, char* option, double interval);

	Matrix3d CalcPlaneRot(Vector3d center, Vector3d norm_vec, double px, double pz);

	Pose PoseProduct(Pose p1, Pose p2);

	Matrix3d RotX(double angle);

	Matrix3d RotY(double angle);

	Matrix3d RotZ(double angle);


}
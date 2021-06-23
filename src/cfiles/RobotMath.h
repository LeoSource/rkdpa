#pragma once

#include <algorithm>
#include <math.h>
#include <Eigen/Dense>

#define	EPS	1e-5
#define	EPS3 1e-3
#define EPS4 1e-4
#define pi	3.141592657

using namespace std;
using namespace Eigen;

namespace MathTools
{
	int Discretize(double* arr, int len, double value);

	int Discretize(VectorXd vec, int len, double value);

	double CalcTransEqua(double a, double b, double c, double pre_value);

	int CalcSinCosEqua(double *x1, double *x2, double a, double b, double c);

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
		Vector6d pos;
		Vector6d vel;
		Vector6d acc;
	};

	struct Pose
	{
		Vector3d pos;
		Matrix3d rot;
	};

	MatrixXd CalcRectanglePath(MatrixXd* corner_pos, char* option, double interval);

	/**
	* @brief	calculate rotation matrix according point and normal vector
	* @author	zxliao
	* @date		2021/6/3
	* @param	center		spatial point
	* @param	norm_vec	normal vector
	* @return	rot			rotation matrix
	**/
	Matrix3d CalcPlaneRot(Vector3d center, Vector3d norm_vec);

	/**
	* @brief	calculate control points with given zone radius
	* @author	zxliao
	* @date		2021/6/3
	* @param	pos1		first corner point
	* @param	pos2		second corner point
	* @param	pos3		third corner point
	* @param	r			transition radius between two lines
	* @param	opt			transition option: 2 points for arc, 5 points for spline
	* @return	ctrlpos		control points of transition
	**/
	MatrixXd CalcSplineTransPos(Vector3d pos1, Vector3d pos2, Vector3d pos3, double r, char* opt);

	double CalcArcRadius(Vector3d pos1, Vector3d pos2, Vector3d pos3);

	Pose PoseProduct(Pose p1, Pose p2);

	Matrix3d RotX(double angle);

	Matrix3d RotY(double angle);

	Matrix3d RotZ(double angle);

	CAVP LinetoSpatial(CLineAVP* line_avp);

	CAVP AngtoSpatial(CLineAVP* ang_avp);

	CAVP toSpatial(CLineAVP* line_avp, CLineAVP* ang_avp);

	Vector4d Tr2Quat1(Matrix3d r);

	Vector4d Tr2Quat2(Matrix3d r);

	Vector4d Tr2AngleAxis(Matrix3d r);

	Matrix3d AngleAxis2Tr(Vector4d E);

	Vector3d Tr2FixedZYX(Matrix3d r);

	Matrix3d FixedZYX2Tr(Vector3d rpy);

	Vector4d CalcAngleAxis(Vector3d rpy0, Vector3d rpyn);
}
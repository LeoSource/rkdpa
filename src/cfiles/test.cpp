// test.cpp : 定义控制台应用程序的入口点。
//

//#include "stdafx.h"
#include <iostream>
#include <algorithm>
#include "PolyTrajPlanner.h"

#include "eigen3/Eigen/Dense"
using namespace std;
using namespace Eigen;

int main()
{
	MatrixXd a = MatrixXd::Zero(3, 4);
	a.setIdentity(3,4);
	a << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
	double arra[] = { 0,2,4,6,8,10 };

	auto b = MathTools::Discretize(arra, 6, 0);
	cout << "idx = " << b << endl;
	Matrix<double, 6, 1> pos1(arra);
	int idx = MathTools::Discretize(pos1, 6, 0);
	cout << "b=" << idx << endl;

	//cout << sizeof(arra) / sizeof(arra[0]) << endl;
	//RowVector3f p, q, r;
	//cout << 'a'<< endl <<a << endl;
	//cout << "block" << endl << a.block(1,2,2,2) << endl;
	//cout << a.rightCols(1) << endl;

	//VectorXd vel;
	//vel.setZero(6);
	//vel(0) = 1.2; vel(1) = 2.3; vel(2) = 3.4;
	//cout << vel << endl;
	//Matrix<double, 6, 6> mat;
	//mat << vel, vel, vel, vel, vel, vel;
	//cout << mat << endl;


	Vector3d pos;
	pos << 0, 10, 5;
	double time[] = { 0,1.2,2 };
	PolyTrajPlanner planner(pos, time, 3);
	cout << planner._poly_params << endl;
	double p = planner.GenerateMotion(1.3547);
	cout << p << endl;

    return 0;
}


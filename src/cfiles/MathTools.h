#pragma once

#include <algorithm>
#include <math.h>
#include <Eigen/Dense>

#define	EPS	1e-8

using namespace std;
using namespace Eigen;

namespace MathTools
{
	int Discretize(double* arr, int len, double value);

	int Discretize(VectorXd vec, int len, double value);

	int Sign(double x);





}
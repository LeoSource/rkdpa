#include "stdafx.h"
#include "MathTools.h"

namespace MathTools
{

	int Discretize(double* arr, int len, double value)
	{
		int idx = 0;
		if (fabs(value - arr[len - 1]) < EPS)
		{
			idx = len - 2;
		}
		else
		{
			auto upper_value = upper_bound(arr, arr + len, value);
			idx = upper_value - arr - 1;
		}

		return idx;
	}

	int Discretize(VectorXd vec, int len, double value)
	{
		int idx = 0, start_idx = 0, mid_idx = 0, end_idx = len;
		if (fabs(value - vec(len - 1)) < EPS)
		{
			idx = len - 2;
		}
		else
		{
			while (end_idx - start_idx>1)
			{
				mid_idx = (start_idx + end_idx) / 2;
				if (vec(mid_idx) > value)
				{
					end_idx = mid_idx;
				}
				else
				{
					start_idx = mid_idx;
				}
			}
			idx = start_idx;
		}


		return idx;

	}

	int Sign(double x)
	{
		if (x < 0)
			return -1;
		else
			return 1;
	}







}
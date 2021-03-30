#include "stdafx.h"
#include "RobotMath.h"

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

	double CalcTransEqua(double a, double b, double c)
	{
		//calculate the equation like a*cos(theta)+b*sin(theta) = c
		double theta = 0, u = 0;
		double deg_thre[] = { -pi / 2, pi / 2 };
		if (fabs(a + c) < EPS)
		{
			u = (c - a) / (2 * b);
			theta = 2 * atan(u);
		}
		else
		{
			double tmp_value1 = (b + sqrt(pow(b,2) + pow(a,2) - pow(c,2))) / (a + c);
			double tmp_value2 = (b - sqrt(pow(b, 2) + pow(a, 2) - pow(c, 2))) / (a + c);
			tmp_value1 = 2 * atan(tmp_value1);
			tmp_value2 = 2 * atan(tmp_value2);
			if ((tmp_value1 > deg_thre[1]) || (tmp_value1 < deg_thre[0]))
				theta = tmp_value2;
			else
				theta = tmp_value1;
		}

		return theta;
	}

	int Sign(double x)
	{
		if (x < 0)
			return -1;
		else
			return 1;
	}

	double LimitNum(double min_value, double& value, double max_value)
	{
		if (value > max_value)
			value = max_value;
		else if (value < min_value)
			value = min_value;
		
		return value;
	}

	double LimitMin(double min_value, double& value)
	{
		if (value < min_value)
			value = min_value;

		return value;
	}

	double LimitMax(double max_value, double &value)
	{
		if (value > max_value)
			value = max_value;

		return value;
	}

	bool Any(VectorXd vec)
	{
		for (int idx = 0; idx<vec.size(); idx++)
		{
			if (fabs(vec(idx))>EPS)
				return true;
		}
		return false;
	}

	double Norm(VectorXd vec)
	{
		double res = 0;
		for (int idx = 0; idx<vec.size(); idx++)
		{
			res += pow(vec(idx), 2);
		}
		res = sqrt(res);

		return res;
	}

	Vector3d Cross(Vector3d v1, Vector3d v2)
	{
		return v1.cross(v2);
	}





}


namespace RobotTools
{
	MatrixXd CalcRectanglePath(MatrixXd corner_pos, int cycle_num, char* option)
	{
		Vector3d step_vec1, step_vec2, start_pos1, start_pos2;
		if (strcmp(option, "s") == 0)
		{
			step_vec1 = corner_pos.col(3) - corner_pos.col(0);
			step_vec2 = corner_pos.col(2) - corner_pos.col(1);
			start_pos1 = corner_pos.col(0);
			start_pos2 = corner_pos.col(1);
		}
		else if (strcmp(option, "m") == 0)
		{
			step_vec1 = corner_pos.col(1) - corner_pos.col(0);
			step_vec2 = corner_pos.col(2) - corner_pos.col(3);
			start_pos1 = corner_pos.col(0);
			start_pos2 = corner_pos.col(3);
		}
		int numvp = 2*cycle_num;
		double step = step_vec1.norm() / (cycle_num-1);
		step_vec1.normalize();
		step_vec2.normalize();
		MatrixXd via_pos;
		via_pos.setZero(3, numvp);
		for (int idx = 0; idx < cycle_num; idx++)
		{
			if (idx%2 == 0)
			{
				via_pos.col(2 * idx) = start_pos1 + step_vec1*step*idx;
				via_pos.col(2 * idx+1) = start_pos2 + step_vec2*step*idx;
			}
			else
			{
				via_pos.col(2 * idx) = start_pos2 + step_vec2*step*idx;
				via_pos.col(2 * idx+1) = start_pos1 + step_vec1*step*idx;
			}
		}

		return via_pos;
	}

	Pose PoseProduct(Pose p1, Pose p2)
	{
		Pose res;
		res.rot = p1.rot*p2.rot;
		res.pos = p1.pos+p1.rot*p2.pos;

		return res;
	}










}
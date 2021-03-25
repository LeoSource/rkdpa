#include "stdafx.h"
#include "PolyTrajPlanner.h"


PolyTrajPlanner::PolyTrajPlanner(VectorXd pos, double tf, int order)
{
	_order = order;
	int n = pos.size();
	_nump = n;
	_tf = tf;
	_tf_vec.setZero(n);
	_dt = tf / (double)(n - 1);
	_poly_params.setZero(order + 1, n - 1);
	_poly_params = CalcPolyParams(pos);
}

PolyTrajPlanner::PolyTrajPlanner(VectorXd pos, double* tf_vec, int order)
{
	_order = order;
	int n = pos.size();
	_nump = n;
	_tf_vec.setZero(n);
	for (int idx = 0; idx < n; idx++)
	{
		_tf_vec(idx) = tf_vec[idx];
	}
	_tf = 0;
	_dt = 0;
	_poly_params.setZero(order + 1, n - 1);
	_poly_params = CalcPolyParams(pos);
}

RowVectorXd PolyTrajPlanner::PolyPos(double t)
{
	RowVectorXd res;
	if (_order == 3)
	{		
		res.setZero(4);
		res << 1.0, t, pow(t,2), pow(t,3);
	}
	else if (_order == 5)
	{
		res.setZero(6);
		res << 1, t, pow(t,2), pow(t,3), pow(t,4), pow(t,5);
	}
		
	return res;
}

RowVectorXd PolyTrajPlanner::PolyVel(double t)
{
	RowVectorXd res;
	if (_order == 3)
	{
		res.setZero(4);
		res << 0, 1, 2 * t, 3 * pow(t, 2);
	}
	else if (_order == 5)
	{
		res.setZero(6);
		res << 0, 1, 2 * t, 3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4);
	}
	return res;
}

RowVectorXd PolyTrajPlanner::PolyAcc(double t)
{
	RowVectorXd res;
	if (_order == 3)
	{
		res.setZero(4);
		res << 0, 0, 2, 6 * t;
	}
	else
	{
		res.setZero(6);
		res << 0, 0, 2, 6 * t, 12 * pow(t, 2), 20 * pow(t, 3);
	}
	return res;
}

MatrixXd PolyTrajPlanner::LhsMat(double t0, double tf)
{
	MatrixXd res;
	if (_order == 3)
	{
		res.setZero(4, 4);
		res.row(0) = PolyPos(t0);
		res.row(1) = PolyPos(tf);
		res.row(2) = PolyVel(t0);
		res.row(3) = PolyVel(tf);
	}
	else if(_order == 5)
	{
		res.setZero(6, 6);
		res.row(0) = PolyPos(t0);
		res.row(1) = PolyPos(tf);
		res.row(2) = PolyVel(t0);
		res.row(3) = PolyVel(tf);
		res.row(4) = PolyAcc(t0);
		res.row(5) = PolyAcc(tf);
	}
	return res;
}

MatrixXd PolyTrajPlanner::CalcPolyParams(VectorXd pos)
{
	MatrixXd params;
	if (_order == 3)
	{
		if (_nump > 2)
		{
			params = PolyContiAcc(pos);
		}
		else
		{
			params = PolyAutoVel(pos);
		}
	}
	else if (_order==5)
	{
		params.setZero(6,1);
		Matrix<double, 6, 1> rhs;
		rhs << pos(0), pos(1), 0, 0, 0, 0;
		MatrixXd lhs = LhsMat(0, _tf);
		params = lhs.lu().solve(rhs);
	}
	return params;
}

MatrixXd PolyTrajPlanner::PolyAutoVel(VectorXd pos)
{
	MatrixXd params;
	params.setZero(4, _nump - 1);
	VectorXd vel, t;
	vel.setZero(_nump);
	t.setLinSpaced(_nump, 0, _tf);
	for (int idx = 0; idx < _nump - 1; idx++)
	{
		vel(0) = 0;
		if (idx == _nump - 2)
		{
			vel(idx + 1) = 0;
		}
		else
		{
			double k1 = (pos(idx + 1) - pos(idx)) / _dt;
			double k2 = (pos(idx + 2) - pos(idx + 1)) / _dt;
			if (MathTools::Sign(k1*k2) == 1)
			{
				vel(idx + 1) = 0.5*(k1 + k2);
			}
			else
			{
				vel(idx + 1) = 0;
			}
		}
		Vector4d rhs;
		rhs << pos(idx), pos(idx + 1), vel(idx), vel(idx + 1);
		MatrixXd lhs = LhsMat(t(idx), t(idx + 1));
		params.col(idx) = lhs.lu().solve(rhs);
	}
	return params;
}

MatrixXd PolyTrajPlanner::PolyContiAcc(VectorXd pos)
{
	int n = _nump;
	MatrixXd params, rhs, lhs;
	params.setZero(4 * (n - 1), 1);
	rhs.setZero(4 * (n - 1), 1);
	lhs.setZero(4 * (n - 1), 4 * (n - 1));
	VectorXd t;
	if (fabs(_tf) < EPS)
	{
		t = _tf_vec;
	}
	else
	{
		t.setLinSpaced(n, 0, _tf);
	}

	rhs(0) = pos(0);
	rhs(2 * (n - 1)-1) = pos(n-1);
	lhs.block(2 * (n - 1), 0, 1, 4) = PolyVel(t(0));
	lhs.block(2 * (n - 1) + n - 1, 4 * n - 8, 1, 4) = PolyVel(t(n - 1));
	for (int idx = 1; idx < n - 1; idx++)
	{
		rhs(2 * idx - 1) = pos(idx);
		rhs(2 * idx) = pos(idx);
		lhs.block(2 * (n - 1) + idx, 4 * idx - 4, 1, 4) = PolyVel(t(idx));
		lhs.block(2 * (n - 1) + idx, 4 * idx, 1, 4) = -PolyVel(t(idx));
		lhs.block(3 * n - 3 + idx, 4 * idx - 4, 1, 4) = PolyAcc(t(idx));
		lhs.block(3 * n - 3 + idx, 4 * idx, 1, 4) = -PolyAcc(t(idx));

	}
	for (int idx = 0; idx < n - 1; idx++)
	{
		lhs.block(2 * idx, 4 * idx, 1, 4) = PolyPos(t(idx));
		lhs.block(2 * idx + 1, 4 * idx, 1, 4) = PolyPos(t(idx + 1));
	}
	params = lhs.lu().solve(rhs);
	params.resize(4, n - 1);

	return params;
}


double PolyTrajPlanner::GenerateMotion(double t)
{
	VectorXd time_vec;
	if (fabs(_tf) < EPS)
	{
		time_vec = _tf_vec;
	}
	else
	{
		time_vec.setLinSpaced(_nump, 0, _tf);
	}	
	int idx = MathTools::Discretize(time_vec, _nump, t);
	double p = PolyPos(t)*_poly_params.col(idx);

	return p;
}

PolyTrajPlanner::~PolyTrajPlanner()
{
}

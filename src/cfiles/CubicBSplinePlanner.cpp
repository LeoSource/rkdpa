#include "CubicBSplinePlanner.h"


CubicBSplinePlanner::CubicBSplinePlanner(MatrixXd* via_pos, char* option)
{
	_nump = via_pos->cols();
	_pdegree = 3;
	_uknot_vec = CalcUKnot(via_pos);
	CalcBSplineParams(via_pos, option);
}

CubicBSplinePlanner::CubicBSplinePlanner(MatrixXd* via_pos, char* option, double uk)
{
	_nump = via_pos->cols();
	_pdegree = 3;
	_uknot_vec = uk*CalcUKnot(via_pos);
	CalcBSplineParams(via_pos, option);
}

CubicBSplinePlanner::CubicBSplinePlanner(MatrixXd* via_pos, char* option, VectorXd uk)
{
	_nump = via_pos->cols();
	_pdegree = 3;
	_uknot_vec = uk;
	CalcBSplineParams(via_pos, option);
}

void CubicBSplinePlanner::CalcBSplineParams(MatrixXd* via_pos, char* option)
{
	if (strcmp(option, "interpolation")==0)
	{
		_num_ctrlp = _nump+2;
		_knot_vec = CalcKnotVec();
		_ctrl_pos = CalcCtrlPos(via_pos);
	}
	else if (strcmp(option, "approximation")==0)
	{
		_num_ctrlp = 20;
		_knot_vec = CalcApproKnotVec();
		_ctrl_pos = CalcApproCtrlPos(via_pos);
	}
}

MatrixXd CubicBSplinePlanner::CalcCtrlPos(MatrixXd* q)
{
	int n = _nump;
	int m = _num_ctrlp;
	VectorXd u_hat = _uknot_vec;
	int dim = q->rows();
	MatrixXd p;
	p.setZero(dim, m);
	VectorXd t1 = (q->col(1)-q->col(0))/(u_hat(1)-u_hat(0));
	VectorXd tn = (q->col(n-1)-q->col(n-2))/(u_hat(n-1)-u_hat(n-2));
	p.col(0) = q->col(0);
	p.col(1) = q->col(0)+(u_hat(1)-u_hat(0))/3*t1;
	p.col(n) = q->col(n-1)-(u_hat(n-1)-u_hat(n-2))/3*tn;
	p.col(n+1) = q->col(n-1);
	MatrixXd B, R;
	B.setZero(n-2, n-2);
	R.setZero(n-2, dim);
	for (int idx = 0; idx<n-2; idx++)
	{
		if (idx==0)
		{
			R.row(idx) = q->col(1).transpose()-CalcBSplineCoeff(3, 1, u_hat(1))*p.col(1).transpose();
			B(0, 0) = CalcBSplineCoeff(3, 2, u_hat(1));
			B(0, 1) = CalcBSplineCoeff(3, 3, u_hat(1));
		}
		else if (idx==n-3)
		{
			R.row(idx) = q->col(n-2).transpose()-CalcBSplineCoeff(3, n, u_hat(n-2))*p.col(n).transpose();
			B(n-3, n-4) = CalcBSplineCoeff(3, n-2, u_hat(n-2));
			B(n-3, n-3) = CalcBSplineCoeff(3, n-1, u_hat(n-2));
		}
		else
		{
			R.row(idx) = q->col(idx+1).transpose();
			B(idx, idx-1) = CalcBSplineCoeff(3, idx+1, u_hat(idx+1));
			B(idx, idx) = CalcBSplineCoeff(3, idx+2, u_hat(idx+1));
			B(idx, idx+1) = CalcBSplineCoeff(3, idx+3, u_hat(idx+1));
		}
	}
	MatrixXd p_tmp = B.lu().solve(R);
	p.middleCols(2, n-2) = p_tmp.transpose();

	return p;
}

MatrixXd CubicBSplinePlanner::CalcApproCtrlPos(MatrixXd* q)
{
	int n = _nump;
	int m = _num_ctrlp;
	VectorXd u_hat = _uknot_vec;
	int dim = q->rows();
	MatrixXd p;
	p.setZero(dim, m);
	p.col(0) = q->col(0);
	p.col(m-1) = q->col(n-1);
	MatrixXd R, B;
	R.setZero(n-2, dim);
	B.setZero(n-2, m-2);
	for (int idx = 1; idx<n-1; idx++)
	{
		R.row(idx-1) = q->col(idx).transpose()-CalcBSplineCoeff(3, 0, u_hat(idx))*q->col(0).transpose()
						-CalcBSplineCoeff(3, m-1, u_hat(idx))*q->col(n-1).transpose();
	}
	for (int nidx = 1; nidx<n-1; nidx++)
	{
		for (int midx = 1; midx<m-1; midx++)
		{
			B(nidx-1, midx-1) = CalcBSplineCoeff(3, midx, u_hat(nidx));
		}
	}
	VectorXd w_vec;
	w_vec.setOnes(n-2);
	MatrixXd W = w_vec.asDiagonal();
	MatrixXd W_tmp = B.transpose()*W*B;
	MatrixXd pseinvB = W_tmp.inverse()*(B.transpose()*W);
	MatrixXd p_tmp = pseinvB*R;
	p.middleCols(1, m-2) = p_tmp.transpose();

	return p;
}

VectorXd CubicBSplinePlanner::CalcKnotVec()
{
	VectorXd uk = _uknot_vec;
	int m = _num_ctrlp;
	int p = _pdegree;
	VectorXd knot_vec;
	knot_vec.setZero(m+p+1);
	knot_vec(0) = knot_vec(1) = knot_vec(2) = uk(0);
	knot_vec(m+p-2) = knot_vec(m+p-1) = knot_vec(m+p) = uk(uk.size()-1);
	knot_vec.segment(3, uk.size()) = uk;

	return knot_vec;
}

VectorXd CubicBSplinePlanner::CalcUKnot(MatrixXd* q)
{
	VectorXd uk;
	uk.setZero(_nump);
	uk(uk.size()-1) = 1;
	double d = 0;
	for (int idx = 1; idx<_nump; idx++)
	{
		d += MathTools::Norm(q->col(idx)-q->col(idx-1));
	}
	for (int idx = 1; idx<_nump-1; idx++)
	{
		uk(idx) = uk(idx-1)+MathTools::Norm(q->col(idx)-q->col(idx-1))/d;
	}

	return uk;
}

VectorXd CubicBSplinePlanner::CalcApproKnotVec()
{
	int n = _nump;
	int m = _num_ctrlp;
	int p = _pdegree;
	double d = (double)(n+1)/(double)(m-p+1);
	VectorXd uk = _uknot_vec;
	VectorXd knot_vec;
	knot_vec.setZero(m+p+1);
	for (int jidx = 1; jidx<m-p; jidx++)
	{
		int idx = floor((jidx)*d);
		double alph = (jidx)*d-idx;
		knot_vec(jidx+p) = (1-alph)*uk(idx-1)+alph*uk(idx);
	}
	for (int idx = 0; idx<p+1; idx++)
	{
		knot_vec(idx) = uk(0);
		knot_vec(m+idx) = uk(uk.size()-1);
	}

	return knot_vec;
}




double CubicBSplinePlanner::CalcBSplineCoeff(int p, int idx, double u)
{
	double b_coeff;
	if (p==0)
	{
		if (u>=_knot_vec(idx) && u<_knot_vec(idx+1))
			b_coeff = 1;
		else
			b_coeff = 0;
	}
	else
	{
		double den1 = _knot_vec(idx+p)-_knot_vec(idx);
		double num1 = u-_knot_vec(idx);
		double coef1 = Divide(num1, den1);
		double den2 = _knot_vec(idx+p+1)-_knot_vec(idx+1);
		double num2 = _knot_vec(idx+p+1)-u;
		double coef2 = Divide(num2, den2);
		b_coeff = coef1*CalcBSplineCoeff(p-1, idx, u)
					+coef2*CalcBSplineCoeff(p-1, idx+1, u);
	}

	return b_coeff;
}

double CubicBSplinePlanner::DiffBSplineCoeff(int p, int jidx, double u, int k)
{
	VectorXd b_coeff;
	RowVectorXd ak;
	b_coeff.setZero(k+1);
	ak.setZero(k+1);
	for (int idx = 0; idx<=k; idx++)
	{
		ak(idx) = CalcDiffCoeff(k, idx, jidx);
		b_coeff(idx) = CalcBSplineCoeff(p-k, jidx+idx, u);
	}
	double diff_bcoeff = MathTools::Factorial(p)/MathTools::Factorial(p-k)*ak*b_coeff;

	return diff_bcoeff;
}

double CubicBSplinePlanner::CalcDiffCoeff(int k, int idx, int jidx)
{
	double ak;
	int p = _pdegree;
	VectorXd u = _knot_vec;
	if (k==0)
		ak = 1;
	else
	{
		if (idx==0)
			ak = Divide(CalcDiffCoeff(k-1, 0, jidx), (u(jidx+p-k+1)-u(jidx)));
		else if (idx==k)
			ak = -Divide(CalcDiffCoeff(k-1, k-1, jidx), u(jidx+p+1)-u(jidx+k));
		else
		{
			double num = CalcDiffCoeff(k-1, idx, jidx)-CalcDiffCoeff(k-1, idx-1, jidx);
			double den = u(jidx+p+idx-k+1)-u(jidx+idx);
			ak = Divide(num, den);
		}
	}
	return ak;
}


RobotTools::CLineAVP CubicBSplinePlanner::GenerateMotion(double u, double du, double ddu)
{
	RobotTools::CLineAVP cavp;
	cavp.pos = GeneratePos(u);
	cavp.vel = GenerateVel(u, du);
	cavp.acc = GenerateAcc(u, du, ddu);

	return cavp;
}

Vector3d CubicBSplinePlanner::GeneratePos(double u)
{
	int m = _num_ctrlp;
	VectorXd b_coeff;
	b_coeff.setZero(m);
	if (fabs(u-_knot_vec.maxCoeff())<EPS)
		b_coeff(m-1) = 1;
	else
	{
		for (int idx = 0; idx<m; idx++)
		{
			b_coeff(idx) = CalcBSplineCoeff(3, idx, u);
		}
	}
	Vector3d pos = _ctrl_pos*b_coeff;
	return pos;
}

Vector3d CubicBSplinePlanner::GenerateVel(double u, double du)
{
	int m = _num_ctrlp;
	VectorXd diff_bcoeff;
	diff_bcoeff.setZero(m);
	for (int idx = 0; idx<m; idx++)
	{
		diff_bcoeff(idx) = DiffBSplineCoeff(3, idx, u, 1);
	}
	//dp/dt=(dp/du)*(du/dt)
	Vector3d vel = _ctrl_pos*diff_bcoeff*du;
	return vel;
}


Vector3d CubicBSplinePlanner::GenerateAcc(double u, double du, double ddu)
{
	int m = _num_ctrlp;
	VectorXd diff_bcoeff, diff2_bcoeff;
	diff_bcoeff.setZero(m);
	diff2_bcoeff.setZero(m);
	for (int idx = 0; idx<m; idx++)
	{
		diff_bcoeff(idx) = DiffBSplineCoeff(3, idx, u, 1);
		diff2_bcoeff(idx) = DiffBSplineCoeff(3, idx, u, 2);
	}
	//ddp/ddt = (dp/du)*ddu+(ddp/ddu)*du^2
	Vector3d acc = _ctrl_pos*diff2_bcoeff*pow(du, 2)
					+_ctrl_pos*diff_bcoeff*ddu;
	return acc;
}


double CubicBSplinePlanner::Divide(double num, double den)
{
	double res;
	if (fabs(num)<EPS && fabs(den)<EPS)
		res = 0;
	else if (fabs(den)<EPS)
		res = num;
	else
		res = num/den;

	return res;
}
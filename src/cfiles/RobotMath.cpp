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

	double CalcTransEqua(double a, double b, double c, double pre_value)
	{
		//calculate the equation like a*cos(theta)+b*sin(theta) = c
		double theta = 0, u = 0;
		double deg_thre[] = { -pi / 2, pi / 2 };
		if (pow(a, 2)+pow(b, 2)<pow(c, 2))
			theta = pre_value;
		else
		{
			if (fabs(a+c) < EPS)
			{
				u = (c-a)/(2*b);
				theta = 2*atan(u);
			}
			else
			{
				double tmp_value1 = (b+sqrt(pow(b, 2)+pow(a, 2)-pow(c, 2)))/(a+c);
				double tmp_value2 = (b-sqrt(pow(b, 2)+pow(a, 2)-pow(c, 2)))/(a+c);
				tmp_value1 = 2*atan(tmp_value1);
				tmp_value2 = 2*atan(tmp_value2);
				if ((tmp_value1 > deg_thre[1])||(tmp_value1 < deg_thre[0]))
					theta = tmp_value2;
				else
					theta = tmp_value1;
			}
		}

		return theta;
	}

	int CalcSinCosEqua(double *x1, double *x2, double a, double b, double c)
	{
		double aa, discr;
		int n;

		if (a == 0. && b == 0. && c == 0.)
		{
			n = 0;
			*x1 = -100;
			*x2 = -100;
		}
		else if (a == b && b == c)
		{
			n = 2;
			*x1 = -pi * 0.5;
			*x2 = pi;
		}
		else if (b == 0. && c == 0.)
		{
			n = 2;
			*x1 = -pi;
			*x2 = pi;
		}
		else if (a == 0. && c == 0.)
		{
			n = 2;
			*x1 = -pi * 0.5;
			*x2 = pi * 0.5;
		}
		else
		{
			discr = a * a + b * b - c * c;
			if (discr >= 0.)
				aa = sqrt(discr);
			else
				aa = 0.;

			if (discr < -0.01)
			{
				n = 0;
				*x1 = -100;
				*x2 = -100;
			}
			else
			{
				*x1 = 2. * atan2((double)(-a + aa), (double)(-b + c));
				*x2 = 2. * atan2((double)(-a - aa), (double)(-b + c));
				if (*x1 > 0 && *x1 > pi)
					*x1 = *x1 - 2. * pi;
				else if (*x1 < 0 && *x1 < -pi)
					*x1 = *x1 + 2. * pi;

				if (*x2 > 0 && *x2 > pi)
					*x2 = *x2 - 2. * pi;
				else if (*x2 < 0 && *x2 < -pi)
					*x2 = *x2 + 2. * pi;
				n = 2;
			}
		}
		return (n);
	}

	int Sign(double x)
	{
		if (x < 0)
			return -1;
		else
			return 1;
	}

	void LimitNum(double min_value, double& value, double max_value)
	{
		if (value > max_value)
			value = max_value;
		else if (value < min_value)
			value = min_value;
		
	}

	void LimitMin(double min_value, double& value)
	{
		if (value < min_value)
			value = min_value;

	}

	void LimitMax(double max_value, double &value)
	{
		if (value > max_value)
			value = max_value;

	}

	void LimitVector(VectorXd min_vec, VectorXd* value, VectorXd max_vec)
	{
		for (int idx = 0; idx<value->size(); idx++)
		{
			LimitNum(min_vec(idx), (*value)(idx), max_vec(idx));
		}
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

	int Factorial(int n)
	{
		if (n==0)
			return 1;
		else
			return n*Factorial(n-1);
	}



}


namespace RobotTools
{
	MatrixXd CalcRectanglePath(MatrixXd* corner_pos, char* option, double interval)
	{
		Vector3d step_vec1, step_vec2, start_pos1, start_pos2;
		if (strcmp(option, "s") == 0)
		{
			step_vec1 = corner_pos->col(3) - corner_pos->col(0);
			step_vec2 = corner_pos->col(2) - corner_pos->col(1);
			start_pos1 = corner_pos->col(0);
			start_pos2 = corner_pos->col(1);
		}
		else if (strcmp(option, "m") == 0)
		{
			step_vec1 = corner_pos->col(1) - corner_pos->col(0);
			step_vec2 = corner_pos->col(2) - corner_pos->col(3);
			start_pos1 = corner_pos->col(0);
			start_pos2 = corner_pos->col(3);
		}
		int cycle_num = round(step_vec1.norm()/interval)+1;
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



	Matrix3d CalcPlaneRot(Vector3d center, Vector3d norm_vec)
	{
		//plane function:Ax+By+Cz+D=0
		double D = -norm_vec.dot(center);
		Vector3d p1;
		p1(2) = center(2);
		p1(0) = center(0)+1;
		p1(1) = (-norm_vec(0)*p1(0)-norm_vec(2)*p1(2)-D)/norm_vec(1);
		Vector3d n, o, a;
		a = norm_vec.normalized();
		n = p1-center;
		n.normalize();
		o = a.cross(n);
		Matrix3d rot;
		rot<<n, o, a;

		return rot;
	}

	MatrixXd CalcSplineTransPos(Vector3d pos1, Vector3d pos2, Vector3d pos3, double r, char* opt)
	{
		MatrixXd ctrlpos;
		if (strcmp(opt, "spline")==0)
		{
			ctrlpos.setZero(3, 5);
			ctrlpos.col(2) = pos2;
			double line1_len = MathTools::Norm(pos2-pos1);
			double line2_len = MathTools::Norm(pos3-pos2);
			ctrlpos.col(0) = pos2+r/line1_len*(pos1-pos2);
			ctrlpos.col(4) = pos2+r/line2_len*(pos3-pos2);
			double ratio = 0.5;
			ctrlpos.col(1) = ctrlpos.col(2)+ratio*(ctrlpos.col(0)-ctrlpos.col(2));
			ctrlpos.col(3) = ctrlpos.col(2)+ratio*(ctrlpos.col(4)-ctrlpos.col(2));
		}
		else if(strcmp(opt,"arc")==0)
		{
			ctrlpos.setZero(3, 2);
			double line1_len = MathTools::Norm(pos2-pos1);
			double line2_len = MathTools::Norm(pos3-pos2);
			ctrlpos.col(0) = pos2+r/line1_len*(pos1-pos2);
			ctrlpos.col(1) = pos2+r/line2_len*(pos3-pos2);
		}
		return ctrlpos;
	}

	double CalcArcRadius(Vector3d pos1, Vector3d pos2, Vector3d pos3)
	{
		Vector3d p2p1 = pos1-pos2;
		Vector3d p2p3 = pos3-pos2;
		double inc_angle = acos(p2p1.dot(p2p3)/p2p1.norm()/p2p3.norm());
		double radius = p2p1.norm()*tan(0.5*inc_angle);

		return radius;
	}

	Pose PoseProduct(Pose p1, Pose p2)
	{
		Pose res;
		res.rot = p1.rot*p2.rot;
		res.pos = p1.pos+p1.rot*p2.pos;

		return res;
	}

	Matrix3d RotX(double angle)
	{
		Matrix3d res;
		res.setIdentity();
		res(1, 1) = cos(angle);
		res(2, 2) = cos(angle);
		res(1, 2) = -sin(angle);
		res(2, 1) = sin(angle);
		
		return res;
	}

	Matrix3d RotY(double angle)
	{
		Matrix3d res;
		res.setIdentity();
		res(0, 0) = cos(angle);
		res(2, 2) = cos(angle);
		res(0, 2) = sin(angle);
		res(2, 0) = -sin(angle);

		return res;
	}

	Matrix3d RotZ(double angle)
	{
		Matrix3d res;
		res.setIdentity();
		res(0, 0) = cos(angle);
		res(1, 1) = cos(angle);
		res(0, 1) = -sin(angle);
		res(1, 0) = sin(angle);

		return res;
	}

	CAVP LinetoSpatial(CLineAVP* line_avp)
	{
		CAVP cavp;
		cavp.pos.setZero();
		cavp.vel.setZero();
		cavp.acc.setZero();
		cavp.pos.head(3) = line_avp->pos;
		cavp.vel.head(3) = line_avp->vel;
		cavp.acc.head(3) = line_avp->acc;

		return cavp;
	}


	CAVP AngtoSpatial(CLineAVP* ang_avp)
	{
		CAVP cavp;
		cavp.pos.setZero();
		cavp.vel.setZero();
		cavp.acc.setZero();
		cavp.pos.segment(3, 3) = ang_avp->pos;
		cavp.vel.segment(3, 3) = ang_avp->vel;
		cavp.acc.segment(3, 3) = ang_avp->acc;

		return cavp;
	}

	CAVP toSpatial(CLineAVP* line_avp, CLineAVP* ang_avp)
	{
		CAVP cavp;
		cavp.pos.head(3) = line_avp->pos;
		cavp.vel.head(3) = line_avp->vel;
		cavp.acc.head(3) = line_avp->acc;
		cavp.pos.segment(3, 3) = ang_avp->pos;
		cavp.vel.segment(3, 3) = ang_avp->vel;
		cavp.acc.segment(3, 3) = ang_avp->acc;

		return cavp;
	}

	Vector4d Tr2Quat1(Matrix3d r)
	{
		Vector4d res;
		res.setZero();
		const double trace = r(0, 0) + r(1, 1) + r(2, 2);
		double root;
		double u0, u1, u2, u3;

		if (trace > 0.0) {
			root = sqrt(trace + 1.0);
			u0 = 0.5 * root;
			root = 0.5 / root;
			u1 = (r(2, 1) - r(1, 2)) * root;
			u2 = (r(0, 2) - r(2, 0)) * root;
			u3 = (r(1, 0) - r(0, 1)) * root;
		}
		else if (r(0, 0) >= r(1, 1) && r(0, 0) >= r(2, 2))
		{
			root = sqrt(1.0 + r(0, 0) - (r(1, 1) + r(2, 2)));
			u1 = 0.5 * root;
			root = 0.5 / root;
			u2 = (r(0, 1) + r(1, 0)) * root;
			u3 = (r(2, 0) + r(0, 2)) * root;
			u0 = (r(2, 1) - r(1, 2)) * root;
		}
		else if (r(1, 1) >= r(2, 2))
		{
			root = sqrt(1.0 + r(1, 1) - (r(2, 2) + r(0, 0)));
			u2 = 0.5 * root;
			root = 0.5 / root;
			u3 = (r(1, 2) + r(2, 1)) * root;
			u1 = (r(0, 1) + r(1, 0)) * root;
			u0 = (r(0, 2) - r(2, 0)) * root;
		}
		else
		{
			root = sqrt(1.0 + r(2, 2) - (r(0, 0) + r(1, 1)));
			u3 = 0.5 * root;
			root = 0.5 / root;
			u1 = (r(2, 0) + r(0, 2)) * root;
			u2 = (r(1, 2) + r(2, 1)) * root;
			u0 = (r(1, 0) - r(0, 1)) * root;
		}

		{
			double norm = sqrt(u0 * u0 + u1 * u1 + u2 * u2 + u3 * u3);

			u0 /= norm;
			u1 /= norm;
			u2 /= norm;
			u3 /= norm;
		}
		if (u0 < 0.0)
		{
			u0 = -u0;
			u1 = -u1;
			u2 = -u2;
			u3 = -u3;
		}
		res << u0, u1, u2, u3;
		return res;
	}

	Vector4d Tr2Quat2(Matrix3d r)
	{
		Vector4d res;
		res.setZero();
		const double trace = r(0, 0) + r(1, 1) + r(2, 2);
		double root;
		double u0, u1, u2, u3;

		if (trace > 0.0) {
			root = sqrt(trace + 1.0);
			u0 = 0.5 * root;
			root = 0.5 / root;
			u1 = (r(2, 1) - r(1, 2)) * root;
			u2 = (r(0, 2) - r(2, 0)) * root;
			u3 = (r(1, 0) - r(0, 1)) * root;
		}
		else if (r(0, 0) >= r(1, 1) && r(0, 0) >= r(2, 2))
		{
			root = sqrt(1.0 + r(0, 0) - (r(1, 1) + r(2, 2)));
			u1 = 0.5 * root;
			root = 0.5 / root;
			u2 = (r(0, 1) + r(1, 0)) * root;
			u3 = (r(2, 0) + r(0, 2)) * root;
			u0 = (r(2, 1) - r(1, 2)) * root;
		}
		else if (r(1, 1) >= r(2, 2))
		{
			root = sqrt(1.0 + r(1, 1) - (r(2, 2) + r(0, 0)));
			u2 = 0.5 * root;
			root = 0.5 / root;
			u3 = (r(1, 2) + r(2, 1)) * root;
			u1 = (r(0, 1) + r(1, 0)) * root;
			u0 = (r(0, 2) - r(2, 0)) * root;
		}
		else
		{
			root = sqrt(1.0 + r(2, 2) - (r(0, 0) + r(1, 1)));
			u3 = 0.5 * root;
			root = 0.5 / root;
			u1 = (r(2, 0) + r(0, 2)) * root;
			u2 = (r(1, 2) + r(2, 1)) * root;
			u0 = (r(1, 0) - r(0, 1)) * root;
		}

		{
			double norm = sqrt(u0 * u0 + u1 * u1 + u2 * u2 + u3 * u3);

			u0 /= norm;
			u1 /= norm;
			u2 /= norm;
			u3 /= norm;
		}
		res << u0, u1, u2, u3;
		return res;
	}
	Vector4d Tr2AngleAxis(Matrix3d r)
	{
		Vector4d res;
		res.setZero();
		Vector4d q = Tr2Quat1(r);
		Vector4d q_temp;
		q_temp.setZero();
		bool offset = false;
		double theta, x, y, z;
		if (q(0) < 0)
		{
			offset = true;
			q_temp(0) = -q(0);
		}
		else
		{
			q_temp(0) = q(0);
		}
		q_temp(1) = q(1);
		q_temp(2) = q(2);
		q_temp(3) = q(3);

		theta = 2.0 * acos(q_temp(0));

		if (fabs(theta) > 2.5e-2)
		{
			double sinth = sin(theta / 2.0);
			x = q_temp(1) / sinth;
			y = q_temp(2) / sinth;
			z = q_temp(3) / sinth;
		}
		else
		{
			double vect_len;
			vect_len = q_temp(1) * q_temp(1) + q_temp(2) * q_temp(2) + q_temp(3) * q_temp(3);
			if (vect_len < EPS)
			{
				theta = 0.0;
				x = 0.0;
				y = 0.0;
				z = 0.0;
			}
			else
			{
				double sin_theta_approx_half = sqrt(vect_len);
				theta = (sin_theta_approx_half * 2.0);
				x = q_temp(1) / sin_theta_approx_half;
				y = q_temp(2) / sin_theta_approx_half;
				z = q_temp(3) / sin_theta_approx_half;
			}
		}
		if (offset == true)
			theta = (2 * pi) - theta;
		res << x, y, z, theta;
		return res;
	}

	Matrix3d AngleAxis2Tr(Vector4d E)
	{
		Matrix3d r;
		r.setIdentity();
		double kx = E(0);
		double ky = E(1);
		double kz = E(2);
		double ct = cos(E(3));
		double st = sin(E(3));
		double vt = 1 - ct;
		r(0, 0) = kx*kx*vt + ct;     r(0, 1) = kx*ky*vt - kz*st;  r(0, 2) = kx*kz*vt + ky*st;
		r(1, 0) = kx*ky*vt + kz*st;  r(1, 1) = ky*ky*vt + ct;     r(1, 2) = ky*kz*vt - kx*st;
		r(2, 0) = kx*kz*vt - ky*st;  r(2, 1) = ky*kz*vt + kx*st;  r(2, 2) = kz*kz*vt + ct;
		return r;
	}

	Vector3d Tr2FixedZYX(Matrix3d r)
	{
		Vector3d res;
		res.setZero();
		double x, y, z;
		double sqr;
		sqr = sqrt(r(1, 2) * r(1, 2) + r(2, 2) * r(2, 2));
		y = atan2(r(0, 2), sqr);

		if (fabs(y - pi * 0.5) <= EPS4)
		{
			z = 0.;
			x = atan2(r(1, 0), r(1, 1));
		}
		else if (fabs(y + pi * 0.5) <= EPS4)
		{
			z = 0.;
			x = atan2(-r(1, 0), r(1, 1));
		}
		else
		{
			/* No degeneration */
			z = atan2(-r(0, 1), r(0, 0));
			x = atan2(-r(1, 2), r(2, 2));
		}
		res << x, y, z;
		return res;
	}
	Matrix3d FixedZYX2Tr(Vector3d rpy)
	{
		Matrix3d res;
		res.setIdentity();
		double Z = rpy(2);
		double Y = rpy(1);
		double X = rpy(0);
		res(0, 0) = cos(Y)*cos(Z);
		res(0, 1) = -cos(Y)*sin(Z);
		res(0, 2) = sin(Y);
		res(1, 0) = sin(X)*sin(Y)*cos(Z) + cos(X)*sin(Z);
		res(1, 1) = -sin(X)*sin(Y)*sin(Z) + cos(X)*cos(Z);
		res(1, 2) = -sin(X)*cos(Y);
		res(2, 0) = -cos(X)*sin(Y)*cos(Z) + sin(X)*sin(Z);
		res(2, 1) = cos(X)*sin(Y)*sin(Z) + sin(X)*cos(Z);
		res(2, 2) = cos(X)*cos(Y);
		return res;
	}
	Vector4d CalcAngleAxis(Vector3d rpy0, Vector3d rpyn)
	{
		Vector4d res;
		res.setZero();
		Matrix3d r0 = FixedZYX2Tr(rpy0);
		Matrix3d rn = FixedZYX2Tr(rpyn);
		Matrix3d r = r0.inverse() * rn;
		res = Tr2AngleAxis(r);
		return res;
	}

}
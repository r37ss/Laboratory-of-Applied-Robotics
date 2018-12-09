#include "Dubins.h"
#include "math.h"
#include <iostream>



using namespace std;

arc::arc(double x0_, double y0_, double th0_, double k_, double L_) {
	x0 = x0_;
	y0 = y0_;
	th0 = th0_;
	k = k_;
	L = L_;
	xf = x0 + L * sinc(k * L / 2.0) * cos(th0 + k * L / 2);
	yf = y0 + L * sinc(k * L / 2.0) * sin(th0 + k * L / 2);
	thf = mod2pi(th0 + k * L);
}

/*
curve::curve(arc a, arc b, arc c, double l) {
	a1 = a;
	a2 = b;
	a3 = c;
	lenght = l;
}*/


vector<Point2f> cut_arc(arc a, arc b, arc c) {
	int nr_points = 300;
	vector<Point2f> list;
	for (int i = 1; i < 300; i++) {
		
		double x, y;
		double s = a.L / nr_points * i;
		x = a.x0 + s * sinc(a.k * s / 2.0) * cos(a.th0 + a.k * s / 2);
		y = a.y0 + s * sinc(a.k * s / 2.0) * sin(a.th0 + a.k * s / 2);
		Point2f temp(x, y);
		list.emplace_back(temp);
	}

	for (int i = 1; i < 300; i++) {

		double x, y;
		double s = b.L / nr_points * i;
		x = b.x0 + s * sinc(b.k * s / 2.0) * cos(b.th0 + b.k * s / 2);
		y = b.y0 + s * sinc(b.k * s / 2.0) * sin(b.th0 + b.k * s / 2);
		Point2f temp(x, y);
		list.emplace_back(temp);
	}

	for (int i = 1; i < 300; i++) {

		double x, y;
		double s = c.L / nr_points * i;
		x = c.x0 + s * sinc(c.k * s / 2.0) * cos(c.th0 + c.k * s / 2);
		y = c.y0 + s * sinc(c.k * s / 2.0) * sin(c.th0 + c.k * s / 2);
		Point2f temp(x, y);
		list.emplace_back(temp);
	}

	return list;
}

//Implementation of function sinc(t), returning 1 for t==0, and sin(t)/t otherwise
double sinc(double t) {
	double s;
	if (fabs(t) < 0.002)
		// For small values of t use Taylor series approximation
		s = 1 - t*t / 6 * (1 - t*t / 20);
	else
		s = sin(t) / t;
	return s;
}

//Normalize an angle(in range[0, 2 * pi))
double mod2pi(double ang) {
	double out=ang;
	while (out < 0) {
		out = out + 2 * PI;
	}
	while (out >= 2 * PI) {
		out = out - 2 * PI;
	}
	return out;
}

//Normalize an angular difference(range(-pi, pi])
double rangeSymm(double ang) {
	double out = ang;
	while (out <= -PI){
		out = out + 2 * PI;
	}
	while (out > PI){
			out = out - 2 * PI;
	}
	return out;
}

// Check validity of a solution by evaluating explicitly the 3 equations  defining a Dubins problem(in standard form)
bool check(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf) {

	double x0 = -1;
	double y0 = 0;
	double xf = 1;
	double yf = 0;

	double eq1, eq2, eq3;
	bool Lpos;

	eq1 = x0 + s1 * sinc((1 / 2.) * k0 * s1) * cos(th0 + (1 / 2.) * k0 * s1) + s2 * sinc((1 / 2.) * k1 * s2) * cos(th0 + k0 * s1 + (1 / 2.) * k1 * s2) 	+ s3 * sinc((1 / 2.) * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + (1 / 2.) * k2 * s3) - xf;
	eq2 = y0 + s1 * sinc((1 / 2.) * k0 * s1) * sin(th0 + (1 / 2.) * k0 * s1) + s2 * sinc((1 / 2.) * k1 * s2) * sin(th0 + k0 * s1 + (1 / 2.) * k1 * s2) 	+ s3 * sinc((1 / 2.) * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + (1 / 2.) * k2 * s3) - yf;
	eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

	if ((s1 > 0) || (s2 > 0) || (s3 > 0))
		Lpos = true;
	else
		Lpos = false;

	if ((sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1.e-6) && Lpos)
		return true;
	else
		return false;

}

// Scale the input problem to standard form (x0: -1, y0: 0, xf: 1, yf: 0)
void scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax, double *sc_th0, double *sc_thf, double *sc_Kmax, double *lambda) {
	
	
	double dx = xf - x0;
	double dy = yf - y0;
	double phi = atan2(dy, dx);
	*lambda = hypot(dx, dy);

	double C = dx / *lambda;
	double S = dy / *lambda;
	*lambda = *lambda / 2;

	// scale and normalize angles and curvature
	*sc_th0 = mod2pi(th0 - phi);
	*sc_thf = mod2pi(thf - phi);
	*sc_Kmax = Kmax * *lambda;

	//cout << *sc_th0 << endl << *sc_thf << endl << *sc_Kmax << endl;
}

void scaleFromStandard(double lambda, double sc_s1, double sc_s2, double sc_s3, double *s1, double *s2, double *s3) {
	*s1 = sc_s1 * lambda;
	*s2 = sc_s2 * lambda;
	*s3 = sc_s3 * lambda;
}

void LSL(double sc_th0, double  sc_thf, double  sc_Kmax, bool  *ok, double  *sc_s1, double  *sc_s2, double  *sc_s3) {
	double invK = 1 / sc_Kmax;
	double C = cos(sc_thf) - cos(sc_th0);
	double S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
	double temp1 = atan2(C, S);
	*sc_s1 = invK * mod2pi(temp1 - sc_th0);

	double temp2 = 2 + 4 * sc_Kmax*sc_Kmax - 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
	
	if (temp2 < 0) {
		*ok = false; *sc_s1 = 0; *sc_s2 = 0; *sc_s3 = 0;
		return;
	}
	
	*sc_s2 = invK * sqrt(temp2);
	
	*sc_s3 = invK * mod2pi(sc_thf - temp1);
	*ok = true;

	cout << *sc_s1 << endl << *sc_s2 << endl<< *sc_s3 << endl;
}

void RSR(double sc_th0, double  sc_thf, double  sc_Kmax, bool  *ok, double  *sc_s1, double  *sc_s2, double  *sc_s3) {
	double invK = 1 / sc_Kmax;
	double C = cos(sc_th0) - cos(sc_thf);
	double S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
	double temp1 = atan2(C, S);
	*sc_s1 = invK * mod2pi(sc_th0 - temp1);
	double temp2 = 2 + 4 * sc_Kmax *sc_Kmax - 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
	if (temp2 < 0) {
		*ok = false; *sc_s1 = 0; *sc_s2 = 0; *sc_s3 = 0;
		return;
	}
	
	*sc_s2 = invK * sqrt(temp2);
	*sc_s3 = invK * mod2pi(temp1 - sc_thf);
	*ok = true;
}

void LSR(double sc_th0, double  sc_thf, double  sc_Kmax, bool  *ok, double  *sc_s1, double  *sc_s2, double  *sc_s3) {
	double invK = 1 / sc_Kmax;
	double C = cos(sc_th0) + cos(sc_thf);
	double S = 2 * sc_Kmax + sin(sc_th0) + sin(sc_thf);
	double temp1 = atan2(-C, S);
	double temp3 = 4 * sc_Kmax*sc_Kmax - 2 + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
	if (temp3 < 0) {
		*ok = false; *sc_s1 = 0; *sc_s2 = 0; *sc_s3 = 0;
		return;
	}
	
	*sc_s2 = invK * sqrt(temp3);
	double temp2 = -atan2(-2, *sc_s2 * sc_Kmax);
	*sc_s1 = invK * mod2pi(temp1 + temp2 - sc_th0);
	*sc_s3 = invK * mod2pi(temp1 + temp2 - sc_thf);
	*ok = true;
}

void RSL(double sc_th0, double  sc_thf, double  sc_Kmax, bool  *ok, double  *sc_s1, double  *sc_s2, double  *sc_s3) {
	double invK = 1 / sc_Kmax;
	double C = cos(sc_th0) + cos(sc_thf);
	double S = 2 * sc_Kmax - sin(sc_th0) - sin(sc_thf);
	double temp1 = atan2(C, S);
	double temp3 = 4 * sc_Kmax*sc_Kmax - 2 + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
	if (temp3 < 0) {
		*ok = false; *sc_s1 = 0; *sc_s2 = 0; *sc_s3 = 0;
		return;
	}
	
	*sc_s2 = invK * sqrt(temp3);
	double temp2 = atan2(2, *sc_s2 * sc_Kmax);
	*sc_s1 = invK * mod2pi(sc_th0 - temp1 + temp2);
	*sc_s3 = invK * mod2pi(sc_thf - temp1 + temp2);
	*ok = true;
}

void RLR(double sc_th0, double  sc_thf, double  sc_Kmax, bool  *ok, double  *sc_s1, double  *sc_s2, double  *sc_s3) {
	double invK = 1 / sc_Kmax;
	double C = cos(sc_th0) - cos(sc_thf);
	double S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
	double temp1 = atan2(C, S);
	double temp2 = 0.125 * (6 - 4 * sc_Kmax* sc_Kmax + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));
	if (fabs(temp2) > 1) {
		*ok = false; *sc_s1 = 0; *sc_s2 = 0; *sc_s3 = 0;
		return;
	}
	*sc_s2 = invK * mod2pi(2 * PI - acos(temp2));
	*sc_s1 = invK * mod2pi(sc_th0 - temp1 + 0.5 * *sc_s2 * sc_Kmax);
	*sc_s3 = invK * mod2pi(sc_th0 - sc_thf + sc_Kmax * (*sc_s2 - *sc_s1));
	*ok = true;
}

void LRL(double sc_th0, double  sc_thf, double  sc_Kmax, bool  *ok, double  *sc_s1, double  *sc_s2, double  *sc_s3) {
	double invK = 1 / sc_Kmax;
	double C = cos(sc_thf) - cos(sc_th0);
	double S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
	double temp1 = atan2(C, S);
	double temp2 = 0.125 * (6 - 4 * sc_Kmax*sc_Kmax + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));
	if (fabs(temp2) > 1) {
		*ok = false; *sc_s1 = 0; *sc_s2 = 0; *sc_s3 = 0;
		return;
	}
	*sc_s2 = invK * mod2pi(2 * PI - acos(temp2));
	*sc_s1 = invK * mod2pi(temp1 - sc_th0 + 0.5 * *sc_s2 * sc_Kmax);
	*sc_s3 = invK * mod2pi(sc_thf - sc_th0 + sc_Kmax * (*sc_s2 - *sc_s1));
	*ok = true;
}


vector<Point2f> dubins(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax) {

	double *sc_th0 = new double(0), *sc_thf = new double(0), *sc_Kmax = new double(0), *lambda = new double(0);
	double *s1 = new double(0), *s2 = new double(0), *s3 = new double(0);
	double  *sc_s1 = new double(0), *sc_s2 = new double(0), *sc_s3 = new double(0);
	double  *sc_s1_c = new double(0), *sc_s2_c = new double(0), *sc_s3_c = new double(0);
	//double  *s1 = new double(0), *s2 = new double(0), *s3 = new double(0);
	bool *ok = new bool(0);
	string optimal;
	
	// Compute params of standard scaled problem
	scaleToStandard(x0, y0, th0, xf, yf, thf, Kmax,sc_th0,sc_thf,sc_Kmax,lambda);

	double	pidx = -1;
	double L = 100000000000000;
	double Lcur=0;
	for (int i = 0; i<=5; i++) {
		switch (i) {

		case 0:
		LSL(*sc_th0, *sc_thf, *sc_Kmax, ok, sc_s1_c, sc_s2_c, sc_s3_c);
		 Lcur = *sc_s1_c + *sc_s2_c + *sc_s3_c;
			cout << "LSL: " << Lcur << endl;
			if (*ok && Lcur < L) {
				L = Lcur;
				*sc_s1 = *sc_s1_c;
				*sc_s2 = *sc_s2_c;
				*sc_s3 = *sc_s3_c;
				pidx = i;
				optimal = "LSL";
			}


		case 1:

			RSR(*sc_th0, *sc_thf, *sc_Kmax, ok, sc_s1_c, sc_s2_c, sc_s3_c);
			 Lcur = *sc_s1_c + *sc_s2_c + *sc_s3_c;
			cout << "RSR: " << Lcur << endl;
			if (*ok && Lcur < L) {
				L = Lcur;
				*sc_s1 = *sc_s1_c;
				*sc_s2 = *sc_s2_c;
				*sc_s3 = *sc_s3_c;
				pidx = i;
				optimal = "RSR";
			}

			break;

		case 2:

			LSR(*sc_th0, *sc_thf, *sc_Kmax, ok, sc_s1_c, sc_s2_c, sc_s3_c);
			 Lcur = *sc_s1_c + *sc_s2_c + *sc_s3_c;
			if (*ok && Lcur < L) {
				L = Lcur;
				*sc_s1 = *sc_s1_c;
				*sc_s2 = *sc_s2_c;
				*sc_s3 = *sc_s3_c;
				pidx = i;
				optimal = "LSR";
			}

			break;

		case 3:

			RSL(*sc_th0, *sc_thf, *sc_Kmax, ok, sc_s1_c, sc_s2_c, sc_s3_c);
			 Lcur = *sc_s1_c + *sc_s2_c + *sc_s3_c;
			cout << "RSL: " << Lcur << endl;
			if (*ok && Lcur < L) {
				L = Lcur;
				*sc_s1 = *sc_s1_c;
				*sc_s2 = *sc_s2_c;
				*sc_s3 = *sc_s3_c;
				pidx = i;
				optimal = "RSL";
			}

			break;

		case 4:
			RLR(*sc_th0, *sc_thf, *sc_Kmax, ok, sc_s1_c, sc_s2_c, sc_s3_c);
			 Lcur = *sc_s1_c + *sc_s2_c + *sc_s3_c;
			cout << "RLR: " << Lcur << endl;
			if (*ok && Lcur < L) {
				L = Lcur;
				*sc_s1 = *sc_s1_c;
				*sc_s2 = *sc_s2_c;
				*sc_s3 = *sc_s3_c;
				pidx = i;
				optimal = "RLR";
			}
			break;

		case 5:
			LRL(*sc_th0, *sc_thf, *sc_Kmax, ok, sc_s1_c, sc_s2_c, sc_s3_c);
			 Lcur = *sc_s1_c + *sc_s2_c + *sc_s3_c;
			cout << "LRL: " << Lcur << endl;
			if (*ok && Lcur < L) {
				L = Lcur;
				*sc_s1 = *sc_s1_c;
				*sc_s2 = *sc_s2_c;
				*sc_s3 = *sc_s3_c;
				pidx = i;
				optimal = "LRL";
			}
			break;

		}
	}

	scaleFromStandard(*lambda, *sc_s1, *sc_s2, *sc_s3, s1, s2, s3);
	L = *s1 + *s2 + *s3;
	
	cout <<endl<< "Start Point: x= " << x0 << " y= " << y0 << "   theta: "<< th0 << endl;
	cout << "End Point: x= " << xf << " y= " << yf << "   theta: " << thf << endl;
	cout << endl << "The shortest path: " << L << " for: " << optimal.c_str() << endl;

	arc a1(x0,y0,th0, ksigns[int(pidx)][0]*Kmax,*s1);
	arc a2(a1.xf, a1.yf, a1.thf, ksigns[int(pidx)][1] * Kmax, *s2);
	arc a3(a2.xf, a2.yf, a2.thf, ksigns[int(pidx)][2] * Kmax, *s3);

	vector<Point2f> list;
	list = cut_arc(a1, a2, a3);

	return list;
	//curve cur(a1,a2,a3,L);
}


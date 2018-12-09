
#ifndef DUBINS_H
#define DUBINS_H


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;



const double PI = 3.141592653589793238463;

const int ksigns[6][3] = { {1,0,-1},{-1,0,-1},{1,0,-1},{-1,0,1},{-1,1,-1},{1,-1,1} };

class arc {
public:
		double x0;
		double y0;
		double th0;
		double k;
		double L;
		double xf;
		double yf;
		double thf;
	
	arc(double , double , double , double , double );
};


/*
class curve {
public:
	arc a1;
	arc a2;
	arc a3;
	double lenght;

	curve(arc , arc , arc , double );
};
*/


//vector<Point_> cut_arc(arc a, arc b, arc c);
vector<Point2f> cut_arc(arc a, arc b, arc c);

double sinc(double t);
double mod2pi(double ang);
double rangeSymm(double ang);
bool check(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf);
void scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax, double *sc_th0, double *sc_thf, double *sc_Kmax, double *lambda);
void scaleFromStandard(double lambda,double sc_s1, double sc_s2, double sc_s3, double *s1, double *s2, double *s3);

void LSL(double sc_th0, double  sc_thf, double  sc_Kmax, bool  *ok, double  *sc_s1, double  *sc_s2, double  *sc_s3);
void RSR(double sc_th0, double  sc_thf, double  sc_Kmax, bool  *ok, double  *sc_s1, double  *sc_s2, double  *sc_s3);
void LSR(double sc_th0, double  sc_thf, double  sc_Kmax, bool  *ok, double  *sc_s1, double  *sc_s2, double  *sc_s3);
void RSL(double sc_th0, double  sc_thf, double  sc_Kmax, bool  *ok, double  *sc_s1, double  *sc_s2, double  *sc_s3);
void RLR(double sc_th0, double  sc_thf, double  sc_Kmax, bool  *ok, double  *sc_s1, double  *sc_s2, double  *sc_s3);
void LRL(double sc_th0, double  sc_thf, double  sc_Kmax, bool  *ok, double  *sc_s1, double  *sc_s2, double  *sc_s3);

vector<Point2f> dubins(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax);

#endif
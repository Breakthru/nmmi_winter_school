

#include <math.h> 


struct FourDoubles{
double q1;
double q2;
double q3;
double q4;	

	FourDoubles(const double & v1, const double & v2, const double & v3, const double & v4){
		q1 = v1;
		q2 = v2;
		q3 = v3;
		q4 = v4;			
	}
	
	double& operator[](const int & i){
		if (i == 1) return q1;
		if (i == 2) return q2;
		if (i == 3) return q3;
		if (i == 4) return q4;
		return q4; //this is wrong, but it is ok...
	}
}

FourDoubles my_stiffness(8.0, 8.0, 8.0, 8.0);

inline FourDoubles gravity_compensated_reference( FourDoubles ref, FourDoubles stif ){

	const double a1 = 0.17;
	const double l1 = 0.12;
	const double l2 = 0.14;
	const double m1 = 0.6;
	const double m2 = 0.44;
	const double g = 9.81;


	FourDoubles G;
	G.q1 = ref.q1 - 1.0/stif.q1 * (g*(a1*m2*cos(ref.q2)*sin(ref.q1) + l1*m1*cos(ref.q2)*sin(ref.q1) + l2*m2*cos(ref.q2)*cos(ref.q4)*sin(ref.q1) + l2*m2*cos(ref.q1)*cos(ref.q3)*sin(ref.q4) - l2*m2*sin(ref.q1)*sin(ref.q2)*sin(ref.q3)*sin(ref.q4)));
	G.q2 = ref.q2 - 1.0/stif.q2 *(g*(a1*m2*cos(ref.q1)*sin(ref.q2) + l1*m1*cos(ref.q1)*sin(ref.q2) + l2*m2*cos(ref.q1)*cos(ref.q4)*sin(ref.q2) + l2*m2*cos(ref.q1)*cos(ref.q2)*sin(ref.q3)*sin(ref.q4)));
	G.q3 = ref.q3 - 1.0/stif.q3 * (-(g*(l2*m2*sin(ref.q1)*sin(ref.q3)*sin(ref.q4) - l2*m2*cos(ref.q1)*cos(ref.q3)*sin(ref.q2)*sin(ref.q4))));
	G.q4 = ref.q4 - 1.0/stif.q4 * ((g*(l2*m2*cos(ref.q1)*cos(ref.q2)*sin(ref.q4) + l2*m2*cos(ref.q3)*cos(ref.q4)*sin(ref.q1) + l2*m2*cos(ref.q1)*cos(ref.q4)*sin(ref.q2)*sin(ref.q3))));
	
	return G;

}


#include <iostream>
#include <random>
#include <math.h>

#include "CKF.h"

class CKF2DPoint : public CKF {
public:
	MatrixXd stateFunction (MatrixXd s)
	{
		MatrixXd state(4,1);
		state(0,0) = s(0,0)+s(2,0);	// x position in 2D point
		state(1,0) = s(1,0)+s(3,0);	// y position in 2D point
		state(2,0) = s(2,0);		// velocity in x
		state(3,0) = s(3,0);		// velocity in y
		return state;
	}

	MatrixXd measurementFunction (MatrixXd m)
	{
		MatrixXd measurement(2,1);
		measurement(0,0) = m(0,0);	// measured x position in 2D point
		measurement(1,0) = m(1,0);	// measured y position in 2D point
		return measurement;
	}
};

int main ()
{

	cout.precision(5);
	cout << fixed;
	default_random_engine generator;			// for artificial noise
	normal_distribution<double> distribution(0.0,1.0);	// for artificial noise

	unsigned int n = 4;
	unsigned int m = 2;
	unsigned int N = 20;	// total number of steps

	CKF2DPoint tracked_point;
	tracked_point.n = n;
	tracked_point.m = m;

	MatrixXd I4(n,n);	// 4x4 Identity Matrix
	I4(0,0) = 1;	I4(1,1) = 1;	I4(2,2) = 1;	I4(3,3) = 1;
	MatrixXd I2(m,m);	// 2x2 Identity Matrix
	I2(0,0) = 1;	I2(1,1) = 1;
	
	MatrixXd s(n,1);	// initial state
	s(0,0) = 1;	s(1,0) = 1;	s(2,0) = 0;	s(3,0) = 0;

	MatrixXd x = s; // initial state
	const double q=0.1;	//std of process. "smoothness". lower the value, smoother the curve
	const double r=0.1;	//std of measurement. "tracking". lower the value, faster the track
	tracked_point.P = I4;	// state covriance
	tracked_point.Q = (q*q) * I4;	// covariance of process     (size must be nxn)
	tracked_point.R = (r*r) * I2;	// covariance of measurement (size must be mxm)
	
	MatrixXd xV(n,N);	// estmate        // allocate memory to show outputs
	MatrixXd sV(n,N);	// actual
	MatrixXd zV(m,N);	// measurement

	MatrixXd oneS = MatrixXd::Ones(n,1);	// Simulate addition
	MatrixXd oneZ = MatrixXd::Ones(m,1);	// Simulate Noise
	MatrixXd MatN = oneZ;  
	
	for (unsigned int k=0; k<N; k++)
	{

		double noise = distribution(generator);
		MatN = oneZ * fabs(noise);

		MatrixXd z = tracked_point.measurementFunction(s);  // make measurments
		z += MatN;			// add artificial noise in the measurement

		//std::cout << "z: \n" << z << "\n\n";

		for (unsigned int i=0; i<n; i++)
		{
			sV(i,k) = s(i,0);	// save actual state
		}
		for (unsigned int i=0; i<m; i++)
		{
			zV(i,k) = z(i,0);	// save measurement
		}
		tracked_point.ckf(x, z);
		for (unsigned int i=0; i<n; i++)
		{
			xV(i,k) = x(i,0);	// save estimate
		}

		s = tracked_point.stateFunction(s);	// update process with artificial increment
		s += oneS;				// simulate increment in state
		//std::cout << "s: \n" << s << "\n\n";
	}

	//cout << "sV:\n" << sV << endl;
	//cout << "\nxV:\n" << xV << endl;
	cout << "\nDifference between actual states and tracked measurements:\n";
	for (unsigned int k=0; k<N; k++)
	{		
		cout << k << ": " << abs( sV(0,k) - xV(0,k) ) << "  (" << sV(0,k) << '-' << xV(0,k) << ")\n";
	}
	

	/* In actual practice, your application should look like this! */
	//for (unsigned int k=0; k<N; k++)
	//{
	//	MatrixXd z = [measurements];  // noised measurment
	//	YourUKF.ukf(x, z);
	//	MatrixXd w = YourUKF.measurement_function(x);	// estimated/corrected measurement
	//}


	return 0;
}

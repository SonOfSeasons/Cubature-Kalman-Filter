
#include <iostream>
#include <cmath>

#include<Eigen/Core>


#include<Eigen/Dense>

using namespace std;
using namespace Eigen;

class CKF {
public:		
	void ckf( MatrixXd& x, const MatrixXd z);
	virtual MatrixXd stateFunction (MatrixXd x) = 0;
	virtual MatrixXd measurementFunction (MatrixXd x) = 0;

	int n;      //number of state
	int m;      //number of measurement
	MatrixXd Q;	//noise covariance of process	(size must be nxn)
	MatrixXd R;	//noise covariance of measurement (size must be mxm)
	MatrixXd P;	//state covariance
};

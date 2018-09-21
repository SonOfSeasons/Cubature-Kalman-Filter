
#include "CKF.h"

void CKF::ckf( MatrixXd& x, const MatrixXd z)
{

	/// Generate Cubature Points


        int k = 2*n;

        MatrixXd sigmas(k,n);

        LLT<MatrixXd> lltOfA(P);
        MatrixXd P_chol = lltOfA.matrixU(); // Cholesky decomposition 
        
        MatrixXd U; 
        U = P_chol * sqrt(n);

        MatrixXd x_trans = x.transpose();

        for (int i = 0; i<n; i++)
        {
            sigmas.row(i)   = x_trans + U.row(i);
            sigmas.row(n+i) = x_trans - U.row(i); 
        }

	/* unscented transformation (ut) of process */

	// Evaluate Cubature Points
        MatrixXd sigmas_f(k,n);     // Evaluated values of Sigma
        MatrixXd sigmas_trans;

        for (int i = 0; i<k; i++)
        {
            sigmas_trans = sigmas.row(i).transpose();
            sigmas_f.row(i) = stateFunction(sigmas_trans).transpose();
        }

        // Cubature Transform 
        MatrixXd sigmas_f_sum = sigmas_f.colwise().sum();
        MatrixXd x_f = sigmas_f_sum / k;
        MatrixXd P_transform = MatrixXd::Zero(n,n);

        MatrixXd temp = x_f.transpose() * x_f;
        MatrixXd sigmas_f_trans;
        
        for(int i = 0; i < k; i++)
        {
            sigmas_f_trans = sigmas_f.row(i).transpose();
            P_transform.noalias() += sigmas_f_trans * sigmas_f.row(i);
            P_transform -= temp;
        }

	// Last Steps
        P_transform /= k;
        P_transform += Q;



	/* unscented transformation (ut) of measurements */
	// Update Step

        MatrixXd sigmas_h(k,m);     // Evaluated values of Sigma

        for (int i = 0; i<k; i++)
        {
            sigmas_f_trans = sigmas_f.row(i).transpose();
            sigmas_h.row(i) = measurementFunction(sigmas_f_trans).transpose();
        }

        // Cubature Transform
        MatrixXd x_h = sigmas_h.colwise().sum() / k;
        MatrixXd P_h = MatrixXd::Zero(m,m);
        
        MatrixXd x_h_temp = x_h.transpose() * x_h;
        
        for(int i = 0; i < k; i++)
        {
            P_h.noalias() += sigmas_h.row(i).transpose() * sigmas_h.row(i);
            P_h -= x_h_temp;
        }

        P_h = P_h / k;
        P_h = P_h + R;

        // Compute the cross-covariance of the state and measurements
        MatrixXd Pxz = MatrixXd::Zero(n,m);

        MatrixXd dx (k,1);
        MatrixXd dz (k,1);
        MatrixXd dx_trans (1,k);

        for(int i=0; i < k; i++)
        {
            dx = sigmas_f.row(i) - x_f;
            dz = sigmas_h.row(i) - x_h;
            dx_trans = dx.transpose();
            Pxz.noalias() += dx_trans * dz;
        }

        Pxz /= k;

        MatrixXd P_h_trans = P_h.inverse();
        MatrixXd K = Pxz * P_h_trans;


        // Calculate Residual
        MatrixXd x_h_trans = x_h.transpose();
        MatrixXd y = z - x_h_trans;
        
        MatrixXd last_x = x_f.transpose() + (K * y);

        MatrixXd K_trans = K.transpose();
        MatrixXd last_P = P_transform - K * P_h * K_trans;

	// Copy Results
	x = last_x;
	P = last_P;
}

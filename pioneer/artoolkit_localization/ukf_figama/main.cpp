#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

MatrixXd prEq(MatrixXd xk_s, Vector2d uk);
vector<MatrixXd> ukfPred(Vector3d xk_1, Matrix3d Pk_1, Vector2d uk, Matrix3d Qk);

int main()
{
  	Vector3d xk_1(1, 2, 3);
			
	Matrix3d Pk_1;
	Pk_1 << 1, 0, 0,
	        0, 1, 0,
	        0, 0, 1;
	              
	Vector2d uk(11, -0.00606060606060606060606060);
	
	Matrix3d Qk;
	Qk << 1, 0, 0,
	      0, 1, 0,
	      0, 0, 1;
	      
	vector <MatrixXd> ret;
	
	ret = ukfPred(xk_1, Pk_1, uk, Qk);
	
	//cout << ret[0] << endl << ret[1] << endl;
  	  
  }
  
vector<MatrixXd> ukfPred(Vector3d xk_1, Matrix3d Pk_1, Vector2d uk, Matrix3d Qk)
{
	double kappa = 1;
	
	Vector3d xk;
	Matrix3d Pk;
	
	MatrixXd xk_s(3,7);
	
	double wc = kappa / (3 + kappa);
	double we = 1/(2*(3+kappa));
	
	xk_s.col(0) = xk_1;
	
	Matrix3d aux = (xk_1 * MatrixXd::Constant(1,3,1));
	Matrix3d aux2 = ((3 + kappa) * Pk_1).llt().matrixL().transpose();
	
	xk_s.block(0,1,3,3) = aux + aux2;
	xk_s.block(0,4,3,3) = aux - aux2;

	xk_s = prEq(xk_s, uk);
	
	xk = wc * xk_s.col(0);
	
	for (int i = 1; i < 7; i++)
	{
		xk = xk + we * xk_s.col(i);
	}
	
	Pk = wc * ((xk_s.col(0) - xk) * ((xk_s.col(0) - xk).transpose()));
	cout << endl << Pk << endl;
	
	for (int i = 1; i < 7; i++)
	{
		Pk = Pk + we * ((xk_s.col(i) - xk) * ((xk_s.col(i) - xk).transpose()));
		cout << endl << Pk << endl;
	}
	
	Pk = Pk + Qk;
	
	vector <MatrixXd> ret;
	
	ret.push_back(xk);
	ret.push_back(Pk);
	
	return ret;
}

MatrixXd prEq(MatrixXd xk_s, Vector2d uk)
{
	int n = xk_s.cols();
	double dt = 0.1;
	
	Vector3d in(uk(0,0), 0, uk(1,0));
	Matrix3d rotMat;
	MatrixXd xk(3,n);
	
	for(int i = 0; i < n; i++)
	{
		rotMat << cos(xk_s(2,i)), sin(xk_s(2,i)), 0,
		          sin(xk_s(2,i)), (-cos(xk_s(2,i))), 0,
		          0, 0, 1;
		          
		xk.col(i) = xk_s.col(i) + rotMat * in * dt;
	}
	
	return xk;
}

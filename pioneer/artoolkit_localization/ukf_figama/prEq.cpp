#include <Eigen/Dense>
#include <math.h>;

using namespace Eigen;

MatrixXd prEq(MatrixXd xk_s, Vector2d uk)
{
	int n = size(xk_s);
	double dt = 0.1;
	
	Vector3d in(uk(0,0), 0, uk(1,0));
	Matrix3d rotMat;
	MatrixXd xk(3,7);
	
	for(int i = 0; i < 7; i++)
	{
		rotMat << cos(xk_s(2,i)), sin(xk_s(2,i)), 0,
		          sin(xk_s(2,i)), (-cos(xk_s(2,i))), 0,
		          0, 0, 1;
		          
		xk.col(i) = rotMat * in * dt;
	}
	
	return xk;
}

#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

vector<MatrixXd> ukfMed(Vector3d xk, Matrix3d Pk, map<int, vector<double>> zk, Matrix3d Rk)
{
	for(map< int, vector <double> >::iterator im = zk.begin(); im != zk.end(); im++)
	{
	
		zk[i]
		
		stringstream ss;

		ss << im->first << ":";

		for(vector<double>::iterator iv = im->second.begin(); iv != im->second.end(); iv++)
			ss << " " << *iv;

		ROS_INFO("%s",ss.str().c_str());
	}
}

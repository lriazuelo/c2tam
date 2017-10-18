#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/Jacobi>
#include <Eigen/Eigenvalues>
#include <iostream>

using namespace Eigen;
using namespace std;

int main(int, char**)
{
  cout.precision(3);
  VectorXd v(10);
v.resize(3);
RowVector3d w;
w.resize(3); // this is legal, but has no effect
cout << "v: " << v.rows() << " rows, " << v.cols() << " cols" << endl;
cout << "w: " << w.rows() << " rows, " << w.cols() << " cols" << endl;

  return 0;
}

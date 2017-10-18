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
  Matrix3d m = Matrix3d::Identity();
m(0,2) = 1e-4;
cout << "Here's the matrix m:" << endl << m << endl;
cout << "m.isUnitary() returns: " << m.isUnitary() << endl;
cout << "m.isUnitary(1e-3) returns: " << m.isUnitary(1e-3) << endl;

  return 0;
}

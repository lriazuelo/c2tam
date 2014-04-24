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
  Matrix3d m = 10000 * Matrix3d::Identity();
m(0,2) = 1;
cout << "Here's the matrix m:" << endl << m << endl;
cout << "m.isDiagonal() returns: " << m.isDiagonal() << endl;
cout << "m.isDiagonal(1e-3) returns: " << m.isDiagonal(1e-3) << endl;


  return 0;
}

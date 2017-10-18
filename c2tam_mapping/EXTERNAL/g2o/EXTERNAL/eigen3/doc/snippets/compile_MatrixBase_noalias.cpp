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
  Matrix2d a, b, c; a << 1,2,3,4; b << 5,6,7,8;
c.noalias() = a * b; // this computes the product directly to c
cout << c << endl;

  return 0;
}

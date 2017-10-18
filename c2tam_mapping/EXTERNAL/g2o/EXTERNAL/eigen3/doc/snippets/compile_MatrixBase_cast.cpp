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
  Matrix2d md = Matrix2d::Identity() * 0.45;
Matrix2f mf = Matrix2f::Identity();
cout << md + mf.cast<double>() << endl;

  return 0;
}

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
  Matrix3d m = Matrix3d::Random();
cout << "Here is the matrix m:" << endl << m << endl;
cout << "Its inverse is:" << endl << m.inverse() << endl;

  return 0;
}

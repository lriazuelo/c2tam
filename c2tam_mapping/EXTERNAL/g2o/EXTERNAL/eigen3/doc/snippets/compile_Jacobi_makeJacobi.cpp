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
  Matrix2f m = Matrix2f::Random();
m = (m + m.adjoint()).eval();
JacobiRotation<float> J;
J.makeJacobi(m, 0, 1);
cout << "Here is the matrix m:" << endl << m << endl;
m.applyOnTheLeft(0, 1, J.adjoint());
m.applyOnTheRight(0, 1, J);
cout << "Here is the matrix J' * m * J:" << endl << m << endl;
  return 0;
}

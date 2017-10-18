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
  Matrix2cf m = Matrix2cf::Random();
cout << "Here is the 2x2 complex matrix m:" << endl << m << endl;
cout << "Here is the adjoint of m:" << endl << m.adjoint() << endl;

  return 0;
}

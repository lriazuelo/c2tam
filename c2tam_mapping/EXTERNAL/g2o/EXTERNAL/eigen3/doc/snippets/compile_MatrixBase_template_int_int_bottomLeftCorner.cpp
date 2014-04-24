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
  Matrix4i m = Matrix4i::Random();
cout << "Here is the matrix m:" << endl << m << endl;
cout << "Here is m.bottomLeftCorner<2,2>():" << endl;
cout << m.bottomLeftCorner<2,2>() << endl;
m.bottomLeftCorner<2,2>().setZero();
cout << "Now the matrix m is:" << endl << m << endl;

  return 0;
}

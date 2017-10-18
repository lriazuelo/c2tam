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
cout << "Here are the coefficients on the 1st super-diagonal and 2nd sub-diagonal of m:" << endl
     << m.diagonal<1>().transpose() << endl
     << m.diagonal<-2>().transpose() << endl;

  return 0;
}

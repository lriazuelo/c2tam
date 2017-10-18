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
  Matrix2i m = Matrix2i::Random();
cout << "Here is the matrix m:" << endl << m << endl;
cout << "Here is the transpose of m:" << endl << m.transpose() << endl;
cout << "Here is the coefficient (1,0) in the transpose of m:" << endl
     << m.transpose()(1,0) << endl;
cout << "Let us overwrite this coefficient with the value 0." << endl;
m.transpose()(1,0) = 0;
cout << "Now the matrix m is:" << endl << m << endl;

  return 0;
}

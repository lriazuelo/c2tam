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
  RowVector4i v = RowVector4i::Random();
cout << "Here is the vector v:" << endl << v << endl;
cout << "Here is v.head(2):" << endl << v.head(2) << endl;
v.head(2).setZero();
cout << "Now the vector v is:" << endl << v << endl;

  return 0;
}

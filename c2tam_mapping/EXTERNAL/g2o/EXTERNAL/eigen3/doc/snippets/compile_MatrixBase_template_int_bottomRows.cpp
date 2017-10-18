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
  Array44i a = Array44i::Random();
cout << "Here is the array a:" << endl << a << endl;
cout << "Here is a.bottomRows<2>():" << endl;
cout << a.bottomRows<2>() << endl;
a.bottomRows<2>().setZero();
cout << "Now the array a is:" << endl << a << endl;

  return 0;
}

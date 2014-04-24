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
  Array33i a = Array33i::Random(), b = Array33i::Random();
Array33i c = a * b;
cout << "a:\n" << a << "\nb:\n" << b << "\nc:\n" << c << endl;


  return 0;
}

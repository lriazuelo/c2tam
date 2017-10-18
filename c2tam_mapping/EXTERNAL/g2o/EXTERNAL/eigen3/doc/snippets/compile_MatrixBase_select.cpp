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
  MatrixXi m(3, 3);
m << 1, 2, 3,
     4, 5, 6,
     7, 8, 9;
m = (m.array() >= 5).select(-m, m);
cout << m << endl;

  return 0;
}

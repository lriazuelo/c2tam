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
  MatrixXi m(2,2);
m << 1, 0,
     1, 1;
cout << "Comparing m with identity matrix:" << endl;
cout << m.cwiseNotEqual(MatrixXi::Identity(2,2)) << endl;
int count = m.cwiseNotEqual(MatrixXi::Identity(2,2)).count();
cout << "Number of coefficients that are not equal: " << count << endl;

  return 0;
}

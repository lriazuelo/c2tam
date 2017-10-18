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
  MatrixXi m = MatrixXi::Random(2,3);
cout << "Here is the matrix m:" << endl << m << endl;
cout << "m.replicate<3,2>() = ..." << endl;
cout << m.replicate<3,2>() << endl;

  return 0;
}

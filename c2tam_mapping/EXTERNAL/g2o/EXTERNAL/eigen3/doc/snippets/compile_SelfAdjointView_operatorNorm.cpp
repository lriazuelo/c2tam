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
  MatrixXd ones = MatrixXd::Ones(3,3);
cout << "The operator norm of the 3x3 matrix of ones is "
     << ones.selfadjointView<Lower>().operatorNorm() << endl;

  return 0;
}

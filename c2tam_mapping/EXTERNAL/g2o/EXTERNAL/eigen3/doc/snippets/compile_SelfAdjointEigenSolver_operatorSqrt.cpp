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
  MatrixXd X = MatrixXd::Random(4,4);
MatrixXd A = X * X.transpose();
cout << "Here is a random positive-definite matrix, A:" << endl << A << endl << endl;

SelfAdjointEigenSolver<MatrixXd> es(A);
MatrixXd sqrtA = es.operatorSqrt();
cout << "The square root of A is: " << endl << sqrtA << endl;
cout << "If we square this, we get: " << endl << sqrtA*sqrtA << endl;

  return 0;
}

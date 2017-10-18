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
  MatrixXd A = MatrixXd::Random(3,3);
MatrixXd B = MatrixXd::Random(3,2);
cout << "Here is the invertible matrix A:" << endl << A << endl;
cout << "Here is the matrix B:" << endl << B << endl;
MatrixXd X = A.lu().solve(B);
cout << "Here is the (unique) solution X to the equation AX=B:" << endl << X << endl;
cout << "Relative error: " << (A*X-B).norm() / B.norm() << endl;

  return 0;
}

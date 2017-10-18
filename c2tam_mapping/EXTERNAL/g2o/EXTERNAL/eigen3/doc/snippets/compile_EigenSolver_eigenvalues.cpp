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
EigenSolver<MatrixXd> es(ones, false);
cout << "The eigenvalues of the 3x3 matrix of ones are:" 
     << endl << es.eigenvalues() << endl;

  return 0;
}

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
  MatrixXcf A = MatrixXcf::Random(4,4);
cout << "Here is a random 4x4 matrix, A:" << endl << A << endl << endl;
ComplexSchur<MatrixXcf> schurOfA(A);
cout << "The unitary matrix U is:" << endl << schurOfA.matrixU() << endl;

  return 0;
}

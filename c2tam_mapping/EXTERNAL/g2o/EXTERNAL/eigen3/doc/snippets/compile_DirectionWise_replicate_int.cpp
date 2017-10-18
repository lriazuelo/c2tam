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
  Vector3i v = Vector3i::Random();
cout << "Here is the vector v:" << endl << v << endl;
cout << "v.rowwise().replicate(5) = ..." << endl;
cout << v.rowwise().replicate(5) << endl;

  return 0;
}

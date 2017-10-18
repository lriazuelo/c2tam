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
  int array[12];
for(int i = 0; i < 12; ++i) array[i] = i;
cout << Map<MatrixXi, 0, OuterStride<> >(array, 3, 3, OuterStride<>(4)) << endl;

  return 0;
}

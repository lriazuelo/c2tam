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
cout << Map<VectorXi, 0, InnerStride<2> >
         (array, 6) // the inner stride has already been passed as template parameter
     << endl;

  return 0;
}

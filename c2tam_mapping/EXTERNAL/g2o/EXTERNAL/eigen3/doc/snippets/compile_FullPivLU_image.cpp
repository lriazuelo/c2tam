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
  Matrix3d m;
m << 1,1,0,
     1,3,2,
     0,1,1;
cout << "Here is the matrix m:" << endl << m << endl;
cout << "Notice that the middle column is the sum of the two others, so the "
     << "columns are linearly dependent." << endl;
cout << "Here is a matrix whose columns have the same span but are linearly independent:"
     << endl << m.fullPivLu().image(m) << endl;

  return 0;
}

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
  #ifndef _MSC_VER
  #warning deprecated
#endif
/* deprecated
Matrix3i m = Matrix3i::Random();
cout << "Here is the matrix m:" << endl << m << endl;
cout << "Here is the upper-triangular matrix extracted from m:" << endl
     << m.part<Eigen::UpperTriangular>() << endl;
cout << "Here is the strictly-upper-triangular matrix extracted from m:" << endl
     << m.part<Eigen::StrictlyUpperTriangular>() << endl;
cout << "Here is the unit-lower-triangular matrix extracted from m:" << endl
     << m.part<Eigen::UnitLowerTriangular>() << endl;
*/
  return 0;
}

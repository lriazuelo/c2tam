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
/*
Matrix3d m = Matrix3d::Zero();
m.part<Eigen::UpperTriangular>().setOnes();
cout << "Here is the matrix m:" << endl << m << endl;
Matrix3d n = Matrix3d::Ones();
n.part<Eigen::LowerTriangular>() *= 2;
cout << "Here is the matrix n:" << endl << n << endl;
cout << "And now here is m.inverse()*n, taking advantage of the fact that"
        " m is upper-triangular:" << endl
     << m.marked<Eigen::UpperTriangular>().solveTriangular(n);
*/
  return 0;
}

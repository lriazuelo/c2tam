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
  Vector3d v(-1,2,-3);
cout << "the absolute values:" << endl << v.array().abs() << endl;
cout << "the absolute values plus one:" << endl << v.array().abs()+1 << endl;
cout << "sum of the squares: " << v.array().square().sum() << endl;

  return 0;
}

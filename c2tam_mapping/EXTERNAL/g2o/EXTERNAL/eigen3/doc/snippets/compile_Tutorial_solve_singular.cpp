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
  Matrix3f A;
Vector3f b;
A << 1,2,3,  4,5,6,  7,8,9;
b << 3, 3, 4;
cout << "Here is the matrix A:" << endl << A << endl;
cout << "Here is the vector b:" << endl << b << endl;
Vector3f x;
x = A.lu().solve(b);
cout << "The solution is:" << endl << x << endl;

  return 0;
}

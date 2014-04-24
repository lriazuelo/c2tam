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
  Vector2f v = Vector2f::Random();
JacobiRotation<float> G;
G.makeGivens(v.x(), v.y());
cout << "Here is the vector v:" << endl << v << endl;
v.applyOnTheLeft(0, 1, G.adjoint());
cout << "Here is the vector J' * v:" << endl << v << endl;
  return 0;
}

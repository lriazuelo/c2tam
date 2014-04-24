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
  RowVectorXd vec1(3);
vec1 << 1, 2, 3;
std::cout << "vec1 = " << vec1 << std::endl;

RowVectorXd vec2(4);
vec2 << 1, 4, 9, 16;;
std::cout << "vec2 = " << vec2 << std::endl;

RowVectorXd joined(7);
joined << vec1, vec2;
std::cout << "joined = " << joined << std::endl;

  return 0;
}

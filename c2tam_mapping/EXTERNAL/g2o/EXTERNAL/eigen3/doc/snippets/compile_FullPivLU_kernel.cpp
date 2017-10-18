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
  MatrixXf m = MatrixXf::Random(3,5);
cout << "Here is the matrix m:" << endl << m << endl;
MatrixXf ker = m.fullPivLu().kernel();
cout << "Here is a matrix whose columns form a basis of the kernel of m:"
     << endl << ker << endl;
cout << "By definition of the kernel, m*ker is zero:"
     << endl << m*ker << endl;

  return 0;
}

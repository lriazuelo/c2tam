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
  MatrixXf matA(2,2), matB(2,2); 
matA << 2, 0,  0, 2;

// Simple but not quite as efficient
matB = matA * matA;
cout << matB << endl << endl;

// More complicated but also more efficient
matB.noalias() = matA * matA;
cout << matB;

  return 0;
}

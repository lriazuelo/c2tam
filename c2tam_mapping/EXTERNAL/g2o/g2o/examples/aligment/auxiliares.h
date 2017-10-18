#ifndef _AUXILIARES_
#define _AUXILIARES_

#include <Eigen/StdVector>
#include <tr1/random>
#include <iostream>
#include <stdint.h>
#include <tr1/unordered_set>

/*
#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/structure_only_solver.h"
*/
#include "tokenizer.h"

#include <iostream>
#include <fstream>

using namespace Eigen;
using namespace std;



// CARGAR DATOS
typedef struct
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector < Matrix4d, aligned_allocator<Matrix4d> > camg2o;
  vector < Matrix4d, aligned_allocator<Matrix4d> > camGT;
} Data;
////////////////////////////////////////////////////////////////////////////////

/*
// MATLAB
typedef Matrix<double,2,1> Vector2;

////////////////////////////////////////////////////////////////////////////////

typedef struct
{
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   int n, fija;
   //SE3AxisAngle c;
   //Vector6 c;
   Matrix4 Ecw;
} CamaraMatlab;

////////////////////////////////////////////////////////////////////////////////

typedef struct
{
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   int n;
   //VertexPointXYZ p;
   Vector3 p;
} PuntoMatlab;

////////////////////////////////////////////////////////////////////////////////

typedef struct
{
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   int cam, pto, espurio;
   Vector2d m;
} MedicionMatlab;

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

/*
void LoadData(const char * fich, Data &dat, unsigned int &CAMARAS);

void generarFicheros
(
  g2o::BlockSolverX *solver_ptr,
  g2o::VertexKFOV4 *vKGT,
  vector<CamaraMatlab*> &camGT,
  vector<PuntoMatlab*> &ptoGT,
  vector<MedicionMatlab*> &medGT,
  g2o::VertexKFOV4 *vK0,
  vector<CamaraMatlab*> &cam0, 
  vector<PuntoMatlab*> &pto0,
  vector<MedicionMatlab*> &med,
  g2o::VertexKFOV4 *vKF,
  vector<CamaraMatlab*> &camF,
  vector<PuntoMatlab*> &ptoF
);

*/
#endif

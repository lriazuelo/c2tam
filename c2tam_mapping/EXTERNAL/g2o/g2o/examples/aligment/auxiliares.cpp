#include "auxiliares.h"


void LoadData(const char * fich, Data &dat, unsigned int &CAMARAS)
{
   Token t;
   if(t.abrirFichero(fich) != Token::ok)
   {
      cerr << "FICHERO INVALIDO\n";
      exit (-1);
   }

   int caso = 0;
   while(t.leeLinea() == Token::ok)
   {
      if(t.tokenizar(" \t") == 0)
      {
        caso++;
        continue;
      }

      Matrix4 cam;
      cam = Matrix4d::Identity();
      cerr << setprecision(10) << fixed;
      switch(caso)
      {
         case 0: // CamG2o
            for(int i = 0; i < t.getnPalabras(); i++)
            {
               int f = i%3, c = i/3;
               //cerr << "i: " << i << "  (f,c): (" << f << "," << c << ")\n";
               //cam(i) = atof(t.token(i));
               cam(f,c) = atof(t.token(i));
            }
            //cerr << fixed << setprecision(5) << cam << endl;
            dat.camg2o.push_back(cam);
            break;
          case 1: // CamG2o
            for(int i = 0; i < t.getnPalabras(); i++)
            {
               int f = i%3, c = i/3;
               //cerr << "i: " << i << "  (f,c): (" << f << "," << c << ")\n";
               //cam(i) = atof(t.token(i));
               cam(f,c) = atof(t.token(i));
            }
            //cerr << fixed << setprecision(5) << cam << endl;
            dat.camGT.push_back(cam);
   
         default:
            ;
      }
   }
   CAMARAS = dat.camGT.size();
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

/*
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
)
{
  solver_ptr->HppToFile("Hpp.m");
  solver_ptr->HplToFile("Hpl.m");
  solver_ptr->HllToFile("Hll.m");

  
  //////////////////////////////////////
  /// GENERACION DEL FICHERO  MATLAB ///
  //////////////////////////////////////

  ofstream file ("g2o.m", ofstream::out | ofstream::trunc);

  if (!file) return;

  file << setprecision(15) << fixed;


  file << "kGT = [" << vKGT->estimate().transpose() << "];\n\n\n";


  file << "camsGT = [\n";
  for(size_t c = 0; c < camGT.size(); c++)
  {
     file << camGT[c]->n << " " << camGT[c]->fija << " "
          << camGT[c]->Ecw.col(0).transpose() << " "
          << camGT[c]->Ecw.col(1).transpose() << " "
          << camGT[c]->Ecw.col(2).transpose() << " "
          << camGT[c]->Ecw.col(3).transpose() << endl;
  }
  file << "];\n\n\n";


  file << "ptosGT = [\n";
  for(size_t p=0; p < ptoGT.size(); p++)
  {
     file << ptoGT[p]->n << " " << ptoGT[p]->p.transpose() << endl;
  }
  file << "];\n\n\n";



  file << "medsGT = [\n";
  for(size_t m=0; m < medGT.size(); m++)
  {
     file << medGT[m]->cam << " " << medGT[m]->pto << " "
          << medGT[m]->espurio << " " << medGT[m]->m.transpose() << endl;
  }
  file << "];\n\n\n";



  file << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n"
       << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n"
       << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";



  file << "k0 = [" << vK0->estimate().transpose() << "];\n\n\n";


  file << "cams0 = [\n";
  for(size_t c = 0; c < cam0.size(); c++)
  {
     file << cam0[c]->n << " " << cam0[c]->fija << " "
          << cam0[c]->Ecw.col(0).transpose() << " "
          << cam0[c]->Ecw.col(1).transpose() << " "
          << cam0[c]->Ecw.col(2).transpose() << " "
          << cam0[c]->Ecw.col(3).transpose() << endl;
  }
  file << "];\n\n\n";


  file << "ptos0 = [\n";
  for(size_t p=0; p < pto0.size(); p++)
  {
     file << pto0[p]->n << " " << pto0[p]->p.transpose() << endl;
  }
  file << "];\n\n\n";


  file << "meds0 = [\n";
  for(size_t m=0; m < med.size(); m++)
  {
     file << med[m]->cam << " " << med[m]->pto << " "
          << med[m]->espurio << " " << med[m]->m.transpose() << endl;
  }
  file << "];\n\n\n";



  file << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n"
       << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n"
       << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";



  file << "kF = [" << vKF->estimate().transpose() << "];\n\n\n";


  file << "camsF = [\n";
  for(size_t c = 0; c < camF.size(); c++)
  {
     file << camF[c]->n << " " << camF[c]->fija << " "
          << camF[c]->Ecw.col(0).transpose() << " "
          << camF[c]->Ecw.col(1).transpose() << " "
          << camF[c]->Ecw.col(2).transpose() << " "
          << camF[c]->Ecw.col(3).transpose() << endl;
  }
  file << "];\n\n\n";


  file << "ptosF = [\n";
  for(size_t p=0; p < ptoF.size(); p++)
  {
     file << ptoF[p]->n << " " << ptoF[p]->p.transpose() << endl;
  }
  file << "];\n\n\n";

  file.close();
}*/

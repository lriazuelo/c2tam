// Copyright 2008 Isis Innovation Limited
#include "MapMaker.h"
#include "MapPoint.h"
#include "Bundle.h"
#include "PatchFinder.h"
#include "SmallMatrixOpts.h"
#include "HomographyInit.h"

#include <cvd/vector_image_ref.h>
#include <cvd/vision.h>
#include <cvd/image_interpolate.h>


#include <TooN/SVD.h>
#include <TooN/SymEigen.h>

#include <gvars3/instances.h>
#include <iostream>
#include <fstream>
#include <sys/time.h>

#include <fstream>
#include <algorithm>   

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

using namespace CVD;
using namespace std;
using namespace GVars3;

// Constructor sets up internal reference variable to Map.
// Most of the intialisation is done by Reset()..
MapMaker::MapMaker(Map& m, Map& mTracker)
 : mMap(m), mMapTracker(mTracker) 
{
  mbResetRequested = false;

  Reset();
  start(); // This CVD::thread func starts the map-maker thread with function run()
  GUI.RegisterCommand("SaveMap", GUICommandCallBack, this);
  GV3::Register(mgvdWiggleScale, "MapMaker.WiggleScale", 0.1, SILENT); // Default to 10cm between keyframes

/*  os.open("salida.txt", ios_base::out | ios_base::trunc);
  timeBAG.open("timeBAG.dat",ios_base::out|ios_base::trunc);
  timeBAG << "%tiempo \t N_KF \t N_Pts \t ConvG\n";
  timeBAL.open("timeBAL.dat",ios_base::out|ios_base::trunc);
  timeBAL << "%tiempo \t N_KF \t N_Pts \t ConvL \n";
  timeT.open("timeT.dat",ios_base::out|ios_base::trunc);
  timeT << "%tiempo \t N_KF \t N_Pts\n";
*/

  //LUIS NEW CAMERA FOR MAPPING
  mCamera = new ATANCamera("Camera");
  mRelocaliser =  new Relocaliser(mMap, mCamera);

};

void MapMaker::Reset()
{
  // This is only called from within the mapmaker thread...
  mMap.Reset();
  mMapTracker.Reset(); //LUIS
  mvFailureQueue.clear();
  while(!mqNewQueue.empty()) mqNewQueue.pop();
  mMap.vpKeyFrames.clear(); // TODO: actually erase old keyframes
  mMapTracker.vpKeyFrames.clear(); //Roboearth
  mvpKeyFrameQueue.clear(); // TODO: actually erase old keyframes
  mbBundleRunning = false;
  mbBundleConverged_Full = true;
  mbBundleConverged_Recent = true;
  mbResetDone = true;
  mbResetRequested = false;
  mbBundleAbortRequested = false;

  aux_KF.dSceneDepthMean = 1.0;
  aux_KF.dSceneDepthSigma = 1.0;


}

// CHECK_RESET is a handy macro which makes the mapmaker thread stop
// what it's doing and reset, if required.
#define CHECK_RESET if(mbResetRequested) {Reset(); continue;};

void MapMaker::run()
{

#ifdef WIN32
  // For some reason, I get tracker thread starvation on Win32 when
  // adding key-frames. Perhaps this will help:
  SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_LOWEST);
#endif

  while(!shouldStop())  // ShouldStop is a CVD::Thread func which return true if the thread is told to exit.
  {
     CHECK_RESET;
     sleep(5); // Sleep not really necessary, especially if mapmaker is busy
     CHECK_RESET;

     // Handle any GUI commands encountered..
     while(!mvQueuedCommands.empty())
     {
        GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
        mvQueuedCommands.erase(mvQueuedCommands.begin());
     }

     if(!mMap.IsGood())  // Nothing to do if there is no map yet!
        continue;

     // From here on, mapmaker does various map-maintenance jobs in a certain priority
     // Hierarchy. For example, if there's a new key-frame to be added (QueueSize() is >0)
     // then that takes high priority.

     CHECK_RESET;
     // Should we run local bundle adjustment?
     if(!mbBundleConverged_Recent && QueueSize() == 0)
     {

        BundleAdjustRecentM();  //LUIS
     }
     CHECK_RESET;
     // Are there any newly-made map points which need more measurements from older key-frames?
     if(mbBundleConverged_Recent && QueueSize() == 0)
        ReFindNewlyMade();

     CHECK_RESET;
     // Run global bundle adjustment?
	// Marta 25th May
     if(mbBundleConverged_Recent && !mbBundleConverged_Full && QueueSize() == 0)
     {
        BundleAdjustAll();
     }
	//End Marta

     CHECK_RESET;
     // Very low priorty: re-find measurements marked as outliers
     if(mbBundleConverged_Recent && mbBundleConverged_Full && rand()%20 == 0 && QueueSize() == 0)
        ReFindFromFailureQueue();

     CHECK_RESET;
     HandleBadPoints();

     CHECK_RESET;
     // Any new key-frames to be added?
     if(QueueSize() > 0)
        AddKeyFrameFromTopOfQueue(); // Integrate into map data struct, and process
  }
}


// Tracker calls this to demand a reset
void MapMaker::RequestReset()
{
  mbResetDone = false;
  mbResetRequested = true;
}

bool MapMaker::ResetDone()
{
  return mbResetDone;
}

// HandleBadPoints() Does some heuristic checks on all points in the map to see if
// they should be flagged as bad, based on tracker feedback.
void MapMaker::HandleBadPoints()
{
  // Did the tracker see this point as an outlier more often than as an inlier?
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
      MapPoint &p = *mMap.vpPoints[i];
      MapPoint &pT = *mMapTracker.vpPoints[i];  //LUIS

      if(p.nMEstimatorOutlierCount > 20 && p.nMEstimatorOutlierCount > p.nMEstimatorInlierCount){
	p.bBad = true;
	pT.bBad = true; //LUIS
      }
    }

  // All points marked as bad will be erased - erase all records of them
  // from keyframes in which they might have been measured.

  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    if(mMap.vpPoints[i]->bBad)
      {
	MapPoint *p = mMap.vpPoints[i];
	for(unsigned int j=0; j<mMap.vpKeyFrames.size(); j++)
	  {
	    KeyFrame &k = *mMap.vpKeyFrames[j];
	    if(k.mMeasurements.count(p))
	      k.mMeasurements.erase(p);
	  }
      }
  // Move bad points to the trash list.

  mMapTracker.MoveBadPointsToTrash(); //LUIS
  mMap.MoveBadPointsToTrash();

  for(unsigned int j=0; j<mMap.vpKeyFrames.size(); j++){
    Vector<3> v = mMap.vpKeyFrames[j]->se3CfromW.inverse().get_translation();
    mMapTracker.vpKeyFrames[j]->se3CfromW = mMap.vpKeyFrames[j]->se3CfromW;
  }


}

MapMaker::~MapMaker()
{
  mbBundleAbortRequested = true;
  cout << "Waiting for mapmaker to die.." << endl;
  stop(); // makes shouldStop() return true
  cout << "Waiting for mapmaker to die.." << endl;
  join();
  cout << " .. mapmaker has died." << endl;
/*
  for (int i=0; i<mMap.vpKeyFrames.size(); i++)
  {
       std::ostringstream filename;
       filename << "keyFrame" << i << ".png";
       cerr << filename.str() << endl;
  }
*/
}


// Finds 3d coords of point in reference frame B from two z=1 plane projections
Vector<3> MapMaker::ReprojectPoint(SE3<> se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B)
{
  // El calculo que hace es:
  // PA(X,Y,Z): Coordenadas del pto en A (Z != 1)
  // PB(X,Y,Z): Coordenadas del pto en B (Z != 1)
  // PA = se3AfromB * PB
  // v2A(x) * PA(Z) - PA(X) = 0
  // v2A(y) * PA(Z) - PA(Y) = 0
  // v2B(x) * PB(Z) - PB(X) = 0
  // v2B(y) * PB(Z) - PB(Y) = 0
  // 3 incognitas PB(X,Y,Z) 4 ecs. => Minimos cuadrados mediante svd.

  Matrix<3,4> PDash;
  PDash.slice<0,0,3,3>() = se3AfromB.get_rotation().get_matrix();
  PDash.slice<0,3,3,1>() = se3AfromB.get_translation().as_col();

  Matrix<4> A;
  A[0][0] = -1.0; A[0][1] =  0.0; A[0][2] = v2B[0]; A[0][3] = 0.0;
  A[1][0] =  0.0; A[1][1] = -1.0; A[1][2] = v2B[1]; A[1][3] = 0.0;
  A[2] = v2A[0] * PDash[2] - PDash[0];
  A[3] = v2A[1] * PDash[2] - PDash[1];

  SVD<4,4> svd(A);
  Vector<4> v4Smallest = svd.get_VT()[3];
  if(v4Smallest[3] == 0.0)
    v4Smallest[3] = 0.00001;
  return project(v4Smallest);
}




// ThinCandidates() Thins out a key-frame's candidate list.
// Candidates are those salient corners where the mapmaker will attempt
// to make a new map point by epipolar search. We don't want to make new points
// where there are already existing map points, this routine erases such candidates.
// Operates on a single level of a keyframe.
void MapMaker::ThinCandidates(KeyFrame &k, int nLevel)
{
  vector<Candidate> &vCSrc = k.aLevels[nLevel].vCandidates;
  vector<Candidate> vCGood;
  vector<ImageRef> irBusyLevelPos;
  // Make a list of `busy' image locations, which already have features at the same level
  // or at one level higher.
  // Rellena de una lista con las mediciones de las caracteristicas
  // en los niveles n y n+1
  //cerr << "**********************************\n";
  for(meas_it it = k.mMeasurements.begin(); it!=k.mMeasurements.end(); it++)
  {
     //cerr << "\n Level: " << it->second.nLevel
     //     << "  PTO: "    << it->second.v2RootPos / LevelScale(nLevel);
     if(!(it->second.nLevel == nLevel || it->second.nLevel == nLevel + 1))
	     continue;
     //cerr << "   Aniadido";
     irBusyLevelPos.push_back(ir_rounded(it->second.v2RootPos / LevelScale(nLevel)));
  }
  //cerr << "\n";



  // Only keep those candidates further than 10 pixels away from busy positions.
  // Usa la lista de arriba para seleccionar las caracteristicas candidatas
  // que se encuentren a mas de 10 px de las medidas
  unsigned int nMinMagSquared = 10*10;
 // cerr << "Nivel: " << nLevel
 //      << "  Nº candidatos: " << vCSrc.size();
  for(unsigned int i=0; i<vCSrc.size(); i++)
  {
     ImageRef irC = vCSrc[i].irLevelPos;
     //cerr << "\n -- PTO: "    << irC;
     bool bGood = true;
     for(unsigned int j=0; j<irBusyLevelPos.size(); j++)
	  {
	     ImageRef irB = irBusyLevelPos[j];
	     if((irB - irC).mag_squared() < nMinMagSquared)
	     {
	        bGood = false;
	        break;
	     }
	  }
     if(bGood){
        //cerr << "   Aniadido";
        vCGood.push_back(vCSrc[i]);
     }
  }
  vCSrc = vCGood;
  //cerr << " Aniadidos: " << vCGood.size() << endl;
  //cerr << "**********************************\n";
}

// Adds map points by epipolar search to the last-added key-frame, at a single
// specified pyramid level. Does epipolar search in the target keyframe as closest by
// the ClosestKeyFrame function.
void MapMaker::AddSomeMapPoints(int nLevel)
{
  KeyFrame &kSrc = *(mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1]); // The new keyframe
  KeyFrame &kTarget = *(ClosestKeyFrame(kSrc));
  Level &l = kSrc.aLevels[nLevel];

  // Selecciona todos los candidatos que se encuentren a mas de 10 px
  // de las caracteristicas medidas.
  // No mira que las candidatas esten juntas o no.
  //cerr << "Antes de entrar: " << kSrc.aLevels[nLevel].vCandidates.size() << endl;
  ThinCandidates(kSrc, nLevel);
  //cerr << "Despues de entrar: " << kSrc.aLevels[nLevel].vCandidates.size() << endl;

  // Intenta aniadir cada candidato seleccionado al mapa.
  // Haciendo una busqueda epipolar
  //cerr << "candidatos = [\n";

//fprintf(stderr,"Candidates %i\n",l.vCandidates.size());
  for(unsigned int i = 0; i<l.vCandidates.size(); i++){

    //cerr << l.vCandidates[i].irLevelPos << endl;
    AddPointEpipolar(kSrc, kTarget, nLevel, i);
  }
  //cerr << "];\n";
};

// Rotates/translates the whole map and all keyframes
void MapMaker::ApplyGlobalTransformationToMap(SE3<> se3NewFromOld)
{
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    mMap.vpKeyFrames[i]->se3CfromW = mMap.vpKeyFrames[i]->se3CfromW * se3NewFromOld.inverse();

  SO3<> so3Rot = se3NewFromOld.get_rotation();
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
      mMap.vpPoints[i]->v3WorldPos =
	se3NewFromOld * mMap.vpPoints[i]->v3WorldPos;
      mMap.vpPoints[i]->RefreshPixelVectors();
    }
}

// Applies a global scale factor to the map
void MapMaker::ApplyGlobalScaleToMap(double dScale)
{
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    mMap.vpKeyFrames[i]->se3CfromW.get_translation() *= dScale;

  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
      (*mMap.vpPoints[i]).v3WorldPos *= dScale;
      (*mMap.vpPoints[i]).v3PixelRight_W *= dScale;
      (*mMap.vpPoints[i]).v3PixelDown_W *= dScale;
      (*mMap.vpPoints[i]).RefreshPixelVectors();
    }
}


// The tracker entry point for adding a new keyframe;
// the tracker thread doesn't want to hang about, so
// just dumps it on the top of the mapmaker's queue to
// be dealt with later, and return.
void MapMaker::AddKeyFrame(KeyFrame &k)
{
  KeyFrame *pK = new KeyFrame;
  *pK = k;
  pK->pSBI = NULL; // Mapmaker uses a different SBI than the tracker, so will re-gen its own
  mvpKeyFrameQueue.push_back(pK);
  if(mbBundleRunning)   // Tell the mapmaker to stop doing low-priority stuff and concentrate on this KF first.
    mbBundleAbortRequested = true;
}

// LUIS
void MapMaker::AddServiceKeyFrame(sKeyFrame k)
{
  //RECONSTRUIR EL sKeyFrame en un KeyFrame
  servicekf.mMeasurements.clear();
  servicekf.MakeKeyFrame_Lite(k.imFrame);
  servicekf.se3CfromW = k.se3CfromW;
  servicekf.bFixed = k.bFixed;
  servicekf.dSceneDepthMean = k.dSceneDepthMean;
  servicekf.dSceneDepthSigma = k.dSceneDepthSigma;

  for(unsigned int i=0; i<k.sMeasurements.size(); i++)
  {
    servicekf.mMeasurements[mMap.vpPoints[k.sMeasurements[i].indexPointMap]] = k.sMeasurements[i].m;
  }


  KeyFrame *pK = new KeyFrame;
  *pK = servicekf;
  pK->pSBI = NULL; // Mapmaker uses a different SBI than the tracker, so will re-gen its own
  mvpKeyFrameQueue.push_back(pK);
  if(mbBundleRunning)   // Tell the mapmaker to stop doing low-priority stuff and concentrate on this KF first.
    mbBundleAbortRequested = true;

}


// Mapmaker's code to handle incoming key-frames.
void MapMaker::AddKeyFrameFromTopOfQueue()
{
  if(mvpKeyFrameQueue.size() == 0)
    return;

  KeyFrame *pK = mvpKeyFrameQueue[0];
  mvpKeyFrameQueue.erase(mvpKeyFrameQueue.begin());
  pK->MakeKeyFrame_Rest();
  mMap.vpKeyFrames.push_back(pK);

  //Roboearth para la posicion de los keyframes en el tracking
  KeyFrame * newK = new KeyFrame();
  mMapTracker.vpKeyFrames.push_back(newK);


  // Any measurements? Update the relevant point's measurement counter status map
  for(meas_it it = pK->mMeasurements.begin();
      it!=pK->mMeasurements.end();
      it++)
    {
      it->first->pMMData->sMeasurementKFs.insert(pK);
      it->second.Source = Measurement::SRC_TRACKER;
    }

  // And maybe we missed some - this now adds to the map itself, too.
  ReFindInSingleKeyFrame(*pK);

  AddSomeMapPoints(3);       // .. and add more map points by epipolar search.
  AddSomeMapPoints(0);
  AddSomeMapPoints(1);
  AddSomeMapPoints(2);

  mbBundleConverged_Full = false;
  mbBundleConverged_Recent = false;
}


// Tries to make a new map point out of a single candidate point
// by searching for that point in another keyframe, and triangulating
// if a match is found.
bool MapMaker::AddPointEpipolar(KeyFrame &kSrc,
				KeyFrame &kTarget,
				int nLevel,
				int nCandidate)
{
  static Image<Vector<2> > imUnProj;
  static bool bMadeCache = false;

  // Proyecta toda la imagen al plano Z=1
  if(!bMadeCache)
  {
     imUnProj.resize(kSrc.aLevels[0].im.size());
     ImageRef ir;
     do imUnProj[ir] = mCamera->UnProject(ir);
     while(ir.next(imUnProj.size()));
     bMadeCache = true;
  }


  // nLevelScale = 2^nLevel
  // Posicion de la caracteristica en el nivel 0
  int nLevelScale = LevelScale(nLevel);
  Candidate &candidate = kSrc.aLevels[nLevel].vCandidates[nCandidate];
  ImageRef irLevelPos = candidate.irLevelPos;
  Vector<2> v2RootPos = LevelZeroPos(irLevelPos, nLevel);


  // Calcula el rayo que une a la camara 2 con la caracteristica candidata
  // Lo calcula en la camara 2 y luego se lo lleva a la camara 1 (mundo)
  Vector<3> v3Ray_SC = unproject(mCamera->UnProject(v2RootPos));
  normalize(v3Ray_SC);
  Vector<3> v3LineDirn_TC =
     kTarget.se3CfromW.get_rotation() *
     (kSrc.se3CfromW.get_rotation().inverse() * v3Ray_SC);
  // En este pto sabemos en que direccion vive la candidata pero no sabemos a
  // que distancia de la camara (profundidad)

  // Restrict epipolar search to a relatively narrow depth range
  // to increase reliability
  // Se restringe la busqueda epipolar a un rango de profundidades estrecho
  // Previamente se ha calculado la profundidad media y su desviacion
  // => el rango será:
  //   [max(mdWiggleScale, media-desv) .. max(40*mdWiggleScale media+desv)]
  // con mdWiggleScale = 0.1 por defecto
  double dMean = kSrc.dSceneDepthMean;
  double dSigma = kSrc.dSceneDepthSigma;
  double dStartDepth = max(mdWiggleScale, dMean - 2*dSigma);
  double dEndDepth = min(40 * mdWiggleScale, dMean + 2*dSigma);

  // Calculo de la posicion de la camara 2 con respecto a la camara 1 (mundo)
  // La camara 1 debería ser Rot == I(3x3)  y Tras = 0(3x1)
  Vector<3> v3CamCenter_TC =
     kTarget.se3CfromW *
     kSrc.se3CfromW.inverse().get_translation(); // The camera end

  // Calculo de la 2a coordenada 3D del rango donde puede vivir la
  // caracteristica en el mundo
  Vector<3> v3RayEnd_TC =
     v3CamCenter_TC +
     dEndDepth * v3LineDirn_TC;                  // the far-away end

  // Calculo de la 1a coordenada 3D del rango donde puede vivir la
  // caracteristica en el mundo
  Vector<3> v3RayStart_TC =
     v3CamCenter_TC +
     dStartDepth * v3LineDirn_TC;                // the far-away end


//cerr << v3RayStart_TC[0] << " | "
//     << v3RayStart_TC / v3RayStart_TC[2]
//     << endl;


  // Si la profundidad del inicio del rango es mayor que la del final
  // Algo raro ocurre, no es posible que la camara hay girado tanto
  if(v3RayEnd_TC[2] <= v3RayStart_TC[2])
    // it's highly unlikely that we'll manage to get anything out
    // if we're facing backwards wrt the other camera's view-ray
    return false;

//cerr << v3RayStart_TC[0] << " | "
//     << v3RayStart_TC / v3RayStart_TC[2]
//     << endl;

  // Si la profundidad del final del rango es negativa => algo raro pasa
  if(v3RayEnd_TC[2] <= 0.0 )  return false;
/*
cerr << "C " << nCandidate << ": "
     << v3RayStart_TC << " | ";
*/

  // Establece todo el rango en valores Z positivos
  if(v3RayStart_TC[2] <= 0.0)
    v3RayStart_TC +=
       v3LineDirn_TC * (0.001 - v3RayStart_TC[2] / v3LineDirn_TC[2]);


  // Hasta aqui tenemos definido el rango 3D donde es mas probable que viva la
  // caracteristica


//cerr << "C " << nCandidate << ": "
//     << v3RayStart_TC[0] << " | "
//     << v3RayEnd_TC << "  "
//     << v3RayStart_TC / v3RayStart_TC[2]
//     << endl;

  // Proyecta el rango al plano Z=1 y calcula la linea dentro de ese plano
  Vector<2> v2A = project(v3RayStart_TC);
  Vector<2> v2B = project(v3RayEnd_TC);
  Vector<2> v2AlongProjectedLine = v2A-v2B;

/*
cerr << "C " << nCandidate << ": "  << v2AlongProjectedLine << " | "
     << v3RayStart_TC << " | "
     << v2A << " | "
     << v3RayEnd_TC << " | "
     << v2B << endl;
*/


  // Se mira si la linea sobre la imagen es suficientemente larga
  if(v2AlongProjectedLine * v2AlongProjectedLine < 0.00000001)
  {
     cout << "v2AlongProjectedLine too small." << endl;
     return false;
  }

  // Se calcula la normal a la linea
  normalize(v2AlongProjectedLine);
  Vector<2> v2Normal;
  v2Normal[0] = v2AlongProjectedLine[1];
  v2Normal[1] = -v2AlongProjectedLine[0];


  // Distancia del inicio del rango a la linea
  double dNormDist = v2A * v2Normal;
  if(fabs(dNormDist) > mCamera->LargestRadiusInImage() )
    return false;


  double dMinLen =
     min(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) - 0.05;
  double dMaxLen =
     max(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) + 0.05;
  if(dMinLen < -2.0)  dMinLen = -2.0;
  if(dMaxLen < -2.0)  dMaxLen = -2.0;
  if(dMinLen > 2.0)   dMinLen = 2.0;
  if(dMaxLen > 2.0)   dMaxLen = 2.0;


  // Find current-frame corners which might match this
  // Se crea el patch y la suma de sus pixeles y la suma al cuadrado
  PatchFinder Finder;
  Finder.MakeTemplateCoarseNoWarp(kSrc, nLevel, irLevelPos);
  if(Finder.TemplateBad())  return false;


  // Representa las coordenadas en 3D sobre el plano Z=1
  vector<Vector<2> > &vv2Corners = kTarget.aLevels[nLevel].vImplaneCorners;
  vector<ImageRef> &vIR = kTarget.aLevels[nLevel].vCorners;
  if(!kTarget.aLevels[nLevel].bImplaneCornersCached)
  {
     for(unsigned int i=0; i<vIR.size(); i++)   // over all corners in target img..
	     vv2Corners.push_back(imUnProj[ir(LevelZeroPos(vIR[i], nLevel))]);
     kTarget.aLevels[nLevel].bImplaneCornersCached = true;
  }

  int nBest = -1;
  int nBestZMSSD = Finder.mnMaxSSD + 1;
  double dMaxDistDiff = mCamera->OnePixelDist() * (4.0 + 1.0 * nLevelScale);
  double dMaxDistSq = dMaxDistDiff * dMaxDistDiff;

  for(unsigned int i=0; i<vv2Corners.size(); i++)   // over all corners in target img..
  {
     Vector<2> v2Im = vv2Corners[i];
     double dDistDiff = dNormDist - v2Im * v2Normal;
     if(dDistDiff * dDistDiff > dMaxDistSq)	continue; // skip if not along epi line
     if(v2Im * v2AlongProjectedLine < dMinLen)	continue; // skip if not far enough along line
     if(v2Im * v2AlongProjectedLine > dMaxLen)	continue; // or too far
     int nZMSSD = Finder.ZMSSDAtPoint(kTarget.aLevels[nLevel].im, vIR[i]);
     if(nZMSSD < nBestZMSSD)
	  {
	     nBest = i;
	     nBestZMSSD = nZMSSD;
	  }
  }

  if(nBest == -1)   return false;   // Nothing found.

  //  Found a likely candidate along epipolar ray
  Finder.MakeSubPixTemplate();
  Finder.SetSubPixPos(LevelZeroPos(vIR[nBest], nLevel));
  bool bSubPixConverges = Finder.IterateSubPixToConvergence(kTarget,10);


  if(!bSubPixConverges) return false;

  // Now triangulate the 3d point...
  Vector<3> v3New;
  v3New = kTarget.se3CfromW.inverse() *
    ReprojectPoint(kSrc.se3CfromW * kTarget.se3CfromW.inverse(),
		   mCamera->UnProject(v2RootPos),
		   mCamera->UnProject(Finder.GetSubPixPos()));

  MapPoint *pNew = new MapPoint;
  MapPoint *pNewT = new MapPoint; //Roboearth
  pNew->v3WorldPos = v3New;
  pNewT->v3WorldPos = v3New; //Roboearth
  pNew->pMMData = new MapMakerData();

  // Patch source stuff:
  pNew->pPatchSourceKF = &kSrc;
  pNewT->pPatchSourceKF = &kSrc;  //Roboearth
  pNew->nSourceLevel = nLevel;
  pNewT->nSourceLevel = nLevel;  //LUIS
  pNew->v3Normal_NC = makeVector( 0,0,-1);
  pNewT->v3Normal_NC = makeVector( 0,0,-1); //Roboearth
  pNew->irCenter = irLevelPos;
  pNewT->irCenter = irLevelPos;  //Roboearth
  pNew->v3Center_NC = unproject(mCamera->UnProject(v2RootPos));
  pNewT->v3Center_NC = unproject(mCamera->UnProject(v2RootPos));  //Roboearth
  pNew->v3OneRightFromCenter_NC =
     unproject(mCamera->UnProject(v2RootPos + vec(ImageRef(nLevelScale,0))));
  pNewT->v3OneRightFromCenter_NC =
     unproject(mCamera->UnProject(v2RootPos + vec(ImageRef(nLevelScale,0))));  //Roboearth
  pNew->v3OneDownFromCenter_NC =
     unproject(mCamera->UnProject(v2RootPos + vec(ImageRef(0,nLevelScale))));
  pNewT->v3OneDownFromCenter_NC =
     unproject(mCamera->UnProject(v2RootPos + vec(ImageRef(0,nLevelScale))));  //Roboearth

  normalize(pNew->v3Center_NC);
  normalize(pNewT->v3Center_NC); //Roboearth
  normalize(pNew->v3OneDownFromCenter_NC);
  normalize(pNewT->v3OneDownFromCenter_NC); //Roboearth
  normalize(pNew->v3OneRightFromCenter_NC);
  normalize(pNewT->v3OneRightFromCenter_NC); //Roboearth

  pNew->RefreshPixelVectors();
  pNewT->RefreshPixelVectors(); //Roboearth

  pNew->indexPointMap = mMap.vpPoints.size();
  pNewT->indexPointMap = mMap.vpPoints.size(); //Roboearth

  mMap.vpPoints.push_back(pNew);
  mMapTracker.vpPoints.push_back(pNewT);  //Roboearth
  mqNewQueue.push(pNew);
  Measurement m;
  m.Source = Measurement::SRC_ROOT;
  m.v2RootPos = v2RootPos;
  m.nLevel = nLevel;
  m.bSubPix = true;
  kSrc.mMeasurements[pNew] = m;

  m.Source = Measurement::SRC_EPIPOLAR;
  m.v2RootPos = Finder.GetSubPixPos();
  kTarget.mMeasurements[pNew] = m;
  pNew->pMMData->sMeasurementKFs.insert(&kSrc);
  pNew->pMMData->sMeasurementKFs.insert(&kTarget);
  return true;

}



double MapMaker::KeyFrameLinearDist(KeyFrame &k1, KeyFrame &k2)
{
  Vector<3> v3KF1_CamPos = k1.se3CfromW.inverse().get_translation();
  Vector<3> v3KF2_CamPos = k2.se3CfromW.inverse().get_translation();
  Vector<3> v3Diff = v3KF2_CamPos - v3KF1_CamPos;
  double dDist = sqrt(v3Diff * v3Diff);
  return dDist;
}

double MapMaker::KeyFrameAngularDist(KeyFrame &k1, KeyFrame &k2)
{
  //Vector<3> v3KF1_CamPos = k1.se3CfromW.get_rotation().ln();
  Vector<3> v3KF2_CamPos = k1.se3CfromW*k2.se3CfromW.get_rotation().ln();
  //Vector<3> v3Diff = v3KF2_CamPos - v3KF1_CamPos;
  double aDist = sin((sqrt(v3KF2_CamPos * v3KF2_CamPos))/2);
  //cerr<<aDist<<endl;
  return aDist;
}


vector<KeyFrame*> MapMaker::NClosestKeyFramesM(KeyFrame &k, unsigned int N)
{
  vector<pair<double, KeyFrame* > > vKFandScores;
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    {
      if(mMap.vpKeyFrames[i] == &k)
	continue;
      double dDist = KeyFrameLinearDist(k, *mMap.vpKeyFrames[i]);
      vKFandScores.push_back(make_pair(dDist, mMap.vpKeyFrames[i]));
    }
  if(N > vKFandScores.size())
    N = vKFandScores.size();
  partial_sort(vKFandScores.begin(), vKFandScores.begin() + N, vKFandScores.end());

  vector<KeyFrame*> vResult;
  for(unsigned int i=0; i<N; i++)
    vResult.push_back(vKFandScores[i].second);
  return vResult;
}

vector<KeyFrame*> MapMaker::NClosestKeyFramesT(KeyFrame &k, unsigned int N)
{
  vector<pair<double, KeyFrame* > > vKFandScores;
  for(unsigned int i=0; i<mMapTracker.vpKeyFrames.size(); i++)
    {
      if(mMapTracker.vpKeyFrames[i] == &k)
	continue;
      double dDist = KeyFrameLinearDist(k, *mMapTracker.vpKeyFrames[i]);
      vKFandScores.push_back(make_pair(dDist, mMapTracker.vpKeyFrames[i]));
    }
  if(N > vKFandScores.size())
    N = vKFandScores.size();
  partial_sort(vKFandScores.begin(), vKFandScores.begin() + N, vKFandScores.end());

  vector<KeyFrame*> vResult;
  for(unsigned int i=0; i<N; i++)
    vResult.push_back(vKFandScores[i].second);
  return vResult;
}


KeyFrame* MapMaker::ClosestKeyFrame(KeyFrame &k)
{
  double dClosestDist = 9999999999.9;
  int nClosest = -1;
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
  {
     if(mMap.vpKeyFrames[i] == &k) continue;

     double dDist = KeyFrameLinearDist(k, *mMap.vpKeyFrames[i]);
     if(dDist < dClosestDist)
     {
        dClosestDist = dDist;
        nClosest = i;
     }
   }
   assert(nClosest != -1);
   return mMap.vpKeyFrames[nClosest];
}

KeyFrame* MapMaker::ClosestKeyFrameT(KeyFrame &k)
{
  double dClosestDist = 9999999999.9;
  int nClosest = -1;
  for(unsigned int i=0; i<mMapTracker.vpKeyFrames.size(); i++)
  {
     if(mMapTracker.vpKeyFrames[i] == &k) continue;

     double dDist = KeyFrameLinearDist(k, *mMapTracker.vpKeyFrames[i]);
     if(dDist < dClosestDist)
     {
        dClosestDist = dDist;
        nClosest = i;
     }
   }
   assert(nClosest != -1);
   return mMapTracker.vpKeyFrames[nClosest];
}



double MapMaker::DistToNearestKeyFrame(KeyFrame &kCurrent)
{
  KeyFrame *pClosest = ClosestKeyFrame(kCurrent);
  double dDist = KeyFrameLinearDist(kCurrent, *pClosest);
  return dDist;
}


bool MapMaker::NeedNewKeyFrame(KeyFrame &kCurrent)
{
  KeyFrame *pClosest = ClosestKeyFrame(kCurrent);
  double dDist = KeyFrameLinearDist(kCurrent, *pClosest);
  double aDist = KeyFrameAngularDist(kCurrent, *pClosest);
  dDist *= (1.0 / kCurrent.dSceneDepthMean);

  if(dDist > GV2.GetDouble("MapMaker.MaxKFDistWiggleMult",1.0,SILENT) * mdWiggleScaleDepthNormalized)
    return true;
  return false;
}


// Perform bundle adjustment on all keyframes, all map points
void MapMaker::BundleAdjustAll()
{
  // construct the sets of kfs/points to be adjusted:
  // in this case, all of them
  set<KeyFrame*> sAdj;
  set<KeyFrame*> sFixed;

  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    if(mMap.vpKeyFrames[i]->bFixed)
      sFixed.insert(mMap.vpKeyFrames[i]);
    else
      sAdj.insert(mMap.vpKeyFrames[i]);

  set<MapPoint*> sMapPoints;
  for(unsigned int i=0; i<mMap.vpPoints.size();i++)
    sMapPoints.insert(mMap.vpPoints[i]);

  BundleAdjustM(sAdj, sFixed, sMapPoints, false);

}



// Peform a local bundle adjustment which only adjusts
// recently added key-frames
void MapMaker::BundleAdjustRecentM()
{
  if(mMap.vpKeyFrames.size() < 8)
    { // Ignore this unless map is big enough
      mbBundleConverged_Recent = true;
      return;
    }

  // First, make a list of the keyframes we want adjusted in the adjuster.
  // This will be the last keyframe inserted, and its four nearest neighbors
  set<KeyFrame*> sAdjustSet;
  KeyFrame *pkfNewest = mMap.vpKeyFrames.back();
  sAdjustSet.insert(pkfNewest);
  vector<KeyFrame*> vClosest = NClosestKeyFramesM(*pkfNewest, 4);
  for(int i=0; i<4; i++)
    if(vClosest[i]->bFixed == false)
      sAdjustSet.insert(vClosest[i]);

  // Now we find the set of features which they contain.
  set<MapPoint*> sMapPoints;
  for(set<KeyFrame*>::iterator iter = sAdjustSet.begin();
      iter!=sAdjustSet.end();
      iter++)
    {
      map<MapPoint*,Measurement> &mKFMeas = (*iter)->mMeasurements;
      for(meas_it jiter = mKFMeas.begin(); jiter!= mKFMeas.end(); jiter++)
	sMapPoints.insert(jiter->first);
    };

  // Finally, add all keyframes which measure above points as fixed keyframes
  set<KeyFrame*> sFixedSet;
  for(vector<KeyFrame*>::iterator it = mMap.vpKeyFrames.begin(); it!=mMap.vpKeyFrames.end(); it++)
    {
      if(sAdjustSet.count(*it))
	continue;
      bool bInclude = false;
      for(meas_it jiter = (*it)->mMeasurements.begin(); jiter!= (*it)->mMeasurements.end(); jiter++)
	if(sMapPoints.count(jiter->first))
	  {
	    bInclude = true;
	    break;
	  }
      if(bInclude)
	sFixedSet.insert(*it);
    }

  BundleAdjustM(sAdjustSet, sFixedSet, sMapPoints, true);
}

// Peform a local bundle adjustment which only adjusts
// recently added key-frames
void MapMaker::BundleAdjustRecentT()
{
  if(mMapTracker.vpKeyFrames.size() < 8)
    { // Ignore this unless map is big enough
      mbBundleConverged_Recent = true;
      return;
    }

  // First, make a list of the keyframes we want adjusted in the adjuster.
  // This will be the last keyframe inserted, and its four nearest neighbors
  set<KeyFrame*> sAdjustSet;
  KeyFrame *pkfNewest = mMapTracker.vpKeyFrames.back();
  sAdjustSet.insert(pkfNewest);
  vector<KeyFrame*> vClosest = NClosestKeyFramesT(*pkfNewest, 4);
  for(int i=0; i<4; i++)
    if(vClosest[i]->bFixed == false)
      sAdjustSet.insert(vClosest[i]);

  // Now we find the set of features which they contain.
  set<MapPoint*> sMapPoints;
  for(set<KeyFrame*>::iterator iter = sAdjustSet.begin();
      iter!=sAdjustSet.end();
      iter++)
    {
      map<MapPoint*,Measurement> &mKFMeas = (*iter)->mMeasurements;
      for(meas_it jiter = mKFMeas.begin(); jiter!= mKFMeas.end(); jiter++)
	sMapPoints.insert(jiter->first);
    };

  // Finally, add all keyframes which measure above points as fixed keyframes
  set<KeyFrame*> sFixedSet;
  for(vector<KeyFrame*>::iterator it = mMapTracker.vpKeyFrames.begin(); it!=mMapTracker.vpKeyFrames.end(); it++)
    {
      if(sAdjustSet.count(*it))
	continue;
      bool bInclude = false;
      for(meas_it jiter = (*it)->mMeasurements.begin(); jiter!= (*it)->mMeasurements.end(); jiter++)
	if(sMapPoints.count(jiter->first))
	  {
	    bInclude = true;
	    break;
	  }
      if(bInclude)
	sFixedSet.insert(*it);
    }

  BundleAdjustT(sAdjustSet, sFixedSet, sMapPoints, true);
}


void MapMaker::BundleAdjust(set<KeyFrame*> sAdjustSet, set<KeyFrame*> sFixedSet, set<MapPoint*> sMapPoints, bool bRecent)
{
BundleAdjustM(sAdjustSet, sFixedSet, sMapPoints, bRecent);
BundleAdjustT(sAdjustSet, sFixedSet, sMapPoints, bRecent);
}

// Common bundle adjustment code. This creates a bundle-adjust instance, populates it, and runs it.
void MapMaker::BundleAdjustM(set<KeyFrame*> sAdjustSet, set<KeyFrame*> sFixedSet, set<MapPoint*> sMapPoints, bool bRecent)
{
  int indexMod = 0;
  int indexBad = 0;


  Bundle b(*mCamera);   // Our bundle adjuster
  mbBundleRunning = true;
  mbBundleRunningIsRecent = bRecent;

  // The bundle adjuster does different accounting of keyframes and map points;
  // Translation maps are stored:
  map<MapPoint*, int> mPoint_BundleID;
  map<int, MapPoint*> mBundleID_Point;
  map<KeyFrame*, int> mView_BundleID;
  map<int, KeyFrame*> mBundleID_View;


  // Add the keyframes' poses to the bundle adjuster. Two parts: first nonfixed, then fixed.
  for(set<KeyFrame*>::iterator it = sAdjustSet.begin(); it!= sAdjustSet.end(); it++)
    {
      int nBundleID = b.AddCamera((*it)->se3CfromW, (*it)->bFixed);
      mView_BundleID[*it] = nBundleID;
      mBundleID_View[nBundleID] = *it;
    }

  for(set<KeyFrame*>::iterator it = sFixedSet.begin(); it!= sFixedSet.end(); it++)
    {
      int nBundleID = b.AddCamera((*it)->se3CfromW, true);
      mView_BundleID[*it] = nBundleID;
      mBundleID_View[nBundleID] = *it;
    }


  // Add the points' 3D position
  for(set<MapPoint*>::iterator it = sMapPoints.begin(); it!=sMapPoints.end(); it++)
    {
      int nBundleID = b.AddPoint((*it)->v3WorldPos);
      mPoint_BundleID[*it] = nBundleID;
      mBundleID_Point[nBundleID] = *it;
    }

  // Add the relevant point-in-keyframe measurements
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
  {
     if(mView_BundleID.count(mMap.vpKeyFrames[i]) == 0) continue;

     int nKF_BundleID = mView_BundleID[mMap.vpKeyFrames[i]];
     for(meas_it it= mMap.vpKeyFrames[i]->mMeasurements.begin();
	      it!= mMap.vpKeyFrames[i]->mMeasurements.end();
	      it++)
	  {
	     if(mPoint_BundleID.count(it->first) == 0) continue;
	     int nPoint_BundleID = mPoint_BundleID[it->first];
	     b.AddMeas
	     (
	        nKF_BundleID, nPoint_BundleID, it->second.v2RootPos,
	        LevelScale(it->second.nLevel) * LevelScale(it->second.nLevel)
        );
	  }
  }

  // Run the bundle adjuster. This returns the number of successful iterations
  int nAccepted = b.Compute(&mbBundleAbortRequested);
//
//  //OSC:: AQUI DEBERIA DE MOSTRAR EL RESULTADO DEL AJUSTE
//  for(map<KeyFrame*,int>::iterator itr = mView_BundleID.begin();
//	   itr!=mView_BundleID.end();
//	   itr++)
//  {
//      SE3<> aux = b.GetCamera(itr->second);
//      cerr << "BA: " << aux << endl;
//  }
//
//  for(map<MapPoint*,int>::iterator itr = mPoint_BundleID.begin();
//	   itr!=mPoint_BundleID.end(); itr++)
//  {
//      cerr << "MAPA: " << b.GetPoint(itr->second) << endl;
//  }


  if(nAccepted < 0)
  {
     // Crap: - LM Ran into a serious problem!
     // This is probably because the initial stereo was messed up.
     // Get rid of this map and start again!
     cout << "!! MapMaker: Cholesky failure in bundle adjust. " << endl
	       << "   The map is probably corrupt: Ditching the map. " << endl;
     mbResetRequested = true;
     return;
  }

  // Bundle adjustment did some updates, apply these to the map  //BALUIS
  if(nAccepted > 0)
  {
     for(map<MapPoint*,int>::iterator itr = mPoint_BundleID.begin();
	      itr!=mPoint_BundleID.end(); itr++)
     {
        itr->first->v3WorldPos = b.GetPoint(itr->second);
        mMapTracker.vpPoints[itr->first->indexPointMap]->v3WorldPos = b.GetPoint(itr->second); //LUIS
      //  fprintf(stderr,"MODIFICADO BA %i\n",itr->first->indexPointMap);  //LUIS
        indexMod++;
     }

     for(map<KeyFrame*,int>::iterator itr = mView_BundleID.begin();
	      itr!=mView_BundleID.end();
	      itr++)
     {
	     itr->first->se3CfromW = b.GetCamera(itr->second);
     }

     if(bRecent)
        mbBundleConverged_Recent = false;
     mbBundleConverged_Full = false;
  }

  if(b.Converged())
  {
     mbBundleConverged_Recent = true;
     if(!bRecent)
	     mbBundleConverged_Full = true;
  }

  mbBundleRunning = false;
  mbBundleAbortRequested = false;

  // Handle outlier measurements:
  vector<pair<int,int> > vOutliers_PC_pair = b.GetOutlierMeasurements();
  for(unsigned int i=0; i<vOutliers_PC_pair.size(); i++)
  {
     MapPoint *pp = mBundleID_Point[vOutliers_PC_pair[i].first];
     KeyFrame *pk = mBundleID_View[vOutliers_PC_pair[i].second];
     Measurement &m = pk->mMeasurements[pp];
     if(pp->pMMData->GoodMeasCount() <= 2 || m.Source == Measurement::SRC_ROOT){   // Is the original source kf considered an outlier? That's bad.
	     pp->bBad = true;
            mMapTracker.vpPoints[pp->indexPointMap]->bBad = true;
indexBad++;
     }
     else
	  {
	     // Do we retry it? Depends where it came from!!
	     if(m.Source == Measurement::SRC_TRACKER || m.Source == Measurement::SRC_EPIPOLAR)
	        mvFailureQueue.push_back(pair<KeyFrame*,MapPoint*>(pk,pp));
	     else
	        pp->pMMData->sNeverRetryKFs.insert(pk);
	     pk->mMeasurements.erase(pp);
	     pp->pMMData->sMeasurementKFs.erase(pk);        
	  }
  }

  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
      MapPoint &p = *mMap.vpPoints[i];
      MapPoint &pT = *mMapTracker.vpPoints[i];  //Roboearth
  }

}

// Common bundle adjustment code. This creates a bundle-adjust instance, populates it, and runs it.
void MapMaker::BundleAdjustT(set<KeyFrame*> sAdjustSet, set<KeyFrame*> sFixedSet, set<MapPoint*> sMapPoints, bool bRecent)
{

  Bundle b(*mCamera);   // Our bundle adjuster
  mbBundleRunning = true;
  mbBundleRunningIsRecent = bRecent;

  // The bundle adjuster does different accounting of keyframes and map points;
  // Translation maps are stored:
  map<MapPoint*, int> mPoint_BundleID;
  map<int, MapPoint*> mBundleID_Point;
  map<KeyFrame*, int> mView_BundleID;
  map<int, KeyFrame*> mBundleID_View;


  // Add the keyframes' poses to the bundle adjuster. Two parts: first nonfixed, then fixed.
  for(set<KeyFrame*>::iterator it = sAdjustSet.begin(); it!= sAdjustSet.end(); it++)
    {
      int nBundleID = b.AddCamera((*it)->se3CfromW, (*it)->bFixed);
      mView_BundleID[*it] = nBundleID;
      mBundleID_View[nBundleID] = *it;
    }

  for(set<KeyFrame*>::iterator it = sFixedSet.begin(); it!= sFixedSet.end(); it++)
    {
      int nBundleID = b.AddCamera((*it)->se3CfromW, true);
      mView_BundleID[*it] = nBundleID;
      mBundleID_View[nBundleID] = *it;
    }


  // Add the points' 3D position
  for(set<MapPoint*>::iterator it = sMapPoints.begin(); it!=sMapPoints.end(); it++)
    {
      int nBundleID = b.AddPoint((*it)->v3WorldPos);
      mPoint_BundleID[*it] = nBundleID;
      mBundleID_Point[nBundleID] = *it;
    }

  // Add the relevant point-in-keyframe measurements
  for(unsigned int i=0; i<mMapTracker.vpKeyFrames.size(); i++)
  {
     if(mView_BundleID.count(mMapTracker.vpKeyFrames[i]) == 0) continue;

     int nKF_BundleID = mView_BundleID[mMapTracker.vpKeyFrames[i]];
     for(meas_it it= mMapTracker.vpKeyFrames[i]->mMeasurements.begin();
	      it!= mMapTracker.vpKeyFrames[i]->mMeasurements.end();
	      it++)
	  {
	     if(mPoint_BundleID.count(it->first) == 0) continue;
	     int nPoint_BundleID = mPoint_BundleID[it->first];
	     b.AddMeas
	     (
	        nKF_BundleID, nPoint_BundleID, it->second.v2RootPos,
	        LevelScale(it->second.nLevel) * LevelScale(it->second.nLevel)
        );
	  }
  }

  // Run the bundle adjuster. This returns the number of successful iterations
  int nAccepted = b.Compute(&mbBundleAbortRequested);
//
//  //OSC:: AQUI DEBERIA DE MOSTRAR EL RESULTADO DEL AJUSTE
//  for(map<KeyFrame*,int>::iterator itr = mView_BundleID.begin();
//	   itr!=mView_BundleID.end();
//	   itr++)
//  {
//      SE3<> aux = b.GetCamera(itr->second);
//      cerr << "BA: " << aux << endl;
//  }
//
//  for(map<MapPoint*,int>::iterator itr = mPoint_BundleID.begin();
//	   itr!=mPoint_BundleID.end(); itr++)
//  {
//      cerr << "MAPA: " << b.GetPoint(itr->second) << endl;
//  }


  if(nAccepted < 0)
  {
     // Crap: - LM Ran into a serious problem!
     // This is probably because the initial stereo was messed up.
     // Get rid of this map and start again!
     cout << "!! MapMaker: Cholesky failure in bundle adjust. " << endl
	       << "   The map is probably corrupt: Ditching the map. " << endl;
     mbResetRequested = true;
     return;
  }

  // Bundle adjustment did some updates, apply these to the map  //BALUIS
  if(nAccepted > 0)
  {
     for(map<MapPoint*,int>::iterator itr = mPoint_BundleID.begin();
	      itr!=mPoint_BundleID.end(); itr++)
     {
        itr->first->v3WorldPos = b.GetPoint(itr->second);
     }

     for(map<KeyFrame*,int>::iterator itr = mView_BundleID.begin();
	      itr!=mView_BundleID.end();
	      itr++)
     {
	     itr->first->se3CfromW = b.GetCamera(itr->second);
     }

     if(bRecent)
        mbBundleConverged_Recent = false;
     mbBundleConverged_Full = false;
  }

  if(b.Converged())
  {
     mbBundleConverged_Recent = true;
     if(!bRecent)
	     mbBundleConverged_Full = true;
  }

  mbBundleRunning = false;
  mbBundleAbortRequested = false;

  // Handle outlier measurements:
  vector<pair<int,int> > vOutliers_PC_pair = b.GetOutlierMeasurements();
  for(unsigned int i=0; i<vOutliers_PC_pair.size(); i++)
  {
     MapPoint *pp = mBundleID_Point[vOutliers_PC_pair[i].first];
     KeyFrame *pk = mBundleID_View[vOutliers_PC_pair[i].second];
     Measurement &m = pk->mMeasurements[pp];
     if(pp->pMMData->GoodMeasCount() <= 2 || m.Source == Measurement::SRC_ROOT){   // Is the original source kf considered an outlier? That's bad.
	     pp->bBad = true;
            mMapTracker.vpPoints[pp->indexPointMap]->bBad = true; //LUIS
     }
     else
	  {
	     // Do we retry it? Depends where it came from!!
	     if(m.Source == Measurement::SRC_TRACKER || m.Source == Measurement::SRC_EPIPOLAR)
	        mvFailureQueue.push_back(pair<KeyFrame*,MapPoint*>(pk,pp));
	     else
	        pp->pMMData->sNeverRetryKFs.insert(pk);
	     pk->mMeasurements.erase(pp);
	     pp->pMMData->sMeasurementKFs.erase(pk);
	  }
  }

}


// Mapmaker's try-to-find-a-point-in-a-keyframe code. This is used to update
// data association if a bad measurement was detected, or if a point
// was never searched for in a keyframe in the first place. This operates
// much like the tracker! So most of the code looks just like in
// TrackerData.h.
bool MapMaker::ReFind_Common(KeyFrame &k, MapPoint &p)
{
  // abort if either a measurement is already in the map, or we've
  // decided that this point-kf combo is beyond redemption
  if(p.pMMData->sMeasurementKFs.count(&k)
     || p.pMMData->sNeverRetryKFs.count(&k))
    return false;

  static PatchFinder Finder;
  Vector<3> v3Cam = k.se3CfromW*p.v3WorldPos;
  if(v3Cam[2] < 0.001)
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
  Vector<2> v2ImPlane = project(v3Cam);
  if(v2ImPlane* v2ImPlane > mCamera->LargestRadiusInImage() * mCamera->LargestRadiusInImage())
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }

  Vector<2> v2Image = mCamera->Project(v2ImPlane);
  if(mCamera->Invalid())
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }

  ImageRef irImageSize = k.aLevels[0].im.size();
  if(v2Image[0] < 0 || v2Image[1] < 0 || v2Image[0] > irImageSize[0] || v2Image[1] > irImageSize[1])
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }

  Matrix<2> m2CamDerivs = mCamera->GetProjectionDerivs();
  Finder.MakeTemplateCoarse(p, k.se3CfromW, m2CamDerivs);

  if(Finder.TemplateBad())
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }

  bool bFound = Finder.FindPatchCoarse(ir(v2Image), k, 4);  // Very tight search radius!
  if(!bFound)
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }

  // If we found something, generate a measurement struct and put it in the map
  Measurement m;
  m.nLevel = Finder.GetLevel();
  m.Source = Measurement::SRC_REFIND;

  if(Finder.GetLevel() > 0)
    {
      Finder.MakeSubPixTemplate();
      Finder.IterateSubPixToConvergence(k,8);
      m.v2RootPos = Finder.GetSubPixPos();
      m.bSubPix = true;
    }
  else
    {
      m.v2RootPos = Finder.GetCoarsePosAsVector();
      m.bSubPix = false;
    };

  if(k.mMeasurements.count(&p))
    {
      assert(0); // This should never happen, we checked for this at the start.
    }
  k.mMeasurements[&p] = m;
  p.pMMData->sMeasurementKFs.insert(&k);
  return true;
}

// A general data-association update for a single keyframe
// Do this on a new key-frame when it's passed in by the tracker
int MapMaker::ReFindInSingleKeyFrame(KeyFrame &k)
{
  vector<MapPoint*> vToFind;
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    vToFind.push_back(mMap.vpPoints[i]);

  int nFoundNow = 0;
  for(unsigned int i=0; i<vToFind.size(); i++)
    if(ReFind_Common(k,*vToFind[i]))
      nFoundNow++;

  return nFoundNow;
};

// When new map points are generated, they're only created from a stereo pair
// this tries to make additional measurements in other KFs which they might
// be in.
void MapMaker::ReFindNewlyMade()
{
  if(mqNewQueue.empty())
    return;
  int nFound = 0;
  int nBad = 0;
  while(!mqNewQueue.empty() && mvpKeyFrameQueue.size() == 0)
    {
      MapPoint* pNew = mqNewQueue.front();
      mqNewQueue.pop();
      if(pNew->bBad)
	{
	  nBad++;
	  continue;
	}
      for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
	if(ReFind_Common(*mMap.vpKeyFrames[i], *pNew))
	  nFound++;
    }
};

// Dud measurements get a second chance.
void MapMaker::ReFindFromFailureQueue()
{
  if(mvFailureQueue.size() == 0)
    return;
  sort(mvFailureQueue.begin(), mvFailureQueue.end());
  vector<pair<KeyFrame*, MapPoint*> >::iterator it;
  int nFound=0;
  for(it = mvFailureQueue.begin(); it!=mvFailureQueue.end(); it++)
    if(ReFind_Common(*it->first, *it->second))
      nFound++;

  mvFailureQueue.erase(mvFailureQueue.begin(), it);
};

// Is the tracker's camera pose in cloud-cuckoo land?
bool MapMaker::IsDistanceToNearestKeyFrameExcessive(KeyFrame &kCurrent)
{
  return DistToNearestKeyFrame(kCurrent) > mdWiggleScale * 10.0;
}

// Find a dominant plane in the map, find an SE3<> to put it as the z=0 plane
SE3<> MapMaker::CalcPlaneAligner()
{
  unsigned int nPoints = mMap.vpPoints.size();
  if(nPoints < 10)
    {
      cout << "  MapMaker: CalcPlane: too few points to calc plane." << endl;
      return SE3<>();
    };

  int nRansacs = GV2.GetInt("MapMaker.PlaneAlignerRansacs", 100, HIDDEN|SILENT);
  Vector<3> v3BestMean;
  Vector<3> v3BestNormal;
  double dBestDistSquared = 9999999999999999.9;

  for(int i=0; i<nRansacs; i++)
    {
      int nA = rand()%nPoints;
      int nB = nA;
      int nC = nA;
      while(nB == nA)
	nB = rand()%nPoints;
      while(nC == nA || nC==nB)
	nC = rand()%nPoints;

      Vector<3> v3Mean = 0.33333333 * (mMap.vpPoints[nA]->v3WorldPos +
				       mMap.vpPoints[nB]->v3WorldPos +
				       mMap.vpPoints[nC]->v3WorldPos);

      Vector<3> v3CA = mMap.vpPoints[nC]->v3WorldPos  - mMap.vpPoints[nA]->v3WorldPos;
      Vector<3> v3BA = mMap.vpPoints[nB]->v3WorldPos  - mMap.vpPoints[nA]->v3WorldPos;
      Vector<3> v3Normal = v3CA ^ v3BA;
      if(v3Normal * v3Normal  == 0)
	continue;
      normalize(v3Normal);

      double dSumError = 0.0;
      for(unsigned int i=0; i<nPoints; i++)
	{
	  Vector<3> v3Diff = mMap.vpPoints[i]->v3WorldPos - v3Mean;
	  double dDistSq = v3Diff * v3Diff;
	  if(dDistSq == 0.0)
	    continue;
	  double dNormDist = fabs(v3Diff * v3Normal);

	  if(dNormDist > 0.05)
	    dNormDist = 0.05;
	  dSumError += dNormDist;
	}
      if(dSumError < dBestDistSquared)
	{
	  dBestDistSquared = dSumError;
	  v3BestMean = v3Mean;
	  v3BestNormal = v3Normal;
	}
    }

  // Done the ransacs, now collect the supposed inlier set
  vector<Vector<3> > vv3Inliers;
  for(unsigned int i=0; i<nPoints; i++)
    {
      Vector<3> v3Diff = mMap.vpPoints[i]->v3WorldPos - v3BestMean;
      double dDistSq = v3Diff * v3Diff;
      if(dDistSq == 0.0)
	continue;
      double dNormDist = fabs(v3Diff * v3BestNormal);
      if(dNormDist < 0.05)
	vv3Inliers.push_back(mMap.vpPoints[i]->v3WorldPos);
    }

  // With these inliers, calculate mean and cov
  Vector<3> v3MeanOfInliers = Zeros;
  for(unsigned int i=0; i<vv3Inliers.size(); i++)
    v3MeanOfInliers+=vv3Inliers[i];
  v3MeanOfInliers *= (1.0 / vv3Inliers.size());

  Matrix<3> m3Cov = Zeros;
  for(unsigned int i=0; i<vv3Inliers.size(); i++)
    {
      Vector<3> v3Diff = vv3Inliers[i] - v3MeanOfInliers;
      m3Cov += v3Diff.as_col() * v3Diff.as_row();
    };

  // Find the principal component with the minimal variance: this is the plane normal
  SymEigen<3> sym(m3Cov);
  Vector<3> v3Normal = sym.get_evectors()[0];

  // Use the version of the normal which points towards the cam center
  if(v3Normal[2] > 0)
    v3Normal *= -1.0;

  Matrix<3> m3Rot = Identity;
  m3Rot[2] = v3Normal;
  m3Rot[0] = m3Rot[0] - (v3Normal * (m3Rot[0] * v3Normal));
  normalize(m3Rot[0]);
  m3Rot[1] = m3Rot[2] ^ m3Rot[0];

  SE3<> se3Aligner;
  se3Aligner.get_rotation() = m3Rot;
  Vector<3> v3RMean = se3Aligner * v3MeanOfInliers;
  se3Aligner.get_translation() = -v3RMean;

  return se3Aligner;
}


// Calculates the depth(z-) distribution of map points visible in a keyframe
// This function is only used for the first two keyframes - all others
// get this filled in by the tracker
void MapMaker::RefreshSceneDepth(KeyFrame *pKF)
{
  double dSumDepth = 0.0;
  double dSumDepthSquared = 0.0;
  int nMeas = 0;
  //vector<double> depths;
  for(meas_it it = pKF->mMeasurements.begin(); it!=pKF->mMeasurements.end(); it++)
    {
      MapPoint &point = *it->first;
      Vector<3> v3PosK = pKF->se3CfromW * point.v3WorldPos;
      dSumDepth += v3PosK[2];
      dSumDepthSquared += v3PosK[2] * v3PosK[2];
      nMeas++;
    //  depths.push_back(v3PosK[2]);
    }
  //sort(depths.begin(), depths.end());
  assert(nMeas > 2); // If not then something is seriously wrong with this KF!!
  //assert(depths.size() > 2);
  pKF->dSceneDepthMean = dSumDepth / nMeas;//depths[depths.size()/2];//trying to use the median instead of mean(dSumDepth / nMeas;)
  pKF->dSceneDepthSigma = sqrt((dSumDepthSquared / nMeas) - (pKF->dSceneDepthMean) * (pKF->dSceneDepthMean));
}

void MapMaker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
  Command c;
  c.sCommand = sCommand;
  c.sParams = sParams;
  ((MapMaker*) ptr)->mvQueuedCommands.push_back(c);
}

void MapMaker::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
  if(sCommand=="SaveMap")
    {
      cout << "  MapMaker: Saving the map.... " << endl;
      ofstream ofs("map.dump");
      for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
	{
	  ofs << mMap.vpPoints[i]->v3WorldPos << "  ";
	  ofs << mMap.vpPoints[i]->nSourceLevel << endl;
	}
      ofs.close();

      for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
	{
	  ostringstream ost1;
	  ost1 << "keyframes/" << i << ".jpg";

	  ostringstream ost2;
	  ost2 << "keyframes/" << i << ".info";
	  ofstream ofs2;
	  ofs2.open(ost2.str().c_str());
	  ofs2 << mMap.vpKeyFrames[i]->se3CfromW << endl;
	  ofs2.close();
	}
      cout << "  ... done saving map." << endl;
      return;
    }

  cout << "! MapMaker::GUICommandHandler: unhandled command "<< sCommand << endl;
  exit(1);
};


// Oscar Garcia: 9/Mar/2011
bool MapMaker::InitFromStereo_EKF
(
  KeyFrame &kF,
  KeyFrame &kS,
  SE3<> &se3
)
{

  mdWiggleScale = *mgvdWiggleScale;
  mCamera->SetImageSize(kF.aLevels[0].im.size());
  cout << "CAMARA SIZE: " << kF.aLevels[0].im.size() << endl;

 // Use EKF estimates to populate new map:

  KeyFrame *pkFirst = new KeyFrame();
  KeyFrame *pkSecond = new KeyFrame();
  *pkFirst = kF;
  *pkSecond = kS;

  pkFirst->bFixed = true;
  pkFirst->se3CfromW = SE3<>();

  // Check that the initialiser estimated a non-zero baseline
  double dTransMagn = sqrt(se3.get_translation() * se3.get_translation());
  if(dTransMagn == 0)
   {
     cout << "  Estimated zero baseline from stereo pair, try again." << endl;
     return false;
   }
  // change the scale of the map so the second camera is wiggleScale away from the first
  se3.get_translation() *= mdWiggleScale/dTransMagn;

  pkSecond->bFixed = false;
  pkSecond->se3CfromW = se3;


  pkFirst->MakeKeyFrame_Rest();
  pkSecond->MakeKeyFrame_Rest();
  mMap.vpKeyFrames.push_back(pkFirst);
  mMap.vpKeyFrames.push_back(pkSecond);

  AddSomeMapPoints(0);
  BundleAdjustAll();
  AddSomeMapPoints(0);
  BundleAdjustAll();
  id = "nivel0";
  os << *this;
  AddSomeMapPoints(3);
  BundleAdjustAll();
  id = "nivel3";
  os << *this;
  AddSomeMapPoints(1);
  BundleAdjustAll();
  id = "nivel1";
  os << *this;
  AddSomeMapPoints(2);

  id = "nivel2";
  os << *this;

  mbBundleConverged_Full = false;
  mbBundleConverged_Recent = false;

  while(!mbBundleConverged_Full)
  {
     BundleAdjustAll();
     if(mbResetRequested) return false;
  }
  id = "ba2";
  os << *this;

  os.close();

  mMap.bGood = true;
  mMapTracker.bGood = true; //TRUE

  se3 = pkSecond->se3CfromW;
  cout << "  MapMaker: made initial map with " << mMap.vpPoints.size() << " points." << endl;

  RefreshSceneDepth(pkFirst);
  mdWiggleScaleDepthNormalized = mdWiggleScale / pkFirst->dSceneDepthMean;

  return true;
}


double MapMaker::InitServFromStereo_EKF
(
 sKeyFrame skF,
  sKeyFrame skS,
  SE3<> &se3
)
{

  mdWiggleScale = *mgvdWiggleScale;

  //RECONSTRUIR EL sKeyFrame en un KeyFrame
  servicekf_ekf.mMeasurements.clear();
  servicekf_ekf.MakeKeyFrame_Lite(skF.imFrame);

  mCamera->SetImageSize(servicekf_ekf.aLevels[0].im.size());
  servicekf_ekf.se3CfromW = skF.se3CfromW;
  servicekf_ekf.bFixed = skF.bFixed;
  servicekf_ekf.dSceneDepthMean = skF.dSceneDepthMean;
  servicekf_ekf.dSceneDepthSigma = skF.dSceneDepthSigma;

  for(unsigned int i=0; i<skF.sMeasurements.size(); i++)
  {
    servicekf_ekf.mMeasurements[mMap.vpPoints[skF.sMeasurements[i].indexPointMap]] = skF.sMeasurements[i].m;
  }

  Vector<3> v_aux;
  v_aux = servicekf_ekf.se3CfromW.get_translation();
  Matrix<3> v_rot = servicekf_ekf.se3CfromW.get_rotation().get_matrix();

  fprintf(stderr,"servicekf_ekf.se3CfromW.get_translation() %f %f %f\n", v_aux[0], v_aux[1], v_aux[2]);
  fprintf(stderr,"servicekf_ekf.se3CfromW.get_rotation() %f %f %f %f %f %f %f %f %f\n", 
			v_rot(0,0), v_rot(0,1), v_rot(0,2),
			v_rot(1,0), v_rot(1,1), v_rot(1,2),
			v_rot(2,0), v_rot(2,1), v_rot(2,2));
  fprintf(stderr,"servicekf_ekf.dSceneDepthMean %f servicekf_ekf.dSceneDepthSigma %f\n", servicekf_ekf.dSceneDepthMean, servicekf_ekf.dSceneDepthSigma);

  //RECONSTRUIR EL sKeyFrame en un KeyFrame
  serviceks_ekf.mMeasurements.clear();
  serviceks_ekf.MakeKeyFrame_Lite(skS.imFrame);
  serviceks_ekf.se3CfromW = skS.se3CfromW;
  serviceks_ekf.bFixed = skS.bFixed;
  serviceks_ekf.dSceneDepthMean = skS.dSceneDepthMean;
  serviceks_ekf.dSceneDepthSigma = skS.dSceneDepthSigma;

  for(unsigned int i=0; i<skS.sMeasurements.size(); i++)
  {
    serviceks_ekf.mMeasurements[mMap.vpPoints[skS.sMeasurements[i].indexPointMap]] = skS.sMeasurements[i].m;
  }

  Vector<3> v_aux_2;
  v_aux_2 = serviceks_ekf.se3CfromW.get_translation();
  Matrix<3> v_rot_2 = serviceks_ekf.se3CfromW.get_rotation().get_matrix();

  fprintf(stderr,"serviceks_ekf.se3CfromW.get_translation() %f %f %f\n", v_aux_2[0], v_aux_2[1], v_aux_2[2]);
  fprintf(stderr,"serviceks_ekf.se3CfromW.get_rotation() %f %f %f %f %f %f %f %f %f\n", 
			v_rot_2(0,0), v_rot_2(0,1), v_rot_2(0,2),
			v_rot_2(1,0), v_rot_2(1,1), v_rot_2(1,2),
			v_rot_2(2,0), v_rot_2(2,1), v_rot_2(2,2));
  fprintf(stderr,"serviceks_ekf.dSceneDepthMean %f serviceks_ekf.dSceneDepthSigma %f\n", serviceks_ekf.dSceneDepthMean, serviceks_ekf.dSceneDepthSigma);



  for(unsigned int i=0; i<LEVELS; i++) 
  {
  fprintf(stderr,"servicekf_ekf LEVEL %i vCandidates %i\t vCorners %i\t vCornerRowLUT %i\t vMaxCorners %i\t vImplaneCorners %i\n",
			i,
			servicekf_ekf.aLevels[i].vCandidates.size(),
			servicekf_ekf.aLevels[i].vCorners.size(),
			servicekf_ekf.aLevels[i].vCornerRowLUT.size(),
			servicekf_ekf.aLevels[i].vMaxCorners.size(),
			servicekf_ekf.aLevels[i].vImplaneCorners.size());
  fprintf(stderr,"serviceks_ekf LEVEL %i vCandidates %i\t vCorners %i\t vCornerRowLUT %i\t vMaxCorners %i\t vImplaneCorners %i\n",
			i,
			serviceks_ekf.aLevels[i].vCandidates.size(),
			serviceks_ekf.aLevels[i].vCorners.size(),
			serviceks_ekf.aLevels[i].vCornerRowLUT.size(),
			serviceks_ekf.aLevels[i].vMaxCorners.size(),
			serviceks_ekf.aLevels[i].vImplaneCorners.size());

  }



 // Use EKF estimates to populate new map:
  KeyFrame *pkFirst = new KeyFrame();
  KeyFrame *pkSecond = new KeyFrame();

  *pkFirst = servicekf_ekf;
  *pkSecond = serviceks_ekf;

  pkFirst->bFixed = true;
  pkFirst->se3CfromW = SE3<>();

  // Check that the initialiser estimated a non-zero baseline
  double dTransMagn = sqrt(se3.get_translation() * se3.get_translation());
  if(dTransMagn == 0)
   {
     cout << "  Estimated zero baseline from stereo pair, try again." << endl;
     return false;
   }
  // change the scale of the map so the second camera is wiggleScale away from the first
  se3.get_translation() *= mdWiggleScale/dTransMagn;

  pkSecond->bFixed = false;
  pkSecond->se3CfromW = se3;


  pkFirst->MakeKeyFrame_Rest();
  pkSecond->MakeKeyFrame_Rest();
  mMap.vpKeyFrames.push_back(pkFirst);
  mMap.vpKeyFrames.push_back(pkSecond);

  KeyFrame *trackerK1 = new KeyFrame();
  KeyFrame *trackerK2 = new KeyFrame();

  mMapTracker.vpKeyFrames.push_back(trackerK1); //LUIS
  mMapTracker.vpKeyFrames.push_back(trackerK2);  //LUIS


  // Estimate the feature depth distribution in the first two key-frames
  // (Needed for epipolar search)
  //mdWiggleScaleDepthNormalized = mdWiggleScale / pkFirst->dSceneDepthMean;

  AddSomeMapPoints(0);
  BundleAdjustAll();
  AddSomeMapPoints(0);
  BundleAdjustAll();
  id = "nivel0";
  os << *this;
  AddSomeMapPoints(3);
  BundleAdjustAll();
  id = "nivel3";
  os << *this;
  AddSomeMapPoints(1);
  BundleAdjustAll();
  id = "nivel1";
  os << *this;
  AddSomeMapPoints(2);

  id = "nivel2";
  os << *this;

  mbBundleConverged_Full = false;
  mbBundleConverged_Recent = false;

  while(!mbBundleConverged_Full)
  {
     BundleAdjustAll();
     if(mbResetRequested) return false;
  }
  id = "ba2";
  os << *this;

  os.close();

  mMap.bGood = true;
  mMapTracker.bGood = true; //TRUE

  se3 = pkSecond->se3CfromW;
  cout << "  MapMaker: made initial map with " << mMap.vpPoints.size() << " points." << endl;

  RefreshSceneDepth(pkFirst);
  mdWiggleScaleDepthNormalized = mdWiggleScale / pkFirst->dSceneDepthMean;

  return mdWiggleScaleDepthNormalized;
}

// OSC
/*
ostream& operator<<(ostream &os, const MapMaker& m)
{
   string id = m.id;
   os << "\n\n%%%%%%%%%%\n\n"
      << id << ".camaras.nCam = " << m.mMap.vpKeyFrames.size() << ";\n";
   for(int i = 0; i < m.mMap.vpKeyFrames.size(); i++)
   {
      os << id << ".camaras.cam{" << i+1 << "}.mundo = [\n"
         << m.mMap.vpKeyFrames[i]->se3CfromW
         <<"];\n";
   }

   os << id << ".mundo.nPtos = " << m.mMap.vpPoints.size() << ";\n"
      << id << ".mundo.ptos = [\n";
   for(int i=0; i < m.mMap.vpPoints.size(); i++)
   {
      os << m.mMap.vpPoints[i]->v3WorldPos << endl;
   }
   os << "];\n\n\n";
   return os;
}
*/



bool MapMaker::AttemptServiceRecovery(CVD::Image<CVD::byte> &imFrame, bool &mbJustRecoveredSoUseCoarse_r,Vector<6> &mv6CameraVelocity_r, SE3<> &mse3StartPos_r, SE3<> &mse3CamFromWorld_r)
{

  ROS_INFO("AttemptServiceRecovery\n");

  aux_KF.mMeasurements.clear();
  aux_KF.MakeKeyFrame_Lite(imFrame);

  bool bRelocGood = mRelocaliser->AttemptRecovery(aux_KF);
  if(!bRelocGood)
    return false;

  SE3<> se3Best = mRelocaliser->BestPose();
  mse3CamFromWorld_r = mse3StartPos_r = se3Best;
  mv6CameraVelocity_r = Zeros;
  mbJustRecoveredSoUseCoarse_r = true;

  Vector<3> aux;
   aux = mse3StartPos_r.get_translation();

  //  fprintf(stderr,"AttemptServiceRecovery mv6CameraVelocity %f %f %f %f %f %f\t", mv6CameraVelocity_r[0],  mv6CameraVelocity_r[1], mv6CameraVelocity_r[2], mv6CameraVelocity_r[3], mv6CameraVelocity_r[4], mv6CameraVelocity_r[5]);

  //fprintf(stderr,"mse3StartPos %f %f %f\t",  aux[0], aux[1], aux[2]);
  Vector<3> aux2;
  aux2 = mse3CamFromWorld_r.get_translation();
//fprintf(stderr,"mse3CamFromWorld %f %f %f\t",  aux[0], aux[1], aux[2]);

  if (mbJustRecoveredSoUseCoarse_r)
    ROS_INFO("mbJustRecoveredSoUseCoarse TRUE");
  else 
    ROS_INFO("mbJustRecoveredSoUseCoarse FALSE");
  return true;
}

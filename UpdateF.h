//
// Created by VElysianP on 11/7/2017.
//

#ifndef MPM_UPDATEF_H
#define MPM_UPDATEF_H

#endif //MPM_UPDATEF_H

#include "Eigen/Eigen"
#include "Eigen/Dense"
#include "Eigen/src/SVD/JacobiSVD.h"
#include "Eigen/LU"
#include "global.h"
#include "interpolation.h"
#include "SVD.h"

using namespace std;
using namespace Eigen;

#define DIMENSIONX 3
#define DIMENSIONY 3
#define DIMENSIONZ 3

void UpdateF(float timeStep, const GridInfo gridInfo, vector<GridAttr> gridAttrs, vector<Particle>& particles, int energyDensityFunction);

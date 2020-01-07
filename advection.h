//
// Created by ziyinqu on 10/1/17.
//
#pragma once
#ifndef MPM_ADVECTION_H
#define MPM_ADVECTION_H

#include "global.h"
#include "constitutiveModel.h"

using namespace std;

void addGravity(vector<GridAttr>& gridAttrs, vector<int> active_nodes, Vector3f gravity);

void addGridForces(vector<GridAttr>& gridAttrs, vector<Particle>& particles, GridInfo gridInfo, int energyDensityFunction);

void updateGridvelocity(vector<GridAttr>& gridAttrs, vector<int> active_nodes, float dt);

#endif //MPM_ADVECTION_H

//
// Created by ziyinqu on 10/2/17.
//
#pragma once
#ifndef MPM_TRANSFER_H
#define MPM_TRANSFER_H

#include "global.h"
#include "interpolation.h"
#include "SVD.h"
#include <math.h>

#define USEAPIC true
using namespace std;

void transferG2P(vector<Particle>& particles, vector<GridAttr>& gridAttrs, const GridInfo gridInfo, float dt, float alpha);

void transferP2G(vector<Particle> particles, vector<GridAttr> &gridAttrs, const GridInfo gridInfo, std::vector<int>& active_nodes);


#endif //MPM_TRANSFER_H

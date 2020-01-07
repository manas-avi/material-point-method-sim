//
// Created by ziyinqu on 10/1/17.
//
#pragma once
#ifndef MPM_WRITEFRAME_H
#define MPM_WRITEFRAME_H

#include <string>
#include "global.h"

using namespace std;
void saveStep(vector<Particle> particles, int step){

    //Write to an object file
    ofstream outfile;

    //later we'll want a second parameter, timeStep to make it spit out differently named files!
    string filename = "Output/step" + to_string(step) + ".obj";

    outfile.open(filename);

    for(int i = 0; i < particles.size(); i++){
        outfile << "v " << particles[i].posP[0] << " " << particles[i].posP[1] << " " << particles[i].posP[2] << "\n";
    }
    outfile.close();

    return;
}

void saveFrame(vector<Particle> particles, int frame){

    //Write to an object file
    ofstream outfile;

    //later we'll want a second parameter, timeStep to make it spit out differently named files!
    string filename = "Output/frame" + to_string(frame) + ".obj";

    outfile.open(filename);

    for(int i = 0; i < particles.size(); i++){
        outfile << "v " << particles[i].posP[0] << " " << particles[i].posP[1] << " " << particles[i].posP[2] << "\n";
    }
    outfile.close();

    return;
}

void saveFramePCD(vector<Particle> particles, int frame){

    //Write to an object file
    ofstream outfile;

    //later we'll want a second parameter, timeStep to make it spit out differently named files!
    string filename = "Output/frame" + to_string(frame) + ".pcd";
    outfile.open(filename);
    outfile << "# .PCD v.7 - Point Cloud Data file format" << "\n";
    outfile << "VERSION .7" << "\n";
    outfile << "FIELDS x y z" << "\n";
    outfile << "SIZE 4 4 4" << "\n";
    outfile << "TYPE F F F" << "\n";
    outfile << "COUNT 1 1 1" << "\n";
    outfile << "WIDTH " << particles.size() << "\n";
    outfile << "HEIGHT 1" << "\n";
    outfile << "VIEWPOINT 0 0 0 5 0 0 0" << "\n";
    outfile << "POINTS " << particles.size() << "\n";
    outfile << "DATA ascii" << "\n";

    for(int i = 0; i < particles.size(); i++){
        outfile << particles[i].posP[0] << " " << particles[i].posP[1] << " " << particles[i].posP[2] << "\n";
    }
    outfile.close();

    return;
}

#endif //MPM_WRITEFRAME_H

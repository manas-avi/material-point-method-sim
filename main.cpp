//
// Created by ziyinqu on 10/1/17.
//

#include <sstream>
#include <iomanip>
#include <string>
#include "mpmInitialize.h"
#include "transfer.h"
#include "advection.h"
#include "writeframe.h"
#include "setBoundaryVelocity.h"
#include "UpdateF.h"
#include "computeMomentum.h"
#include "Test/kernelTest.h"
#include "Test/derivativeTest.h"
#include "AnalyticCollisionObject.h"
#include <ctime>
    
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

#define SANITYCHECK false
#define DERIVATIVETEST false
#define TIMER true


void render_particles(pcl::visualization::PCLVisualizer::Ptr viewer , std::vector<Particle> particles, int frame_num)
{
    if (frame_num > 0)
        viewer->removeAllPointClouds();

    int num_particles = particles.size();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.resize(num_particles);
    for (int i = 0; i < num_particles; ++i)
    {
        cloud->points[i].x = particles[i].posP.x();
        cloud->points[i].y = particles[i].posP.y();
        cloud->points[i].z = particles[i].posP.z();
    }

    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    // viewer->updatePointCloud<pcl::PointXYZ> (cloud, "sample cloud");

    std::stringstream ss;
    ss << std::setw(4) << std::setfill('0') << frame_num;
    std::string fnum = ss.str();

    viewer->spinOnce (0);
    std::string filename = "Output/images/img_" + fnum + ".png";
    viewer->saveScreenshot(filename);

}

int main()
{

    if (SANITYCHECK)
        quadraticTest();

    if(DERIVATIVETEST)
        derivativeTest();
    else 
    {
        // MPM simulation parameters setting up
        float dt = 1e-4f; //50 FPS
        float frameRate = 60;
        // int stepsPerFrame = 1;
        int stepsPerFrame = (int)ceil(1 / (dt / (1 / frameRate))); //calculate how many steps per frame we should have based on our desired frame rate and dt!
        float alpha = 0.95;
        int num_frames = 1000;

        Vector3f gravity = Vector3f(0, -10, 0);

        // particles attributes initialize
        float density = 1.0f;
        float mass = 10;
        float volume = mass/density;
        std::string filenameLeft = "../Models/smallLeftCube.obj";
        // std::string filenameRight = "../Models/smallRightCube.obj";
        // std::vector<Particle> particlesLeft;
        // std::vector<Particle> particlesRight;
        // Vector3f velocityLeft = Vector3f(0.f, 0.f, 0.f);
        // Vector3f velocityRight = Vector3f(-0.f, 0.f, 0.f);
        // mpmParticleInitialize(filenameLeft, particlesLeft, mass, volume, velocityLeft);
        // mpmParticleInitialize(filenameRight, particlesRight, mass, volume, velocityRight);
        // particlesLeft.insert(particlesLeft.end(), particlesRight.begin(), particlesRight.end());
        // std::vector<Particle> particles = particlesLeft;
        // std::string filenameLeft = "../Models/translatedRandomDragon_2500000.obj";
        std::vector<Particle> particles;
        Vector3f velocity = Vector3f(0.f, 0.f, 0.f);
        mpmParticleInitialize(filenameLeft, particles, mass, volume, velocity);

        // grid attributes initialize
        float dx = 0.02f;
        Vector3i simArea = Vector3i::Ones();
        std::vector<GridAttr> gridAttrs;
        GridInfo gridInfo;
        mpmGridInitialize(gridAttrs, gridInfo, simArea, dx);

        // start simulation
        int step = 0;
        int frame = 0;
        cout << "INFO: ------------ Simulation Start! ------------ " << endl;

        // create simulation window 
        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->initCameraParameters ();
        viewer->setCameraPosition(0, 0.5, 2,    0, 0, 0,   0, 1, 0);
        viewer->setCameraFieldOfView(0.523599);
        viewer->setCameraClipDistances(0.00522511, 50); 
        viewer->setBackgroundColor (0, 0, 0);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

        // add a plane to visualizer
        pcl::ModelCoefficients plane_coeff;
        plane_coeff.values.resize(4);    // We need 4 values
        plane_coeff.values[0] = 0;
        plane_coeff.values[1] = 1;
        plane_coeff.values[2] = 0;
        plane_coeff.values[3] = 0;
        viewer->addPlane(plane_coeff);



        while (frame < num_frames) 
        {
            // set timer
            std::clock_t start, p2gstart, g2pstart, updatestart, forcestart, updatefstart;
            double duration, p2gduration, g2pduration, updateduration, forceduration, updatefduration;
            start = std::clock();

            cout << "INFO: Current simulation step is " << step << endl;
            mpmGridReinitialize(gridAttrs, gridInfo);
            std::vector<int> active_nodes;

            // transfer from Particles to Grid
            p2gstart = std::clock();
            //Vector3f Lpp2g = computeParticleMomentum(particles);
            transferP2G(particles, gridAttrs, gridInfo, active_nodes);

            //Vector3f Lgp2g = computeGridMomentum(gridAttrs, false);
            //cout << "      P2G Momentum Difference: " << (Lgp2g - Lpp2g).transpose() << endl;

            p2gduration = ( std::clock() - p2gstart ) / (double) CLOCKS_PER_SEC;

            // advection part, add forces and update grid velocity
            addGravity(gridAttrs, active_nodes, gravity);

            //Add external forces based on our defined energy density function
            forcestart = std::clock();
            int energyDensityFunction = 3; //define which density function we wish to use!
            addGridForces(gridAttrs, particles, gridInfo, energyDensityFunction);
            forceduration = ( std::clock() - forcestart ) / (double) CLOCKS_PER_SEC;

            updatestart = std::clock();
            updateGridvelocity(gridAttrs, active_nodes, dt);
            updateduration = ( std::clock() - updatestart ) / (double) CLOCKS_PER_SEC;

            // boundary collision
            setBoundaryVelocity(gridAttrs, gridInfo);
            Vector3f center = Vector3f(0.5f, 0.f, 0.5f);
            float radius = 0.2f;
            float friction  = 0.f;
            SphereGridCollision(gridAttrs, active_nodes, center, radius, friction, dx);

            //update deformation gradient here
            updatefstart = std::clock();
            UpdateF(dt, gridInfo, gridAttrs, particles, energyDensityFunction);
            updatefduration = ( std::clock() - updatefstart ) / (double) CLOCKS_PER_SEC;

            // transfer from Grid to particles
            g2pstart = std::clock();
            //Vector3f Lgg2p = computeGridMomentum(gridAttrs, true);
            transferG2P(particles, gridAttrs, gridInfo, dt, alpha);
            //Vector3f Lpg2p = computeParticleMomentum(particles);
            SphereParticleCollision(particles, center, friction, radius);

            //cout << "      G2P Momentum Difference: " << (Lgg2p - Lpg2p).transpose() << endl;
            g2pduration = ( std::clock() - g2pstart ) / (double) CLOCKS_PER_SEC;


            if (step % stepsPerFrame == 0) 
            {
                //Save the particles!
                cout << "INFO: Current Frame is " << frame << endl;
                render_particles(viewer, particles, frame);
                // saveFrame(particles, frame); //call the saving routine
                // saveFramePCD(particles, frame); //call the saving routine
                // PointCloud* cloud = new PointCloud;
                // int num_particles = particles.size();
                // for (int i = 0; i < num_particles; ++i)
                // {
                //     cloud->add_vertex(vec3(particles[i][0], particles[i][1], particles[i][2]));// z = 0: all points are on XY plane
                // }


                frame = frame + 1;
            }
                //Save the particles!
//                saveStep(particles, step); //call the saving routine

            step = step + 1;
            // output time calculation
            duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
            if (TIMER)
            {
                cout << "      P2G time is: " << p2gduration << endl;
                cout << "      UPV time is: " << updateduration << endl;
                cout << "      FRC time is: " << forceduration << endl;
                cout << "      UPF time is: " << updatefduration << endl;
                cout << "      G2P time is: " << g2pduration << endl;
                cout << "      overall time is: " << duration << endl;
            }
        }
    }

    return 0;
}

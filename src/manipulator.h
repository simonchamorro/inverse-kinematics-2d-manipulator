/********
 * manipulator.h
 * Author: Simon Chamorro
 * Robot Manipulator class header
********/

#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <iostream>
#include "robot_configuration.h"

using namespace std;


class Manipulator{
    public:
        Manipulator();
        ~Manipulator();
        
        Configuration get_config();
        bool reset();
        bool set_parameters(int num_links, double links[MAX_LINKS]);
        bool forward_kinematics(double angles[MAX_LINKS]);
        bool intersection(double x, double y, double r, double angles[MAX_LINKS]);
        bool inverse_kinematics(double x, double y, double theta);
        bool inverse_dynamics(double fx, double fy, double tau, double *torques[MAX_LINKS]);
        double clip_angle_180(double angle);

    private:
        Configuration robot_config;
};

#endif


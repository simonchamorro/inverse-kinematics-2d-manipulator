/********
 * manipulator.cpp
 * Author: Simon Chamorro
 * Robot Manipulator class
********/

#include <iostream>
#include <math.h>
#include "manipulator.h"
#include "robot_configuration.h"

using namespace std;

#define PI 3.14159265359


// Constructor
Manipulator::Manipulator(){
    reset();
}


// Destructor
Manipulator::~Manipulator(){
}


// Reset to default params
bool Manipulator::reset(){
    int n_links = 3;
    double links[n_links] = {1.0, 1.0, 1.0};
    set_parameters(n_links, links);
    double angles[n_links] = {0.0, 0.0, 0.0};
    forward_kinematics(angles);
    return true;
}


Configuration Manipulator::get_config(){
    return robot_config;
}


/**
 * Set Robot Manipulator parameters.
 *
 * @param[in] num_links Number of links.
 * @param[in] links Array with links' lengths.
 */
bool Manipulator::set_parameters(int num_links, double links[MAX_LINKS]){
    robot_config.num_links = num_links;
    for (int i = 0; i < num_links; i+=1 ){
        robot_config.links[i] = links[i];
    }
    return true;
}


/**
 * Move each joint of the Robot Manipulator to a specific angle.
 * Angles are assumed to be in degres.
 * Final x, y and theta position is stored internally
 *
 * @param[in] angles Array with desired joint angles. 
 * @return true once done. 
 */
bool Manipulator::forward_kinematics(double angles[MAX_LINKS]){
    double theta = 0;
    double x = 0;
    double y = 0;
    for (int i = 0; i < robot_config.num_links; i += 1){
        x += robot_config.links[i]*cos((theta + angles[i])*PI/180.0);
        y += robot_config.links[i]*sin((theta + angles[i])*PI/180.0);
        theta += angles[i];
        robot_config.angles[i] = angles[i];
    }
    robot_config.x = x;
    robot_config.y = y;
    robot_config.theta = clip_angle_180(theta);

    return true;
}


/**
 * Checks if end effector is within a given circle.
 * Array must contain at least num_links angles.
 *
 * @param[in] x coordinate of circle center.
 * @param[in] y coordinate of circle center.
 * @param[in] r radius of circle.
 * @param[in] angles Array with desired joint angles. 
 * @return is_within_circle bool.
 */
bool Manipulator::intersection(double x, double y, double r, double angles[MAX_LINKS]){
    bool is_within_circle;
    forward_kinematics(angles);
    double r_squared = pow(r, 2);
    double d_squared = pow(robot_config.x - x, 2) + pow(robot_config.y - y, 2);
    is_within_circle = (d_squared <= r_squared);
    return is_within_circle;
}


bool Manipulator::inverse_kinematics(double x, double y, double theta){
    cout << "Not implemented \n";
    return false;
}


bool Manipulator::inverse_dynamics(double fx, double fy, double tau, double *torques[MAX_LINKS]){
    cout << "Not implemented \n";
    return false;
}


double Manipulator::clip_angle_180(double angle){
    while (angle > 180){
        angle -= 360;
    }

    while (angle <= -180){
        angle += 360;
    }

 
    return angle;
}
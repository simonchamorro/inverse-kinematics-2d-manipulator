/********
 * robot_configuration.h
 * Author: Simon Chamorro
 * Struct to store Robot Configuration
********/

#ifndef ROBOT_CONFIGURATION_H
#define ROBOT_CONFIGURATION_H

using namespace std;

const int MAX_LINKS = 10;


struct Configuration{
    
    int num_links;
    double links[MAX_LINKS];
    double angles[MAX_LINKS];
    double x;
    double y;
    double theta;
};

#endif
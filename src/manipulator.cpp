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

// Utils

// Keep angle between -180 and 180 degres
double clip_angle_180(double angle){
    while (angle > 180){
        angle -= 360;
    }
    while (angle <= -180){
        angle += 360;
    }
    return angle;
}


// Check if point is within center
bool point_in_circle(double x_center, double y_center, double radius, 
                    double x, double y){
    double r_squared = pow(radius, 2);
    double d_squared = pow(x - x_center, 2) + pow(y - y_center, 2);
    return (d_squared <= r_squared);
}


// Robot Manipulator class functions

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
        robot_config.angles[i] = 0.0;
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
    forward_kinematics(angles);
    return point_in_circle(x, y, r, robot_config.x, robot_config.y);
}


/**
 * Solve for angle of joint 1 form angle of joint 2.
 *
 * @param[in] theta2 Angle of joint 2.
 * @return theta1 angle of joint 1.
 */
double Manipulator::solve_theta_1(double theta2, double x, double y){
    double A = robot_config.links[0] + robot_config.links[1] * cos(theta2*PI/180);
    double B = robot_config.links[1] * sin(theta2*PI/180);

    // Find theta1 in radians
    double theta_c1 = acos( (A*x + B*y) / (pow(A, 2) + pow(B, 2)) ) *180/PI;
    double theta_c2 = -theta_c1;
    double theta_s1 = asin( (A*y - B*x) / (pow(A, 2) + pow(B, 2)) ) *180/PI; 
    double theta_s2 = clip_angle_180( 180 - theta_s1 );

    double theta1;
    if ( abs(theta_c1 - theta_s1) < 1e-6 || abs(theta_c1 - theta_s2) < 1e-6 ){
        theta1 = theta_c1;
    }
    else{
        theta1 = theta_c2;
    }  
    return theta1;
}


/**
 * Inverse kinematics of Robot Manipulator.
 *
 * @param[in] x coordinate of end effector.
 * @param[in] y coordinate of end effector.
 * @param[in] y orientation of end effector.
 * @param[out] angles_1 Angles of joints.
 * @param[out] angles_1 Angles of joints, other possible configuration.
 * @return bool: true if success, false if not.
 */
bool Manipulator::inverse_kinematics(double x, double y, double theta, 
                                    double *angles_1, double *angles_2){
    if (robot_config.num_links != 3){
        return false;
    }
    // Find pos of J3 and check reachability
    double x3 = x - robot_config.links[2]*cos(theta*PI/180.0);
    double y3 = y - robot_config.links[2]*sin(theta*PI/180.0);
    double radius = robot_config.links[0] + robot_config.links[1];
    if (!point_in_circle(0.0, 0.0, radius, x3, y3)){
        return false;
    }

    // Find configuration
    // source: https://drive.google.com/file/d/1j-UEZHs-4KvykbWKMLxDwkFE_MvqaI3l/view
    double d = 2*robot_config.links[0]*robot_config.links[1];
    double f = pow(x3, 2) + pow(y3, 2) - pow(robot_config.links[0], 2) - pow(robot_config.links[1], 2);
    double theta2_a = acos(f/d) * 180/PI;
    double theta2_b = -theta2_a;

    double theta1_a = solve_theta_1(theta2_a, x3, y3);
    double theta1_b = solve_theta_1(theta2_b, x3, y3);

    angles_1[0] = theta1_a;
    angles_1[1] = theta2_a;
    angles_1[2] = clip_angle_180( theta - theta1_a - theta2_a );
    angles_2[0] = theta1_b;
    angles_2[1] = theta2_b;
    angles_2[2] = clip_angle_180( theta - theta1_b - theta2_b);
    return true;
}


/**
 * Inverse dynamics of Robot Manipulator.
 *
 * @param[in] fx desired force at end effector.
 * @param[in] fy desired force at end effector.
 * @param[in] tau desired torque at end effector.
 * @param[out] torques at robot joints.
 * @return bool: true if success, false otherwise.
 */
bool Manipulator::inverse_dynamics(double fx, double fy, double tau, double *torques){
    if (robot_config.num_links != 3){
        return false;
    }

    // source: http://robotics.sjtu.edu.cn/upload/course/5/files/Jacobian.pdf
    double l1 = robot_config.links[0];
    double l2 = robot_config.links[1];
    double l3 = robot_config.links[2];
    double a1 = robot_config.angles[0]*PI/180;
    double a2 = robot_config.angles[1]*PI/180;
    double a3 = robot_config.angles[2]*PI/180;

    double jacobian[3][3] = {{-l1*sin(a1) -l2*sin(a1 + a2) -l3*sin(a1 + a2 + a3), -l2*sin(a1 + a2) -l3*sin(a1 + a2 + a3), -l3*sin(a1 + a2 + a3)}, 
                             {l1*cos(a1) + l2*cos(a1 + a2) + l3*cos(a1 + a2 + a3), l2*cos(a1 + a2) + l3*cos(a1 + a2 + a3), l3*cos(a1 + a2 + a3)}, 
                             {1, 1, 1}};

    double jacobian_t[3][3];
    for(int i = 0; i < 3; i += 1)
      for(int j = 0; j < 3; j += 1) {
         jacobian_t[j][i] = jacobian[i][j];
      }

    int i, j, k;
    double mult[3][1];
    double forces[3][1] = {{fx}, {fy}, {tau}};
    for(i = 0; i < 3; ++i)
        for(j = 0; j < 1; ++j)
            for(k = 0; k < 3; ++k)
            {
                mult[i][j] += jacobian_t[i][k] * forces[k][j];
            }

    torques[0] = mult[0][0];
    torques[1] = mult[1][0];
    torques[2] = mult[2][0];
    return true;
}

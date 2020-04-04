/********
 * tests.cpp
 * Author: Simon Chamorro
 * Tests for Manipulator Robot using Catch.
 * https://github.com/catchorg/Catch2
********/

#define CATCH_CONFIG_MAIN

#include <stdlib.h>
#include "catch.h"
#include "robot_configuration.h"
#include "manipulator.h"


TEST_CASE( "Manipulator Robot Tests" ) {

    Manipulator manipulator;
    manipulator.reset();
    Configuration config = manipulator.get_config();
    srand(0);

    SECTION( "Initialization" ) {
        REQUIRE( config.num_links == 3 );
        REQUIRE( config.links[0] == 1.0 );
        REQUIRE( config.links[1] == 1.0 );
        REQUIRE( config.links[2] == 1.0 );
        REQUIRE( config.angles[0] == 0.0 );
        REQUIRE( config.angles[1] == 0.0 );
        REQUIRE( config.angles[2] == 0.0 );
        REQUIRE( config.x == 3.0 );
        REQUIRE( config.y == 0.0 );
        REQUIRE( config.theta == 0.0 );
    }

    SECTION( "Forward kinematics" ) {
        double angles[MAX_LINKS];

        angles[0] = 0.0;
        angles[1] = 90.0;
        angles[2] = 90.0;
        manipulator.forward_kinematics(angles);
        Configuration config = manipulator.get_config();
        REQUIRE( abs(config.x - 0.0) < 1e-6 );
        REQUIRE( abs(config.y - 1.0) < 1e-6 );
        REQUIRE( abs(config.theta - 180.0) < 1e-6 );

        angles[0] = 45.0;
        angles[1] = 0.0;
        angles[2] = 0.0;
        manipulator.forward_kinematics(angles);
        config = manipulator.get_config();
        REQUIRE( abs(config.x - 2.12132034356) < 1e-6 );
        REQUIRE( abs(config.y - 2.12132034356) < 1e-6 );
        REQUIRE( abs(config.theta - 45.0) < 1e-6 );
    }

    SECTION( "Intersection" ) {
        double x, y, r;
        double angles[MAX_LINKS];

        x = 3.0;
        y = 0.0;
        r = 0.1;
        angles[0] = 0.0;
        angles[1] = 0.0;
        angles[2] = 0.0;
        REQUIRE( manipulator.intersection(x, y, r, angles) );

        x = 1.0;
        y = 1.0;
        r = 0.3;
        angles[0] = 0.0;
        angles[1] = 0.0;
        angles[2] = 0.0;
        REQUIRE( !manipulator.intersection(x, y, r, angles) );

        x = -2.4;
        y = -1.4;
        r = 0.2;
        angles[0] = 180.0;
        angles[1] = 45.0;
        angles[2] = 0.0;
        REQUIRE( manipulator.intersection(x, y, r, angles) );
    }

    SECTION( "Inverse kinematics" ){
        double x, y, theta;
        double ang_1[3];
        double ang_2[3];
        double joints[3];

        for (int i = 0; i < 10; i += 1){
            joints[0] = (double)rand() * 360 / RAND_MAX - 180;
            joints[1] = (double)rand() * 360 / RAND_MAX - 180;
            joints[2] = (double)rand() * 360 / RAND_MAX - 180;
            manipulator.forward_kinematics(joints);
            Configuration config = manipulator.get_config();
            REQUIRE( manipulator.inverse_kinematics(config.x, config.y, config.theta, ang_1, ang_2) );
            bool valid_configuration = ( abs(joints[0] - ang_1[0]) < 1e-3 && 
                            abs(joints[1] - ang_1[1]) < 1e-3 &&
                            abs(joints[2] - ang_1[2]) < 1e-3 ||
                            abs(joints[0] - ang_2[0]) < 1e-3 &&
                            abs(joints[1] - ang_2[1]) < 1e-3 &&
                            abs(joints[2] - ang_2[2]) < 1e-3);
            REQUIRE( valid_configuration );
        }
        
        // Test fail cases
        x = 4;
        y = 0;
        theta = 0;
        REQUIRE( !manipulator.inverse_kinematics(x, y, theta, ang_1, ang_2) );
    }

    SECTION( "Inverse dynamics" ){
        double fx, fy, tau;
        double torques[3];
        double joints[3];

        fx = 0;
        fy = 1;
        tau = 0;
        manipulator.inverse_dynamics(fx, fy, tau, torques);
        REQUIRE( torques[0] == 3.0 );
        REQUIRE( torques[1] == 2.0 );
        REQUIRE( torques[2] == 1.0 );

        joints[0] = 90.0;
        joints[1] = 0.0;
        joints[2] = 0.0;
        fx = 1;
        fy = 0;
        tau = 0;
        manipulator.forward_kinematics(joints);
        manipulator.inverse_dynamics(fx, fy, tau, torques);
        REQUIRE( torques[0] == -3.0 );
        REQUIRE( torques[1] == -2.0 );
        REQUIRE( torques[2] == -1.0 );

        joints[0] = 30.0;
        joints[1] = -30.0;
        joints[2] = 30.0;
        fx = 1;
        fy = 1;
        tau = 1;
        manipulator.forward_kinematics(joints);
        manipulator.inverse_dynamics(fx, fy, tau, torques);
        REQUIRE( abs(torques[0] - 2.732) < 1e-3 );
        REQUIRE( abs(torques[1] - 2.366) < 1e-3 );
        REQUIRE( abs(torques[2] - 1.366) < 1e-3 );
    }

    SECTION( "Changing Robot Configuration" ){
        double x_circle, y_circle, r, n_links;

        double links[MAX_LINKS];
        double angles[MAX_LINKS];

        // Change links
        n_links = 4;
        links[0] = 3.0;
        links[1] = 2.0;
        links[2] = 1.0;
        links[3] = 1.0;
        manipulator.set_parameters(n_links, links);
        config = manipulator.get_config();
        REQUIRE( config.num_links == n_links );
        REQUIRE( config.links[0] == links[0] );
        REQUIRE( config.links[1] == links[1] );
        REQUIRE( config.links[2] == links[2] );
        REQUIRE( config.links[2] == links[3] );

        // Forward
        angles[0] = 0.0;
        angles[1] = 90.0;
        angles[2] = -90.0;
        angles[3] = -90.0;
        manipulator.forward_kinematics(angles);
        config = manipulator.get_config();
        REQUIRE( abs(config.x - 4.0) < 1e-6 );
        REQUIRE( abs(config.y - 1.0) < 1e-6 );
        REQUIRE( abs(config.theta - (-90.0)) < 1e-6 );

        // Intersection
        x_circle = 4.1;
        y_circle = 1.1;
        r = 0.2;
        REQUIRE( manipulator.intersection(x_circle, y_circle, r, angles) );

        x_circle = 4.1;
        y_circle = 1.1;
        r = 0.05;
        REQUIRE( !manipulator.intersection(x_circle, y_circle, r, angles) );

        // Inverse kinematics and dynamics fail
        double x = 0;
        double y = 0;
        double theta = 0;
        double ang_1[4];
        double ang_2[4];
        REQUIRE( !manipulator.inverse_kinematics(x, y, theta, ang_1, ang_2) );

        double fx = 0;
        double fy = 0;
        double tau = 0;
        double torques[4];
        REQUIRE( !manipulator.inverse_dynamics(fx, fy, tau, torques) );
    }
}

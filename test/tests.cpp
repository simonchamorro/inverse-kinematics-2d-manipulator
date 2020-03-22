/********
 * tests.cpp
 * Author: Simon Chamorro
 * Tests for Manipulator Robot using Catch.
 * https://github.com/catchorg/Catch2
********/

#define CATCH_CONFIG_MAIN

#include <stdlib.h>
#include "catch.h"
#include "../src/robot_configuration.h"
#include "../src/manipulator.h"


TEST_CASE( "Manipulator Robot Tests" ) {

    Manipulator manipulator;
    manipulator.reset();
    Configuration config = manipulator.get_config();

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

}

/********
 * main.cpp
 * Author: Simon Chamorro
 * Main file to test Robot Manipulator class
********/

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include "manipulator.h"
#include "robot_configuration.h"


using namespace std;


void print_available_commands(){
    cout << "Available Commands:\n";
    cout << "  - help\n";
    cout << "  - reset\n";
    cout << "  - parameters LINK_1 LINK_2 ...\n";
    cout << "  - forward THETA_1 THETA_2 ...\n";
    cout << "  - exit\n";
    cout << "--------------------------------\n";
}

void print_robot_config(Configuration config){
    cout << "----- 2D Robot Manipulator -----\n";
    cout << "Configuration:\n";
    cout << "  - Number of links: " << config.num_links << "\n";
    cout << "  - Links: ";
    for (int i = 0; i < config.num_links; i += 1){
        cout << config.links[i] << " ";
    }
    cout << "\n";
    cout << "--------------------------------\n";
}

int main()
{
    bool exit_flag = false;
    Manipulator manipulator;
    manipulator.reset();
    print_robot_config(manipulator.get_config());
    print_available_commands();

    while (!exit_flag){      
        string input;
        getline(std::cin, input);
        string buffer;
        istringstream ss(input);
        vector<string> commands;
        while (ss >> buffer)
            commands.push_back(buffer);
        if (commands.size() < 1){
            continue;
        }

        if (commands[0] == "help"){
            print_available_commands();            
        }
        
        // Reset robot parameters
        else if (commands[0] == "reset"){
            manipulator.reset();
            print_robot_config(manipulator.get_config());     
        }

        // Change robot parameters
        else if (commands[0] == "parameters"){

            if (commands.size() < MAX_LINKS && commands.size() > 1){
                int n_links = atof(commands[1].c_str());
                double links[n_links];
                for (int i = 0; i < commands.size() - 1; i += 1){
                    links[i] = atof(commands[i + 1].c_str());
                }
                manipulator.set_parameters(n_links, links);
                print_robot_config(manipulator.get_config());
            }

            else{
                cout << "Invalid number of links.\n";
            }
        }

        // Forward kinematics
        else if (commands[0] == "forward"){
            Configuration config = manipulator.get_config();
            int n_links = config.num_links;
            if (commands.size() == n_links + 1){
                double angles[n_links];
                for (int i = 0; i < commands.size() - 1; i += 1){
                    angles[i] = atof(commands[i + 1].c_str());
                }
                manipulator.forward_kinematics(angles);
            }

            else{
                cout << "Invalid number of angles.\n";
            }        
        }

        // Intersection
        else if (commands[0] == "intersection"){
            // manipulator.intersection();     
        }

        // Inverse kinematics
        else if (commands[0] == "inverse_kinematics"){
            // manipulator.inverse_kinematics();            
        }

        // Inverse dynamics
        else if (commands[0] == "inverse_dynamics"){
            // manipulator.inverse_dynamics();         
        }

        // Exit program
        else if (commands[0] == "exit"){
            exit_flag = true;
        }

        else{
            cout << "\nNot a valid command\n";
        }
    }
        
    return 0;
}
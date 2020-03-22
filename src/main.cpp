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
    cout << "  - intersection X Y R THETA_1 THETA_2 ...\n";
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
    //Init Manipulator
    bool exit_flag = false;
    Manipulator manipulator;
    manipulator.reset();
    print_robot_config(manipulator.get_config());
    print_available_commands();

    // Set print settings
    cout.precision(3);
    cout << fixed << boolalpha;

    while (!exit_flag){

        // Get user input
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
                config = manipulator.get_config();
                cout << "End effector position x: " << config.x << ", y: " 
                        << config.y << ", theta: " << config.theta << endl; 
            }

            else{
                cout << "Invalid number of angles.\n";
            }        
        }

        // Intersection
        else if (commands[0] == "intersection"){
            Configuration config = manipulator.get_config();
            int n_links = config.num_links;
            if (commands.size() == n_links + 4){
                double angles[n_links];
                double x = atof(commands[1].c_str());
                double y = atof(commands[2].c_str());
                double r = atof(commands[3].c_str());
                for (int i = 4; i < commands.size(); i += 1){
                    angles[i] = atof(commands[i].c_str());
                }
                bool is_in_circle = manipulator.intersection(x, y, r, angles);
                config = manipulator.get_config();
                cout << "Circle x: " << x << ", y: " << y 
                    << ", r: " << r << endl;
                cout << "End effector position x: " << config.x << ", y: " 
                    << config.y << ", theta: " << config.theta << endl;
                cout << "Is within circle: " << is_in_circle << endl;
            }

            else{
                cout << "Invalid number of arguments.\n";
            }      
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
        cout << endl; 
    }
        
    return 0;
}
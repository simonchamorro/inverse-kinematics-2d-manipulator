/********
 * main.cpp
 * Author: Simon Chamorro
 * Main file to test Robot Manipulator class interactively
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
    cout << "  - links LINK_1 LINK_2 ...\n";
    cout << "  - forward THETA_1 THETA_2 ...\n";
    cout << "  - intersection X Y R THETA_1 THETA_2 ...\n";
    cout << "  - inverse_k X Y THETA\n";
    cout << "  - inverse_d FX FY TAU\n";
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
    cout << "  - Joints (deg): ";
    for (int i = 0; i < config.num_links; i += 1){
        cout << config.angles[i] << " ";
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
            print_robot_config(manipulator.get_config());
            print_available_commands();            
        }
        
        // Reset robot parameters
        else if (commands[0] == "reset"){
            manipulator.reset();
            print_robot_config(manipulator.get_config());     
        }

        // Change robot parameters
        else if (commands[0] == "links"){

            if (commands.size() < MAX_LINKS && commands.size() > 1){
                int n_links = commands.size() - 1;
                double links[n_links];
                for (int i = 1; i < commands.size(); i += 1){
                    links[i - 1] = atof(commands[i].c_str());
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
        else if (commands[0] == "inverse_k"){
            if (commands.size() == 4 && manipulator.get_config().num_links == 3){
                double x = atof(commands[1].c_str());
                double y = atof(commands[2].c_str());
                double theta = atof(commands[3].c_str());
                double angles_1[MAX_LINKS];
                double angles_2[MAX_LINKS];
                if (manipulator.inverse_kinematics(x, y, theta, angles_1, angles_2)){
                    cout << "Configuration 1: " << angles_1[0] << ", " << angles_1[1] 
                        << ", " << angles_1[2] << endl;
                    cout << "Configuration 2: " << angles_2[0] << ", " << angles_2[1] 
                        << ", " << angles_2[2] << endl;
                }
                else{
                    cout << "Position unreachable.\n";
                }
                
            }
            else{
                cout << "Invalid arguments or number of links.\n";
            }        
        }

        // Inverse dynamics
        else if (commands[0] == "inverse_d"){
            if (commands.size() == 4 && manipulator.get_config().num_links == 3){
                double fx = atof(commands[1].c_str());
                double fy = atof(commands[2].c_str());
                double tau = atof(commands[3].c_str());
                double torques[MAX_LINKS];
                if (manipulator.inverse_dynamics(fx, fy, tau, torques)){
                    cout << "Torques: " << torques[0] << ", " << torques[1] 
                        << ", " << torques[2] << endl;
                }
                
            }
            else{
                cout << "Invalid arguments or number of links.\n";
            }          
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
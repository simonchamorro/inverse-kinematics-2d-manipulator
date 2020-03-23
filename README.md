# inverse-kinematics-2d-manipulator

Planar Robotic Manipulator.

## Build and usage

### Build
To build main program and test program:

```bash
 cd inverse-kinematics-2d-manipulator/
 make
```
This will create two executables: `bin/run` and `bin/test`.

### Testing

To run tests:
```bash
bin/test 
```

The framework used for testing is [Catch](https://github.com/catchorg/Catch2)

### Run 

To launch the main program:
```bash
bin/run 
```

This will create an instance of a Robot Manipulator and show this interface:
```bash
----- 2D Robot Manipulator -----
Configuration:
  - Number of links: 3
  - Links: 1 1 1 
  - Joints (deg): 0 0 0 
--------------------------------
Available Commands:
  - help
  - reset
  - links LINK_1 LINK_2 ...
  - forward THETA_1 THETA_2 ...
  - intersection X Y R THETA_1 THETA_2 ...
  - inverse_k X Y THETA
  - inverse_d FX FY TAU
  - exit
--------------------------------
```

### Functions

#### help
Shows the current state of the robot and the commands available.

#### reset
Resets the robot to the default state.

#### links
Changes the robot links. The desired lengths for the links are given as parameters. There have to be between 1 and 10 links.  

#### forward
Changes the robot configuration using its forward dynamics. The joint positions are given as parameters and the end effector's position (x, y, theta) is computed.

#### intersection
Given a circle (x, y center and radius) and joint positions, the functions checks if the end effector is within that circle. The circle parameters and the joint angles are given as parameters.

#### inverse_k
Inverse kinematics. Given the desired position of the end effector (x, y, theta), the function returns the joint angles if the position is reachable. Only works when the robot has 3 links.

#### inverse_d
Inverse dynamics. Given a desired force at the end effector (fx, fy, tau), the function returns the joint torques. Only works when the robot has 3 links.




# A-path-planning-with-LIDAR
This is a code for an autonomous robot to navigate through an unknown environment using LIDAR.
The robot contains a LIDAR sensor to take input from the envirinment. The information gets processed in Raspberry Pi. PWM motor control is enebaled.
The layout of the program is as follows. The raspberry pi and the operator computer should be connected to a common local network. The LIDAR sensor is programmed to generate a 2D map of the environment,when the robot is kept at a place. This map is visible in the screen of the operator. The ooperator now can select a start and stop position in the map by simply clicking by mouse. The robot will find the shortest path to go to that final position autonomously. and this shortest path will be displayed in the screen.

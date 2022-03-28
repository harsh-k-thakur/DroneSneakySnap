# DroneSneakySnap

This project was build as a part of my quarter studies at University of California San Diego.
This project was executed in a group of two. My partner Rye Gleason and I shared the idea and code together to build an end product. 

This project started with understanding dynamics and kinematics of drone. How does Roll, Pitch, Yaw and Throttle works in an open space.

Once I understood the basic dynamics of the drone. 

I started building with the Hardware. For this I used '''Fusion360''' to create new libraries for my IMU, LEDS and Resistors.
Some of the libraries were directly provided to us, and which are readily available in Fusion360 as well. Thus, understanding the basic schematic of a particular part and utilizing the knowledge of that part to integrate in the main project was important. 
You can see datasheet and schmetics of different electronics component under [datasheeet]('/datasheeet/') folder.

All the necessary things related to Hardware are available in [Hardware]('/Hardware/') folder. Things like Schematics, Fusion360 files, Gerber Files are located in the [Hardware]('/Hardware/') folder. Feel free to check those out.

Once done with Hardware, we started working on software. 
While our Printed Circuit Board (PCB) got printed and came back to us we were given a temporary quadcopter in order to start with programming. 
For the software we had Atmega Arduino Micro-controller on the Flight Controller Board. 
Started the project with basic communication between the remote and the quadcopter using Radio Frequency, testing each motors individually works at different spped using Pulse Width Modulation (PWM).
Integrated throttle control from remote and RF together so that we can control the speed using the remote.

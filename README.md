# SwissCheese
Virtual Swiss Cheese Game

Creating a environment for the game of jeu gruyere using pygame.

https://makezine.com/projects/the-game-of-jeu-gruyere/

#Inputs
The input into the system will always be the length of the cord regardless of what control method is used. 

# Goals
The goal is to solve this a couple of ways:

* Manually by controlling the cords
* Using motors to run the inverse kinematics to allow control of x,y via a joystick
* AI algorithm that controls the lengths of the cords
* AI algorithm that controls the x,y position, and then computes the lengths of cords

# Stepper Motors
For the motion axes, going to try to use stepper motors running some GT2 belt in order to maintain position. The stepper motors are used because the RAMPS 1.4 board was available and does not requrie me to make a PCB for this project as the RAMPS already supports all of the I/O that are required, including some FETS for the solenoid on the crane project. 

The engineering challenge here is creating a dual axes motion control. The inverse and forward kinematics are solved in the python game script. The motion control consists of generating timing pulses to control the stepper motors which is pretty straightforward. The issue becomes how to control both of the motors when going between two different points.

```
 X  (start)
  \
   \
    \ 
     X (end)
```   
The delta Y here is larger than the delta X, so the Y axes must move faster than X axis such that both of them arrive at the same time. The issue is that the machine is not linear, so the angle of the cables come into place. 



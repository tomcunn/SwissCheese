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
For the motion axes, going to try to use stepper motors running some GT2 belt in order to maintain position. The stepper motors are used because the RAMPS 1.6

board was available and does not requrie me to make a PCB for this project as the RAMPS already supports all of the I/O that are required, including some FETS for the solenoid on the crane project. 

The engineering challenge here is creating a dual axes motion control. The inverse and forward kinematics are solved in the python game script. The motion control consists of generating timing pulses to control the stepper motors which is pretty straightforward. The issue becomes how to control both of the motors when going between two different points.

```
 X  (start)
  \
   \
    \ 
     X (end)
```   
The delta Y here is larger than the delta X, so the Y axes must move faster than X axis such that both of them arrive at the same time. The issue is that the machine is not linear, so the angle of the cables come into place. 

# Kinematics
The arduino only uses the inverse kinematics. 

```
#Set the kinematics type to use
#FORWARD - Control the lengths of the cords , then compute x,y
#INVERSE - Control the x,y , then compute lengths
KINEMATICS_TYPE = 'INVERSE'
```

Here are the kinematics for the two cable. 

```
.
(0,0)
  A    ab    B
   \       /
    \     /
 ad  \   / bd
      \ /
       D
#Given x,y (0,0) is top left and the length between the two joints, compute the ad and bd

    ad = math.sqrt(dx*dx+yx*yx)
    bd = math.sqrt((Bx-x)*(Bx-x)+y*y)
   
```

# Path Planning

How do you go from (2,10) to (5,5), with (0,0) being the upper left. The delta is 3 in the X, and is 5 in the Y. Maybe we just simply have a master axis, and then have a dependent axis. Meaning that we command the X to 10 inches/min, and then we know that X needs to travel 5/3 faster, so we command the Y to 16.66 inches/min. The difficult becomes what about the acceleration phase and computing time and position to sync the two axes. The next question is what is good enough. For manual control 


# Hardware Hookup

```
//These pins have been verified to work
#define X_STEP    A0
#define X_DIR     A1
#define X_ENABLE  38

#define Z_STEP    46
#define Z_DIR     48
#define Z_ENABLE  A8
```

The stepper motor connection to the RAMPS 1.6 Board. 

![image](https://user-images.githubusercontent.com/4383135/206915399-de5b6786-f67e-4cdd-bd44-950103174a18.png)

So far this appears to be the pinout of the RAMP 1.6 Board.

![image](https://user-images.githubusercontent.com/4383135/206915461-dd8e6569-5e97-4169-9c45-559e67a225a7.png)

# Dual Joystick Control

Creating a dual joystick to demo. There will be two modes:

 * Each joystick y controls the length (no kinematics required, the human does the kinematics)
 * One joystick controls the x,y 

```
//Joystick Wiring
Wire Color    Physical  Jumper Board   MEGA
Green/White   5V         Power          5V
Blue          Gnd        Gnd            GND
Orange/White  Right Y    AN0            A3
Orange        Left Y     AN1            A4
Blue/White    Right X    Switch A       A5
Brown/White   Toggle     Switch B       D44
```



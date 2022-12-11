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

# Kinematics
There are two types of kinematics that need to be considered here.

```
#Set the kinematics type to use
#FORWARD - Control the lengths of the cords , then compute x,y
#INVERSE - Control the x,y , then compute lengths
KINEMATICS_TYPE = 'INVERSE'
```

Here are the kinematics for the two cable. 

```
Not sure if L_a and L_b are swapped on this diagram.

  A   L_c   B
   \       /
    \     /
 L_a \   / L_b
      \ /
       C

#Given the lengths of the strings (L_a and L_b) and the distance between the two (L_c) 
#compute the x,y (0,0) is top left
def ForwardKinematics(L_a,  L_b,  L_c):
    #Compute the angle of A
    Angle_A = ((L_b*L_b)+(L_c*L_c)-(L_a*L_a))/(2*L_b*L_c)
    Angle_A = math.acos(Angle_A)
    #Compute the x,y based on a triangle
    x = math.cos(Angle_A)*L_b
    y = math.sin(Angle_A)*L_b
    return x,y

#Given x,y (0,0) is top left and the length between the two joints, compute the L_a and L_b
def InverseKinematics(x,y,L_c):
    L_a = math.sqrt(x*x+y*y)
    L_b = math.sqrt((L_c-x)*(L_c-x)+y*y)
    return L_a, L_b
```

# Path Planning

How do you go from (2,10) to (5,5), with (0,0) being the upper left. The delta is 3 in the X, and is 5 in the Y. Maybe we just simply have a master axis, and then have a dependent axis. Meaning that we command the X to 10 inches/min, and then we know that X needs to travel 5/3 faster, so we command the Y to 16.66 inches/min. The difficult becomes what about the acceleration phase and computing time and position to sync the two axes. The next question is what is good enough. For manual control 

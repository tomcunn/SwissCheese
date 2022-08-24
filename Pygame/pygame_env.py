import pygame
import math
import numpy as np

pygame.init()

#Set the kinematics type to use
#FORWARD - Control the lengths of the cords , then compute x,y
#INVERSE - Control the x,y , then compute lengths
KINEMATICS_TYPE = 'INVERSE'

# Setting up color objects
BLUE  = (0, 0, 255)
RED   = (255, 0, 0)
WOOD = (226, 206, 114)
BLACK = (0, 0, 0)
WHITE = (230, 230, 230)
GREY = (140,140,140)

# Creating a board that is 30" Wide by 48" tall.
# Scale is # of pixels per inch
scale = 20
height = 40 * scale
width = 24 * scale

#setup the position of the actuators
#Compute everything in inches, then scale later
A = ((0,0))
B = ((30, 0))
L_a = 40
L_b = 40
L_c = 24
current_position_x = 12
current_position_y = 30


#Create a list of circles
#(center x, center y, radius)
circle_def_list = [(5,4,1.5),
                   (12,5,1),
                   (17,6,1.5),
                   (21,3,0.5),
                   (4,10,2),
                   (8,8,1),
                   (12,10,0.5),
                   (16,11,1.5),
                   (10,11,1.0)]

DISPLAYSURF = pygame.display.set_mode((width,height))
pygame.display.set_caption("Jeu Gruyere")
DISPLAYSURF.fill(WOOD)

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

def DetectFall(x,y,circle_x,circle_y, circle_r):

    #Figure out if the distance between the current point and the circle center is less than the radius
    distX = x - circle_x;
    distY = y - circle_y;
    distance = math.sqrt( (distX*distX) + (distY*distY))

    if(distance < circle_r):
       Fall = True
    else:
       Fall = False
    return Fall


# Beginning Game Loop
while True:
    if(KINEMATICS_TYPE == 'FORWARD'):
        #Compute the position of the ball based on the line lengths
        current_position_x,current_position_y = ForwardKinematics(L_a,L_b,L_c)
    else:
        #Testing the inverse kinematics
        L_a, L_b = InverseKinematics(current_position_x,current_position_y,L_c)

    DISPLAYSURF.fill(WOOD)

    #Use the keyboard to change the line lengths
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

        if ((event.type == pygame.KEYDOWN) and (KINEMATICS_TYPE == 'FORWARD')):
            if event.key == pygame.K_q:
               L_b -= 0.25
            if event.key == pygame.K_w:
               L_a -= 0.25
            if event.key == pygame.K_a:
               L_b += 0.25
            if event.key == pygame.K_s:
               L_a += 0.25
        elif ((event.type == pygame.KEYDOWN) and (KINEMATICS_TYPE == 'INVERSE')):
            if event.key == pygame.K_UP:
               current_position_y -= 0.25
            if event.key == pygame.K_DOWN:
               current_position_y += 0.25
            if event.key == pygame.K_RIGHT:
               current_position_x += 0.25
            if event.key == pygame.K_LEFT:
               current_position_x -= 0.25

    #Scale from inches to pixels for drawing
    current_position = ((scale * current_position_x, scale * current_position_y))

    #Debugging
    print(L_a,L_b,current_position_x,current_position_y)

    #Draw the failure areas
    for a in range(0,len(circle_def_list)):

        #Extract the position and radius of circles from the list
        x,y,r = circle_def_list[a]

        #Determine the failure criteria
        Failure = DetectFall(x,y,current_position_x,current_position_y,r)

        #Draw and color the circles
        if(Failure == True):
          color = RED
        else:
          color = BLACK
        pygame.draw.circle(DISPLAYSURF, color, (x*scale,y*scale), r*scale)

    # Draw a the ball and lines
    pygame.draw.line(DISPLAYSURF, GREY, (0,0),current_position,1)
    pygame.draw.line(DISPLAYSURF, GREY, (width,0),current_position, 1)
    pygame.draw.circle(DISPLAYSURF, WHITE, current_position, 10)


    # Flip the display
    pygame.display.flip()

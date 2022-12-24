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
# 1 pixel per mm
height = 600
width = 470

#setup the position of the actuators
#Compute everything in inches, then scale later
A = ((0,0))
B = ((470, 0))
AD = 394
BD = 394
AB = 470
current_position_x = 235
current_position_y = 316


#Create a list of circles
#(center x, center y, radius)
circle_def_list = [(127,102,38),
                   (305,127,25),
                   (432,152,38),
                   (533,76,13),
                   (102,254,51),
                   (203,203,25),
                   (305,254,13),
                   (406,280,38)]

DISPLAYSURF = pygame.display.set_mode((width,height))
#Create an array to tract the data
tracking_data = np.zeros((width,height))

pygame.display.set_caption("Jeu Gruyere")
DISPLAYSURF.fill(WOOD)

#Kinematics (distance in mm)
#  A(0,0)        B(470,0)
#   A          B
#     \       /
#      \     /
#       \   /
#         D
#Given x,y, find the distances AD, BD
def ForwardKinematics(AD,  BD,  AB):
    #Compute the angle of A
    AD2 = AD * AD
    BD2 = BD * BD

    x = (AD2 - BD2 + (Bx_distance * Bx_distance)) / (2 * Bx_distance)
    y2 =  AD2 - (x * x);

    y = math.sqrt(y2)

    return x,y

#Given x,y (0,0) is top left and the length between the two joints, compute the L_a and L_b
def InverseKinematics(x,y,AB):
    x2 = x * x;
    y2 = y * y;
    AD = math.sqrt(x2+y2)
    BD = math.sqrt((AB-x)*(AB-x)+y2);
    return AD, BD

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
        current_position_x,current_position_y = ForwardKinematics(AD,BD,AB)
    else:
        #Testing the inverse kinematics
        AD, BD = InverseKinematics(current_position_x,current_position_y,AB)

    DISPLAYSURF.fill(WOOD)

    #Use the keyboard to change the line lengths
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

        if ((event.type == pygame.KEYDOWN) and (KINEMATICS_TYPE == 'FORWARD')):
            if event.key == pygame.K_q:
               BD -= 1
            if event.key == pygame.K_w:
               AD -= 1
            if event.key == pygame.K_a:
               BD += 1
            if event.key == pygame.K_s:
               AD += 1
        elif ((event.type == pygame.KEYDOWN) and (KINEMATICS_TYPE == 'INVERSE')):
            if event.key == pygame.K_UP:
               current_position_y -= 5
            if event.key == pygame.K_DOWN:
               current_position_y += 5
            if event.key == pygame.K_RIGHT:
               current_position_x += 5
            if event.key == pygame.K_LEFT:
               current_position_x -= 5

    #Scale from mm to pixels for drawing
    current_position = ((current_position_x,current_position_y))

    #Debugging
    print(AD,BD,current_position_x,current_position_y)

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
        pygame.draw.circle(DISPLAYSURF, color, (x,y), r)

    # Draw a the ball and lines
    pygame.draw.line(DISPLAYSURF, GREY, (0,0),current_position,1)
    pygame.draw.line(DISPLAYSURF, GREY, (width,0),current_position, 1)
    pygame.draw.circle(DISPLAYSURF, WHITE, current_position, 27)
    tracking_data[current_position_x,current_position_y] = 1
    
    i= 0
    b = 0
    #draw the tracking line
    for i in range(0,height):
        
        
        for b in range(0,width):

            #Check to see if we should draw this pixel
            if(tracking_data[b][i] == 1):
                DISPLAYSURF.set_at((b,i),(255,0,0))
            b=b+1
        i=i+1
    

    # Flip the display
    pygame.display.flip()

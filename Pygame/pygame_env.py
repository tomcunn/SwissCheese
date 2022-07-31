import pygame
import math

pygame.init()

# Setting up color objects
BLUE  = (0, 0, 255)
RED   = (255, 0, 0)
GREEN = (0, 255, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

# Creating a board that is 30" Wide by 48" tall.
# Scale is # of pixels per inch
scale = 20
height = 48 * scale
width = 30 * scale

#setup the position of the actuators
#Compute everything in inches, then scale later
A = ((0,0))
B = ((30, 0))
L_a = 40
L_b = 40
L_c = 30

# set the current position of the ball
current_position_x = 15*scale
current_position_y = 24*scale



DISPLAYSURF = pygame.display.set_mode((width,height))
pygame.display.set_caption("Jeu Gruyere")
DISPLAYSURF.fill(WHITE)

def ForwardKinematics(L_a,  L_b,  L_c):
    #Compute the angle of A
    Angle_A = ((L_b*L_b)+(L_c*L_c)-(L_a*L_a))/(2*L_b*L_c)
    Angle_A = math.acos(Angle_A)
    print(math.degrees(Angle_A))
    #Computer the x,y based on a triangle
    x = math.cos(Angle_A)*L_b
    y = math.sin(Angle_A)*L_b
    return x,y

# Beginning Game Loop
while True:
    #Compute the position of the ball based on the line lengths
    current_position_x,current_position_y = ForwardKinematics(L_a,L_b,L_c)
    DISPLAYSURF.fill(WHITE)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
               L_a -= 0.25
            if event.key == pygame.K_w:
               L_b -= 0.25
            if event.key == pygame.K_a:
               L_a += 0.25
            if event.key == pygame.K_s:
               L_b += 0.25

    current_position = ((scale * current_position_x, scale * current_position_y))

    print(L_a,L_b,current_position_x,current_position_y)

    pygame.draw.circle(DISPLAYSURF, (0, 255, 255), (20*scale,15*scale), 3*scale)
    pygame.draw.circle(DISPLAYSURF, (0, 255, 255), (30*scale,20*scale), 3*scale)
    # Draw a solid blue circle in the center
    pygame.draw.circle(DISPLAYSURF, (0, 0, 255), current_position, 10)
    pygame.draw.line(DISPLAYSURF, (0, 0, 255), (0,0),current_position,1)
    pygame.draw.line(DISPLAYSURF, (0, 0, 255), (width,0),current_position, 1)

    # Flip the display
    pygame.display.flip()

import pygame

pygame.init()

# Setting up color objects
BLUE  = (0, 0, 255)
RED   = (255, 0, 0)
GREEN = (0, 255, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

# Creating a board that is 30" Wide by 48" tall.
scale = 20
height = 48 * scale
width = 30 * scale

# set the current position of the ball
current_position_x = 15*scale
current_position_y = 24*scale


DISPLAYSURF = pygame.display.set_mode((width,height))
pygame.display.set_caption("Jeu Gruyere")
DISPLAYSURF.fill(WHITE)

# Beginning Game Loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYPRESS:
            if event.key == pygame.K_LEFT:
               current_position_x -= 1

    current_position = ((current_position_x,current_position_y))
    # Draw a solid blue circle in the center
    pygame.draw.circle(DISPLAYSURF, (0, 0, 255), current_position, 10)
    pygame.draw.line(DISPLAYSURF, (0, 0, 255), (0,0),current_position,1)
    pygame.draw.line(DISPLAYSURF, (0, 0, 255), (width,0),current_position, 1)

    # Flip the display
    pygame.display.flip()

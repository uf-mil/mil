#!/usr/bin/env python
"""

This program is for use in aiding the development of the navigation 
system. It will not be part of what we give to Lockheed, it is just
to help move the robot around during testing 

This code creates a GUI grid system which we can then click on to move the 
robot to that position. It is designated in half meter moves for now
and can only do half and whole meter moves. 

Source:
    Sample Python/Pygame Programs
    Simpson College Computer Science
    http://programarcadegames.com/
    http://simpson.edu/computer-science/

"""
import pygame
import rospy
from geometry_msgs.msg import PoseStamped
from robotx_msgs.msg import PoseTwistStamped
 
# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
 
# This sets the WIDTH and HEIGHT of each grid location
WIDTH = 42
HEIGHT = 42
 
# This sets the margin between each cell
MARGIN = 3
 
# Create a 2 dimensional array. A two dimensional
# array is simply a list of lists.
grid = []
for row in range(11):
    # Add an empty array that will hold each cell
    # in this row
    grid.append([])
    for column in range(11):
        grid[row].append(0)  # Append a cell
 
# Set row 1, cell 5 to one. (Remember rows and
# column numbers start at zero.)
grid[1][5] = 1
 
# Initialize pygame
pygame.init()
 
# Set the HEIGHT and WIDTH of the screen
WINDOW_SIZE = [500, 500]
screen = pygame.display.set_mode(WINDOW_SIZE)
 
# Set title of screen
pygame.display.set_caption("Array Backed Grid")
 
# Loop until the user clicks the close button.
done = False
 
# Used to manage how fast the screen updates
clock = pygame.time.Clock()

rospy.init_node("move_helper_node")
desired_pos = rospy.Publisher('/trajectory', PoseTwistStamped, queue_size=1)

 
# -------- Main Program Loop -----------
while not done and not rospy.is_shutdown():
    for event in pygame.event.get():  # User did something
        if event.type == pygame.QUIT:  # If user clicked close
            done = True  # Flag that we are done so we exit this loop
        elif event.type == pygame.MOUSEBUTTONDOWN:
            # User clicks the mouse. Get the position
            pos = pygame.mouse.get_pos()
            # Change the x/y screen coordinates to grid coordinates
            column = pos[0] // (WIDTH + MARGIN)
            row = pos[1] // (HEIGHT + MARGIN)
            # Set that location to zero
            grid[row][column] = 1

            print("Click ", pos, "Grid coordinates: ", -(row - 5.0)/2.0, (column  - 5.0)/2.0)
            # On click, call the ROS trajectory service to command the robot to move to a given location
            to_send = PoseTwistStamped()
            # Fill the x and y positions with the offset commands
            to_send.posetwist.pose.position.x = -(row - 5.0)/2.0
            to_send.posetwist.pose.position.y = (column  - 5.0)/2.0
            desired_pos.publish(to_send)
        
 
    # Set the screen background
    screen.fill(BLACK)
 
    # Draw the grid
    for row in range(11):
        for column in range(11):
            color = WHITE
            if row == 5 and column == 5:
                color = BLACK
            pygame.draw.rect(screen,
                             color,
                             [(MARGIN + WIDTH) * column + MARGIN,
                              (MARGIN + HEIGHT) * row + MARGIN,
                              WIDTH,
                              HEIGHT])
 
    # Limit to 60 frames per second
    clock.tick(60)
 
    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()
 
# Be IDLE friendly. If you forget this line, the program will 'hang'
# on exit.
pygame.quit()
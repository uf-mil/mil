#!/usr/bin/env python

"""
 Example program to show using an array to back a grid on-screen.
 
 Sample Python/Pygame Programs
 Simpson College Computer Science
 http://programarcadegames.com/
 http://simpson.edu/computer-science/
 
 Explanation video: http://youtu.be/mdTeqiWyFnc
"""
import pygame
from nav_msgs.msg import Odometry
import rospy
from time import time
 
# Defines
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
grid_size = 200
 
# This sets the WIDTH and HEIGHT of each grid location
WIDTH = 2
HEIGHT = 2
 
# This sets the margin between each cell
MARGIN = 1

# Initialize pygame
pygame.init()
pygame.display.set_caption("Navigation visualization")
WINDOW_SIZE = [500, 500]
screen = pygame.display.set_mode(WINDOW_SIZE)
clock = pygame.time.Clock()
rospy.init_node("nav_mapper")

class nav_vis_vessel(object):
    def __init__(self):
        self.grid = []
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        for row in range(grid_size):
            self.grid.append([])
            for column in range(grid_size):
                self.grid[row].append(0)  # Append a 0 into that cell

    def odom_callback(self, odom):
        column = int((2*odom.pose.pose.position.x)-50)
        row = int((2*odom.pose.pose.position.y)-40)
        self.grid[column][row] = 1

    def draw(self):
        for row in range(grid_size):
            for column in range(grid_size):
                color = WHITE
                if self.grid[row][column] == 1:
                    color = BLACK
                pygame.draw.rect(screen,
                                 color,
                                 [(MARGIN + WIDTH) * column + MARGIN,
                                  (MARGIN + HEIGHT) * row + MARGIN,
                                  WIDTH,
                                  HEIGHT])

boat = nav_vis_vessel()
while not rospy.is_shutdown():

    screen.fill(BLACK)
    boat.draw()
    clock.tick(60)
    pygame.display.flip()

pygame.quit()
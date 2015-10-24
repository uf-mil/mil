#!/usr/bin/env python

from __future__ import division
import rospy
import roslib
import pygame
import sys
from time import time
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import WrenchStamped

# Color Definitions
red = (255,0,0)
green = (0,255,0)
blue = (0,0,255)
darkBlue = (0,0,128)
white = (255,255,255)
black = (0,0,0)
pink = (255,200,200)

DISPLAY_SIZE = (640,640)
ROBOT_SIZE = (100,150)
WHEEL_SIZE = (10, 10)

rospy.init_node('mecanum_simulator')


class robot(object):

	def __init__(self):

		self.ROBOT_SIZE = ROBOT_SIZE
		self.robot_position = 0
		self.x_pos = DISPLAY_SIZE[0]/2.5
		self.y_pos = DISPLAY_SIZE[1]/2.5
		self.bigger = pygame.Rect(0, 0, 100, 50)
		self.vehicle = pygame.Surface(ROBOT_SIZE)
		self.cur_theda = 0

		# To become to end position of the line drawn
		self.desired_rotations = [self.x_pos + 30, self.x_pos + 30, self.x_pos + 120, self.x_pos + 120]
		self.wrench = [0,0,0]

		# Font Definitions
		self.myfont = pygame.font.SysFont("arial", 20)
		self.myfont2 = pygame.font.SysFont("arial", 15)

		self.mecanum2_data = (0,0,10,10)
		self.mecanum1_data = (0,0,10,10)
		self.mecanum3_data = (0,0,10,10)
		self.mecanum4_data = (0,0,10,10)
		self.m1_start = (0,0)
		self.m2_start = (0,0)
		self.m3_start = (0,0)
		self.m4_start = (0,0)
		self.m1_end = (0,0)
		self.m2_end = (0,0)
		self.m3_end = (0,0)
		self.m4_end = (0,0)

		# ROS pieces
		rospy.Subscriber('/mecanum', Int32MultiArray, self.mecanum_callback, queue_size = 1)
		rospy.Subscriber('/wrench', WrenchStamped, self.wrench_callback)

	def wrench_callback(self, msg):
		self.wrench[0] = msg.wrench.force.x
		self.wrench[1] = msg.wrench.force.y
		self.wrench[2] = msg.wrench.torque.z

	def mecanum_callback(self, msg):
		for i in range(0,len(msg.data)):

			if i == 0: self.desired_rotations[i] = -1 * msg.data[i] 
			if i == 1: self.desired_rotations[i] = -1 * msg.data[i] 
			if i == 2: self.desired_rotations[i] = -1 * msg.data[i] 
			if i == 3: self.desired_rotations[i] = -1 * msg.data[i] 

	def format_visualization(self):

		self.mecanum2_data = (self.x_pos - 10, self.y_pos + 30, 10,10)
		self.mecanum1_data = (self.x_pos + ROBOT_SIZE[0], self.y_pos + 30, 10,10)
		self.mecanum3_data = (self.x_pos - 10, self.y_pos + ROBOT_SIZE[1] - 30, 10,10)
		self.mecanum4_data = (self.x_pos + ROBOT_SIZE[0], self.y_pos + ROBOT_SIZE[1] - 30, 10,10)

		self.m1_start = (self.x_pos+ROBOT_SIZE[0] + 5, self.y_pos + 30)
		self.m2_start = (self.x_pos-5 , self.y_pos + 30)
		self.m3_start = (self.x_pos - 5 , self.y_pos + ROBOT_SIZE[1] - 30)
		self.m4_start = (self.x_pos + ROBOT_SIZE[0] + 5 , self.y_pos + ROBOT_SIZE[1] - 30)

		self.m1_end = (self.x_pos - 35 + (.03*self.desired_rotations[0])*self.desired_rotations[0] + self.ROBOT_SIZE[1], 5*self.desired_rotations[0] + 30 + self.y_pos )
		self.m2_end = (self.x_pos - 15 - (.03*self.desired_rotations[1])*self.desired_rotations[1], 5*self.desired_rotations[1] + 30 + self.y_pos)
		self.m3_end = (self.x_pos - 15 - (.03*self.desired_rotations[2])*self.desired_rotations[2], 5*self.desired_rotations[2] + 120 + self.y_pos )
		self.m4_end = (self.x_pos - 35 + (.03*self.desired_rotations[3])*self.desired_rotations[3] +  self.ROBOT_SIZE[1], 5*self.desired_rotations[3] + 120 + self.y_pos )

	def draw(self):
 
		# Draw vehicle and wheels
		screen.blit(self.vehicle, (self.x_pos,self.y_pos))
		pygame.draw.rect(screen, red, (self.mecanum1_data))
		pygame.draw.rect(screen, red, (self.mecanum2_data))
		pygame.draw.rect(screen, red, (self.mecanum3_data))
		pygame.draw.rect(screen, red, (self.mecanum4_data))
		self.vehicle.fill(white)

		# Array used to store text labels
		label = [0,0,0,0,0,0,0,0,0,0,0,0]

		# Render all text
		label[1] = self.myfont.render("Vx: " + str(self.wrench[0]), 1, blue)
		label[2] = self.myfont.render("Vy: " + str(self.wrench[1]), 1, blue)
		label[3] = self.myfont.render("Vz: " + str(self.wrench[2]), 1, blue)
		label[4] = self.myfont.render("M1: " + str(-self.desired_rotations[0]), 1, green)
		label[5] = self.myfont.render("M2: " + str(-self.desired_rotations[1]), 1, green)
		label[6] = self.myfont.render("M3: " + str(-self.desired_rotations[2]), 1, green)
		label[7] = self.myfont.render("M4: " + str(-self.desired_rotations[3]), 1, green)
		label[8] = self.myfont2.render("M1", 1, black)
		label[9] = self.myfont2.render("M2", 1, black)
		label[10] = self.myfont2.render("M3", 1, black)
		label[11] = self.myfont2.render("M4", 1, black)

		# Draw vectors to visualize wheel configuration
		pygame.draw.line(screen, blue, self.m1_start, self.m1_end)
		pygame.draw.line(screen, blue, self.m2_start, self.m2_end)
		pygame.draw.line(screen, blue, self.m3_start, self.m3_end)
		pygame.draw.line(screen, blue, self.m4_start, self.m4_end)

		# Print text showing 
		for i in range(1,8):
			screen.blit(label[i], (450, (50+(25*i))))

		# Print wheel labels 
		screen.blit(label[8], (self.x_pos+ROBOT_SIZE[0] - 25, self.y_pos + 25))
		screen.blit(label[9], (self.x_pos + 3 , self.y_pos + 25))
		screen.blit(label[10], (self.x_pos + 3, self.y_pos + ROBOT_SIZE[1] - 35))
		screen.blit(label[11], (self.x_pos + ROBOT_SIZE[0] -25, self.y_pos + ROBOT_SIZE[1] - 35))

if __name__ == '__main__':

	pygame.init()
	pygame.key.set_repeat(1,10)
	screen = pygame.display.set_mode(DISPLAY_SIZE)
	surf =  pygame.Surface(ROBOT_SIZE)
	screen.fill(black)
	clock = pygame.time.Clock()
	bot = robot()

	while True and not rospy.is_shutdown():
		bot.format_visualization()
		bot.draw()
		clock.tick(60)
		pygame.display.flip()
		screen.fill(black)



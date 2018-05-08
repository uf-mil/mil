#!/usr/bin/python

import sys
import os
import docker
import curses
from cursesmenu import *
from cursesmenu.items import *
import distutils.spawn
import subprocess

__author__ = 'Daniel Volya (RustyBamboo)'


class DockerController(object):

    def __init__(self, stdscr):
        self.stdscr = stdscr

        # Check if docker server is running
        try:
            self.docker_client = docker.from_env()
        except Exception:
            assert False, 'Is Docker server running?'

        # Attempt to load container, if can't then later ask user to select image
        self.container = None
        # If container is currently running
        if "RustyROS" in [x.name for x in self.docker_client.containers.list()]:
            self.container = self.docker_client.containers.get('RustyROS')
        # Check if container is stopped, but still exists
        else:
            try:
                self.container = self.docker_client.containers.get('RustyROS')
                self.container.start()
            except Exception:
                self.container = None

        # if no docker images are avaliable, go ahead and build one
        if not self.docker_client.images.list():
            self.brand_new_docker_image()

        # attempt to find what kind of terminal emulator is being used
        self.terminal = None
        self.find_avaliable_terminal()
        assert not self.terminal is None, 'No terminal found'

        # run curses stuff
        self.draw()

    # Finds terminal emulator
    def find_avaliable_terminal(self):
        # possible terminal emulators, with last having largest priorty
        terminals = ["gnome-terminal", "xterm", "konsole"]
        for t in terminals:
            x = distutils.spawn.find_executable(t)
            if not x is None:
                self.terminal = t

    # Builds the docker image, and shows some info for user
    def brand_new_docker_image(self):
        self.stdscr.clear()

        height, width = self.stdscr.getmaxyx()
        text = "Building image..."[:width-1]
        text1 = "To view build process:"[:width-1]
        text2 = "docker ps"[:width-1]
        text3 = "docker attach NAME"[:width-1]

        # Find center of window
        x = int((width // 2) - (len(text) // 2) - len(text) % 2)
        y = int((height // 2) - 2)

        self.stdscr.addstr(y, x, text)
        self.stdscr.addstr(y+1, x, text1)
        self.stdscr.addstr(y+2, x, text2)
        self.stdscr.addstr(y+3, x, text3)
        self.stdscr.refresh()

        # Assume dockerfile is in the same relative directory
        self.docker_client.images.build(path='./', tag='mil_image:latest')

    def open_ros_container(self, img):
        # Shouldn't happen, but if container was already set, ignore reset
        if not self.container is None:
            return

        # Similar to running 'docker run -it IMG'
        self.container = self.docker_client.containers.run(
            img, detach=True, tty=True, stdin_open=True, name="RustyROS", hostname=os.uname()[1]+"-ros")

    # Main draw application for curses
    def draw(self):
        k = 0
        cursor_x = 0
        cursor_y = 0

        # Clear and refresh the screen for a blank canvas
        self.stdscr.clear()
        self.stdscr.refresh()

        # Start colors in curses
        curses.start_color()
        curses.init_pair(1, curses.COLOR_CYAN, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
        curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_WHITE)

        # Loop until a key ('k') is pressed
        while (k == 0):
            # draw start menu
            self.draw_start_menu(k)
            self.draw_quit_bar()

            # Refresh the screen
            self.stdscr.refresh()

            # Wait for next input
            k = self.stdscr.getch()
            if k == ord('q'):
                return

        # Build image if user clicked b
        if k == ord('b'):
            self.brand_new_docker_image()

        self.stdscr.clear()

        # If no avaliable container, then ask user to select an image
        if self.container is None:
            selected_image = self.draw_images_menu()
            self.open_ros_container(selected_image)

        self.stdscr.clear()

        while (True):
            self.stdscr.clear()
            select = self.draw_options_menu()
            if select == 0:
                subprocess.Popen(
                    [self.terminal, '-e', 'docker exec -it RustyROS bash'])
            elif select == 1:
                self.container.stop()
                return
            elif select == 2:
                self.container.commit(repository='mil_image', tag='latest')
            elif select == 3:
                self.brand_new_docker_image()
            elif select == 4:
                return

    def draw_images_menu(self):
        l = [img.tags for img in self.docker_client.images.list()]
        selection = SelectionMenu.get_selection(l)
        return self.docker_client.images.list()[selection]

    def draw_options_menu(self):
        l = ['Open Terminal -- opens a terminal window attached to container', 'Stop -- stops the container', 'Save Image -- commits container to image',
             'Build Image -- runs docker build']
        selection = SelectionMenu.get_selection(l)
        return selection

    def draw_quit_bar(self):
        height, width = self.stdscr.getmaxyx()
        statusbarstr = "Press 'q' to exit "
        # Render status bar
        self.stdscr.attron(curses.color_pair(3))
        self.stdscr.addstr(height-1, 0, statusbarstr)
        self.stdscr.addstr(height-1, len(statusbarstr),
                           " " * (width - len(statusbarstr) - 1))
        self.stdscr.attroff(curses.color_pair(3))

    def draw_start_menu(self, k):
        # Initialization
        self.stdscr.clear()
        height, width = self.stdscr.getmaxyx()

        # Declaration of strings
        title = "ROS Docker Controller"[:width-1]
        subtitle = "Daniel Volya (RustyBamboo)"[:width-1]
        keystr = "Press any key to continue or b to build fresh MIL image"

        # Centering calculations
        start_x_title = int((width // 2) - (len(title) // 2) - len(title) % 2)
        start_x_subtitle = int(
            (width // 2) - (len(subtitle) // 2) - len(subtitle) % 2)
        start_x_keystr = int(
            (width // 2) - (len(keystr) // 2) - len(keystr) % 2)
        start_y = int((height // 2) - 2)

        # Turning on attributes for title
        self.stdscr.attron(curses.color_pair(2))
        self.stdscr.attron(curses.A_BOLD)

        # Rendering title
        self.stdscr.addstr(start_y, start_x_title, title)

        # Turning off attributes for title
        self.stdscr.attroff(curses.color_pair(2))
        self.stdscr.attroff(curses.A_BOLD)

        # Print rest of text
        self.stdscr.addstr(start_y + 1, start_x_subtitle, subtitle)
        self.stdscr.addstr(start_y + 3, (width // 2) - 2, '-' * 4)
        self.stdscr.addstr(start_y + 5, start_x_keystr, keystr)


if __name__ == "__main__":
    curses.wrapper(DockerController)

#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import math

sig = 0.5
f0 = np.array([-10, 5])
deg = -15
rad = (np.pi / 180 ) * deg
screen0 = np.array([np.cos(rad),np.sin(rad)])*3 + f0

sig = 0.5
f1 = np.array([-10,-5])
deg = 15
rad = (np.pi / 180 ) * deg
screen1 = np.array([np.cos(rad),np.sin(rad)])*3 + f1

def dist_x_to_v(x, f, screen):
    p3 = x
    p1 = f
    p2 = screen
    ret = np.abs(np.cross(p2-p1,p3-p1)/np.linalg.norm(p2-p1))
    return ret


def pdf(x, y, f, screen):
    x = np.array([x,y])
    sig_p = (sig / np.linalg.norm(screen-f)) * np.linalg.norm(x-f)
    a = (sig_p * np.sqrt(2* np.pi))
    b = np.exp(-0.5*(dist_x_to_v(x, f, screen)/sig_p)**2)
    return b/a * np.linalg.norm(x-f)


vpdf = np.vectorize(pdf)


x = np.linspace(-10, 10, 100)
y = np.linspace(-10, 100, 100)
Y,X = np.meshgrid(x,y)

poses0 = np.array([X.flatten(), Y.flatten(), [f0]*len(x)*len(y), [screen0]*len(x)*len(y)]).T
p0 = vpdf(poses0[:, 0], poses0[:, 1], poses0[:, 2], poses0[:, 3])

poses1 = np.array([X.flatten(), Y.flatten(), [f1]*len(x)*len(y), [screen1]*len(x)*len(y)]).T
p1 = vpdf(poses1[:, 0], poses1[:, 1], poses1[:, 2], poses1[:, 3])

p = np.multiply(p0, p1)

fig = plt.figure()
ax = fig.add_subplot(111)#, projection='3d')
ax.scatter(X, Y, c = p)
ax.scatter(f0[0], f0[1], c = 'red')
ax.scatter(f1[0], f1[1], c = 'red')
ax.scatter(screen0[0], screen0[1], c = 'red')
ax.scatter(screen1[0], screen1[1], c = 'red')


fig = plt.figure()
ax0 = fig.add_subplot(111)#, projection='3d')
ax0.scatter(X, Y, c = p0)
ax0.scatter(f0[0], f0[1], c = 'red')
ax0.scatter(screen0[0], screen0[1], c = 'red')

fig = plt.figure()
ax1 = fig.add_subplot(111)#, projection='3d')
ax1.scatter(X, Y, c = p1)
ax1.scatter(f1[0], f1[1], c = 'red')
ax1.scatter(screen1[0], screen1[1], c = 'red')


plt.show()


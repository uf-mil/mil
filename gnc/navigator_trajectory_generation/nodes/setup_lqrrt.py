"""
Class for storing setup info for an LQRRT node.
Here we do an example set-up for a boat.

"""

################################################# DEPENDENCIES

from __future__ import division
import numpy as np
import numpy.linalg as npl

################################################# PHYSICAL PARAMETERS

# Experimentally determined mass and inertia
m = 500  # kg
I = 400  # kg/m^2
invM = np.array([1/m, 1/m, 1/I])

# Experimentally determined top speeds and thrusts of the boat
velmax_pos = np.array([2.5, 1, 0.7])  # (m/s, m/s, rad/s), body-frame forward
velmax_neg = np.array([-0.8, -1, -0.7])  # (m/s, m/s, rad/s), body-frame backward
thrust_max = np.array([220, 220, 220, 220])  # N, per thruster

# Thruster layout, back-left, back-right, front-left front-right (m)
thruster_positions = np.array([[-1.9000,  1.0000, -0.0123],
							   [-1.9000, -1.0000, -0.0123],
							   [ 1.6000,  0.6000, -0.0123],
							   [ 1.6000, -0.6000, -0.0123]])
thruster_directions = np.array([[ 0.7071,  0.7071,  0.0000],
								[ 0.7071, -0.7071,  0.0000],
								[ 0.7071, -0.7071,  0.0000],
								[ 0.7071,  0.7071,  0.0000]])
thrust_levers = np.cross(thruster_positions, thruster_directions)
B = np.concatenate((thruster_directions.T, thrust_levers.T))[[0, 1, 5]]
invB = npl.pinv(B)

# Effective linear drag coefficients given thrust and speed limits
Fx_max = B.dot(thrust_max * [1, 1, 1, 1])[0]
Fy_max = B.dot(thrust_max * [1, -1, -1, 1])[1]
Mz_max = B.dot(thrust_max * [-1, 1, -1, 1])[2]
D_pos = np.abs([Fx_max, Fy_max, Mz_max] / velmax_pos)
D_neg = np.abs([Fx_max, Fy_max, Mz_max] / velmax_neg)

# Boat shape and resolution
boat_length = 210 * 0.0254  # m
boat_width = 96 * 0.0254  # m
boat_buffer = 0.25  # m
vps_spacing = .5  # m

# Grid of points defining boat
vps_grid_x, vps_grid_y = np.mgrid[slice(-(boat_length+boat_buffer)/2, (boat_length+boat_buffer)/2+vps_spacing, vps_spacing),
								  slice(-(boat_width+boat_buffer)/2, (boat_width+boat_buffer)/2+vps_spacing, vps_spacing)]
vps_grid_x = vps_grid_x.reshape(vps_grid_x.size)
vps_grid_y = vps_grid_y.reshape(vps_grid_y.size)
vps = np.zeros((vps_grid_x.size, 2))
for i in range(len(vps)):
	vps[i] = [vps_grid_x[i], vps_grid_y[i]]
vps = vps.T

################################################# DYNAMICS

nstates = 6
ncontrols = 3
magic_rudder = 1000

def dynamics(x, u, dt):
	"""
	Returns next state given last state x, wrench u, and timestep dt.
	Simple holonomic boat-like dynamics, with some "force of nature"
	keeping the boat looking along its velocity line.

	"""
	# Rotation matrix (orientation, converts body to world)
	R = np.array([
				  [np.cos(x[2]), -np.sin(x[2]), 0],
				  [np.sin(x[2]),  np.cos(x[2]), 0],
				  [           0,             0, 1]
				])

	# Construct drag coefficients based on our motion signs
	D = np.zeros(3)
	for i, v in enumerate(x[3:]):
		if v >= 0:
			D[i] = D_pos[i]
		else:
			D[i] = D_neg[i]

	# Heading controller trying to keep us car-like
	# if x[3] > 0:
	# 	vw = R[:2, :2].dot(x[3:5])
	# 	ang = np.arctan2(vw[1], vw[0])
	# 	c = np.cos(x[2])
	# 	s = np.sin(x[2])
	# 	cg = np.cos(ang)
	# 	sg = np.sin(ang)
	# 	u[2] = u[2] + magic_rudder*np.arctan2(sg*c - cg*s, cg*c + sg*s)

	# Actuator saturation
	u = B.dot(np.clip(invB.dot(u), -thrust_max, thrust_max))

	# M*vdot + D*v = u  and  pdot = R*v
	xdot = np.concatenate((R.dot(x[3:]), invM*(u - D*x[3:])))

	# First-order integrate
	xnext = x + xdot*dt

	# Impose not turning in place
	if x[3] > 0:
		xnext[5] = np.clip(np.abs(xnext[3]/velmax_pos[0]), 0, 1) * xnext[5]
	elif x[3] < 0:
		xnext[5] = np.clip(np.abs(xnext[3]/velmax_neg[0]), 0, 1) * xnext[5]

	# Impose not driving backwards
	#if xnext[3] < 0:
	#		xnext[3] = 0

	return xnext

################################################# CONTROL POLICY

# Body-frame gains
kp = np.diag([120, 50, 0])
kd = np.diag([120, 50, 0])

def lqr(x, u):
	"""
	Returns cost-to-go matrix S and policy matrix K given local state x and effort u.

	"""
	R = np.array([
				  [np.cos(x[2]), -np.sin(x[2]), 0],
				  [np.sin(x[2]),  np.cos(x[2]), 0],
				  [           0,             0, 1]
				])
	# if x[3] < 0:
	# 	backup_factor = 2
	# else:
	# 	backup_factor = 1
	S = np.diag([1, 1, 1, 1, 1, 1])
	K = np.hstack((kp.dot(R.T), kd))
	return (S, K)

def erf(xgoal, x):
	"""
	Returns error e given two states xgoal and x.
	Angle differences are taken properly on SO3.

	"""
	e = xgoal - x
	c = np.cos(x[2])
	s = np.sin(x[2])
	cg = np.cos(xgoal[2])
	sg = np.sin(xgoal[2])
	e[2] = np.arctan2(sg*c - cg*s, cg*c + sg*s)
	return e

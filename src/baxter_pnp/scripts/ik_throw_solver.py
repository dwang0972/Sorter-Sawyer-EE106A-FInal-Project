#!/usr/bin/env python

import sys

import math
import scipy.optimize as opt
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


g = 9.81

m = 0.1

y3 = 1
z3 = -0.1
alpha = math.atan2(z3, y3)

L4 = 0.4
L6 = 0.13375
Lg = 0.07

w3 = 1.957
w5 = 3.485

class IkThrowSolver:
	def __init__(self, x3, z3, x_target, z_target):
		self.x_target = x_target
		self.z_target = z_target

		self.x3 = x3
		self.z3 = z3
		self.alpha = math.atan2(x3, z3)


	def x5(self, theta3):
		return self.x3 + L4*math.cos(-theta3 + alpha)

	def z5(self, theta3):
		return self.z3 + L4*math.sin(-theta3 + alpha)


	def x_start(self, theta3, theta5):
		return self.x5(theta3) + (L6+Lg)*math.cos(-theta5-theta3+alpha)

	def z_start(self, theta3, theta5):
		return self.z5(theta3) + (L6+Lg)*math.sin(-theta5-theta3+alpha)

	def xdot_start(self, theta3, theta5):
		return +L4*w3*math.sin(-theta3+alpha) + (L6+Lg)*(w3+w5)*math.sin(-theta5-theta3+alpha)

	def zdot_start(self, theta3, theta5):
		return -L4*w3*math.cos(-theta3+alpha) - (L6+Lg)*(w3+w5)*math.cos(-theta5-theta3+alpha)

	def x_eq(self, var):
		theta3, theta5, t = var
		return self.x_start(theta3, theta5) + self.xdot_start(theta3, theta5) * t

	def z_eq(self, var):
		theta3, theta5, t = var
		return self.z_start(theta3, theta5) + self.zdot_start(theta3, theta5) * t - m*g*t*t/2



	def distance(self, var):
		return math.sqrt(math.pow(self.x_eq(var)-self.x_target, 2) + math.pow(self.z_eq(var)-self.z_target, 2))


	def solve(self):
		return opt.minimize(self.distance, (1,1,1), 
			constraints=({'type': 'ineq', 'fun': lambda x:  x[2]}))


	def plot(self, solutions):
		theta3s, theta5s, ts = solutions
		t = np.linspace(0, 3, 200)
		var = [theta3s, theta5s, t]

		plt.plot(self.x_eq(var), self.z_eq(var))

		xx = self.x_eq(var)
		zz = self.z_eq(var)

		#fig = plt.figure()
		#line,  = plt.plot([], [], 'r-')


		#ani = animation.FuncAnimation(fig, lambda num, x, y, line: line.set_data(x[:num], y[:num]), len(xx), fargs=[xx, zz, line], interval=25, blit=True)
		#ani.save('test.gif')

		plt.plot(self.x_target, self.z_target, marker='o')
		plt.text(self.x_target, self.z_target+0.05, 'Target')

		circle4 = plt.Circle((self.x3, self.z3), L4, fill=False)
		plt.gcf().gca().add_artist(circle4)

		plt.plot(self.x3, self.z3, marker='o')
		plt.text(self.x3 + 0.05, self.z3, 'Joint 3')
		
		circle6 = plt.Circle((self.x5(theta3s), self.z5(theta3s)), L6+Lg, fill=False)
		plt.gcf().gca().add_artist(circle6)

		plt.plot(self.x5(theta3s), self.z5(theta3s), marker='o')
		plt.text(self.x5(theta3s) - 0.15, self.z5(theta3s), 'Joint 5')

		plt.plot([self.x3, self.x5(theta3s)], [self.z3, self.z5(theta3s)], label='L4')
		plt.text(self.x3 - 0.05, self.z3 + 0.1, 'L4')

		plt.plot([self.x5(theta3s), self.x_start(theta3s, theta5s)], [self.z5(theta3s), self.z_start(theta3s, theta5s)], label='L6')
		plt.text(self.x5(theta3s) + 0.05, self.z5(theta3s) + 0.1, 'L6')

		plt.xlim(-1, 5)
		plt.ylim(-1, 1)
		plt.gca().set_aspect('equal')
		plt.draw()
		plt.show()


def main():
	solver = IkThrowSolver(0, 0, 10, -0.5)
	solutions = solver.solve()
	print(solutions.x)
	theta3, theta5, t = solutions.x

	solver.plot([theta3, theta5, t])


if __name__ == "__main__":
	sys.exit(main())
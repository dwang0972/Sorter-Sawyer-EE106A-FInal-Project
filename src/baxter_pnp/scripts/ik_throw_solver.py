#!/usr/bin/env python

import sys

import math
import scipy.optimize as opt
import numpy as np
import matplotlib.pyplot as plt


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
	def __init__(self, y3, z3, y_target, z_target):
		self.y_target = y_target
		self.z_target = z_target

		self.y3 = y3
		self.z3 = z3
		self.alpha = math.atan2(y3, z3)


	def y5(self, theta3):
		return self.y3 - L4*math.cos(theta3 + alpha)

	def z5(self, theta3):
		return self.z3 + L4*math.sin(theta3 + alpha)


	def y_start(self, theta3, theta5):
		return self.y5(theta3) - (L6+Lg)*math.cos(theta5+theta3+alpha)

	def z_start(self, theta3, theta5):
		return self.z5(theta3) + (L6+Lg)*math.sin(theta5+theta3+alpha)

	def ydot_start(self, theta3, theta5):
		return L4*w3*math.sin(theta3+alpha) + (L6+Lg)*(w3+w5)*math.sin(theta5+theta3+alpha)

	def zdot_start(self, theta3, theta5):
		return L4*w3*math.cos(theta3+alpha) + (L6+Lg)*(w3+w5)*math.cos(theta5+theta3+alpha)

	def y_eq(self, var):
		theta3, theta5, t = var
		return self.y_start(theta3, theta5) + self.ydot_start(theta3, theta5) * t

	def z_eq(self, var):
		theta3, theta5, t = var
		return self.z_start(theta3, theta5) + self.zdot_start(theta3, theta5) * t - m*g*t*t/2



	def distance(self, var):
		return math.sqrt(math.pow(self.y_eq(var)-self.y_target, 2) + math.pow(self.z_eq(var)-self.z_target, 2))


	def solve(self):
		return opt.minimize(self.distance, (1,1,1))


	def plot(self, solutions):
		theta3s, theta5s, ts = solutions
		t = np.linspace(0, 3, 200)
		var = [theta3s, theta5s, t]

		plt.plot(self.y_eq(var), self.z_eq(var))
		
		plt.plot(self.y_target, self.z_target, marker='o')
		plt.text(self.y_target, self.z_target+0.05, 'Target')

		circle4 = plt.Circle((self.y3, self.z3), L4, fill=False)
		plt.gcf().gca().add_artist(circle4)

		plt.plot(self.y3, self.z3, marker='o')
		plt.text(self.y3 + 0.05, self.z3, 'Joint 3')
		
		circle6 = plt.Circle((self.y5(theta3s), self.z5(theta3s)), L6+Lg, fill=False)
		plt.gcf().gca().add_artist(circle6)

		plt.plot(self.y5(theta3s), self.z5(theta3s), marker='o')
		plt.text(self.y5(theta3s) - 0.15, self.z5(theta3s), 'Joint 5')

		plt.plot([self.y3, self.y5(theta3s)], [self.z3, self.z5(theta3s)], label='L4')
		plt.text(self.y3 - 0.05, self.z3 + 0.1, 'L4')

		plt.plot([self.y5(theta3s), self.y_start(theta3s, theta5s)], [self.z5(theta3s), self.z_start(theta3s, theta5s)], label='L6')
		plt.text(self.y5(theta3s) + 0.05, self.z5(theta3s) + 0.1, 'L6')

		plt.xlim(-1, 3)
		plt.ylim(-1, 1)
		plt.gca().set_aspect('equal')
		plt.draw()
		plt.show()


def main():
	solver = IkThrowSolver(1, 0.1, 3, -0.2)
	solutions = solver.solve()
	solver.plot(solutions.x)


if __name__ == "__main__":
	sys.exit(main())
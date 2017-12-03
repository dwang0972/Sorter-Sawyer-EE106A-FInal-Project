#!/usr/bin/env python

import sys

import math
import scipy.optimize as opt
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


g = 9.81

m = 0.05

y3 = 1
z3 = -0.1
alpha = math.atan2(z3, y3)

L4 = 0.4
L6 = 0.13375
Lg = 0.07

w3 = 1.957
w5 = 3.485

class IkThrowSolver:
    def __init__(self, x3, z3, alpha, x_target, z_target):
        self.x_target = x_target
        self.z_target = z_target

        self.x3 = x3
        self.z3 = z3
        self.alpha = alpha
        print("Alpha: {0}".format(self.alpha))


    def x5(self, theta3):
        return self.x3 + L4*math.cos(theta3 - alpha)

    def z5(self, theta3):
        return self.z3 + L4*math.sin(theta3 - alpha)


    def x_start(self, theta3, theta5):
        return self.x5(theta3) + (L6+Lg)*math.cos(-theta5+theta3-alpha)

    def z_start(self, theta3, theta5):
        return self.z5(theta3) + (L6+Lg)*math.sin(-theta5+theta3-alpha)

    def xdot_start(self, theta3, theta5):
        return -L4*w3*math.sin(theta3+alpha) + (L6+Lg)*(-w3+w5)*math.sin(-theta5+theta3-alpha)

    def zdot_start(self, theta3, theta5):
        return +L4*w3*math.cos(theta3+alpha) - (L6+Lg)*(-w3+w5)*math.cos(-theta5+theta3-alpha)

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
        	bounds=((-math.pi, math.pi), (-math.pi/2, math.pi), (0, 999999)),
            constraints=({'type': 'ineq', 'fun': lambda x:  x[2]}))






    def get_start_times(self, theta3_start, theta5_start, theta3_goal, theta5_goal):
        exec3, exec5 = self.get_execution_times(theta3_start, theta5_start, theta3_goal, theta5_goal)

        if exec3 > exec5:
            return 0, exec3-exec5
        else:
            return exec5-exec3, 0

    def get_execution_times(self, theta3_start, theta5_start, theta3_goal, theta5_goal):
        exec3 = abs(theta3_goal - theta3_start) / w3
        exec5 = abs(theta5_goal - theta5_start) / w5

        return exec3, exec5


    def get_total_execution_time(self, theta3_start, theta5_start, theta3_goal, theta5_goal):
        return max(self.get_execution_times(theta3_start, theta5_start, theta3_goal, theta5_goal))



    def plot(self, solutions):
        theta3s, theta5s, ts = solutions
        t = np.linspace(0, 4, 200)
        t_exec = np.linspace(0, 2, 200)
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
        plt.ylim(-1, 2)
        plt.gca().set_aspect('equal')

        plt.xlabel('X (m)')
        plt.ylabel('Z (m)')
        plt.title('Trajectory of the object when released by the arm')

        plt.draw()

        plt.figure()

        start3, start5 = self.get_start_times(-math.pi, -math.pi, theta3s, theta5s)

        t_total = self.get_total_execution_time(-math.pi, -math.pi, theta3s, theta5s)

        theta3 = [0] * len(t_exec)
        theta5 = [0] * len(t_exec)
        for i, tt in enumerate(t_exec):
            if tt < start3:
                theta3[i] = -math.pi
            else:
                theta3[i] = min(-math.pi + w3*(tt-start3), theta3s)
            if tt < start5:
                theta5[i] = -math.pi
            else:
                theta5[i] = min(-math.pi + w5*(tt-start5), theta5s)

        line3, = plt.plot(t_exec, theta3, label="Theta 3")
        line5, = plt.plot(t_exec, theta5, label="Theta 5")

        plt.axvline(t_total, color="black")
        plt.text(t_total+0.1, -1.5, "Release", color="black")

        plt.legend(handles=[line3, line5])
        plt.xlabel('Time (s)')
        plt.ylabel('Angle (rad)')
        plt.title('Evolution of the joint angles with time')

        plt.show()



def main():
    solver = IkThrowSolver(0, 0, 4, -0.5)
    solutions = solver.solve()
    print(solutions.x)
    theta3, theta5, t = solutions.x

    solver.plot([theta3, theta5, t])


if __name__ == "__main__":
    sys.exit(main())
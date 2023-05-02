#!/usr/bin/env python3

# read text files and plot them

import matplotlib.pyplot as plt
import numpy as np
import sys
import math

# data files to read
file1 = "../data_files/q1.txt"
file2a = "../data_files/q2-a.txt"
file2c = "../data_files/q2-c.txt"
file2d = "../data_files/q2-d.txt"
file3 = "../data_files/q3.txt"
file4i = "../data_files/q4-i.txt"
file4ii = "../data_files/q4-ii.txt"
file4iii = "../data_files/q4-iii.txt"
file4iv = "../data_files/q4-iv.txt"

q7_des = 0.1 # for Q1
x_des = (0.3, 0.1, 0.5) # desired end-effector position, for Q2 onwards

traj1 = np.loadtxt(file1, skiprows=0)[:, 7]
traj2a_q = np.loadtxt(file2a, skiprows=0)[:, 1:8]
traj2a_x = np.loadtxt(file2a, skiprows=0)[:, 8:]
traj2c_q = np.loadtxt(file2c, skiprows=0)[:, 1:8]
traj2c_x = np.loadtxt(file2c, skiprows=0)[:, 8:]
traj2d_q = np.loadtxt(file2d, skiprows=0)[:, 1:8]
traj2d_x = np.loadtxt(file2d, skiprows=0)[:, 8:]
traj3_q = np.loadtxt(file3, skiprows=0)[:, 1:8]
traj3_x = np.loadtxt(file3, skiprows=0)[:, 8:]
traj4i_q = np.loadtxt(file4i, skiprows=0)[:, 1:8]
traj4i_x = np.loadtxt(file4i, skiprows=0)[:, 8:]
traj4ii_q = np.loadtxt(file4ii, skiprows=0)[:, 1:8]
traj4ii_x = np.loadtxt(file4ii, skiprows=0)[:, 8:]
traj4iii_q = np.loadtxt(file4iii, skiprows=0)[:, 1:8]
traj4iii_x = np.loadtxt(file4iii, skiprows=0)[:, 8:]
traj4iv_q = np.loadtxt(file4iv, skiprows=0)[:, 1:8]
traj4iv_x = np.loadtxt(file4iv, skiprows=0)[:, 8:]

time1 = np.loadtxt(file1, skiprows=0)[:, 0]
time2a = np.loadtxt(file2a, skiprows=0)[:, 0]
time2c = np.loadtxt(file2c, skiprows=0)[:, 0]
time2d = np.loadtxt(file2d, skiprows=0)[:, 0]
time3 = np.loadtxt(file3, skiprows=0)[:, 0]
time4i = np.loadtxt(file4i, skiprows=0)[:, 0]
time4ii = np.loadtxt(file4ii, skiprows=0)[:, 0]
time4iii = np.loadtxt(file4iii, skiprows=0)[:, 0]
time4iv = np.loadtxt(file4iv, skiprows=0)[:, 0]

# desired circular trajectory for Q4
xd4i = 0.3 + 0.1*np.sin(np.pi*time4i)
yd4i = 0.1 + 0.1*np.cos(np.pi*time4i)
xd4ii = 0.3 + 0.1*np.sin(np.pi*time4ii)
yd4ii = 0.1 + 0.1*np.cos(np.pi*time4ii)
xd4iii = 0.3 + 0.1*np.sin(np.pi*time4iii)
yd4iii = 0.1 + 0.1*np.cos(np.pi*time4iii)
xd4iv = 0.3 + 0.1*np.sin(np.pi*time4iv)
yd4iv = 0.1 + 0.1*np.cos(np.pi*time4iv)

# plotting
############# Question 1 #############

plt.figure(1)
plt.plot(time1, traj1, 'b')
plt.axhline(y=q7_des, color='b', linestyle='--')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('$q_7$ (rad) $\\rightarrow$')
plt.legend(['$q_7$', '$q_{7d}$'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.title('Plot of $q_7$ versus time')
plt.grid()
plt.savefig("Figure1.png", bbox_inches="tight")
plt.show()

############# Question 2 #############

plt.figure(2)
for i in range(traj2a_q.shape[1]):
	plt.plot(time2a, traj2a_q[:, i])
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Joint trajectories $q$ (rad) $\\rightarrow$')
plt.legend(['$q_1$', '$q_2$', '$q_3$', '$q_4$', '$q_5$', '$q_6$', '$q_7$'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.xlim(0, 3)
plt.title('Plot of joint trajectories versus time')
plt.grid()
plt.savefig("Figure2.png", bbox_inches="tight")
plt.show()

plt.figure(3)
for i in range(traj2a_x.shape[1]):
	plt.plot(time2a, traj2a_x[:, i])
plt.gca().set_prop_cycle(None)
for i in range(traj2a_x.shape[1]):
	plt.plot(time2a, np.full_like(time2a, x_des[i]), '--')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point trajectory (m) $\\rightarrow$')
plt.legend(['$x$', '$y$', '$z$', '$x_d$', '$y_d$', '$z_d$'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.xlim(0, 3)
plt.title('Plot of operational point trajectory versus time')
plt.grid()
plt.savefig("Figure3.png", bbox_inches="tight")
plt.show()

plt.figure(4)
for i in range(traj2a_q.shape[1]):
	plt.plot(time2a, traj2a_q[:, i], '--')
plt.gca().set_prop_cycle(None)
for i in range(traj2c_q.shape[1]):
	plt.plot(time2c, traj2c_q[:, i])	
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Joint trajectories $q$ (rad) $\\rightarrow$')
plt.legend(['$q_1$ from part (a)', '$q_2$ from part (a)', '$q_3$ from part (a)', '$q_4$ from part (a)', '$q_5$ from part (a)', 
	'$q_6$ from part (a)', '$q_7$ from part (a)', '$q_1$ from part (c)', '$q_2$ from part (c)', '$q_3$ from part (c)', 
	'$q_4$ from part (c)', '$q_5$ from part (c)', '$q_6$ from part (c)', '$q_7$ from part (c)'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.xlim(0, 3)
plt.title('Plot of joint trajectories versus time')
plt.grid()
plt.savefig("Figure4.png", bbox_inches="tight")
plt.show()

plt.figure(5)
for i in range(traj2a_x.shape[1]):
	plt.plot(time2a, traj2a_x[:, i], '--')
plt.gca().set_prop_cycle(None)
for i in range(traj2c_x.shape[1]):
	plt.plot(time2c, traj2c_x[:, i])
plt.gca().set_prop_cycle(None)
for i in range(traj2c_x.shape[1]):
	plt.plot(time2c, np.full_like(time2c, x_des[i]), ':')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point trajectory (m) $\\rightarrow$')
plt.legend(['$x$ from part (a)', '$y$ from part (a)', '$z$ from part (a)', '$x$ from part (c)', '$y$ from part (c)', 
	'$z$ from part (c)', '$x_d$', '$y_d$', '$z_d$'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.xlim(0, 3)
plt.title('Plot of operational point trajectory versus time')
plt.grid()
plt.savefig("Figure5.png", bbox_inches="tight")
plt.show()

plt.figure(6)
for i in range(traj2a_q.shape[1]):
	plt.plot(time2a, traj2a_q[:, i], '--')
plt.gca().set_prop_cycle(None)
for i in range(traj2d_q.shape[1]):
	plt.plot(time2d, traj2d_q[:, i])	
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Joint trajectories $q$ (rad) $\\rightarrow$')
plt.legend(['$q_1$ from part (a)', '$q_2$ from part (a)', '$q_3$ from part (a)', '$q_4$ from part (a)', '$q_5$ from part (a)', 
	'$q_6$ from part (a)', '$q_7$ from part (a)', '$q_1$ from part (d)', '$q_2$ from part (d)', '$q_3$ from part (d)', 
	'$q_4$ from part (d)', '$q_5$ from part (d)', '$q_6$ from part (d)', '$q_7$ from part (d)'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.xlim(0, 3)
plt.title('Plot of joint trajectories versus time')
plt.grid()
plt.savefig("Figure6.png", bbox_inches="tight")
plt.show()

plt.figure(7)
for i in range(traj2a_x.shape[1]):
	plt.plot(time2a, traj2a_x[:, i], '--')
plt.gca().set_prop_cycle(None)
for i in range(traj2d_x.shape[1]):
	plt.plot(time2d, traj2d_x[:, i])
plt.gca().set_prop_cycle(None)
for i in range(traj2d_x.shape[1]):
	plt.plot(time2d, np.full_like(time2d, x_des[i]), ':')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point trajectory (m) $\\rightarrow$')
plt.legend(['$x$ from part (a)', '$y$ from part (a)', '$z$ from part (a)', '$x$ from part (d)', '$y$ from part (d)', 
	'$z$ from part (d)', '$x_d$', '$y_d$', '$z_d$'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.xlim(0, 3)
plt.title('Plot of operational point trajectory versus time')
plt.grid()
plt.savefig("Figure7.png", bbox_inches="tight")
plt.show()

plt.figure(8)
for i in range(traj2c_q.shape[1]):
	plt.plot(time2c, traj2c_q[:, i], '--')
plt.gca().set_prop_cycle(None)
for i in range(traj2d_q.shape[1]):
	plt.plot(time2d, traj2d_q[:, i])	
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Joint trajectories $q$ (rad) $\\rightarrow$')
plt.legend(['$q_1$ from part (c)', '$q_2$ from part (c)', '$q_3$ from part (c)', '$q_4$ from part (c)', '$q_5$ from part (c)', 
	'$q_6$ from part (c)', '$q_7$ from part (c)', '$q_1$ from part (d)', '$q_2$ from part (d)', '$q_3$ from part (d)', 
	'$q_4$ from part (d)', '$q_5$ from part (d)', '$q_6$ from part (d)', '$q_7$ from part (d)'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.xlim(0, 3)
plt.title('Plot of joint trajectories versus time')
plt.grid()
plt.savefig("Figure8.png", bbox_inches="tight")
plt.show()

plt.figure(9)
for i in range(traj2c_x.shape[1]):
	plt.plot(time2c, traj2c_x[:, i], '--')
plt.gca().set_prop_cycle(None)
for i in range(traj2d_x.shape[1]):
	plt.plot(time2d, traj2d_x[:, i])
plt.gca().set_prop_cycle(None)
for i in range(traj2d_x.shape[1]):
	plt.plot(time2d, np.full_like(time2d, x_des[i]), ':')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point trajectory (m) $\\rightarrow$')
plt.legend(['$x$ from part (c)', '$y$ from part (c)', '$z$ from part (c)', '$x$ from part (d)', '$y$ from part (d)', 
	'$z$ from part (d)', '$x_d$', '$y_d$', '$z_d$'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.xlim(0, 3)
plt.title('Plot of operational point trajectory versus time')
plt.grid()
plt.savefig("Figure9.png", bbox_inches="tight")
plt.show()

############# Question 3 #############

plt.figure(10)
for i in range(traj3_q.shape[1]):
	plt.plot(time3, traj3_q[:, i])
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Joint trajectories $q$ (rad) $\\rightarrow$')
plt.legend(['$q_1$', '$q_2$', '$q_3$', '$q_4$', '$q_5$', '$q_6$', '$q_7$'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.title('Plot of joint trajectories versus time')
plt.grid()
plt.savefig("Figure10.png", bbox_inches="tight")
plt.show()

plt.figure(11)
for i in range(traj3_x.shape[1]):
	plt.plot(time3, traj3_x[:, i])
plt.gca().set_prop_cycle(None)
for i in range(traj3_x.shape[1]):
	plt.plot(time3, np.full_like(time3, x_des[i]), '--')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point trajectory (m) $\\rightarrow$')
plt.legend(['$x$', '$y$', '$z$', '$x_d$', '$y_d$', '$z_d$'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.title('Plot of operational point trajectory versus time')
plt.grid()
plt.savefig("Figure11.png", bbox_inches="tight")
plt.show()

############# Question 4 #############

plt.figure(12)
for i in range(traj4i_q.shape[1]):
	plt.plot(time4i, traj4i_q[:, i])
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Joint trajectories $q$ (rad) $\\rightarrow$')
plt.legend(['$q_1$', '$q_2$', '$q_3$', '$q_4$', '$q_5$', '$q_6$', '$q_7$'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.title('Plot of joint trajectories versus time')
plt.grid()
plt.savefig("Figure12.png", bbox_inches="tight")
plt.show()

plt.figure(13)
for i in range(traj4i_x.shape[1]):
	plt.plot(time4i, traj4i_x[:, i])
plt.gca().set_prop_cycle(None)
plt.plot(time4i, xd4i, '--')
plt.plot(time4i, yd4i, '--')
plt.plot(time4i, np.full_like(time4i, 0.5), '--')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point trajectory (m) $\\rightarrow$')
plt.legend(['$x$', '$y$', '$z$', '$x_d$', '$y_d$', '$z_d$'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.title('Plot of operational point trajectory versus time')
plt.grid()
plt.savefig("Figure13.png", bbox_inches="tight")
plt.show()

plt.figure(14)
plt.plot(traj4i_x[:, 0], traj4i_x[:, 1])
plt.xlabel('x (m) $\\rightarrow$')
plt.ylabel('y (m) $\\rightarrow$')
plt.title('Plot of y versus x')
plt.axis('equal')
plt.grid()
plt.savefig("Figure14.png", bbox_inches="tight")
plt.show()

############################################################

plt.figure(15)
for i in range(traj4ii_q.shape[1]):
	plt.plot(time4ii, traj4ii_q[:, i])
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Joint trajectories $q$ (rad) $\\rightarrow$')
plt.legend(['$q_1$', '$q_2$', '$q_3$', '$q_4$', '$q_5$', '$q_6$', '$q_7$'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.title('Plot of joint trajectories versus time')
plt.grid()
plt.savefig("Figure15.png", bbox_inches="tight")
plt.show()

plt.figure(16)
for i in range(traj4ii_x.shape[1]):
	plt.plot(time4ii, traj4ii_x[:, i])
plt.gca().set_prop_cycle(None)
plt.plot(time4ii, xd4ii, '--')
plt.plot(time4ii, yd4ii, '--')
plt.plot(time4ii, np.full_like(time4ii, 0.5), '--')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point trajectory (m) $\\rightarrow$')
plt.legend(['$x$', '$y$', '$z$', '$x_d$', '$y_d$', '$z_d$'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.title('Plot of operational point trajectory versus time')
plt.grid()
plt.savefig("Figure16.png", bbox_inches="tight")
plt.show()

plt.figure(17)
plt.plot(traj4ii_x[:, 0], traj4ii_x[:, 1])
plt.xlabel('x (m) $\\rightarrow$')
plt.ylabel('y (m) $\\rightarrow$')
plt.title('Plot of y versus x')
plt.axis('equal')
plt.grid()
plt.savefig("Figure17.png", bbox_inches="tight")
plt.show()

############################################################

plt.figure(18)
for i in range(traj4iii_q.shape[1]):
	plt.plot(time4iii, traj4iii_q[:, i])
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Joint trajectories $q$ (rad) $\\rightarrow$')
plt.legend(['$q_1$', '$q_2$', '$q_3$', '$q_4$', '$q_5$', '$q_6$', '$q_7$'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.title('Plot of joint trajectories versus time')
plt.grid()
plt.savefig("Figure18.png", bbox_inches="tight")
plt.show()

plt.figure(19)
for i in range(traj4iii_x.shape[1]):
	plt.plot(time4iii, traj4iii_x[:, i])
plt.gca().set_prop_cycle(None)
plt.plot(time4iii, xd4iii, '--')
plt.plot(time4iii, yd4iii, '--')
plt.plot(time4iii, np.full_like(time4iii, 0.5), '--')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point trajectory (m) $\\rightarrow$')
plt.legend(['$x$', '$y$', '$z$', '$x_d$', '$y_d$', '$z_d$'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.title('Plot of operational point trajectory versus time')
plt.grid()
plt.savefig("Figure19.png", bbox_inches="tight")
plt.show()

plt.figure(20)
plt.plot(traj4iii_x[:, 0], traj4iii_x[:, 1])
plt.xlabel('x (m) $\\rightarrow$')
plt.ylabel('y (m) $\\rightarrow$')
plt.title('Plot of y versus x')
plt.axis('equal')
plt.grid()
plt.savefig("Figure20.png", bbox_inches="tight")
plt.show()

############################################################

plt.figure(21)
for i in range(traj4iv_q.shape[1]):
	plt.plot(time4iv, traj4iv_q[:, i])
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Joint trajectories $q$ (rad) $\\rightarrow$')
plt.legend(['$q_1$', '$q_2$', '$q_3$', '$q_4$', '$q_5$', '$q_6$', '$q_7$'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.title('Plot of joint trajectories versus time')
plt.grid()
plt.savefig("Figure21.png", bbox_inches="tight")
plt.show()

plt.figure(22)
for i in range(traj4iv_x.shape[1]):
	plt.plot(time4iv, traj4iv_x[:, i])
plt.gca().set_prop_cycle(None)
plt.plot(time4iv, xd4iv, '--')
plt.plot(time4iv, yd4iv, '--')
plt.plot(time4iv, np.full_like(time4iv, 0.5), '--')
plt.xlabel('time (sec) $\\rightarrow$')
plt.ylabel('Operational point trajectory (m) $\\rightarrow$')
plt.legend(['$x$', '$y$', '$z$', '$x_d$', '$y_d$', '$z_d$'], bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.title('Plot of operational point trajectory versus time')
plt.grid()
plt.savefig("Figure22.png", bbox_inches="tight")
plt.show()

plt.figure(23)
plt.plot(traj4iv_x[:, 0], traj4iv_x[:, 1])
plt.xlabel('x (m) $\\rightarrow$')
plt.ylabel('y (m) $\\rightarrow$')
plt.title('Plot of y versus x')
plt.axis('equal')
plt.grid()
plt.savefig("Figure23.png", bbox_inches="tight")
plt.show()
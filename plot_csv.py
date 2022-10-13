import numpy as np
import matplotlib.pyplot as plt
import csv

from camera_stand_math import *

alpha = np.arange(0, 2 * np.pi, 0.1)

distance = 0.4
radius = 0.1

calc_arm_extension = []
calc_arm_speed = []
calc_beta = []
calc_beta_speed1 = []
calc_beta_speed2 = []
calc_beta_speed3 = []

for a in alpha:
    this_calc_arm_extension = arm_extension(a, distance, radius)
    calc_arm_extension.append(this_calc_arm_extension)
    calc_arm_speed.append(slider_speed(a, distance, radius))
    calc_beta.append(beta(a, distance, radius, this_calc_arm_extension))
    calc_beta_speed1.append(angular_speed(a, distance, radius, this_calc_arm_extension))
    calc_beta_speed2.append(angular_speed_simple(a, distance, radius))
    # calc_beta_speed3.append(angular_speed_real(a, distance, radius))

fig = plt.figure()

ax1 = fig.add_subplot(311, polar=True)
ax1.plot(calc_beta, calc_arm_extension)

ax1.set_thetamin(-20)
ax1.set_thetamax(20)
ax1.set_ylim(0, 0.6)

ax2 = fig.add_subplot(312)

ax2.plot(alpha, calc_arm_extension, color='g', label="Arm extension")
ax2.plot(alpha, calc_beta, color='b', label="Arm angle")

ax2.set_xlabel('Alpha')
ax2.set_title('Position', fontsize=20)
ax2.grid()
ax2.legend()

ax3 = fig.add_subplot(313)

ax3.plot(alpha, calc_arm_speed, color='g', label="Arm extension speed")
ax3.plot(alpha, calc_beta_speed1, color='b', label="Arm angle speed 1")
ax3.plot(alpha, calc_beta_speed2, color='r', label="Arm angle speed 2")
# ax3.plot(alpha, calc_beta_speed3, color='b', label="Arm angle speed 3")

ax3.set_xlabel('Alpha')
ax3.set_title('Speed', fontsize=20)
ax3.grid()
ax3.legend()

# alpha = []
# arm_calc = []
# arm_act = []
# beta = []
# speed = []

# with open('flat_circle_run_export.csv', 'r') as csvfile:
#     lines = csv.DictReader(csvfile, delimiter=',')
#     for row in lines:
#         alpha.append(float(row['Alpha']))
#         arm_calc.append(float(row['Calculated arm position']))
#         arm_act.append(float(row['Actual arm position']))
#         beta.append(float(row['Beta']))
#         speed.append(float(row['Speed']))


# plt.plot(alpha, arm_calc, color='g', label="Arm calculated")
# plt.plot(alpha, arm_act, color='b', label="Arm actual")
# plt.plot(alpha, beta, color='r', label="Beta")
# plt.plot(alpha, speed, color='black', label="Angular speed")

# ax1.xticks(rotation=25)

plt.show()

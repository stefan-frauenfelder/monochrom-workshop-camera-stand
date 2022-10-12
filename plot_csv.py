import matplotlib.pyplot as plt
import csv

alpha = []
arm_calc = []
arm_act = []
beta = []
speed = []

with open('flat_circle_run_export.csv', 'r') as csvfile:
    lines = csv.DictReader(csvfile, delimiter=',')
    for row in lines:
        alpha.append(float(row['Alpha']))
        arm_calc.append(float(row['Calculated arm position']))
        arm_act.append(float(row['Actual arm position']))
        beta.append(float(row['Beta']))
        speed.append(float(row['Speed']))


plt.plot(alpha, arm_calc, color='g', label="Arm calculated")
plt.plot(alpha, arm_act, color='b', label="Arm actual")
plt.plot(alpha, beta, color='r', label="Beta")
plt.plot(alpha, speed, color='black', label="Angular speed")

plt.xticks(rotation=25)
plt.xlabel('Alpha')
plt.ylabel('Arm position')
plt.title('Movement plot', fontsize=20)
plt.grid()
plt.legend()
plt.show()

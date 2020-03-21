import math
import matplotlib.pyplot as plt

# Read sensor input
f = "../data/obj_pose-laser-radar-synthetic-input.txt"
data = open(f, "r")
radar_x_values = []
radar_y_values = []
ladar_x_values = []
ladar_y_values = []
filter_y_values = []
filter_x_values = []

if data.mode == 'r':
    contents = data.readlines()
    for line in contents:
        values = line.split("\t")
        if(values[0] == 'L'):
            x = float(values[1])
            y = float(values[2])
            ladar_x_values.append(x)
            ladar_y_values.append(y)
        else:
            rho = float(values[1])
            phi = float(values[2])
            rho_dot = float(values[3])
            x = rho * math.cos(phi)
            y = rho * math.sin(phi)
            radar_x_values.append(x)
            radar_y_values.append(y)

# Read filter output
data2 = open('../data/filter_output.txt', "r")
if data2.mode == 'r':
    contents = data2.readlines()
    for line in contents:
        values = line.split(" ")
        x = float(values[0])
        y = float(values[1])
        filter_x_values.append(x)
        filter_y_values.append(y)

# display results
fig = plt.figure()
ax=fig.add_axes([0,0,1,1])
ax.scatter(radar_x_values, radar_y_values, color='r', s=2)
ax.scatter(ladar_x_values, ladar_y_values, color='b', s=2)
ax.scatter(filter_x_values, filter_y_values, color='g', s=2)
plt.show()
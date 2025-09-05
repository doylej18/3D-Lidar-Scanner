import serial
import open3d as o3d
import numpy as np
import time
import math
 

num_layers = 3
measurments = 32                # coordinate sets per scan
delta_z = 200                     # vertical spacing between layers (mm or any unit)
z = 0

ser = serial.Serial('COM3', 115200, timeout=5) 
print("Opening:", ser.name)
 
#ser.write(b'r')  # tell MCU to start scan
 
ser.reset_input_buffer()
ser.reset_output_buffer()
 
points = []
 
print("Waiting for PJ0 button press for layer...")
 
while True:
    msg = ser.readline().decode().strip()
    if msg == "x":
        print("Button press detected, starting scan...")
        break
 
#for every layer in the scan
for layer in range(num_layers):
 
    coords = [] #cords holds all coordinates for each layer
    scan_points = 0 #variable holds the current amount of scanned points 
    i = 0
    r = 0
 
    while scan_points < measurments: #scans all points from 0 to 32
        line = ser.readline().decode().strip() #strips the message sent through UART

        if line != 'x': #x is the char sent through UART to signal that button press has occured
            print("Received:", line) #prints recieved, distance value
        parts = line.split(',')
        if len(parts) == 2: #ensures no garbage data is used in the code
            curr_measurments = int(parts[0]) #current measurment is the i value from c code
            r = float(parts[1]) #r is the distance value sent from mc

            #angle calculation , where curr_measurment is the iteration variable from 0-32
            angle_rad = (curr_measurments * 11.25) * (math.pi / 180.0)


            x = r*math.cos(angle_rad) #calculates x coord
            y = r*math.sin(angle_rad) #calculates y coord
            z = layer*delta_z #calculates z coord

            coords.append((x, y, z)) #appends all coords to list called coords
            scan_points += 1
 
    # wait for SCAN_DONE from MCU
    while True:
        done_msg = ser.readline().decode().strip() 
        if done_msg == "SCAN_DONE": #signal that total scan is done
            print(f"Layer {layer} scan complete")
            break
 
    points.extend(coords) #append list of coordinates to list called points which holds coords for all layers
 
 
ser.close() #close serial communication 
 
# save as .xyz file
with open("full_scan.xyz", "w") as f:
    for x, y, z in points:
        f.write(f"{x:.2f} {y:.2f} {z:.2f}\n") #write to .xyz file
 
# create point cloud and line connections
points_np = np.array(points)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points_np)
 
lines = []
 
# connections in the xy plane
for layer in range(num_layers):
    base = layer * measurments
    for i in range(measurments - 1):
        lines.append([base + i, base + i + 1])
    lines.append([base + measurments - 1, base])  # close the loop
 
# connections between layers
for step in range(measurments):
    for layer in range(num_layers - 1):
        idx1 = layer * measurments + step
        idx2 = (layer + 1) * measurments + step
        lines.append([idx1, idx2])
 
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points_np)
line_set.lines = o3d.utility.Vector2iVector(lines)
line_set.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in lines])
 
# displays 3D graph
o3d.visualization.draw_geometries([pcd, line_set])
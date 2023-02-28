
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

plot_roadmap = True
write_to_maps = True

turnarounds = 1
nodes_per_turnaround = 20
phi_intervals = 2*np.pi / nodes_per_turnaround
radius_circle = 60

spline_length = turnarounds * np.pi*2

distance_on_spline = np.arange(0, spline_length, phi_intervals)

x_coordinates = np.sin(distance_on_spline)*radius_circle
y_coordinates = np.cos(distance_on_spline)*radius_circle-radius_circle
phi_coordinates = -distance_on_spline - np.pi/2

file_content = """\
<?xml version="1.0" encoding="UTF-8"?>
<!-- 1.5708 = pi / 2 -->
<!-- 
uint8 LANECENTER_FREEWAY=1
uint8 LANECENTER_SURFACESTREET=2
uint8 LANECENTER_BIKELANE=3
uint8 ROADLINE_BROKENSINGLEWHITE=6
uint8 ROADLINE_SOLIDSINGLEWHITE=7
uint8 ROADLINE_SOLIDDOUBLEWHITE=8
uint8 ROADLINE_BROKENSINGLEYELLOW=9
uint8 ROADLINE_BROKENDOUBLEYELLOW=10
uint8 ROADLINE_SOLIDSINGLEYELLOW=11
uint8 ROADLINE_SOLIDDOUBLEYELLOW=12
uint8 ROADLINE_PASSINGDOUBLEYELLOW=13
uint8 ROADEDGEBOUNDARY=15
uint8 ROADEDGEMEDIAN=16
uint8 STOPSIGN=17
uint8 CROSSWALK=18
uint8 SPEEDBUMP=19 
-->
<way>\
"""

if plot_roadmap:
    roadmap_plot = plt.plot(x_coordinates,y_coordinates, 'b')

for index, (x, y, theta) in enumerate(zip(x_coordinates,y_coordinates,phi_coordinates)):
    if plot_roadmap:
        if index == 0:
            roadmap_plot = plt.plot(x, y, 'g', marker=(2, 0, theta*180/np.pi), markersize=10, linestyle='None')
            roadmap_plot = plt.plot(x, y, 'g', marker=(3, 0, theta*180/np.pi), markersize=10, linestyle='None')
        elif index == len(phi_coordinates)-1: 
            roadmap_plot = plt.plot(x, y, 'r', marker=(2, 0, theta*180/np.pi), markersize=10, linestyle='None')
            roadmap_plot = plt.plot(x, y, 'r', marker=(3, 0, theta*180/np.pi), markersize=10, linestyle='None')
        else:
            roadmap_plot = plt.plot(x, y, 'b', marker=(2, 0, theta*180/np.pi), markersize=10, linestyle='None')
            roadmap_plot = plt.plot(x, y, 'b', marker=(3, 0, theta*180/np.pi), markersize=10, linestyle='None')
        # print(theta)

    file_content += f'\n  <nd x="{x}" y="{y}" theta="{theta+np.pi/2}"/>'

file_content += """
  <lane type="road" width="4.0" two_way="0"/>
</way>
"""

if write_to_maps:
    file_name = f"simple_circle_radius_{str(radius_circle)}"
    path = Path(__file__).parent
    file = open(f"{path}/../maps/{file_name}.xml", "w")
    file.write(file_content)
    file.close()

if plot_roadmap:
    plt.show()
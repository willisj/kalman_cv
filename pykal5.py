#!/usr/local/bin/python
# coding: latin-1
'''
# <codecell>    pykal5.py    Kalman Filter for GPS trail (path trace) smoothing
#
#    Ian Douglas   forked 2015-August updated through 2016-August
'''

from pykalman import KalmanFilter
import matplotlib.pyplot as plt
import numpy as np
import random
from mpl_toolkits.mplot3d import Axes3D    
from haversine import haversine
from pyproj import Proj
from os import mkdir

'''
#     Haversine computes great circle distance from latitude and longitude angles
#              = 1/2 * versine = (1 − cos(θ))/2 or sin**2(θ/2)
'''
def do_hav(source_lat,source_lng,dest_lat,dest_lng):
    return haversine((source_lat,source_lng),(dest_lat,dest_lng),miles = True)
#   END do_hav'

'''
#     Convert [latitude, longitude, altitude] to UTM cartesian values
#            GPS uses WGS84 standard 
'''
def gps_to_cartesian(coordinates,inverted=False):
    p = Proj(proj='utm',zone=17,ellps='WGS84')
    converted = []
    for point in coordinates:
        lat = point[0] 
        lng = point[1] 
        alt = point[2] 
        
        if inverted:
            x,y = p(lng,lat,inverse=True)    
        else:
            x,y = p(lng,lat)
        
        converted.append((y,x,alt))

    return converted
#   END gps_to_cartesian

def calculate_point_variance(measurements, corrected,absolute=True):
    if len(measurements) != len(corrected):
            print "* 50 * calculate_point_variance: Lists must be same size", len(measurements), len(corrected)
            exit(1)
        
    variance = []    
    for i in range(len(measurements)):
        if absolute:
            variance.append((abs(measurements[i][0] - corrected[i][0]),
                abs(measurements[i][1] - corrected[i][1]),
                abs(measurements[i][2] - corrected[i][2])))
        else:
            variance.append((measurements[i][0] - corrected[i][0],
                measurements[i][1] - corrected[i][1],
                measurements[i][2] - corrected[i][2]))
    return variance
#   END calculate_point_variance

def calculate_offsets(measurements, cartesian=False):
    offsets = []
    for i in range(len(measurements)):
        if i == 0:
            continue
        offsets.append(
            ( measurements[i][0] - measurements[i-1][0],
            measurements[i][1] - measurements[i-1][1],
            measurements[i][2] - measurements[i-1][2] ))
    return offsets
#   END calculate_offsets
    
def positions_from_offsets_additive(ref,offsets):
    
    positions = []
    
    current_pos = list(ref)
    positions.append(current_pos)
    
    for offset in offsets:
        for i in range(3):
            current_pos[i] += offset[i]
        positions.append(tuple(current_pos))

    return positions
#   END positions_from_offsets_additive
    
def positions_from_offsets_corrective(refs,offsets):
    
    if not (len(refs) == len(offsets)):
        print "ERROR 96: positions from offsets not matching in length. %d  vs.  %d" % (len(refs), len(offsets))
        exit(1)
    
    positions = []
    
    for j in range(len(refs)):
        current_pos = list(refs[j])
        for i in range(3):
            current_pos[i] += offsets[j][i]
        positions.append(tuple(current_pos))

    return positions
#   END positions_from_offsets_corrective

'''    
# measurements must be in the format 
# [ (x, y, z), ... ]
'''
def run_kal(measurements, training_size=150):
    ' count the number of measurements                                      '
    num_measurements  = len(measurements)
    ' find the dimension of each row                                        '
    dim =  len(measurements[0])

    if dim == 3:
        simple_mode = True
    elif dim == 9:
        simple_mode = False
    else:
        print "Error: Dimensions for run_kal must be 3 or 9"
        exit(1)
    print("run_kal 127 - run_kal smooth --> simple_mode={0}".format(simple_mode))
    
    training_set = []
    for i in range(min(training_size,len(measurements))):
        training_set.append(measurements[i])

    if simple_mode:
        ' run the filter                                                    '
        kf = KalmanFilter(initial_state_mean=[0,0,0], n_dim_obs=3)
        (smoothed_state_means, smoothed_state_covariances) = kf.em(training_set).smooth(measurements)
    else:
        kf = KalmanFilter(initial_state_mean=[0,0,0,0,0,0,0,0,0], n_dim_obs=9)
        (smoothed_state_means, smoothed_state_covariances) = kf.em(training_set).smooth(measurements)
        
    ' means represent corrected points                                        '
    return smoothed_state_means, smoothed_state_covariances, simple_mode
#   END run_kal

'''
#     Process input [latitude, longitude, altitude] vehicle path 
'''
def load_gps_trail(extra_fields = False):
    i = 0
    measurements = []
    header=True
    print("load_gps_trail 152 - Failure here means the input gps file did not have correct Unix LF endings...")
    for line in open('nazeer-2735-3485-xyz-vel-accel.csv'):        #'nazeer_trip.csv'):
        if header:
            header=False
            continue
        i += 1
        fields = line.strip().split(',')

        if i % 6 == 0:     # Display every sixth entry, plus first and 100th
            print i,fields
            continue
        elif i == 1:
            print i,fields, '\n Length of line = ', len(fields), " extra_fields = ", extra_fields
            continue
        elif i == 100:
            print i,fields, "\n extra_fields = ", extra_fields #, measurements
            continue

#       print "Past the 6 - 1 and 100 clause", i,fields
        px = float(fields[0])
        py = float(fields[1])
        pz = float(fields[2])

        if len(fields) == 3:
            extra_fields = False
            measurements.append((px, py, pz))
            continue
        else:
            extra_fields = True
            px_vel = float(fields[3])
            py_vel = float(fields[4])
            pz_vel = float(fields[5])
            px_accel = float(fields[6])
            py_accel = float(fields[7])
            pz_accel = float(fields[8])
            measurements.append((px, py, pz, px_vel, py_vel, pz_vel, px_accel, py_accel, pz_accel))
            continue
                
    return measurements
#   END load_gps_trail 

'''
#     Plot [latitude, longitude, altitude] 
#    (need to flip lat & long to get x & y looking right on the output
'''
def plot_first_dim(data,flip_xy=True,xlbl="",ylbl="",title="",filename=None):
    x_axis = []
    y_axis = []
    
    if flip_xy:
        minx = maxx = data[0][1]
        miny = maxy = data[0][0]
    else:
        minx = maxx = data[0][0]
        miny = maxy = data[0][1]
    
    print "plot_first_dim 208 - length data = ", len(data)
    
    for x,y,z, xdot, ydot, zdot, xddot, yddot, zddot in data:
        
        if flip_xy:
            t = x
            x = y
            y = t
        
        x_axis.append(x)
        y_axis.append(y)
        
        minx = min(x,minx)
        miny = min(y,miny)
        
        maxx = max(x,maxx)
        maxy = max(y,maxy)        

    plt.xlim((minx,maxx))
    plt.ylim((miny,maxy))
    plt.xlabel(xlbl)
    plt.ylabel(ylbl)
    plt.title(title)
    plt.scatter(x_axis,y_axis)

    if filename:
        plt.savefig( filename)
    else:    
        plt.show()

    plt.close()
#   END plot_first_dim

def plot_all_dimensions(positions,file_prefix):
    split = [list(), list(), list()]

    j = 0 
    for position in positions:
        for i in range(3):
            split[i].append((j,position[i],0))
        j+=1

    x_vals = split[0]
    y_vals = split[1]
    z_vals = split[2]

    plot_first_dim(x_vals,flip_xy=False,xlbl="Time (s)",ylbl="WGS84 Latitude",title="WGS84 LATITUDE OVER TIME",filename=output_dir +file_prefix + "all_dim_x")
    plot_first_dim(y_vals,flip_xy=False,xlbl="Time (s)",ylbl="WGS84 Longitude",title="WGS84 LONGITUDE OVER TIME",filename=output_dir +file_prefix + "all_dim_y")
    plot_first_dim(z_vals,flip_xy=False,xlbl="Time (s)",ylbl="Altitude ",title="Altitude OVER TIME",filename=output_dir +file_prefix + "all_dim_z")
#   END plot_all_dimensions    

'''    
## MAIN Program here
'''
output_dir = "output5/"

try:
    mkdir(output_dir)
except OSError, e:
    pass

#EXTRA
parse_extra_fields = True

# Load the path in from the file 
measurements = load_gps_trail(extra_fields = parse_extra_fields )
print "Main 274 - measurements ", measurements, "\n273 parse_extra_fields = ", parse_extra_fields

# Convert these measurements to meters
cart_measurements = gps_to_cartesian(measurements)
print "Main 278 - cart_measurements ", cart_measurements

#############################
##  Positions              ##
#############################

# Run the Kalman filter on the measurements
print "Main 285 - Run KF - measurements", measurements
ss_means_meas, ss_covariance_meas,cart_simple = run_kal(measurements,training_size=150)
filtered_measurements = ss_means_meas        # Give this a more meaningful name
print "Main 288 - Run KF - filtered_measurements " , len(filtered_measurements)
f = open(output_dir + 'skf_trail_filtered.csv', 'w')
f.write('lat,lng,alt,xdot,ydot,zdot,xddot,yddot,zddot\n')
for lat,lng,alt,xdot,ydot,zdot,xddot,yddot,zddot in filtered_measurements:
   f.write("{0},{1},{2},{3},{4},{5},{6},{7},{8}\n".format(lat,lng,alt,xdot,ydot,zdot,xddot,yddot,zddot))
f.close()
print "Main 294"
#############################
##  OFFSETS                ##
#############################

# Find the offsets between points
offsets = calculate_offsets(measurements)

# Run the Kalman filter on the offsets
ss_means, ss_covariance,offset_simple = run_kal(offsets)
corrected_offsets = ss_means    # Give this a more meaningful name

# Reconstruct the cartesian positions from the offsets and a reference point
#corrected_cart_positions = positions_from_offsets_additive(cart_measurements[0],corrected_offsets)
corrected_positions = positions_from_offsets_corrective(measurements[1:len(measurements)], corrected_offsets)

# Convert the positions back to wgs84
corrected_gps_positions = gps_to_cartesian(corrected_positions,inverted=True)

### OFFSET Results ##
print "Main 314 - plot_first_dim - measurements[:3] "
plot_first_dim(
    measurements[:3],
    title = "Measurements in WGS84 - Lat/Long/alt",
    xlbl = 'X Position (latitude degrees)',
    ylbl = 'Y Position (longitude degrees)',
    filename = output_dir + "measurements.png" )

print "Main 322 - plot_first_dim - corrected_positions "
plot_first_dim(
    corrected_positions,
    title = "Corrected GPS Points from Offsets ", 
    xlbl = 'X Position (latitude degrees)',
    ylbl = 'Y Position (longitude degrees)',
    filename = output_dir + "corrected_fromoff.png")
#    filename = output_dir + "filtered_fromoff.png")

print "Main 331 - plot_first_dim - filtered_measurements "
plot_first_dim(
    filtered_measurements,
    title = "Corrected WGS84 Points from Positions", 
    xlbl = 'X Position (latitude degrees)',
    ylbl = 'Y Position (longitude degrees)',
    filename = output_dir + "filtered_frompos.png")

print "Main 339 - length corrected_positions = ", len(corrected_positions), " length measurements = ", len(measurements)
plot_first_dim(corrected_positions,"fromFilteredOffset-")
plot_first_dim(measurements,"fromMeasurements-")

# Split the variance into it's three components and draw it

# Calculate the variance between the two offset lists
offset_variance = calculate_point_variance(measurements[1:], corrected_positions)

offset_variance_split = [[],[],[]]
for x,y,z in offset_variance:
    offset_variance_split[0].append(x)
    offset_variance_split[1].append(y)
    offset_variance_split[2].append(z)

plt.plot(offset_variance_split[0])
plt.plot(offset_variance_split[1])
plt.plot(offset_variance_split[2])
plt.xlabel('Measurement ID')
plt.title('Variance Between Measurement and Filtered (Offsets)')
plt.ylabel('Measurement offset_variance')
#plt.show()
plt.savefig(output_dir + "offset_variance.png")

# calculate variance for positions
position_variance = calculate_point_variance(measurements, filtered_measurements)
position_variance_split = [[],[],[]]
for x,y,z in offset_variance:
    position_variance_split[0].append(x)
    position_variance_split[1].append(y)
    position_variance_split[2].append(z)

plt.plot(position_variance_split[0])
plt.plot(position_variance_split[1])
plt.plot(position_variance_split[2])
plt.xlabel('Measurement ID')
plt.title('Variance Between Measurement and Filtered (Positions)')
plt.ylabel('Measurement offset_variance')
#plt.show()
plt.savefig(output_dir + "point_variance.png")

# output corrected file

f = open(output_dir + 'gps_trail_filtered.csv', 'w')
f.write('lat,lng,alt\n')
i = 0
for x,y,z in corrected_gps_positions:
   i += 1
   f.write("{0}, {1}, {2}\n".format(x,y,z))
   if i % 6 == 0:     # Display every sixth entry, plus first and 100th
      print i,x,y,z
      continue
   elif i == 1:
       print i,x,y,z, '\n Length of line = ', len(corrected_gps_positions)
       continue
   elif i == 100:
       print i,x,y,z #, "\n extra_fields = ", extra_fields #, measurements
       continue

f.close()

'''
##    GPS Trail input and standard Kalman filter smoothing...
##
##  End <pykal5.py>
'''
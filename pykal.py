from pykalman import KalmanFilter
import matplotlib.pyplot as plt
import numpy as np
import random
from mpl_toolkits.mplot3d import Axes3D	
from haversine import haversine
from pyproj import Proj
from os import mkdir

##
# 	TODO
#	Parse gaurav trip	
#   distribute speed over x/y dimensions
#	( x2' - x1' ) / time
#   modify do_kal + other functitons to handle 3x3 or 9x9 arrays
##

def do_hav(source_lat,source_lng,dest_lat,dest_lng):
	return haversine((source_lat,source_lng),(dest_lat,dest_lng),miles = True)

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

def calculate_point_variance(measurements, corrected,absolute=True):
	if len(measurements) != len(corrected):
			print "calculate_point_variance: Lists must be same size", len(measurements), len(corrected)
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
	
def positions_from_offsets_additive(ref,offsets):
	
	positions = []
	
	current_pos = list(ref)
	positions.append(current_pos)
	
	for offset in offsets:
		for i in range(3):
			current_pos[i] += offset[i]
		positions.append(tuple(current_pos))

	return positions
	
def positions_from_offsets_corrective(refs,offsets):
	
	if not (len(refs) == len(offsets)):
		print "ERROR: positions from offsets not maching in length. %d  vs.  %d" % (len(refs), len(offsets))
		exit(1)
	
	positions = []
	
	for j in range(len(refs)):
		current_pos = list(refs[j])
		for i in range(3):
			current_pos[i] += offsets[j][i]
		positions.append(tuple(current_pos))

	return positions
	
# measurements must be in the format 
# [ (x, y, z), ... ]
def run_kal(measurements, training_size=40):

	
	# count the number of measurements	
	num_measurements  = len(measurements)
	
	# find the dimension of each row
	dim =  len(measurements[0])
	if dim == 3:
		simple_mode = True
	elif dim == 9:
		trans_matrix = [
			[1, 0, 0, 1, 0, 0, 0.5, 0, 0 ],
			[0, 1, 0, 0, 1, 0, 0, 0.5, 0 ],
			[0, 0, 1, 0, 0, 1, 0, 0, 0.5 ],
			[0, 0, 0, 1, 0, 0, 1, 0, 0 ],
			[0, 0, 0, 0, 1, 0, 0, 1, 0 ],
			[0, 0, 0, 0, 0, 1, 0, 0, 1 ],
			[0, 0, 0, 0, 0, 0, 1, 0, 0 ],
			[0, 0, 0, 0, 0, 0, 0, 1, 0 ],
			[0, 0, 0, 0, 0, 0, 0, 0, 1 ] ];
		simple_mode = False
	else:
		print "Error: Dimensions for run_kal must be 3 or 9"
		exit(1)
	
	training_set = []
	for i in range(min(training_size,len(measurements))):
		training_set.append(measurements[i])

	if simple_mode:
		# run the filter
		kf = KalmanFilter(initial_state_mean=[0,0,0], n_dim_obs=3)
		(smoothed_state_means, smoothed_state_covariances) = kf.em(training_set).smooth(measurements)
	else:
		# kf = KalmanFilter(initial_state_mean=[0,0,0,0,0,0,0,0,0], n_dim_obs=9)
		kf = KalmanFilter(initial_state_mean=[0,0,0,0,0,0,0,0,0], transition_matrix=trans_matrix, n_dim_obs=9)
		(smoothed_state_means, smoothed_state_covariances) = kf.em(training_set).smooth(measurements)
		
	# means represent corrected points
	return smoothed_state_means, smoothed_state_covariances, simple_mode

def load_gps_trail(extra_fields = False):
	i = 0
	measurements = []
	header=True
	for line in open('nazeer_trip.csv'):
		if header:
			header=False
			continue
		i += 1
		fields = line.strip().split(',')
		print i,fields
		px = float(fields[0])
		py = float(fields[1])
		pz = float(fields[2])
		
		if extra_fields:
			px_vel = float(fields[0])
			py_vel = float(fields[0])
 			pz_vel = float(fields[0])
			px_accel = float(fields[0])
			py_accel = float(fields[0])
			pz_accel = float(fields[0])
			measurements.append((px, py, pz, px_vel, py_vel, pz_vel, px_accel, py_accel, pz_accel))
		else:
			measurements.append((px, py, pz))
			
	
	return measurements

def plot_first_dim(data,flip_xy=True,xlbl="",ylbl="",title="",filename=None):
	x_axis = []
	y_axis = []
	
	if flip_xy:
		minx = maxx = data[0][1]
		miny = maxy = data[0][0]
	else:
		minx = maxx = data[0][0]
		miny = maxy = data[0][1]
	
	for x,y,z in data:
		
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

	plot_first_dim(x_vals,flip_xy=False,xlbl="Time (s)",ylbl="UTM Latitude",title="UTM LATITUDE OVER TIME",filename=output_dir +file_prefix + "all_dim_x")
	plot_first_dim(y_vals,flip_xy=False,xlbl="Time (s)",ylbl="UTM Longiute",title="UTM LATITUDE OVER TIME",filename=output_dir +file_prefix + "all_dim_y")
	plot_first_dim(z_vals,flip_xy=False,xlbl="Time (s)",ylbl="Altitude ",title="Altitude OVER TIME",filename=output_dir +file_prefix + "all_dim_z")
	
	
	
	
## MAIN HERE

output_dir = "output/"

try:
	mkdir(output_dir)
except OSError, e:
	pass

#EXTRA
parse_extra_fields = True

# Load the path in from the file 
measurements = load_gps_trail(extra_fields = parse_extra_fields )

# Convert these measurements to meters
cart_measurements = gps_to_cartesian(measurements)

#############################
##  Positions              ##
#############################

# Run the Kalman filter on the measurements
ss_means_meas, ss_covariance_meas,cart_simple = run_kal(cart_measurements,training_size=60)
filtered_cart_measurements = ss_means_meas		# Give this a more meaningful name

#############################
##  OFFSETS                ##
#############################

# Find the offsets between points
offsets = calculate_offsets(cart_measurements)

# Run the Kalman filter on the offsets
ss_means, ss_covariance,offset_simple = run_kal(offsets)
corrected_offsets = ss_means	# Give this a more meaningful name

# Reconstruct the cartesian positions from the offsets and a reference point
#corrected_cart_positions = positions_from_offsets_additive(cart_measurements[0],corrected_offsets)
corrected_cart_positions = positions_from_offsets_corrective(cart_measurements[1:len(cart_measurements)], corrected_offsets)

# Convert the positions back to wgs84
corrected_gps_positions = gps_to_cartesian(corrected_cart_positions,inverted=True)

### OFFSET Results ##
plot_first_dim(
	cart_measurements,
	title = "Measurements in Cartesian",
	xlbl = 'X Position (meters)',
	ylbl = 'Y Position (meters)',
	filename = output_dir + "measurements_cart.png" )
	
plot_first_dim(
	corrected_cart_positions,
	title = "Corrected Cartesian Points from Offsets ", 
	xlbl = 'X Position (meters)',
	ylbl = 'Y Position (meters)',
	filename = output_dir + "filtered_cart_fromoff.png")
	
plot_first_dim(
	filtered_cart_measurements,
	title = "Corrected Cartesian Points from Positions", 
	xlbl = 'X Position (meters)',
	ylbl = 'Y Position (meters)',
	filename = output_dir + "filtered_cart_frompos.png")

plot_all_dimensions(corrected_cart_positions,"fromFilteredOffset-")
plot_all_dimensions(cart_measurements,"fromMeasurements-")


# Split the variance into it's three components and draw it

# Calculate the variance between the two offset lists
offset_variance = calculate_point_variance(cart_measurements[1:], corrected_cart_positions)

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
position_variance = calculate_point_variance(cart_measurements, filtered_cart_measurements)
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
for x,y,z in corrected_gps_positions:
	f.write("{0}, {1}, {2}\n".format(x,y,z))
	
f.close()
	
	





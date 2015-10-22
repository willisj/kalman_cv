#!/usr/bin/env python
# -*- coding: utf-8 -*-
import re
import os
import sys
import time

## CONFIG

START_TIMESTAMP = 55120				# 0 shows all
CAN_OUTPUT_MODE	= 'DEMO'			# Valid Options: HUD, CSV, DEMO

REALTIME_DISPLAY = True
WAIT_BEFORE_PRINTING = 0.10 	# seconds
sleep_factor = 1.0 # 1 realtime

ALL_CAN_FIELDS = ['ESP_Lamp_Req', 'PrkBrake_Indicator', 'AccelPedalPosition', 'ABS_Lamp_Req', 'BrkSw1Stat', 'Odometer', 'TSC_SUPP', 'PARK_BRK_EGD', 'BATT_VOLT', 'BrakeSwitch', 'FullBrk_Actv1', 'BRK_ACTIVE', 'FrontWiperInUse', 'ESP_TrqRqEnabl', 'ESP_PD_EN', 'ESP_AVL', 'BRK_ACT_LCM', 'AmbientTemp', 'BrkSw2Stat', 'LowBeam', 'ETC_LmpOn', 'TSC_EN', 'FullBrk_Actv', 'ABS_PRSNT', 'VEH_SPEED', 'ESP_BrakeSwitch', 'BrkEnbl_LCM', 'HighBeams', 'ETC_LmpFlash', 'YawRate', 'SteeringWheel_Angle', 'REF_VEH_SPEED', 'ESP_PD_SUPP', 'ESP_PRSNT', 'BarometricPressure', 'ABS_BrkEvt', 'LAT_ACCEL', 'EngineSpeed', 'ESP_Disabled']
CAN_FIELDS = {'VEH_SPEED':'Vel','EngineSpeed':'RPM','SteeringWheel_Angle':'Steering','BrakeSwitch':'BRK','AccelPedalPosition':'Gas'}
TRACK_ALL_FIELDS = False

## OUTPUT FILES

OUTPUT_DIR = "out"
TSV_FILE_ = "gps.tsv"
CSV_FILE_ = "gps.csv"
VIS_FILE_ = "gps_visualizer.dat"

############## 

# Directory / File Prep

try:
    os.stat(OUTPUT_DIR)
except:
    os.mkdir(OUTPUT_DIR)

VIS_FILE = open(OUTPUT_DIR + "/" + VIS_FILE_,'w')
TSV_FILE = open(OUTPUT_DIR + "/" + TSV_FILE_,'w')
CSV_FILE = open(OUTPUT_DIR + "/" + CSV_FILE_,'w')

TSV_FILE.write('%s\t%s\t%s\t%s\t%s\t%s\t%s\n' % ('nice_timestamp','timestamp','lat','lon','speed(m/s)','angle','alt'))
CSV_FILE.write('%s, %s, %s, %s, %s, %s, %s\n' % ('nice_timestamp','timestamp','lat','lon','speed(m/s)','angle','alt'))

## Field Prep

if TRACK_ALL_FIELDS or CAN_OUTPUT_MODE == 'DEMO':
	for f in ALL_CAN_FIELDS:
		if f not in CAN_FIELDS:
			CAN_FIELDS[f] = f

CAN_VAL = {}
for key in CAN_FIELDS:
	CAN_VAL[key] = 0.0
	
enumerated_can_fields = set()

## REGEX ##

timestamp_re = re.compile(r'\d\d(:\d\d)+')
rmc_pos = re.compile(r'(\d+\.?\d*),A,(\d+\.\d+),(N|S),(\d+\.\d+),(W|E),(\d+.\d+),(\d+.\d+)?')
#gga_pos = re.compile(r'(\d+\.\d+),(\d+\.\d+),[NA],(\d+\.\d+),[NW]')
gga_pos = re.compile(r'GPGGA,(\d+\.\d+),(\d+\.\d+),([NS]),(\d+\.\d+),([EW]),(\d+),(\d+),(\d+\.\d+),(\d+\.\d+),(\w)')
can_line = re.compile(r'^(\d+):(\d+):(\d+):(\d+)\s+([^:]+):\s*(.*)$')


def dms_tp_dd(dms,direction):
	point = dms.index('.')
	
	d = float(dms[:point-2]) 
	m = float(dms[point-2:point])
	s = float(dms[point:]) * 60.0
	dd = d + (m/60.0) + (s/3600.0)
	
	if direction == 'W' or direction == 'S':
		dd *= -1
		
	return dd
	

## ##### ## 
# CLASSES
class can_point:
	def __init__(self):
		self.nice_timestamp = 1
		self.parameter = 'unset'
		self.value = None
		self.hour = 0
		self.minute = 0
		self.second = 0
		self.msecond = 0
		
	def timestamp_seconds(self):
		time = (self.hour * 60.0 * 60.0) + (self.minute * 60.0) + self.second + (self.msecond / 1000.0)
		return time

class gps_pos:
	def __init__(self):
		self.nice_timestamp = 1
		self.timestamp=-1
		self.mode = 'unset'
		self.lat=-1
		self.lat_d='N'
		self.lon=-1
		self.lon_d='W'
		self.speed=-1
		self.angle=0
	
	def parse_GPRMC(self,line):
		pos = re.search(rmc_pos,line)
		print line
		if pos:
			self.mode = 'RMC'
			#self.nice_timestamp=re.search(timestamp_re,line).group(0)
			self.pos = re.search(rmc_pos,line)

			full_ts = pos.group(1)

			self.timestamp = str((int(full_ts[0:2]) * 60 * 60) + (int(full_ts[2:4]) * 60)+ int(full_ts[4:6]))
			
			self.lat = pos.group(2)
			self.lat_d = pos.group(3)
			self.lon = pos.group(4)
			self.lon_d = pos.group(5)
			self.speed = pos.group(6)
			self.angle = pos.group(7)
			return True
		else:
			return False

	
	## Correct POS
	# puts position into WGS94
	# changes speed from knots to m/s
	def correct_pos(self):
		self.lat = dms_tp_dd(self.lat,self.lat_d)
		self.lon = dms_tp_dd(self.lon,self.lon_d)
		
		self.speed = float(self.speed)*0.514444
		

## ##### ## 


def process_GPS_line(line):
	point = gps_pos()	

	if 'GPRMC' in line:		
		if not point.parse_GPRMC(line):
			return
	
	#DISABLED
	elif 'GPGGA' in line:
		mode = 'GGA'
		pos = re.search(gga_pos,line)
		# timestamp = pos.group(1)
		# lat = pos.group(2)
		# lon = pos.group(3)	
		if pos >= 10:
			process_GPS_line.alt = pos.group(9)
	
	if not point.mode == "unset":
		
		# Convert units
		point.correct_pos()

		if process_GPS_line.last_print_sec < point.timestamp and float(point.timestamp) > START_TIMESTAMP: 
			print point.timestamp, process_GPS_line.last_print_sec
			process_GPS_line.last_print_sec = point.timestamp
			VIS_FILE.write('%s,1,%f,%f,%f\n' % (point.timestamp,point.lat,point.lon,point.speed))
			TSV_FILE.write( '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' % (point.nice_timestamp,point.timestamp,point.lat,point.lon,point.speed,point.angle,process_GPS_line.alt))
			CSV_FILE.write( '%s, %s, %s, %s, %s, %s, %s\n' % (point.nice_timestamp,point.timestamp,point.lat,point.lon,point.speed,point.angle,process_GPS_line.alt))
			
process_GPS_line.alt = 0
process_GPS_line.last_print_sec = 0

def print_CAN_header():
	if CAN_OUTPUT_MODE == 'DEMO':
		return
	elif CAN_OUTPUT_MODE == 'HUD':
		sep='\t'
	else:
		sep = ', '
	
	print 'timestamp',
	for k,v in CAN_FIELDS.items():
		sys.stdout.write( sep + k)
		
	print ''


def print_CAN_line(timestamp):
	if CAN_OUTPUT_MODE == 'HUD':
		print timestamp,
		for k,v in CAN_FIELDS.items():
			sys.stdout.write('\t ' + v + ': '+ str(CAN_VAL[k]))
		print ''
		
	elif CAN_OUTPUT_MODE == 'CSV':
		print timestamp,
		for k,v in CAN_FIELDS.items():
			sys.stdout.write(', ' + str(CAN_VAL[k]))
		print ''
		
	elif CAN_OUTPUT_MODE == 'DEMO':
		
		## Construct PEDAL STRING
		pedal_string = ('BRK ' if (CAN_VAL['BrakeSwitch'] > 0.0) else '    ') + ('GAS ' if float(CAN_VAL['AccelPedalPosition']) > 0.0 else '   ')
		#print timestamp + '\tPedal:',pedal_string,
		
		## Calculate ACCELERATION
		accel = print_CAN_line.last_speed - CAN_VAL['VEH_SPEED']
		dir_char = ' '
		if accel > 0:
			dir_char = 'v'
		elif accel < 0:
			dir_char = '^'
			
		speed_string = "%.2f km/h %s" % (CAN_VAL['VEH_SPEED'],dir_char)
		
		# PRINT IT 
		# print '\tVel: %.2f km/h %s'% (CAN_VAL['VEH_SPEED'],dir_char),
		
		## Construct LIGHT STRING
		light_string = ('LO' if (CAN_VAL['LowBeam'] > 0.0) else '  ') + ' ' + ('HI' if (CAN_VAL['HighBeams'] > 0.0) else '  ') 
		## PRINT LIGHTS
		# print '\tLights:',light_string ,
		
		## Calculate STEERING DIRECTION
		change_steer = print_CAN_line.last_steering_angle - CAN_VAL['SteeringWheel_Angle']
		dir_char = '  '
		if change_steer < 0:
			dir_char = '->'
		elif change_steer > 0:
			dir_char = '<-'
		# PRINT IT 
		steering_str = "%s %d°" % (dir_char,int(CAN_VAL['SteeringWheel_Angle']))
		#print 'Steering Angle:',steering_str
		
		## ACCESSORY STRING
		wpr_string = "OFF"
		if CAN_VAL['FrontWiperInUse'] > 0:
			wpr_string = " ON"
			
		## RPM STRING
		rpm_string = "%.0f" % CAN_VAL['EngineSpeed']
		
		## YAW STRING
		yaw_string = "%.3f deg/s" % CAN_VAL['YawRate']
		
		# ALL 
		if False: 
			print ''
			print "| %12s | %8s | %4s | %8s | %9s | %s |"% ("   SPEED    "," PEDALS "," RPM"," LIGHTS ","STEERING", "WPR")
			print "| %12s | %8s | %4s | %8s | %9s  | %s |"% (speed_string,pedal_string,rpm_string,light_string,steering_str,wpr_string)
		# No Lights 
		if True: 
			print ''
			print "| %12s | %8s | %4s | %9s | %s |"% ("   SPEED    "," PEDALS "," RPM","STEERING", "WPR")
			print "| %12s | %8s | %4s | %9s  | %s |"% (speed_string,pedal_string,rpm_string,steering_str,wpr_string)
		# No Lights, no wiper
		if False: 
			print ''
			print "| %12s | %8s | %4s | %9s | %13s | " % ("   SPEED    "," PEDALS "," RPM","STEERING","YAW RATE")
			print "| %12s | %8s | %4s | %9s  | %13s |" % (speed_string,pedal_string,rpm_string,steering_str, yaw_string)
		# ONly Lights
		if False: 
			print ''
			print "| %8s | %8s |"% (" PEDALS ", " LIGHTS ")
			print "| %8s | %8s |"% (pedal_string,light_string)
			
		print_CAN_line.last_speed = CAN_VAL['VEH_SPEED']
		print_CAN_line.last_steering_angle = CAN_VAL['SteeringWheel_Angle']
		
		
print_CAN_line.last_speed = 0
print_CAN_line.last_steering_angle = 0
		

def process_CAN_line(line):
	match = re.search(can_line,line)
	sleep_time = 0
	point = can_point()
	
	if match:
		point.hour=int(match.group(1))
		point.minute=int(match.group(2))
		point.second=int(match.group(3))
		point.msecond=int(match.group(4))
		point.parameter=match.group(5)
		point.value=float(match.group(6))
		point.nice_timestamp = "%02.d:%02.d:%02.d:%03.d" % ( point.hour,point.minute, point.second, point.msecond)
		
		enumerated_can_fields.add(point.parameter)
		
		if point.parameter in CAN_FIELDS.keys():
			CAN_VAL[point.parameter] = point.value
			
			
			
			if process_CAN_line.last_point_time:
				sleep_time = point.timestamp_seconds() - process_CAN_line.last_point_time
			else:
				process_CAN_line.last_point_time = point.timestamp_seconds()
			
			sleep_time -= process_CAN_line.time_since_print - time.time()
			sleep_time *= sleep_factor
			if sleep_time > WAIT_BEFORE_PRINTING:
				if(REALTIME_DISPLAY):
					time.sleep(sleep_time)
				print_CAN_line(point.nice_timestamp)
				process_CAN_line.time_since_print = time.time()
			
				process_CAN_line.last_point_time = point.timestamp_seconds()

process_CAN_line.last_point_time = 0
process_CAN_line.time_since_print = time.time()

## MAIN ##

if __name__ == "__main__":
	
	#Parse Args
	cur_arg_id = 1
	gps_filename = None
	can_filename = None
	while cur_arg_id < len(sys.argv):
		cur_arg = sys.argv[cur_arg_id];
		
		if cur_arg == '-h' or cur_arg == "--help":
			print "Usage: ./do_trail"
			print "\t-g [gps_filename]\t\tParse GPS trail"
			print "\t-c [can_filename]\t\tParse can file"
			break
		elif cur_arg == "-g":
			gps_filename = sys.argv[cur_arg_id+1]
			cur_arg_id += 2
		elif cur_arg == "-c":
			can_filename = sys.argv[cur_arg_id+1]
			cur_arg_id += 2
	
	if gps_filename:
		for line in open(gps_filename):
			process_GPS_line(line)

	if can_filename:
		print_CAN_header()
		for line in open(can_filename):
			process_CAN_line(line)

	if not gps_filename and not can_filename:
		print "For usage try:\t\t./do_trail --help"

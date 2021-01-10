#!/usr/bin/env python

# a span: when the lidar scan is devided into len(midi_settings_) parts, those parts are called spans

import roslib
import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Int16MultiArray
import numpy as np

midi_settings_ = None # holds all the midi settings from param server
laser_frame_id_ = None #holds laser frame id
span_vis_pub_ = None # hold publsiher for the visualisation messages (radial lines)
span_range_vis_pub_ = None # holds publisher for ranges detected visualisation marker
midi_pub_ = None # holds publisher of "midi" messages
old_midi_values_ = None #holds old values of the midi messages
min_lid_rng_ = 0 # hold min lidar distance value
max_lid_rng_ = 0 # holds max lidar distance value
invert_distance_midi_conversion_ = True # if false, max lidar dist means min midi value. If true max lidar dist value means max midi value


def scan_callback(data):
	global laser_frame_id_
	global old_midi_values_
	global min_lid_rng_
	global max_lid_rng_
	scan = LaserScan()
	scan = data

	# if min and max ranges werent set from params, set them from scan message
	if(min_lid_rng_ == 0):
		min_lid_rng_ = scan.range_min
		rospy.loginfo("Taking the min laser range from scan message: %lf", min_lid_rng_)
	if(max_lid_rng_ == 0):
		max_lid_rng_ = scan.range_max
		rospy.loginfo("Taking the max laser range from scan message: %lf", max_lid_rng_)

	laser_frame_id_ = scan.header.frame_id

	scan_angle = scan.angle_max-scan.angle_min
	nr_of_spans = len(midi_settings_)
	angle_per_span = scan_angle/nr_of_spans

	# create and publish the division lines of the laser for visualisation
	marker_line_list = []
	for i in range(0,nr_of_spans+1):
		marker_line_list.append(0.0)
		marker_line_list.append(0.0)	
		x,y = pol2cart(max_lid_rng_, scan.angle_min+i*angle_per_span)
		marker_line_list.append(x)
		marker_line_list.append(y)		
	create_and_pub_line_list_marker(marker_line_list, span_vis_pub_)

	# calculate a range in each span min and avarage
	avarage_ranges_in_spans = []
	min_ranges_in_spans = [max_lid_rng_]*nr_of_spans # populate list of lenght nr_of_spans 
	laser_points_per_span = len(scan.ranges)/nr_of_spans
	for span in range(0, nr_of_spans):
		avg_sum=0
		avg_dev=0
		for rang in range(0, laser_points_per_span):
			i = span*laser_points_per_span+rang
			# print i
			if(scan.ranges[i] > scan.range_min and scan.ranges[i] < max_lid_rng_):
				avg_sum = avg_sum+scan.ranges[i]
				avg_dev = avg_dev+1
				# find the minimum range in each span
				if scan.ranges[i]<min_ranges_in_spans[span]:
					min_ranges_in_spans[span] = scan.ranges[i]

		if(avg_dev != 0):
			avarage_ranges_in_spans.append(avg_sum/avg_dev)
		else:
			avarage_ranges_in_spans.append(0)

	# decide which ranges to use further from here
	applied_ranges_in_spans = min_ranges_in_spans # avarage_ranges_in_spans

	# create and publish the range lines for visualisation
	ranges_marker_line_list = []
	for i in range(0,len(applied_ranges_in_spans)):
		x,y = pol2cart(applied_ranges_in_spans[i], scan.angle_min+i*angle_per_span)
		ranges_marker_line_list.append(x)
		ranges_marker_line_list.append(y)
		x,y = pol2cart(applied_ranges_in_spans[i], scan.angle_min+(i+1)*angle_per_span)
		ranges_marker_line_list.append(x)
		ranges_marker_line_list.append(y)
	create_and_pub_line_list_marker(ranges_marker_line_list, span_range_vis_pub_)

	# map midi values to ranges
	midi_values = [0]*nr_of_spans # midi_values list is aligned with midi_settings_ list
	for i in range(0, nr_of_spans):
		midi_value = int(map(	applied_ranges_in_spans[i],
													min_lid_rng_, 
													max_lid_rng_, 
													midi_settings_[i]['min_val'], 
													midi_settings_[i]['max_val'], 
													invert_distance_midi_conversion_))
		# if mapping has failed write a midi value 0
		if type(midi_value) != None: 
			midi_values[i] = midi_value
		else:
			midi_values[i] = 0

	# publish extracted midi values
	for i in range(0, len(midi_values)):
		# publish a midi message only on change
		if(midi_values[i] != old_midi_values_[i]):
			msg = Int16MultiArray()
			msg.data.append(midi_settings_[i]['midi_id'])
			msg.data.append(midi_values[i])
			midi_pub_.publish(msg)
			old_midi_values_[i] = midi_values[i]

# maps a value that is leftMin<value<leftMax to a value that is rightMin<output<rightMax
def map(value, leftMin, leftMax, rightMin, rightMax, invert=False):
	#make sure input value is within bounds
	if (value<leftMin):
		rospy.logwarn("Input value that is being mapped is out of bounds. Setting it to min bound")
		value = leftMin
	if (value>leftMax):
		rospy.logwarn("Input value that is being mapped is out of bounds. Setting it to max bound")
		value = leftMax

	# Figure out how 'wide' each range is
	leftSpan = leftMax - leftMin
	rightSpan = rightMax - rightMin
	
	# check if the spans are large enough
	if abs(leftSpan) < 1e-6:
		rospy.logerr("left span in mapping values too small")
		return None
	if abs(rightSpan) < 1e-6:
		rospy.logerr("right span in mapping values too small")
		return None

	# Convert the left range into a 0-1 range (float)
	valueScaled = float(value - leftMin) / float(leftSpan)
	
	# if inversion is enables, invert
	if invert:
		valueScaled = 1-valueScaled

	# Convert the 0-1 range into a value in the right range.
	return rightMin + (valueScaled * rightSpan)

def create_and_pub_line_list_marker(marker_line_list, publisher):
	marker = Marker()
	marker.header.frame_id = laser_frame_id_
	marker.header.stamp = rospy.Time.now()
	marker.ns = "scan_devider_lines"
	marker.id = 0
	marker.type = marker.LINE_LIST
	marker.action = marker.ADD
	for i in range(0, len(marker_line_list)/2):
		point = Point()
		point.x = marker_line_list[i*2]
		point.y = marker_line_list[i*2+1]
		marker.points.append(point)
	marker.pose.orientation.w= 1.0
	marker.scale.x = 0.05
	marker.color.a = 0.4
	marker.color.r = 0.0
	marker.color.g = 1.0
	marker.color.b = 0.0
	publisher.publish(marker)

def pol2cart(radius, angle):
	x = radius * np.cos(angle)
	y = radius * np.sin(angle)
	return(x, y)

def main():
	global midi_settings_
	global span_vis_pub_
	global span_range_vis_pub_
	global midi_pub_
	global old_midi_values_
	global min_lid_rng_
	global max_lid_rng_
	global invert_distance_midi_conversion_

	rospy.init_node('lidar_midi_node', anonymous=True)
	rospy.Subscriber("/scan", LaserScan, scan_callback)
	span_vis_pub_ = rospy.Publisher('/scan_devider_lines', Marker, queue_size=10)
	span_range_vis_pub_ = rospy.Publisher('/scan_ranges_lines', Marker, queue_size=10)
	midi_pub_ = rospy.Publisher('/phosporm_midi_input', Int16MultiArray, queue_size=10)
	
	if rospy.has_param('/midi_settings'):
		rospy.loginfo("Got midi settings param")
		midi_settings_ = rospy.get_param('/midi_settings')
	else:
		rospy.logerr("Did not detect midi settings param")
		return

	min_lid_rng_ = rospy.get_param("~min_lidar_range", 0)
	max_lid_rng_ = rospy.get_param("~max_lidar_range", 0)
	invert_distance_midi_conversion_ = rospy.get_param("invert_distance_midi_conversion", True)

	old_midi_values_=[0]*len(midi_settings_) #populate list of lenght len(midi_settings_) with zeroes
	
	rospy.spin()

if __name__ == '__main__':
	main()

"""
    Description: Publisher for sensor_msg/Imu message from our Sparkfun20948
    Author: Victor & this lovely github post
    https://github.com/ev3dev-lang-java/ev3dev-lang-java/issues/356
    Date: Feb 2024
"""

import sys
from time import time, sleep

# ros packages for publishing
import rospy
from sensor_msgs.msg import Imu, MagneticField

# not sure what these two do...
from dynamic_reconfigure.server import Server
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# used for Kalaman Extended Filter for Euler angles to Quaternion
import numpy as np
from ahrs.filters import EKF

import argparse

# libraries required for imu / circuitpython wrapper (I think it's circuitpython?)
import board
import busio
import adafruit_icm20x

ap = argparse.ArgumentParser()
ap.add_argument('-p', 
		'--publish_custom_ekf', 
		required = False, 
		help = 'Whether to publish custom imu/custom_filtered ekf values topic',
		default = 0,
)
args = vars(ap.parse_args())

try:
	args['publish_custom_ekf'] = int(args['publish_custom_ekf'])
	if args['publish_custom_ekf'] < 0 or args['publish_custom_ekf'] > 1:
		args['publish_custom_ekf'] = 0
		print("provided --pub_custom_ekf flag was not 0 or 1; defaulted to 0...") 
except ValueError:
	print("--publish_custom_ekf flag must be int 0 or 1; defaulted to 0...")
	args['publish_custom_ekf'] = 0
	
pub_custom_ekf = bool(args['publish_custom_ekf'])

imu_raw = Imu()             # Raw data
mag_msg = MagneticField()   # Magnetometer data

if pub_custom_ekf:
	imu_data = Imu()    # Filtered data

i2c = busio.I2C(board.SCL, board.SDA)  # uses board.SCL and board.SDA
icm = adafruit_icm20x.ICM20948(i2c)

print("I2C connection to IMU board established successfully...")

def get_imu_values():
	imu_raw = {
		'linear_acceleration': icm.acceleration,
		'gyro': icm.gyro,
		'magnetometer': icm.magnetic,
	}

	# print(icm.acceleration, "<-- type of icm.acceleration:", type(icm.acceleration))

	# print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (icm.acceleration))
	# print("Gyro X:%.2f, Y: %.2f, Z: %.2f rads/s" % (icm.gyro))
	# print("Magnetometer X:%.2f, Y: %.2f, Z: %.2f uT" % (icm.magnetic))

	return imu_raw

# Main function
if __name__ == '__main__':
	# change "imu" to whatever you want Node name to be
	rospy.init_node("imu")

	# Sensor measurements publishers
	pub_raw = rospy.Publisher('imu/data_raw', Imu, queue_size=1)
	pub_mag = rospy.Publisher('imu/mag', MagneticField, queue_size=1)

	if pub_custom_ekf:
		pub_data = rospy.Publisher('imu/custom_filtered', Imu, queue_size=1)

	# Get parameters values
	frame_id = rospy.get_param('~frame_id', 'imu_link')
	frequency = rospy.get_param('~frequency', 100)

	rate = rospy.Rate(frequency)

	print("Topics 'imu/data_raw', 'imu/mag' created successfully")
	if pub_custom_ekf:
		print("'imu/custom_filtered' topic created successfully, custom ekf filter values will also be published")

	# required for Ros header, keeping track of num of broadcasts
	seq = 0

	while not rospy.is_shutdown():
		raw_imu_vals = get_imu_values()

		# Publish raw data
		imu_raw.header.stamp = rospy.Time.now()
		imu_raw.header.frame_id = frame_id
		imu_raw.header.seq = seq

		imu_raw.orientation_covariance[0] = -1

		imu_raw.linear_acceleration.x = float(raw_imu_vals['linear_acceleration'][0])
		imu_raw.linear_acceleration.y = float(raw_imu_vals['linear_acceleration'][1])
		imu_raw.linear_acceleration.z = float(raw_imu_vals['linear_acceleration'][2])
		imu_raw.linear_acceleration_covariance[0] = -1

		imu_raw.angular_velocity.x = float(raw_imu_vals['gyro'][0])
		imu_raw.angular_velocity.y = float(raw_imu_vals['gyro'][1])
		imu_raw.angular_velocity.z = float(raw_imu_vals['gyro'][2])
		imu_raw.angular_velocity_covariance[0] = -1

		pub_raw.publish(imu_raw)

		if pub_custom_ekf:
			# Publish filtered data
			imu_data.header.stamp = rospy.Time.now()
			imu_data.header.frame_id = frame_id
			imu_data.header.seq = seq

			# extended kalaman filter; for acc and gyro values to quaternion
			ekf = EKF(
				gyr = np.array([raw_imu_vals['gyro']]),
				acc = np.array([raw_imu_vals['linear_acceleration']]),
			)
			# access with ekf.Q.shape

			imu_data.orientation.w = float(ekf.Q[0][0])
			imu_data.orientation.x = float(ekf.Q[0][1])
			imu_data.orientation.y = float(ekf.Q[0][2])
			imu_data.orientation.z = float(ekf.Q[0][3])
			imu_raw.orientation_covariance[0] = -1

			imu_data.linear_acceleration.x = float(raw_imu_vals['linear_acceleration'][0])
			imu_data.linear_acceleration.y = float(raw_imu_vals['linear_acceleration'][1])
			imu_data.linear_acceleration.z = float(raw_imu_vals['linear_acceleration'][2])
			imu_data.linear_acceleration_covariance[0] = -1

			imu_data.angular_velocity.x = float(raw_imu_vals['gyro'][0])
			imu_data.angular_velocity.y = float(raw_imu_vals['gyro'][1])
			imu_data.angular_velocity.z = float(raw_imu_vals['gyro'][2])
			imu_data.angular_velocity_covariance[0] = -1

			pub_data.publish(imu_data)

		# Publish magnetometer data
		mag_msg.header.stamp = rospy.Time.now()
		mag_msg.header.frame_id = frame_id
		mag_msg.header.seq = seq

		mag_msg.magnetic_field.x = float(raw_imu_vals['magnetometer'][0])
		mag_msg.magnetic_field.y = float(raw_imu_vals['magnetometer'][1])
		mag_msg.magnetic_field.z = float(raw_imu_vals['magnetometer'][2])

		pub_mag.publish(mag_msg)

		if seq == 0:
			print("Successfully published on all topics, ekf filter should be working, good stuff")

		seq = seq + 1
		rate.sleep()

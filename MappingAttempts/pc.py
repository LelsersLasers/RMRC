# from base64 import decode
# import cv2
import rospy 

import numpy as np
import quaternion

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2

from nav_msgs.msg import Odometry

from std_msgs.msg import Header
# from pprint import pprint
# from cv_bridge import CvBridge

import read


rosTopicPointsIn = "/camera/depth/color/points"
rosTopicPositionIn = "/t265/odom/sample"
rosTopicPointsOut = "/out_cloud"

class TODO:
	...
	# TODO! not all TODO uses mean the same class


class MutState:
	def __init__(self):
		self.points = []

		self.transform = Odometry()

		self.frame = 0

		self.pub = rospy.Publisher(rosTopicPointsOut, PointCloud2, queue_size=1)
		self.header = Header()
		self.header.frame_id = "camera_depth_optical_frame"
		# self.header.frame_id = "joe"

	def publish_points(self):

		fields = [
			PointField('x', 0, PointField.FLOAT32, 1),
			PointField('y', 4, PointField.FLOAT32, 1),
			PointField('z', 8, PointField.FLOAT32, 1)
		]

		# points = []
		# for point in self.points:
		# 	points.append([point.x, point.y, point.z])

		pc2 = point_cloud2.create_cloud(self.header, fields, self.points)
		pc2.header.stamp = rospy.Time.now()

		print("Publish\t %i" % len(self.points), flush=True)
		
		self.pub.publish(pc2)


def listener():
	mutState = MutState()

	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber(rosTopicPositionIn, Odometry, read_transform, callback_args=mutState, queue_size=1, buff_size=9999999)
	rospy.Subscriber(rosTopicPointsIn, PointCloud2, read_cloud, callback_args=mutState, queue_size=1, buff_size=9999999)
	
	rospy.spin()

	# rate = rospy.Rate(1)
	# while not rospy.is_shutdown():
	# 	mutState.publish_points()
	# 	rate.sleep()

# def pick_closest_point(cloud):
# 	list_of_points = read.read_points_list(cloud)

# 	list_of_distances = [point.x ** 2 + point.y ** 2 + point.z ** 2 for point in list_of_points]
# 	min_dist = min(list_of_distances)
# 	min_idx = list_of_distances.index(min_dist)

# 	return list_of_points[min_idx]


def read_cloud(cloud, mutState):

	"""
	SYNC NEW POINTS WITH MASTER LIST
	1) Quaterion to translation
	2) Subtract (1) from new point cloud
	3) Apply quaterion to (2)
	"""

	ori = mutState.transform.pose.pose.orientation
	quat = np.quaternion(ori.w, ori.x, ori.y, ori.z)
	rotate = quaternion.as_rotation_matrix(quat)

	# 1)
	translate = mutState.transform.pose.pose.position
	np_translate = np.array([translate.x, translate.y, translate.z])
	rotated_translation = rotate @ np_translate

	# 2)
	list_of_points = read.read_points_list(cloud)
	# list_of_points = [ pick_closest_point(cloud) ]

	list_of_np_points = []
	for point in list_of_points:
		np_point = np.array( [point.x, point.y, point.z])
		moved_point = np_point - rotated_translation
		list_of_np_points.append(moved_point)


	# 3)
	transformed_points = quaternion.rotate_vectors(quat, list_of_np_points)

	# Format for RVIZ
	list_of_list_of_points = []
	for point in transformed_points:
		new_point = [
			point[0],
			point[1],
			point[2],
		]
		list_of_list_of_points.append(new_point)

	# for point in list_of_points:
	# 	new_point = [point.x, point.y, point.z]
	# 	np_point = np.array(new_point)

	# 	rotated_point = rotate @ np_point

	# 	new_point[0] = rotated_point[0] + mutState.transform.pose.pose.position.x
	# 	new_point[1] = rotated_point[1] + mutState.transform.pose.pose.position.y
	# 	new_point[2] = rotated_point[2] + mutState.transform.pose.pose.position.z

	# 	transformed_points.append(new_point)


	# mutState.points = transformed_points + mutState.points

	# Update state
	mutState.points = list_of_list_of_points + mutState.points
	mutState.frame += 1

	print("Listen\t", mutState.frame, len(transformed_points), len(mutState.points), flush=True)

	# Send new points to RVIZ
	mutState.publish_points()

def read_transform(transform, mutState):
	mutState.transform = transform

	# print("Transform recieved")
	# print(transform.pose.pose.position)




def main():
    listener()

main()

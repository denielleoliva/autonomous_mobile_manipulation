#!/usr/bin/python2

import copy

import rospy
import numpy as np
import math
import sys

from geometry_msgs.msg import WrenchStamped, PoseStamped, Twist, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from threading import Thread

import tf2_ros
import PyKDL as kdl 
import kdl_parser_py.urdf as kdl_parser

def get_smallest_dist_and_direction(ang1, ang2):

    temp1 = ang1
    temp2 = ang2

    if ang1 <= 0:
        temp1 = math.pi * 2 + ang1

    if ang2 <= 0:
        temp2 = math.pi * 2 + ang2

    distance1 = temp1 - temp2
    distance2 = temp2 - temp1

    if distance1 < 0:
        distance1 += math.pi * 2

    if distance2 < 0:
        distance2 += math.pi * 2

    if distance1 >= distance2:
        return distance2, 'left'
    else:
	    return distance1, 'right'

# quaternion multiplication function from here: https://stackoverflow.com/questions/39000758/how-to-multiply-two-quaternions-by-python-or-numpy
def hamilton_product(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

class Follower:

	FORCE_THRESHOLD = .4

	def __init__(self):
		self.end_effector_force_due_to_gravity = np.array([0, -6.85163828398, 28.5188294731, -0.0527265813691 ])

		self.saved_force = []
		self.obstacles = []
		self.wrist_force = []
		self.arm_force = []
		self.twist = Twist()
		self.is_first = True

		self.tfBuffer = tf2_ros.Buffer()
		self.broadcaster = tf2_ros.TransformBroadcaster()
		self.listener = tf2_ros.TransformListener(self.tfBuffer) 

		flag, self.tree = kdl_parser.treeFromParam("/bvr_SIM/robot_description")
		self.arm_chain = self.tree.getChain("bvr_SIM/main_arm_SIM/base_link", "bvr_SIM/main_arm_SIM/wrist_3_link")

		# print self.arm_chain.getNrOfJoints()

		# for i in xrange(self.arm_chain.getNrOfSegments()):
		# 	print self.arm_chain.getSegment(i).getName()

		# sys.exit(0)

		gravity = kdl.Vector(0.0, 0.0, 9.81)

		self.dyn_solver = kdl.ChainDynParam(self.arm_chain, gravity)
		self.jac_solver = kdl.ChainJntToJacSolver(self.arm_chain)

		self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)
		self.force_sub = rospy.Subscriber('/bvr_SIM/ft_sensor_topic', WrenchStamped, self.force_cb)
		# self.joint_sub = rospy.Subscriber('/bvr_SIM/joint_states', JointState, self.joint_cb)

		self.cmd_vel_pub = rospy.Publisher('/bvr_SIM/cmd_vel', Twist, queue_size=10)
  		self.twist_stamped_publisher = rospy.Publisher('/bvr_SIM/interactive_markers_main_arm_SIM/follower_node/cmd_vel', Twist, queue_size=1)

	# filter the incoming force data specifically for the motion
	def lp_filter (self, filtered_force, measured_force, alpha=.1):
		return filtered_force - ( alpha * (filtered_force - measured_force) )

	def hp_filter (self, filtered_force, measured_force, alpha=.5):
		return filtered_force - ( alpha * (filtered_force - measured_force) )

	def joint_cb (self, data):
		
		# get joint states in kdl format
		joint_state = kdl.JntArray(self.arm_chain.getNrOfJoints())
		joint_state[0] = data.position[5]#shoulder_pan
		joint_state[1] = data.position[4]#shoulder_lift
		joint_state[2] = data.position[2]#elbow_joint
		joint_state[3] = data.position[6]#wrist_1_joint
		joint_state[4] = data.position[7]#wrist_2_joint
		joint_state[4] = data.position[8]#wrist_3_joint

		# now get torques due to gravity
		gravity_torques = kdl.JntArray(self.arm_chain.getNrOfJoints())
		self.dyn_solver.JntToGravity(joint_state, gravity_torques)

		# now transform to get the wrench at wrist_1_link
		jac = kdl.Jacobian(self.arm_chain.getNrOfJoints())
		self.jac_solver.JntToJac(joint_state, jac, -2)

		# convert to numpy format
		gravity_torques_np = np.array([ i for i in gravity_torques ])
		jac_np = np.zeros((6, self.arm_chain.getNrOfJoints()))

		for i in xrange(6):
			for j in xrange(self.arm_chain.getNrOfJoints()):
				jac_np[i, j] = jac[i, j]

		# F = J^T * tau	
		gravity_wrench_np = np.matmul(np.linalg.pinv(jac_np.transpose), gravity_torques_np)

		# now rotate from base_link to wrist_1_link
		gravity_wrench_transformed = np.array([0.0, gravity_wrench_np[0], gravity_wrench_np[1], gravity_wrench_np[2]])

		print gravity_wrench_transformed

		try:
			transform = self.tfBuffer.lookup_transform( 'bvr_SIM/main_arm_SIM/wrist_1_link', 'bvr_SIM/main_arm_SIM/base_link', rospy.Time())
		except:
			return 

		trans = [ transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z, 0]
		rot = [ transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z ]

		gravity_wrench_transformed = hamilton_product(rot, hamilton_product(gravity_wrench_transformed, [rot[0], -1 * rot[1], -1 * rot[2], -1 * rot[3]]))

		self.end_effector_force_due_to_gravity = gravity_wrench_transformed


	def update_obstacles_from_laser(self, data):
		n_obstacles = []

		range_max = 10.0

		for index, val in enumerate(data.ranges):

			angle = data.angle_min + index * data.angle_increment			

			if val < data.range_min or val > range_max or val == float("inf") or angle < (-5 * math.pi / 6) or angle > (5 * math.pi / 6):
				continue

			n_x = val * math.cos(angle)
			n_y = val * math.sin(angle)

			n_obstacles.append(np.array([n_x, n_y]))

		self.obstacles = n_obstacles

	def publish_vector_in_frame(self, translation, rotation, parent_frame, child_frame):

		to_pub = TransformStamped()

		to_pub.header.stamp = rospy.Time.now()
		to_pub.header.frame_id = parent_frame
		to_pub.child_frame_id = child_frame

		to_pub.transform.translation.x = translation[0]
		to_pub.transform.translation.y = translation[1]
		to_pub.transform.translation.z = translation[2]
		to_pub.transform.rotation.w = rotation[0]
		to_pub.transform.rotation.x = rotation[1]
		to_pub.transform.rotation.y = rotation[2]
		to_pub.transform.rotation.z = rotation[3]

		self.broadcaster.sendTransform(to_pub)

	def laser_cb(self, data):
		self.update_obstacles_from_laser(data)

	def force_cb(self, data):
		force = np.array([0, data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])

		# force -= self.end_effector_force_due_to_gravity

		# print force
		
		self.wrist_force = force[1:]
		# print self.wrist_force 

		# print '_'*30s

		# print force

		self.transform_and_save_force(force)
		# print self.saved_force

	def transform_and_save_force(self, force):
		try:
			transform = self.tfBuffer.lookup_transform( 'bvr_SIM/bvr_base_link', 'bvr_SIM/main_arm_SIM/wrist_1_link', rospy.Time())
		except:
			return 

		trans = [ 0, transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
		rot = [ transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z ]

		f_rotated = hamilton_product(rot, hamilton_product(force, [rot[0], -1 * rot[1], -1 * rot[2], -1 * rot[3]]))

		# print f_rotated

		# f_transformed = f_rotated + trans

		to_ret = f_rotated[1:3]

		if self.saved_force == []:
			self.saved_force = to_ret
		else:	
			self.saved_force = self.lp_filter(self.saved_force, to_ret)

		print self.saved_force

	def calc_overall_force(self):

		attr_gain = 10.0

		total_force = copy.deepcopy(self.saved_force)
		current_obstacles = copy.deepcopy(self.obstacles) 

		total_force *= attr_gain

		# equations for attractive and repulsive force are taken from here: https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf
		# max distance is one meter
		q_star = 3.0
		gains = np.array([5, 5])

		# print total_force

		# print '-'*30

		repulsive_force = np.array([0.0, 0.0])

		for index, val in enumerate(current_obstacles):
			distance = np.linalg.norm(val)

			if distance > q_star or distance == float("inf"):
				continue
			# print "[]"*30	

			# print "{0}, {1}, {2}: [{3}]".format(q_star, distance, gains, i)
			# print total_force

			# I think the sign here is correct but it's questionable
			repulsive_force_i = ( (1 / q_star) - ( 1 / distance ) ) * ( 1 / distance ** 2 ) * gains * ( ( val ) / distance )
			# print distance

			# if index % 3 == 0:
			# 	self.publish_vector_in_frame(
			# 			translation=[repulsive_force_i[0], repulsive_force_i[1], 0],
			# 			rotation=[1, 0, 0, 0],
			# 			parent_frame="bvr_SIM/bvr_base_link",
			# 			child_frame="obstacle_force_{0}".format(index)
			# 		)

			repulsive_force += repulsive_force_i


		# print "_"*30
		# print repulsive_force
		# print np.linalg.norm(repulsive_force)

		total_force += repulsive_force

		# print total_force
		# print np.linalg.norm(total_force)

		return total_force

	def wait_for_initial_scans(self):
		while ( self.saved_force == [] or self.obstacles == [] ) and not rospy.is_shutdown():
			pass

	def get_twist_from_force(self, vel, delta_t=.2):

		true_angle = math.atan2(vel[1], vel[0])

		if true_angle > math.pi:
			true_angle -= 2 * math.pi

		# dist, direction = get_smallest_dist_and_direction( 0, true_angle )

		# print dist, direction

		twist = Twist()

		twist.linear.x = max(np.linalg.norm(vel) * math.cos(true_angle), 0)

		twist.angular.z = true_angle / delta_t

		return twist

	def publish_transforms(self):

		while not rospy.is_shutdown():
			self.publish_vector_in_frame(
					self.wrist_force,
					[1, 0, 0, 0],
					"bvr_SIM/main_arm_SIM/wrist_1_link",
					"output_force_arm"
				)
			self.publish_vector_in_frame(
					[ self.saved_force[0], self.saved_force[1], 0],
					[1, 0, 0, 0],
					"map",
					"output_force"
				)

	def calc_arm_attr_force(self):

		try:
			transform = self.tfBuffer.lookup_transform( 'bvr_SIM/main_arm_SIM/wrist_1_link', 'bvr_SIM/bvr_base_link', rospy.Time())
		except:
			return [0, 0]

		current_pos = np.array([ transform.transform.translation.x, transform.transform.translation.y ])

		distance = np.linalg.norm(current_pos - self.home_pos)

		# print ":)" + str( self.home_pos - current_pos ) 

		# if distance < .01:
		# 	return [ 0, 0 ]

		return 100 * ( self.home_pos - current_pos ) 

	def move_arm(self):

		twist_out = Twist()

		force = copy.deepcopy(self.saved_force)

		if self.arm_force == []:
			self.arm_force = force
		else:
			self.arm_force = self.lp_filter(self.arm_force, force, alpha=.6)

		# if np.linalg.norm(self.arm_force) > 0:
		# 	self.arm_force /= np.linalg.norm(self.arm_force)

		# self.arm_force *= .1

		attr_force = self.calc_arm_attr_force()

		total_force = self.arm_force - attr_force

		# :(
		total_force[0] = 0

		# print '_'*30
		# print total_force	

		try:
			transform = self.tfBuffer.lookup_transform( 'bvr_SIM/main_arm_SIM/wrist_3_link', 'bvr_SIM/bvr_base_link', rospy.Time())
		except:
			return 

		rot = [ transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z ]

		force_to_transform = [0, total_force[0], total_force[1], 0 ]

		f_rotated = hamilton_product(rot, hamilton_product(force_to_transform, [rot[0], -1 * rot[1], -1 * rot[2], -1 * rot[3]]))

		# print f_rotated 

		twist_out.linear.x = (f_rotated[1] if (abs(f_rotated[1]) > .1) else 0 )
		twist_out.linear.y = (f_rotated[2] if (abs(f_rotated[2]) > .1) else 0 )
		twist_out.linear.z = (f_rotated[3] if (abs(f_rotated[3]) > .1) else 0 )

		if abs(twist_out.linear.x) > .05:
			twist_out.linear.x = .05 * (twist_out.linear.x / abs(twist_out.linear.x))
		if abs(twist_out.linear.y) > .05:
			twist_out.linear.y = .05 * (twist_out.linear.y / abs(twist_out.linear.y))
		if abs(twist_out.linear.z) > .05:
			twist_out.linear.z = .05 * (twist_out.linear.z / abs(twist_out.linear.z))

		# print twist_out

		self.twist_stamped_publisher.publish(twist_out)

	def tick(self):

		self.move_arm()

		if np.linalg.norm(self.saved_force) < Follower.FORCE_THRESHOLD:
			self.twist = Twist()

			# print "!"*30
			# print self.saved_force

			return

		# print "?"*30
		# print self.saved_force

		total_force = self.calc_overall_force()

		self.twist = self.get_twist_from_force(total_force)

		self.cmd_vel_pub.publish(self.twist)

if __name__ == '__main__':

	rospy.init_node("follower_node", anonymous=True)

	follower_obj = Follower()
	follower_obj.wait_for_initial_scans()

	try:
		transform = follower_obj.tfBuffer.lookup_transform( 'bvr_SIM/main_arm_SIM/wrist_1_link', 'bvr_SIM/bvr_base_link', rospy.Time())
	except:
		sys.exit(0)

	follower_obj.home_pos = np.array([ transform.transform.translation.x, transform.transform.translation.y ])

	pub_thread = Thread(target=follower_obj.publish_transforms)

	pub_thread.start()

	while not rospy.is_shutdown():
		follower_obj.tick()

	pub_thread.join()
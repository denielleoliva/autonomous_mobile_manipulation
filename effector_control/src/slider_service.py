#!/usr/bin/python2

from __future__ import absolute_import
import sys
import rospy
import rospkg
import tf2_ros
import tf
import threading

import numpy as np
import sys

from PyQt5.QtWidgets import QApplication, QWidget, QSlider, QLabel, QVBoxLayout, QPushButton
from PyQt5.QtCore import Qt, QObject, QThread
from threading import Thread

from geometry_msgs.msg import WrenchStamped, Vector3, TransformStamped
from gazebo_msgs.srv import ApplyBodyWrench
# from tf.transformations import euler_from_quaternion

# quaternion multiplication function from here: https://stackoverflow.com/questions/39000758/how-to-multiply-two-quaternions-by-python-or-numpy
def hamilton_product(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

class SliderWorker(QObject):

	wrench = WrenchStamped()
	wrench.wrench.force.x  = 0 #this is really z
	wrench.wrench.force.y  = 0
	wrench.wrench.force.z  = 0
	wrench.wrench.torque.x = 0
	wrench.wrench.torque.y = 0
	wrench.wrench.torque.z = 0

	def run(self):
		rospy.wait_for_service('/gazebo/apply_body_wrench') 
		force = rospy.ServiceProxy('/gazebo/apply_body_wrench',ApplyBodyWrench)

		while not rospy.is_shutdown():
			# This essentially is just a constructor for a ApplyBodyWrench srv object that is then passed through a service
			force(
					body_name = "bvr_SIM::bvr_SIM/main_arm_SIM/wrist_1_link",
					reference_frame= "map", 
					wrench = SliderWorker.wrench.wrench, 
					duration = rospy.Duration(10)
				)
			rospy.sleep(.1)

class SliderApp(QWidget):
	def __init__(self):
		super(SliderApp, self).__init__()
		self.setWindowTitle("Slider App")
		self.setGeometry(100, 100, 400, 200)  # (x, y, width, height)

		layout = QVBoxLayout()

		self.wrench_in = WrenchStamped()

		self.wrench_in.header.frame_id = 'bvr_SIM/main_arm_SIM/wrist_1_link'

		# Create a QSlider widget
		self.slider_x = QSlider(Qt.Horizontal, self)
		self.slider_x.setValue(50)
		layout.addWidget(self.slider_x)

		# Create a QLabel widget
		self.label_x = QLabel("x vector: 0.0", self)
		layout.addWidget(self.label_x)

		#slider y
		self.slider_y = QSlider(Qt.Horizontal, self)
		self.slider_y.setValue(50)
		layout.addWidget(self.slider_y)

		self.label_y = QLabel("y vector: 0.0", self)
		layout.addWidget(self.label_y)

		self.reset_button = QPushButton("&Reset", self)
		layout.addWidget(self.reset_button)

		self.label_vector = QLabel("Transformed vector: [0.0, 0.0, 0.0]")
		layout.addWidget(self.label_vector)

		self.transformed_wrench = [0, 0, 0, 0]


		# Connect the slider's valueChanged signal to the update_label slot
		# self.slider_x.valueChanged.connect(self.update_label)

		# self.slider_x.valueChanged.connect(self.update_vectors)
		self.slider_x.valueChanged.connect(self.applyForce)
		self.slider_x.valueChanged.connect(lambda : self.label_x.setText("X Value: {0}".format(float(self.slider_x.value()) / 10.0 - 5)))
		self.slider_x.valueChanged.connect(lambda : self.label_vector.setText("Transformed vector: [{0}, {1}, {2}]".format(self.transformed_wrench[0], self.transformed_wrench[1], self.transformed_wrench[2])))
		self.slider_y.valueChanged.connect(self.applyForce)
		self.slider_y.valueChanged.connect(lambda : self.label_y.setText("Y Value: {0}".format(float(self.slider_y.value()) / 10.0 - 5)))
		self.slider_y.valueChanged.connect(lambda : self.label_vector.setText("Transformed vector: [{0}, {1}, {2}]".format(self.transformed_wrench[0], self.transformed_wrench[1], self.transformed_wrench[2])))
		
		self.reset_button.clicked.connect(lambda : self.slider_x.setValue(50) or self.slider_y.setValue(50) )
		# self.slider_x.valueChanged.connect(self.publish_vectors)
		# self.slider_y.valueChanged.connect(self.publish_vectors)
		# self.slider_z.valueChanged.connect(self.publish_vectors)

		# print tf2_ros.__file__
		self.tfBuffer = tf2_ros.Buffer()
		self.broadcaster = tf2_ros.TransformBroadcaster()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)
		self.setLayout(layout)

		self.wrench = WrenchStamped()
		self.wrench.wrench.force.x  = 0 #this is really z
		self.wrench.wrench.force.y  = 0
		self.wrench.wrench.force.z  = 0
		self.wrench.wrench.torque.x = 0
		self.wrench.wrench.torque.y = 0
		self.wrench.wrench.torque.z = 0

		self.thread = QThread()
		self.worker = SliderWorker()

		SliderWorker.wrench.header.frame_id = self.wrench_in.header.frame_id

		self.worker.moveToThread(self.thread)
		
		self.thread.started.connect(self.worker.run)
		# self.workers.connect(self.worker.get_thread,  pyqtSignal("finished()"), self.thread.deleteLater)
		# self.thread.finished.connect(self.thread.deleteLater)
		self.thread.start()

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

	def applyForce(self):
		wrench          = WrenchStamped()
		wrench.wrench.force.x  = float(self.slider_x.value()) / 10.0 - 5 #this is really z
		wrench.wrench.force.y  = float(self.slider_y.value()) / 10.0 - 5
		wrench.wrench.force.z  = 0
		wrench.wrench.torque.x = 0
		wrench.wrench.torque.y = 0
		wrench.wrench.torque.z = 0

		self.publish_vector_in_frame(
				[wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z],
				[1, 0, 0, 0],
				"map",
				"input_force"
			)

		self.transformed_wrench = self.moveJoint(wrench.wrench)

		self.publish_vector_in_frame(
				self.transformed_wrench[:3],
				[1, 0, 0, 0],
				"bvr_SIM/main_arm_SIM/wrist_1_link",
				"input_force_arm"
			)

		wrench.wrench.force.x  = self.transformed_wrench[0] #this is really z
		wrench.wrench.force.y  = self.transformed_wrench[1]
		wrench.wrench.force.z  = self.transformed_wrench[2]
		wrench.wrench.torque.x = self.transformed_wrench[3]
		wrench.wrench.torque.y = self.transformed_wrench[4]
		wrench.wrench.torque.z = self.transformed_wrench[5]

		SliderWorker.wrench = wrench


	def moveJoint(self, wrench):

		try:
			transform = self.tfBuffer.lookup_transform('bvr_SIM/main_arm_SIM/wrist_1_link', 'map', rospy.Time())
		except:
			return 

		# trans = [ transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z, 0]
		rot = [ transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z ]

		# print transform

		force_comp =np.array([0, wrench.force.x, wrench.force.y, wrench.force.z])
		moment_comp = np.array([wrench.torque.x, wrench.torque.y, wrench.torque.z, 0])

		# trans = np.array(trans)
		rot = np.array(rot)

		f_rotated = hamilton_product(rot, hamilton_product(force_comp, [rot[0], -1 * rot[1], -1 * rot[2], -1 * rot[3]]))

		# m_rotated = np.dot(rot, np.dot(moment_comp, np.conjugate(rot))) + ( np.cross(np.atleast_2d(trans).transpose(), np.atleast_2d(f_rotated).transpose(), axis=0) )

		# f_transformed = f_rotated + trans
		# m_transformed = m_rotated + np.cross(trans, f_rotated)

		# to_ret = f_rotated[1:]

		# to_ret = np.append(to_ret, [0, 0, 0])

		to_ret = force_comp[1:]

		to_ret = np.append(to_ret, [0, 0, 0])

		return np.array(to_ret)

def main():
	app = QApplication(sys.argv)

	rospy.init_node("force_qt_pub")

	# print u":)", rospkg.get_ros_root()

	window = SliderApp()

	window.show()

	sys.exit(app.exec_())

if __name__ == "__main__":
	main()
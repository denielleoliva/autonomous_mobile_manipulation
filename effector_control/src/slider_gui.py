#!/usr/bin/python3

import sys
import rospy
import rospkg

from PyQt5.QtWidgets import QApplication, QWidget, QSlider, QLabel, QVBoxLayout
from PyQt5.QtCore import Qt
from threading import Thread

from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Vector3
from gazebo_msgs.srv import ApplyBodyWrench

class SliderApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Slider App")
        self.setGeometry(100, 100, 400, 200)  # (x, y, width, height)

        layout = QVBoxLayout()

        self.wrench_in = WrenchStamped()

        self.wrench_in.header.frame_id = 'bvr_SIM/main_arm_SIM/wrist_3_link'

        # Create a QSlider widget
        self.slider_x = QSlider(Qt.Horizontal, self)
        layout.addWidget(self.slider_x)

        # Create a QLabel widget
        self.label_x = QLabel("x vector: 0", self)
        layout.addWidget(self.label_x)


        #slider y
        self.slider_y = QSlider(Qt.Horizontal, self)
        layout.addWidget(self.slider_y)

        self.label_y = QLabel("y vector: 0", self)
        layout.addWidget(self.label_y)

        #slider z
        self.slider_z = QSlider(Qt.Horizontal, self)
        layout.addWidget(self.slider_z)

        self.label_y = QLabel("z vector: 0", self)
        layout.addWidget(self.label_y)

        # Connect the slider's valueChanged signal to the update_label slot
        self.slider_x.valueChanged.connect(self.update_label)

        self.slider_x.valueChanged.connect(self.update_vectors)
        self.slider_x.valueChanged.connect(self.applyForce)
        self.slider_y.valueChanged.connect(self.update_vectors)
        self.slider_z.valueChanged.connect(self.update_vectors)

        self.slider_x.valueChanged.connect(self.publish_vectors)
        self.slider_y.valueChanged.connect(self.publish_vectors)
        self.slider_z.valueChanged.connect(self.publish_vectors)

        self.setLayout(layout)

    # Slot function to update the label with the current slider value
    def update_label(self):
        value = self.slider_x.value()
        self.label_x.setText(f"Slider Value: {value}")

    def update_vectors(self):
        # float64 x
        # float64 y
        # float64 z

        vector_in = Vector3()
        vector_in.x = self.slider_x.value()*1000
        vector_in.y = self.slider_y.value()
        vector_in.z = self.slider_z.value()

        self.wrench_in.wrench.force = vector_in


    def publish_vectors(self):
        thread = Thread(target = self.force_pub_thread)

        thread.start()
        thread.join()

    def force_pub_thread(self):
        
        pub = rospy.Publisher("/gazebo_force_app", WrenchStamped, queue_size=10)

        pub.publish(self.wrench_in)

    def applyForce(self): 
        rospy.wait_for_service('/gazebo/apply_body_wrench') 
        force = rospy.ServiceProxy('/gazebo/apply_body_wrench',ApplyBodyWrench)

        wrench          = WrenchStamped()
        wrench.force.x  = self.slider_x.value()
        wrench.force.y  = 0
        wrench.force.z  = 0
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = 0
        # You can also define the start time if necessary... 
        force(body_name = "bvr_SIM/main_arm_SIM/wrist_3_link",wrench = wrench, duration = rospy.Duration(10))





def main():
    app = QApplication(sys.argv)

    rospy.init_node("force_qt_pub")

    print(":)", rospkg.get_ros_root())

    window = SliderApp()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

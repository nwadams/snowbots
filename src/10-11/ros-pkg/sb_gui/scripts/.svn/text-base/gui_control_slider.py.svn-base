#!/usr/bin/env python
# this code should be able to drive robot by changing the slider value
# from GUI, it does not listen to any of the sensors for now...
# therefore, when testing with robot, make sure the car is not on the ground
# slider1.py

LOOP_FREQ = 10.0 # Hz
CAR_PUBLISH_TOPIC = "servo_command"
TURRET_PUBLISH_TOPIC = "turret_command"
NODE_NAME = "GUICommander"

car_throttle =0
car_steer = 0
estop_status = True
turret_pan = 0
turret_pan = 0


import roslib; roslib.load_manifest('sb_commander')
from sb_msgs.msg import CarCommand
from sb_msgs.msg import TurretCommand
import rospy
import sys
from geometry_msgs import Twist,Vector3
from PyQt4 import QtGui
from PyQt4 import QtCore

car_pub = rospy.Publisher(CAR_PUBLISH_TOPIC, CarCommand)
turret_pub = rospy.Publisher(TURRET_PUBLISH_TOPIC, TurretCommand)

class Slider(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        rospy.init_node(NODE_NAME)
        rate = rospy.Rate(LOOP_FREQ)
        rospy.loginfo("ready to test GUI")


        self.setGeometry(300, 300, 250, 200)
        self.setWindowTitle('Servo Control GUI')

        self.slider1 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.slider1.setFocusPolicy(QtCore.Qt.NoFocus)
        self.slider1.setGeometry(80, 40, 100, 30)
        self.slider1.setValue(50)
        self.connect(self.slider1, QtCore.SIGNAL('valueChanged(int)'), self.changeValue)

        self.label1 = QtGui.QLabel('throttle', self)
        self.label1.move(15,50)


        self.slider2 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.slider2.setFocusPolicy(QtCore.Qt.NoFocus)
        self.slider2.setGeometry(80, 70, 100, 30)
        self.slider2.setValue(50)
        self.connect(self.slider2, QtCore.SIGNAL('valueChanged(int)'), self.changeValue)
        
        self.label2 = QtGui.QLabel('steering', self)
        self.label2.move(15,80)

        #above is Qt Slider for throttle and steer

        self.slider3 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.slider3.setFocusPolicy(QtCore.Qt.NoFocus)

        self.slider3.setGeometry(80, 100, 100, 30)
        self.slider3.setValue(50)
        self.connect(self.slider3, QtCore.SIGNAL('valueChanged(int)'), self.changeValue)
        
        self.label3 = QtGui.QLabel('pan', self)
        self.label3.move(15,110)


        self.slider4 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.slider4.setFocusPolicy(QtCore.Qt.NoFocus)

        self.slider4.setGeometry(80, 130, 100, 30)
        self.slider4.setValue(50)
        self.connect(self.slider4, QtCore.SIGNAL('valueChanged(int)'), self.changeValue)
        
        self.label4 = QtGui.QLabel('tilt', self)
        self.label4.move(15,140)

        #above is Qt Slider for camera turret pan and tilt

        self.estop = QtGui.QPushButton('Estop',self)
        self.estop.setCheckable(True)
        self.estop.move(90,15)
        self.connect(self.estop, QtCore.SIGNAL('clicked()'), self.setEstop)

    def setEstop(self):
        global estop_status
        if self.estop.isChecked():
            estop_status = not estop_status
            car_pub.publish(throttle = 0, steering = 0)
        else: estop_status = False
        

    def changeValue(self, value):
        global estop_status
        gui_pos = self.slider1.value()
        gui_pos1 = self.slider2.value()
        gui_pos2 = self.slider3.value()
        gui_pos3 = self.slider4.value()

        car_throttle = (gui_pos - 50) / 50.0
        car_steer = (gui_pos1 - 50) / 50.0
        turret_pan = (gui_pos2 - 50) / 50.0
        turret_tilt = (gui_pos3 - 50) / 50.0

        if estop_status:
            car_pub.publish(throttle = car_throttle, steering = car_steer)
        else: car_pub.publish(throttle = 0, steering = 0)

        turret_pub.publish(pan = turret_pan, tilt = turret_tilt)

class Twistmsg(geometry_msgs.Twist):
    def __init__(self,parent = None):
        geometry_msg.Twist.__init__(self,parent)
        twistreturn = geometry_msg.Twist
        twistreturn.linearVel = Vectormsg.setVector(throttle)
        twistreturn.angularVel = Vectormsg.setVector(steering)
        
        
        
class Vectormsg(geometry_msgs.Vector3):
    def __init__(self,parent = None):
        geometry_msgs.Vector3.__init__(self,parent)
        Vector3.x = 0
        Vector3.y = 0
        Vector3.z = 0        
        
    def setVecotr(self,value):
        Vector3.x = value


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    icon = Slider()
    icon.show()
    app.exec_()s

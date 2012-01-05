#!/usr/bin/env python
# this code should be able to drive robot by changing the textbox value
# from GUI, it does not listen to any of the sensors for now...
# therefore, when testing with robot, make sure the car is not on the ground
# slider1.py

LOOP_FREQ = 10.0 # Hz
CAR_PUBLISH_TOPIC = "servo_command"
TURRET_PUBLISH_TOPIC = "turret_command"
NODE_NAME = "GUICommander"

car_throttle =0
car_steer = 0
estop_status = 0
turret_pan = 0
turret_tilt = 0


import roslib; roslib.load_manifest('sb_commander')
from sb_msgs.msg import CarCommand
from sb_msgs.msg import TurretCommand
import rospy
import sys
from PyQt4 import QtGui
from PyQt4 import QtCore

car_pub = rospy.Publisher(CAR_PUBLISH_TOPIC, CarCommand)
turret_pub = rospy.Publisher(TURRET_PUBLISH_TOPIC, TurretCommand)

class GuiControl(QtGui.QWidget):
    def __init__(self,parent =None):
        QtGui.QWidget.__init__(self, parent)
        rospy.init_node(NODE_NAME)
        rate = rospy.Rate(LOOP_FREQ)
        rospy.loginfo("ready to go")

        self.setGeometry(300,300,350,200)
        self.setWindowTitle('GUI Driver')

        self.estop = QtGui.QPushButton('Estop',self)
        self.estop.setCheckable(True)
        self.estop.move(90,15)
        self.connect(self.estop, QtCore.SIGNAL('clicked()'), self.setEstop)

        self.button0 = QtGui.QPushButton('throttle',self)
        self.button0.setFocusPolicy(QtCore.Qt.NoFocus)
        self.button0.move(20,50)
        self.connect(self.button0, QtCore.SIGNAL('clicked()'),self.showValue0)
        self.setFocus()

        self.label0 = QtGui.QLineEdit(self)
        self.label0.move(130,52)

        self.button1 = QtGui.QPushButton('steering',self)
        self.button1.setFocusPolicy(QtCore.Qt.NoFocus)
        self.button1.move(20,80)
        self.connect(self.button1, QtCore.SIGNAL('clicked()'),self.showValue1)
        self.setFocus()

        self.label1 = QtGui.QLineEdit(self)
        self.label1.move(130,82)

#throttle and steer

        self.button2 = QtGui.QPushButton('pan',self)
        self.button2.setFocusPolicy(QtCore.Qt.NoFocus)
        self.button2.move(20,110)
        self.connect(self.button2, QtCore.SIGNAL('clicked()'),self.showValue2)
        self.setFocus()

        self.label2 = QtGui.QLineEdit(self)
        self.label2.move(130,112)

        self.button3 = QtGui.QPushButton('tilt',self)
        self.button3.setFocusPolicy(QtCore.Qt.NoFocus)
        self.button3.move(20,140)
        self.connect(self.button3, QtCore.SIGNAL('clicked()'),self.showValue3)
        self.setFocus()

        self.label3 = QtGui.QLineEdit(self)
        self.label3.move(130,142)

#pan and tilt

    def showValue0(self):
        text, ok = QtGui.QInputDialog.getText(self, 'throttle', 'Enter Value:')
        if ok:
            text_float = float(text)
            if text_float >= -1.0 and text_float <= 1.0 and estop_status:
                self.label0.setText(str(text))
                car_throttle = text_float
                car_pub.publish(throttle = car_throttle, steering = car_steer)

    def showValue1(self):
        text, ok = QtGui.QInputDialog.getText(self, 'steering', 'Enter Value:')
        if ok:
            text_float = float(text)
            if text_float >= -1.0 and text_float <= 1.0 and estop_status:
                self.label1.setText(str(text))
                car_steer = text_float
                car_pub.publish(throttle = car_throttle, steering = car_steer)

    def showValue2(self):
        text, ok = QtGui.QInputDialog.getText(self, 'pan', 'Enter Value:')
        if ok:
            text_float = float(text)
            if text_float >= -1.0 and text_float <= 1.0:
                self.label2.setText(str(text))
                turret_pan = text_float
                turret_pub.publish(pan = turret_pan, tilt = turret_tilt)

    def showValue3(self):
        text, ok = QtGui.QInputDialog.getText(self, 'tilt', 'Enter Value:')
        if ok:
            text_float = float(text)
            if text_float >= -1.0 and text_float <= 1.0:
                self.label3.setText(str(text))
                turret_pub.publish(pan = turret_pan, tilt = turret_tilt)

    def setEstop(self):
        global estop_status
        if self.estop.isChecked():
            estop_status = 0
            car_pub.publish(throttle = 0, steering = 0)
        else: estop_status = 1


app = QtGui.QApplication(sys.argv)
foo = GuiControl()
foo.show()
app.exec_()

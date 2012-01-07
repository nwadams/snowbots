#!/usr/bin/env python

"""
A graphical program for setting the calibration values in your config
file.

Publishes
=========
  - sb_msgs/ServoCommand on the "servo" topic

@author: Ian Phillips <ianfp@freeshell.org>
"""

PUBLISH_TOPIC = "servo"
NODE_NAME = "calibrator"

from optparse import OptionParser
import os
import sys
from ConfigParser import RawConfigParser

from PyQt4 import QtGui, QtCore

import roslib
roslib.load_manifest("sb_config")
import rospy

from sb_msgs.msg import ServoCommand
from sb_config import find_config_file

class ServoControl(QtGui.QWidget):
  """
  A Qt Widget for adjusting the value of a servo and setting its
  minimum, center, and maximum values.
  """
  # Servos connected to the Furious board take PPM signal values 
  # between 0 and 500.
  SLIDER_MIN = 0
  SLIDER_MAX = 500
  SLIDER_STEP = 5

  def __init__(self, parent, config, name):
    QtGui.QWidget.__init__(self, parent)
    self.name = name
    self.config = config
    self.servo_id = config.getint(self.name, "id")
    current_pos = config.getint(self.name, "center")

    # A servo config control widget (this class) has several sub-widgets:
    # - an LCD display that shows the current value of the slider
    lcd = QtGui.QLCDNumber(3)
    lcd.setSegmentStyle(QtGui.QLCDNumber.Flat)
    # set the LCD to display the current center value (if any)
    lcd.display(current_pos)

    # - a slider to control the value sent to the servo 
    self.slider = QtGui.QSlider(QtCore.Qt.Horizontal)
    self.slider.setRange(ServoControl.SLIDER_MIN, ServoControl.SLIDER_MAX)
    self.slider.setSingleStep(ServoControl.SLIDER_STEP)
    self.slider.setValue(current_pos)
    self.connect(self.slider, QtCore.SIGNAL('valueChanged(int)'),
      lcd, QtCore.SLOT('display(int)'))
    self.connect(self.slider, QtCore.SIGNAL('valueChanged(int)'),
        self.onSliderChange)

    # - buttons for settin the min, center, and max values of the servo
    minLbl = QtGui.QLabel( "min" )
    self.minBtn = QtGui.QPushButton()
    self.connect(self.minBtn, QtCore.SIGNAL('clicked()'), 
        self.onMinClick)

    ctrLbl = QtGui.QLabel( "center" )
    self.ctrBtn = QtGui.QPushButton()
    self.connect(self.ctrBtn, QtCore.SIGNAL('clicked()'), 
        self.onCtrClick)

    maxLbl = QtGui.QLabel( "max" )
    self.maxBtn = QtGui.QPushButton()
    self.connect(self.maxBtn, QtCore.SIGNAL('clicked()'), 
        self.onMaxClick)

    # - labels for showing the name and index of the servo
    servoLbl = QtGui.QLabel(self.name)
    servoNoLbl = QtGui.QLabel(str(self.servo_id))

    layout = QtGui.QGridLayout()
    layout.setColumnMinimumWidth(0, 50)
    layout.setColumnMinimumWidth(1, 200)
    layout.setColumnMinimumWidth(2, 100)
    layout.setColumnMinimumWidth(3, 100)
    layout.setColumnMinimumWidth(4, 100)
    layout.addWidget(servoLbl,    0, 0);
    layout.addWidget(servoNoLbl,  1, 0);
    layout.addWidget(lcd,         0, 1);
    layout.addWidget(self.slider, 1, 1);
    layout.addWidget(minLbl,      0, 2);
    layout.addWidget(self.minBtn, 1, 2);
    layout.addWidget(ctrLbl,      0, 3);
    layout.addWidget(self.ctrBtn, 1, 3);
    layout.addWidget(maxLbl,      0, 4);
    layout.addWidget(self.maxBtn, 1, 4);
    self.setLayout(layout);

  def onMinClick(self):
    val = self.slider.value()
    self.config.set(self.name, "min", val)
    self.minBtn.setText(str(val))

  def onCtrClick(self):
    val = self.slider.value()
    self.config.set(self.name, "center", val)
    self.ctrBtn.setText(str(val))

  def onMaxClick(self):
    val = self.slider.value()
    self.config.set(self.name, "max", val)
    self.maxBtn.setText(str(val))

  def onSliderChange(self, val):
    self.parentWidget().updateBoardState(self.servo_id, val)

###  end ServoControl  ###



class CalibrationScreen(QtGui.QWidget):
  """
  The main window in this application.
  """
  def __init__(self, filename):
    QtGui.QWidget.__init__(self, None)

    # Create main layout object
    layout = QtGui.QVBoxLayout()

    # Load config file and add widgets for each servo
    self.filename = filename
    self.config = RawConfigParser()
    cfg = self.config   # for convenience
    cfg.read(filename)
    for section in cfg.sections():
      # Detect servos in the config file
      parts = section.split(".")
      if parts[0] == "servo":
        sc = ServoControl(self, cfg, section)
        layout.addWidget(sc)

    # Add save and quit buttons to layout
    quitBtn = QtGui.QPushButton("Quit")
    self.connect(quitBtn, QtCore.SIGNAL('clicked()'), 
        QtGui.qApp, QtCore.SLOT('quit()'))

    saveBtn = QtGui.QPushButton("Save")
    self.connect(saveBtn, QtCore.SIGNAL('clicked()'), self.saveConfig)

    buttons = QtGui.QHBoxLayout()
    buttons.addWidget(saveBtn)
    buttons.addWidget(quitBtn)

    layout.addLayout(buttons)
    self.setLayout(layout)

    # Create a ROS publisher for sending the slider values to the servos
    self.publisher = rospy.Publisher(PUBLISH_TOPIC, ServoCommand)
    rospy.init_node(NODE_NAME)

  def updateBoardState(self, servo_id, val):
    msg = ServoCommand()
    msg.id = servo_id
    msg.pwm = val
    self.publisher.publish(msg)

  def saveConfig(self):
    self.config.write(open(self.filename, "w"))
    QtGui.QMessageBox.information(
        self,
        "Message",
        "Your configuration has been saved to %s." % self.filename,
        QtGui.QMessageBox.Ok )

###  end CalibrationScreen  ###




def main():
  app = QtGui.QApplication(rospy.myargv())

  parser = OptionParser()
  parser.add_option("--config-file", "-c")
  options, args = parser.parse_args(rospy.myargv())
  cfgfile = find_config_file(options.config_file)
  if not cfgfile:
    rospy.logfatal("Unable to find config file")
    exit(1)
  rospy.loginfo("Calibrating file %s", cfgfile)

  screen = CalibrationScreen(cfgfile)
  screen.show()
  retval = app.exec_()
  return retval


if __name__ == "__main__":
  main()

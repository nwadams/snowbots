#!/usr/bin/env python

"""
A graphical program for setting the calibration values in your config
file for furiousDriver.py.

Usage:
  %s config_file [-p PORT]
Command-line arguments:
  config_file - (required)  The config file to edit.
  -p PORT     - connect to the board on port PORT
"""

import os
import sys

from PyQt4 import QtGui, QtCore

from furious.devices import FuriousBoard
import furious.config


class ServoControl(QtGui.QWidget):
  """
  A Qt Widget for adjusting the value of a servo and setting its
  minimum, center, and maximum values.
  """
  # Servos connected to the Furious board take PPM signal values 
  # between 0 and 500.
  SLIDER_MIN = 0
  SLIDER_MAX = 500
  SLIDER_STEP = 10

  S_ID = 0
  S_MIN = 1
  S_MID = 2
  S_MAX = 3
  def __init__(self, parent, servo_name, servo_cfg):
    QtGui.QWidget.__init__(self, parent)
    self.name = servo_name
    self.servo = servo_cfg

    # A servo config control widget (this class) has several sub-widgets:
    # - an LCD display that shows the current value of the slider
    lcd = QtGui.QLCDNumber(3)
    lcd.setSegmentStyle(QtGui.QLCDNumber.Flat)
    # set the LCD to display the current center value (if any)
    lcd.display(self.servo[ServoControl.S_MID])

    # - a slider to control the value sent to the servo 
    self.slider = QtGui.QSlider(QtCore.Qt.Horizontal)
    self.slider.setRange(ServoControl.SLIDER_MIN, ServoControl.SLIDER_MAX)
    self.slider.setSingleStep(ServoControl.SLIDER_STEP)
    self.slider.setValue(self.servo[ServoControl.S_MID])
    self.connect(self.slider, QtCore.SIGNAL('valueChanged(int)'),
      lcd, QtCore.SLOT('display(int)'))
    self.connect(self.slider, QtCore.SIGNAL('valueChanged(int)'),
        self.onSliderChange)

    # - buttons for settin the min, center, and max values of the servo
    minLbl = QtGui.QLabel( self.getMinLabel() )
    self.minBtn = QtGui.QPushButton()
    self.connect(self.minBtn, QtCore.SIGNAL('clicked()'), 
        self.onMinClick)

    ctrLbl = QtGui.QLabel( self.getCenterLabel() )
    self.ctrBtn = QtGui.QPushButton()
    self.connect(self.ctrBtn, QtCore.SIGNAL('clicked()'), 
        self.onCtrClick)

    maxLbl = QtGui.QLabel( self.getMaxLabel() )
    self.maxBtn = QtGui.QPushButton()
    self.connect(self.maxBtn, QtCore.SIGNAL('clicked()'), 
        self.onMaxClick)

    # - labels for showing the name and index of the servo
    servoLbl = QtGui.QLabel(self.name)
    servoNoLbl = QtGui.QLabel(str(self.servo[ServoControl.S_ID]))

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
    self.servo[ServoControl.S_MIN] = val
    self.minBtn.setText(str(val))

  def onCtrClick(self):
    val = self.slider.value()
    self.servo[ServoControl.S_MID] = val
    self.ctrBtn.setText(str(val))

  def onMaxClick(self):
    val = self.slider.value()
    self.servo[ServoControl.S_MAX] = val
    self.maxBtn.setText(str(val))

  def onSliderChange(self, val):
    self.parentWidget().updateBoardState(self.servo[ServoControl.S_ID], val)

  def getMinLabel(self):
    """
    Try to provide an intelligent, context-sensitive label for the
    servo's "min" value.
    """
    if self.nameMatches(["steer", "pan"]):
      return "Left"
    if self.nameMatches(["throt", "drive", "track", "tread"]):
      return "Reverse"
    if self.nameMatches(["tilt"]):
      return "Down"
    return "Min"

  def getCenterLabel(self):
    """
    Try to provide an intelligent, context-sensitive label for the
    servo's "center" value.
    """
    if self.nameMatches(["steer", "pan"]):
      return "Straight"
    if self.nameMatches(["throt", "drive", "track", "tread"]):
      return "Stop"
    return "Center"

  def getMaxLabel(self):
    """
    Try to provide an intelligent, context-sensitive label for the
    servo's "max" value.
    """
    if self.nameMatches(["steer", "pan"]):
      return "Right"
    if self.nameMatches(["throt", "drive", "track", "tread"]):
      return "Forward"
    if self.nameMatches(["tilt"]):
      return "Up"
    return "Max"

  def nameMatches(self, match_list):
    """
    Helper method for get___Label() methods.
    @param match_list: a list of servo name fragments to compare against
    @return: True if the name of this servo matches any of the fragments
      in match_list
    """
    for match in match_list:
      if self.name.lower().find(match) >= 0:
        return True
    return False

###  end ServoControl  ###


class CalibrationScreen(QtGui.QWidget):
  """
  The main window in this application.
  """
  def __init__(self, cfgfile, port):
    QtGui.QWidget.__init__(self, None)
    self.config_file = cfgfile
    self.config = furious.config.parse_config(self.config_file)
    self.board = FuriousBoard()
    if not port:
      port = self.board.probe()
    if not port:
      QtGui.QMessageBox.information(
          self,
          "Error",
          "Unable to detect a Furious board.",
          QtGui.QMessageBox.Ok )
      QtGui.qApp.exit(1)
      # For some reason, the app doesn't exit immediately after the call
      # to qApp.exit() above, so this next line is needed for a clean
      # shutdown.
      exit(1)  

    self.board.open(port)

    if not self.board.is_ready():
      QtGui.QMessageBox.information(
          self,
          "Error",
          "Unable to connect to Furious board on %s." % port,
          QtGui.QMessageBox.Ok )
      QtGui.qApp.exit(1)

    quitBtn = QtGui.QPushButton("Quit")
    self.connect(quitBtn, QtCore.SIGNAL('clicked()'), 
        QtGui.qApp, QtCore.SLOT('quit()'))

    saveBtn = QtGui.QPushButton("Save")
    self.connect(saveBtn, QtCore.SIGNAL('clicked()'), self.saveConfig)

    buttons = QtGui.QHBoxLayout()
    buttons.addWidget(saveBtn)
    buttons.addWidget(quitBtn)

    layout = QtGui.QVBoxLayout()

    statusLbl = QtGui.QLabel("Connected on %s" % port)
    layout.addWidget(statusLbl)
    for sname in self.config['servo']:
      servo = self.config['servo'][sname]
      sc = ServoControl(self, sname, servo)
      layout.addWidget(sc)
    layout.addLayout(buttons)
    self.setLayout(layout)

  def __del__(self):
    self.board.close()

  def updateBoardState(self, servo_id, val):
    self.board.servos[int(servo_id)] = val
    self.board.update()

  def saveConfig(self):
    furious.config.save_config(self.config, self.config_file)
    QtGui.QMessageBox.information(
        self,
        "Message",
        "Your configuration has been saved to %s." % self.config_file,
        QtGui.QMessageBox.Ok )

###  end CalibrationScreen  ###




def main():
  app = QtGui.QApplication(sys.argv[:])

  cfgfile = None
  port = None
  args = sys.argv[1:]
  while len(args) > 0:
    arg = args.pop(0)
    if arg == "-p":
      port = args.pop(0)
    elif arg.find("help") >= 0:
      print __doc__ % sys.argv[0]
      exit(0)
    else:
      cfgfile = arg

  if not cfgfile:
    print >>sys.stderr, "Missing command line argument: config_file"
    print >>sys.stderr, __doc__ % sys.argv[0]
    exit(1)

  screen = CalibrationScreen(cfgfile, port)
  screen.show()
  retval = app.exec_()
  return retval


if __name__ == "__main__": main()

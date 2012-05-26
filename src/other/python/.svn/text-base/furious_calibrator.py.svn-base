#!/usr/bin/env python

"""
A graphical program for calibrating the servos in your pyFurious.py
configuration file.  See also:
  pyFurious.py
  pyFuriousConfig.py
  robot.cfg.example

Usage:
  %s config_file
Command-line arguments:
  config_file - (required)  The config file to edit.
"""

import sys
from PyQt4 import QtGui, QtCore
from pyFurious import FuriousBoard
import pyFuriousConfig

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
    minLbl = QtGui.QLabel("Min")
    self.minBtn = QtGui.QPushButton()
    self.connect(self.minBtn, QtCore.SIGNAL('clicked()'), 
        self.onMinClick)

    ctrLbl = QtGui.QLabel("Center")
    self.ctrBtn = QtGui.QPushButton()
    self.connect(self.ctrBtn, QtCore.SIGNAL('clicked()'), 
        self.onCtrClick)

    maxLbl = QtGui.QLabel("Max")
    self.maxBtn = QtGui.QPushButton()
    self.connect(self.maxBtn, QtCore.SIGNAL('clicked()'), 
        self.onMaxClick)

    # - labels for showing the name and index of the servo
    servoLbl = QtGui.QLabel(servo_name)
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


class CalibrationScreen(QtGui.QWidget):
  """
  The main window in this application.
  """
  def __init__(self, cfgfile):
    QtGui.QWidget.__init__(self, None)
    self.config_file = cfgfile
    self.config = pyFuriousConfig.parseConfig(self.config_file)
    self.board = FuriousBoard()
    self.board.open(self.config['port'])
    if not self.board.isReady():
      QtGui.QMessageBox.information(
          self,
          "Error",
          "Unable to connect to Furious board on %s." % self.config['port'],
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

    statusLbl = QtGui.QLabel("Connected on %s" % self.config['port'])
    layout.addWidget(statusLbl)
    for sname in self.config['servo']:
      servo = self.config['servo'][sname]
      sc = ServoControl(self, sname, servo)
      layout.addWidget(sc)
    layout.addLayout(buttons)
    self.setLayout(layout)

  def updateBoardState(self, servo_id, val):
    self.board.setServo(int(servo_id), val)
    self.board.update()

  def saveConfig(self):
    pyFuriousConfig.saveConfig(self.config, self.config_file)
    QtGui.QMessageBox.information(
        self,
        "Message",
        "Your configuration has been saved to %s." % self.config_file,
        QtGui.QMessageBox.Ok )




def main():
  app = QtGui.QApplication(sys.argv[:])

  if len(sys.argv) < 2:
    print "Missing command line argument: config_file"
    print __doc__ % sys.argv[0]
    exit(1)

  cfgfile = sys.argv[1]
  screen = CalibrationScreen(cfgfile)
  screen.show()
  retval = app.exec_()
  return retval


if __name__ == "__main__": main()

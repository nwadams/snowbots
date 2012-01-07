"""
Main class for the L{furiousDriver.py} node.

@author: Ian Phillips
"""

import errno
import os
import select
import time

from furious.config import parse_config
from furious.devices import Battery, FuriousBoard, Odometer, Servo


class FuriousDriver(object):
  """
  This is the high-level class that encapsulates the devices from 
  L{furious.devices} and translates raw values from the Furious
  board into units that are meaningful to humans.
  """

  def __init__(self, config_file=None):
    """
    The constructor does some default configuration, but the driver
    usually needs to load a config file via L{load_config} to be 
    fully initialized.
    """
    self._board = FuriousBoard()
    self._servos = {}
    self._odometer = Odometer(self._board)
    self._logic_batt = Battery(self._board, port=7)
    self._motor_batt = Battery(self._board, port=6)
    self._dbg = None  # Observer object for debug messages

    if config_file:
      self.load_config(config_file)


  ### configuration ###

  def load_config(self, filename):
    """
    Configures the robot according to the settings in the given
    configuration file.
    @param filename: a filename containing configuration settings
    @type filename: string
    @return: None
    """
    # parse_config() takes care of the messiness of reading and cleaning
    # up the config file.
    config = parse_config(filename)
    for key in config:
      if key == "servo":
        for servo_name in config[key]:
          servo  = config[key][servo_name]
          id     = servo[0]
          min    = servo[1]
          center = servo[2]
          max    = servo[3]
          self._servos[servo_name] = Servo(self._board, id, min, center, max)

      elif key == "odometer":
        self._odometer = Odometer(self._board, config[key])

      # TODO: add battery level customizations


  ### connecting, disconnecting, etc ###

  def connect(self, port):
    """
    Attempts to establish a serial connection to the Furious board on
    the specified port (eg, "/dev/ttyACM0").
    """
    self._board.open(port)
    time.sleep(0.5)
    self.refresh()
    time.sleep(0.5)
    self.refresh()

  def disconnect(self):
    """
    Immediately stops all servos and closes the board's serial connection.
    """
    cmd = self.stop()
    self._board.close()
    return cmd

  def probe(self):
    """
    Probes the computer for a Furious board and returns the port that has
    one attached to it.  Returns None if no board can be found.  Note that 
    this does not connect the driver to the board; use L{open()} for that.
    """
    return self._board.probe()


  def is_ready(self):
    """
    Returns true if the robot is ready to accept commands.
    """
    return self._board.is_ready()


  ###  communicating with the board ###

  def stop(self):
    """
    Immediately stops all servos.  Good for emergencies and when you 
    reach your goal (or the end of your program).
    @return: the serial command string that was sent on this update,
      for debugging purposes
    """
    for i in range(len(self._board.servos)):
      self._board.servos[i] = 0
    return self.refresh()

  def refresh(self):
    """
    Sends all pending commands to and receives all updates from
    the board.
    @return: the serial command string that was sent on this update,
      for debugging purposes
    """
    # Update the board
    sent = ""
    try:
      sent = self._board.update()
    # Handle the case when a read or write is interrupted.
    except OSError, e:
      self.stop()
      if e.errno != errno.EINTR:  # if not interrupted system call
        raise e  # propagate the exception
    except select.error, e:
      self.stop()
      if e[0] != errno.EINTR:
        raise e

    # Update Odometer object with new values from the board
    self._odometer.update()
    return sent


  ###  Message handling  ###

  def on_servo_command(self, msg):
    """
    Callback when a servo command message is received.  Updates the
    appropriate servo with the new value.
    @param msg: a L{ServoCommand} message object
    """
    servo = self._servos[msg.servo]
    if not servo:
      raise ValueError("No servo named %s has been added" % msg.servo)
    servo.set(msg.value)

  def get_state(self, msg):
    """
    Populates the given FuriousState message with the current state values.
    @param msg: a L{FuriousState} message object
    @return: the updated L{FuriousState} message
    """
    msg.portname = self._board.get_port_name()
    for i in range(6):
      msg.analog[i] = self.get_analog(i)
      msg.ir[i] = self.get_ir(i)
    msg.logic_battery = self._logic_batt.get_charge()
    msg.motor_battery = self._motor_batt.get_charge()
    msg.odometer = self._odometer.get_distance()
    return msg

  def get_analog(self, analog_id):
    """
    Returns the voltage (between 0 and 5.0v) coming from the specified 
    analog port as a float.
    """
    # Analog values from from the board as integers between 0 and 1024.
    # 0 means zero volts; 1024 means 5.0 volts.
    return (self._board.state[analog_id] * 5.0) / 1024

  def get_ir(self, ir_id):
    """
    Returns the distance from the IR sensor on the specified port as
    a float.
    """
    # The raw values returned by the sensor describe an exponential
    # curve with respect to distance.  This 4th-order polynomial equation
    # roughly fits that response curve.  (Credit: Matt Baumann)
    val = self.get_analog(ir_id)
    return (
      (0.2006 * (val**4)) -
      (1.4103 * (val**3)) +
      (3.5853 * (val**2)) -
      (4.0081 *  val)     +
       1.8929 )

 
  ###  utility methods  ###

  def set_debugger(self, dbg):
    """
    Sets the debugger file object (eg, stdout), to which calls to 
    _debug() will be written.
    """
    self._dbg = dbg
    self._board.set_debugger(dbg)
    self._debug("Debugging enabled in %s" % self.__class__.__name__)

  def _debug(self, msg):
    if self._dbg:
      print >>self._dbg, msg

###  end FuriousDriver  ###


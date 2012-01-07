"""
This module contains helper classes for L{furious.driver.FuriousDriver}.
L{FuriousBoard} handles the serial communication with the microcontroller,
while the others represent devices on the robot and translate raw values
from the board into meaningful units (eg, volts, meters).

@author: Ian Phillips
"""

import os
import serial

from util import clamp, interpolate, saturate

SCAN_PORTS = []
"""
The ports that should be scanned for the Furious board.  This depends
on the operating system.
"""
if os.name == "posix":
  SCAN_PORTS = ["/dev/ttyACM%i" % _i for _i in xrange(9, -1, -1)]


FURIOUS_COMMAND_DELIM = "G"
"""
Character that delimits the end of a serial command or response.
"""


class FuriousBoard(object):
  """
  This is the low-level driver that communicates with the Furious
  microcontroller board over a serial (USB) connection.  For more 
  information on the Furious board, see the inventor's website at: 

  U{http://ashleymckay.com}
  """

  def __init__(self):
    """
    Constructor.
    """
    # The serial port for communication with the physical board.
    self._port = serial.Serial()
    self._port.baudrate = 19200 # baud
    self._port.timeout = 0.5 # seconds

    # These variables hold the command values that will be sent
    # to the board the next time update() is called.
    self.i2c_poll = 0
    self.servos = [0 for x in range(8)] 

    # This list holds the state values returned by the board.  See
    # the accessor methods below for the interpretation of these values.
    self.state = [0 for x in range(21)]

    # Observer object for debug messages
    self._dbg = None

  def __del__(self):
    """
    Destructor.  Closes the serial connection in case you forgot to call
    L{close()} yourself.  Don't forget!
    """
    self.close()

  def open(self, port):
    """
    Opens a connection to the given port.
    @param port: the port name to which the Furious board is attached (eg, 
      "/dev/ttyACM0").
    @type port: string
    """
    if not self.is_valid_port(port):
      raise ValueError("Port '%s' is not valid." % port)
    if self._port.isOpen():
      self._port.close()
    self._port.port = port
    self._port.open()
    self._port.flushInput()
    self._port.flushOutput()

  def probe(self):
    """
    Probes the computer for a Furious board and returns the port that has
    one attached to it.  Returns None if no board can be found.  Note that 
    this does not connect the driver to the board; use L{open()} for that.
    """
    device = None
    for port in SCAN_PORTS:
      self._debug("Scanning on port %s" % port)
      if self.is_valid_port(port):
        try:
          self.open(port) # Try to open device
          self.update()   # Attempt handshake with Furious board
        except serial.serialutil.SerialException, e:
          self._debug("Unable to open %s.  Reason: %s" % (port, e))
        else:
          self._debug("Furious device found on port %s" % port)
          device = port
          break
    self.close()
    return device

  def is_valid_port(self, port):
    """
    Returns True if the given port is a valid port that a Furious board
    might be connected to.  This does NOT mean that a board is necessarily
    there.
    @param port: the port name to be checked
    @return: True if the given port is a potentially valid port, False
      otherwise
    """
    return port and os.path.exists(port)

  def close(self):
    """
    Closes the serial connection to the board.
    """
    self._port.close()

  def is_ready(self):
    """
    @return: True if the serial connection to the board is open, False
      otherwise
    """
    return self._port.isOpen()

  def update(self):
    """
    Sends any pending commands to the board and receives all state
    updates.
    @return: the serial command string that was sent on this update,
      for debugging purposes
    """
    cmd = self._get_command_string()
    # It is necessary to send the string one byte at a time, due
    # to spooky low-level byte-alignment issues.  Ouch.
    for c in cmd:
      self._port.write(c)

    reply = ""
    c = self._port.read(1)
    while c != FURIOUS_COMMAND_DELIM:
      # Filter out illegal characters.  The package brltty can add
      # illegal characters to the output, so you should uninstall it
      # if you have problems.
      if c in ",0123456789-":
        reply += c
      c = self._port.read(1)

    # Chop the reply into parts and convert them to integers.
    parts = reply.split(",")
    parts.pop()  # pop off the blank due to trailing comma
    self.state = [ int(x.strip()) for x in parts ]
    return cmd

  def get_port_name(self):
    """
    @return: the name of the port to which the Furious board is attached
    """
    return self._port.portstr

  def _get_command_string(self):
    """
    Takes the pending commands and returns a string that can be
    sent over the serial connection to command the board.
    @return: a command string in the format expected by the Furious board
    """
    parts = []
    parts.append(   str(self.i2c_poll) )
    # These four digits are for future features, currently unused 
    parts.extend( [ "0" for x in range(4) ] )
    parts.extend( [ str(x) for x in self.servos] )
    parts.append( FURIOUS_COMMAND_DELIM )
    return ",".join(parts)

  def set_debugger(self, dbg):
    """
    Enables internal debugging output.
    @param dbg: The writable file object (eg, sys.stdout) to which 
      debug statements will be written.
    @type dbg: file
    """
    self._dbg = dbg
    self._debug("Debugging enabled in %s" % self.__class__.__name__)

  def _debug(self, msg):
    """
    Writes the given message to the debug file (if any).
    @param msg: a debug message
    @type msg: string
    """
    if self._dbg:
      print >>self._dbg, msg
        
###  end FuriousBoard  ###




class Servo(object):
  """
  Represents a single servo and its configuration parameters.
  """
  def __init__(self, board, id, min, center, max):
    """
    Constructor.
    @param board: the L{FuriousBoard} instance to which this servo is
      connected.
    @param id: the physical port that this servo is connected to on the board.
    @param min: the minimum raw PPM signal value that is within the servo's
      range.  For example, on a throttle, this would correspond to full 
      reverse.
    @param center: the center PPM value of this servo's range.  For example,
      on a throttle, this would correspond to stopped.
    @param max: the maximum PPM value in this servo's range.  For example,
      on a throttle, this would correspond to full forward.
    """
    self.board = board
    self.id = id
    self.min = min
    self.center = center
    self.max = max

  def set(self, percentage):
    """
    Updates this servo's port on the board with the raw PPM signal value
    that corresponds to the given percentage value.
    @param percentage: a servo setting between -100 and 100
    """
    percentage = clamp(percentage, -100, 100)
    raw_ppm = 0
    if percentage < 0:
      raw_ppm = int(interpolate(percentage, -100, 0, self.min, self.center))
    else:
      raw_ppm = int(interpolate(percentage, 0, 100, self.center, self.max))
    self.board.servos[self.id] = raw_ppm

###  end Servo  ###





class Battery(object):
  """
  Represents a battery and defines its analog port and charge limits (eg,
  full, empty).
  """
  def __init__(self, board, port, full=150, empty=120):
    """
    Constructor.
    @param board: the L{FuriousBoard} instance to which this battery is
      connected
    @param port: the analog port id on which this battery's level is
      monitored
    @param full: the raw signal value that indicates full battery
    @param empty: the raw signal value that indicates empty battery
    """
    self.board = board
    self.port = port
    self.full = full
    self.empty = empty

  def get_charge(self):
    """
    Converts the given raw value to a percentage.
    @param raw_value: the raw signal value to be converted to a percentage
    @return: the raw input value as a percentage (100 = full, 0 = empty)
    """
    raw_value = self.board.state[self.port]
    return int(saturate(raw_value, self.empty, self.full, 0, 100))

###  end Battery  ###




class Odometer(object):
  """
  Represents an odometer and defines the number of meters that
  the robot travels per "tick" of the odometer.
  """

  def __init__(self, board, meters_per_tick=0.33):
    """
    Constructor.
    @param board: the L{FuriousBoard} instance to which this odometer
      is connected
    @param meters_per_tick: the number of meters the vehicle has traveled
      each time the odometer registers a tick
    """
    self.board = board
    self.port = 9  # the odometer port number on Furious
    self.num_ticks = 0
    self.meters_per_tick = meters_per_tick

  def update(self):
    """
    Reads the number of new ticks (eg, wheel rotations) that the board
    has registered since the last update.
    """
    self.num_ticks += self.board.state[self.port]

  def get_distance(self):
    """
    @return: the total distance, in meters, that this odometer has measured
      so far.
    """
    return self.num_ticks * self.meters_per_tick

###  end Odometer  ###


"""
This module contains the L{FuriousBoard} class that handles communication
with the Furious microcontroller.  For more information on the Furious
microcontroller, go to the inventor's website at

U{http://ashleymckay.com/ashleymckay/cms/?q=node/195}

@author: Ian Phillips <ianfp@freeshell.org>
"""

from ConfigParser import RawConfigParser
import os
import select
import serial
import sys
from sb_util.range import saturate

SCAN_PORTS = []
"""
The ports that should be scanned for the Furious board.  This depends
on the operating system.
"""
if os.name == "posix":
  SCAN_PORTS = ["/dev/ttyACM%i" % _i for _i in xrange(9, -1, -1)]


class FuriousBoard(object):
  """
  This is the driver class that communicates with the Furious
  microcontroller board over a serial USB connection.
  """

  SERVO_SHIM_INDEX = 2
  SERVO_SHIM_VALUE = 426
  NUM_ANALOG = 6
  NUM_SERVOS = 8
  NUM_BATTERIES = 2
  MOTOR_BATTERY_INDEX = 6
  LOGIC_BATTERY_INDEX = 7
  ODOMETER_INDEX = 9

  COMMAND_DELIM = "G"
  """
  Character that delimits the end of a serial command or response.
  """

  ###  Constructor, destructor  ###

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
    self.servos = [0 for x in range(FuriousBoard.NUM_SERVOS)]

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


  ###  Connecting, disconnecting  ###

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


  ###  Updating  ###

  def update(self):
    """
    Sends any pending commands to the board and receives all state
    updates.
    @return: the serial command string that was sent on this update,
      for debugging/output purposes
    """
    cmd = self._get_command_string()
    # It is necessary to send the string one byte at a time, due
    # to spooky low-level byte-alignment issues.  Ouch.
    try:
      for c in cmd:
        self._port.write(c)
    except select.error:
      self.reset()
      return "Error while writing: %s" % select.error

    reply = ""
    try:
      c = self._port.read(1)
      while c != FuriousBoard.COMMAND_DELIM:
        # Filter out illegal characters.  The package brltty can add
        # illegal characters to the output, so you should uninstall it
        # if you have problems.
        if c in ",0123456789-":
          reply += c
        c = self._port.read(1)
    except select.error:
      self.reset()
      return "Error while reading: %s" % select.error

    # Chop the reply into parts and convert them to integers.
    parts = reply.split(",")
    parts.pop()  # pop off the blank due to trailing comma
    self.state = [ int(x.strip()) for x in parts ]
    return cmd

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
    # Shim to fix firmware bug
    self.servos[FuriousBoard.SERVO_SHIM_INDEX] = FuriousBoard.SERVO_SHIM_VALUE
    parts.extend( [ str(x) for x in self.servos] )
    parts.append( FuriousBoard.COMMAND_DELIM )
    return ",".join(parts)

  def reset(self):
    """
    Sends a zero to all servos.
    @return: the command message sent
    """
    for i in range(FuriousBoard.NUM_SERVOS):
      self.servos[i] = 0
    return self.update()


  ###  Accessors and Mutators  ###

  def set_servo(self, servo_id, ppm):
    self.servos[servo_id] = ppm


  def get_analog(self, analog_id):
    """
    Returns the voltage (between 0 and 5.0v) coming from the specified 
    analog port as a float.
    """
    # Analog values from from the board as integers between 0 and 1024.
    # 0 means zero volts; 1024 means 5.0 volts.
    return (self.state[analog_id] * 5.0) / 1024

  def get_motor_battery(self):
    return self.state[FuriousBoard.MOTOR_BATTERY_INDEX]

  def get_logic_battery(self):
    return self.state[FuriousBoard.LOGIC_BATTERY_INDEX]

  def get_odometer(self):
    return self.state[FuriousBoard.ODOMETER_INDEX]
  
  def get_port_name(self):
    """
    @return: the name of the port to which the Furious board is attached
    """
    return self._port.portstr


  ###  Debugging  ###

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


class ServoError(Exception):
  """
  Base exception class for L{Servo}.
  """
  pass



class Servo(object):
  """
  Handles reading a servo configuration section of a config file
  and converting servo position values (floats in the -1.0 to 1.0 range)
  into raw PWM integer values.
  """

  def __init__(self, name):
    """
    @param name: The name of this servo.  This determines which
    servo section of the config file L{load_config} will read.
    """
    self._name = name
    self._id = None
    self._min = self._center = self._max = 0

  def load_config(self, filename):
    """
    Loads configuration settings from the file whose name is given.
    It will read the section called "servo.[name]", where [name] is
    the name of this servo as given to the constructor.
    @param filename: the name of the config file to be read
    """
    cfg = RawConfigParser();
    cfg.read(filename)
    section = "servo.%s" % self._name
    self._id = cfg.getint(section, "id")
    self._min = cfg.getint(section, "min")
    self._center = cfg.getint(section, "center")
    self._max = cfg.getint(section, "max")

  def get_id(self):
    """
    @return: the ID of this servo
    """
    if self._id is None:
      raise ServoError(
        "The servo '%s' has not been configured" % self._name)
    return self._id

  def get_raw_value(self, position):
    """
    @param position: a human-friendly servo position, specified as a
      floating-point number between -1.0 and 1.0
    @return: the corresponding raw PWM integer value
    """
    if self._id is None:
      raise ServoError(
        "The servo '%s' has not been configured" % self._name)
    if position < 0:
      return int(saturate(position, -1.0, 0.0, self._min, self._center))
    else:
      return int(saturate(position, 0.0, 1.0, self._center, self._max))

###  end Servo  ###

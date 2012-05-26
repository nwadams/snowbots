"""
SUMMARY:
This module contains classes for interacting with the Furious robotics
microcontroller board in low- and high-level ways.  

WHERE TO START:
You'll probably want to use FuriousRobot for most things.  Check out
furious_example.py if you need help.

@author  Ian Phillips
"""

import errno 
import os
import serial
import sys

from robolib import clamp, interpolate


# Character that delimits the end of a serial command or response.
FURIOUS_COMMAND_DELIM = "G"

class FuriousBoard(object):
  """
  This is the low-level driver that communicates with the Furious
  microcontroller board over a serial (USB) connection.  You probably
  will not want to use this for most things -- use FuriousRobot (below)
  instead.  For more information on the Furious board, see the 
  inventor's website at: 

  http://ashleymckay.com
  """

  # Default parameters for serial connection
  _BAUD = 19200
  _TIMEOUT = 0.5 # seconds

  def __init__(self):
    """
    Constructor.
    """
    # The serial port for communication with the physical board.
    self._port = serial.Serial()
    self._port.baudrate = FuriousBoard._BAUD
    self._port.timeout = FuriousBoard._TIMEOUT

    # These variables hold the command values that will be sent
    # to the board the next time update() is called.
    self._sonar_req = 0
    self._unused = [0 for x in range(4)]
    self._servos = [0 for x in range(8)] 

    # This list holds the state values returned by the board.  See
    # the accessor methods below for the interpretation of these values.
    self._state = [0 for x in range(11)]

    # Observer object for debug messages
    self._dbg = None

  def open(self, port):
    """
    Opens a connection to the given port (eg, "/dev/ttyACM0").
    """
    if not self._is_valid_port(port):
      raise ValueError("Port '%s' is not valid." % port)
    if self._port.isOpen():
      self._port.close()
    self._port.port = port
    self._port.open()
    self._port.flushInput()
    self._port.flushOutput()

  def probe(self, ports):
    """
    Probes the given list of ports and returns the first one (if any) that
    has a Furious board attached to it.  Note that this does not connect
    the driver to the board; use open() for that.
    """
    device = None
    for port in ports:
      if self._is_valid_port(port):
        self._debug("Scanning on port %s" % port)
        try:
          self.open(port) # Try to open device
          self.update()   # Attempt handshake with Furious board
        except serial.serialutil.SerialException, e:
          self._debug("Unable to open %s.  Reason: %s" % (port, e.message))
        else:
          self._debug("Furious device found on port %s" % port)
          device = port
          break
    self.close()
#    self._port.timeout = 0
    return device

  def _is_valid_port(self, port):
    """
    Returns True if the given port is a valid port that a Furious board
    might be connected to.  This does NOT mean that a board is necessarily
    there.
    """
    return port and os.path.exists(port)

  def close(self):
    """
    Closes the serial connection to the board.
    """
    self._port.close()

  def is_ready(self):
    """
    Returns True if the serial connection to the board is open.
    """
    return self._port.isOpen()

  def update(self):
    """
    Sends any pending commands to the board and receives all state
    updates.
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
      # illegal characters to the output, so you should uninstall it.
      if c in ",0123456789-":
        reply += c
      c = self._port.read(1)

    # Chop the reply into parts and convert them to integers.
    parts = reply.split(",")
    parts.pop()  # pop off the blank due to trailing comma
    self._state = [ int(x.strip()) for x in parts ]

  def set_servo(self, servo_id, val):
    """
    Takes a servo id and a raw PPM value.  Future calls to update()
    will send the PPM value to the specified servo (unless future
    calls to this method overwrite it).
    """
    self._servos[servo_id] = val

  def set_sonar_request(self, sonar_id):
    """
    Takes a sonar id.  The next call to update() will cause the specified
    sonar to ping.
    """
    self._sonar_req = sonar_id

  def get_port_name(self):
    return self._port.portstr

  def get_analog(self, analog_id):
    """
    Returns the value of the specified analog port as an integer between
    0 and 1024.
    """
    return self._state[analog_id]

  def get_logic_battery(self):
    return self._state[6]
 
  def get_motor_battery(self):
    return self._state[7]

  def get_sonar_request(self):
    return self._sonar_req

  def get_sonarVal(self):
    return self._state[8]
 
  def get_odometer(self):
    return self._state[9]

  def _get_command_string(self):
    """
    Takes the pending commands and returns a string that can be
    sent over the serial connection to command the board.
    """
    parts = []
    parts.append(   str(self._sonar_req) )
    parts.extend( [ str(x) for x in self._unused] )
    parts.extend( [ str(x) for x in self._servos] )
    parts.append( FURIOUS_COMMAND_DELIM )
    return ",".join(parts)

  def set_debugger(self, dbg):
    """
    Sets the "observer" file object (eg, stdout), to which calls to 
    _debug() will be written.
    """
    self._dbg = dbg
    self._debug("Debugging enabled in %s" % self.__class__.__name__)

  def _debug(self, msg):
    if self._dbg:
      print >>self._dbg, msg
        
###  end FuriousBoard  ###




class Servo(object):
  """
  A helper class for FuriousRobot.  Represents the configuration 
  parameters of a single servo.
  """
  def __init__(self, id, min, center, max):
    """
    Constructor parameters:
      id - the physical port that this servo is connected to on
           the board.
      min - the minimum raw PPM signal value that is within the servo's
            range.  For example, on a throttle, this would correspond
            to full reverse.
      center - the center PPM value of this servo's range.  For example,
               on a throttle, this would correspond to stopped.
      max - the maximum PPM value in this servo's range.  For example,
            on a throttle, this would correspond to full forward.
    """
    self._id = id
    self._min = min
    self._center = center
    self._max = max

  def get_id(self):
    return self._id

  def get_raw_value(self, percentage):
    """
    Takes a percentage value and returns the corresponding raw PPM 
    signal value.
    """
    percentage = clamp(percentage, -100, 100)
    if percentage < 0:
      return int(interpolate(percentage, -100, 0, self._min, self._center))
    else:
      return int(interpolate(percentage, 0, 100, self._center, self._max))

###  end Servo  ###





class Battery(object):
  """
  Helper class for FuriousRobot that defines the charge limits (eg,
  full, empty) of a battery.
  """
  def __init__(self, full, empty):
    self._full = full
    self._empty = empty

  def get_charge(self, raw_value):
    """
    Returns the given raw value as a percentage.
    100 = full
      0 = empty
    """
    return int(interpolate(raw_value, self._empty, self._full, 0, 100))

###  end Battery  ###


class Odometer(object):
  """
  Helper class for FuriousRobot that represents an odometer.
  """
  def __init__(self, meters_per_tick):
    """
    Constructor parameters:
      meters_per_tick - the number of meters the vehicle has traveled
        each time the odometer registers a tick.
    """
    self._num_ticks = 0
    self._meters_per_tick = meters_per_tick

  def increment(self, num_ticks):
    """
    Adds num_ticks to the total number of odometer ticks counted so far.
    """
    self._num_ticks += num_ticks

  def get_distance(self):
    """
    Returns the total distance, in meters, that this odometer has measured
    so far.
    """
    return self._num_ticks * self._meters_per_tick

###  end Odometer  ###



class SonarArray(object):
  """
  Represents an array of sonar rangefinders and handles the sequential
  manner in which they must be polled.
  """
  def __init__(self):
    # i2c bus ids of all added sonars
    self._ids = []        
    # Most recent readings from all sonars.  The value at self._values[n]
    # is the value for the sonar whose id is at self._ids[n].
    self._values = []
    # The index of the next sonar to be polled.
    self._poll_idx = None

  def is_enabled(self):
    """
    Returns true if sonars have been added via add_sonar().
    """
    return self._poll_idx is not None

  def get_poll_id(self):
    """
    Returns the i2c bus id of the next sonar to be polled.  Returns
    None if no sonars have been added via add_sonar().
    """
    if self._poll_idx is not None:
      return self._ids[self._poll_idx]
    return None

  def add_sonar(self, id):
    """
    Adds a sonar rangefinder with the given i2c bus id to this
    sonar array.
    """
    if id not in self._ids:
      self._ids.append(id)
      self._values.append(0.0)
      self._poll_idx = 0

  def update(self, value):
    if self._poll_idx is None:
      return
    self._values[self._poll_idx] = value
    self._poll_idx = (self._poll_idx + 1) % len(self._ids)

  def get_value(self, id):
    try:
      index = self._ids.index(id)
    except ValueError:
      # Translate the raw list ValueError into something sonar-specific.
      raise ValueError("Sonar id %i is not valid." % id)
    return self._values[index]

###  end SonarArray  ###



class FuriousRobot(object):
  """
  This is the high-level class that encapsulates FuriousBoard (below)
  and translates the raw values it provides into units that are meaningful
  to humans.  This is the class you'll want to use most of the time.
  """

  _DEFAULT_METERS_PER_TICK = 0.33
  _DEFAULT_BATTERY_FULL = 150
  _DEFAULT_BATTERY_EMPTY = 120

  def __init__(self):
    """
    The constructor does some default configuration, but the robot
    usually needs servo configuration via add_servo() before it is
    fully initialized.
    """
    self._board = FuriousBoard()
    self._sonars = SonarArray()
    self._servos = {}
    self._odometer = None
    self._logic_batt = None # Furious board battery
    self._motor_batt = None # motor battery
    self._dbg = None  # Observer object for debug messages

    ## Configuration
    self.configure_odometer(
        FuriousRobot._DEFAULT_METERS_PER_TICK )
    self.configure_logic_battery(
        FuriousRobot._DEFAULT_BATTERY_FULL, 
        FuriousRobot._DEFAULT_BATTERY_EMPTY )
    self.configure_motor_battery(
        FuriousRobot._DEFAULT_BATTERY_FULL, 
        FuriousRobot._DEFAULT_BATTERY_EMPTY )


  ### configuration ###

  def add_servo(self, name, id, min, center, max):
    """
    Adds a servo to this robot's internal configuration.  Arguments:
      name   - a text name for the servo (eg, "steering", "throttle")
      id     - physical bus id (0 - 7)
      min    - minimum raw PPM value
      center - center PPM value
      max    - max PPM value
    Servos added by this method can be controlled by set_servo() and
    refresh().
    """
    self._servos[name] = Servo(id, min, center, max)

  def add_sonars(self, *ids):
    """
    Takes one or more sonar ids and registers them to be polled.  For
    example:
      robot = FuriousRobot()
      robot.add_sonars(12)    # robot now has one sonar: id 12
      robot.add_sonars(13,14) # robot now has three: 12, 13, 14
    """
    # Add sonars
    for id in ids:
      self._sonars.add_sonar(id)

  def configure_odometer(self, meters_per_tick):
    """
    Configures the odometer to calculate distance based on meters_per_tick.
    NOTE: calling this method will reset the odometer to zero.
    """
    self._odometer = Odometer(meters_per_tick)

  def configure_logic_battery(self, full, empty):
    """
    Configures the logic battery to use the given raw values to 
    determine the logic battery level.
    """
    self._logic_batt = Battery(full, empty)

  def configure_motor_battery(self, full, empty):
    """
    Configures the motor battery to use the given raw values to 
    determine the motor battery level.
    """
    self._motor_batt = Battery(full, empty)


  ### connecting, disconnecting, etc ###

  def connect(self, port):
    """
    Attempts to establish a serial connection to the Furious board on
    the specified port (eg, "/dev/ttyACM0").
    """
    self._board.open(port)
    self.refresh()

  def disconnect(self):
    """
    Immediately stops all servos and closes the board's serial connection.
    """
    self.stop()
    self._board.close()

  def probe(self, ports):
    """
    Probes the given list of ports and returns the first one (if any) that
    has a Furious board attached to it.  Note that this does not connect
    the driver to the board; use open() for that.
    """
    return self._board.probe(ports)

  def is_ready(self):
    """
    Returns true if the robot is ready to accept commands.
    """
    return self._board.is_ready()

  def stop(self):
    """
    Immediately stops all servos.  Good for emergencies and when you 
    reach your goal (or the end of your program).
    """
    for servo_name in self._servos:
      servo = self._servos[servo_name]
      self._board.set_servo(servo.get_id(), servo.get_raw_value(0))
    self.refresh()

  def refresh(self):
    """
    Sends all pending commands to and receives all updates from
    the board.
    """
    # Determine which sonar id to poll next
    if self._sonars.is_enabled():
      self._board.set_sonar_request(self._sonars.get_poll_id())

    # Update the board
    try:
      self._board.update()
    except OSError, e:
      # Handle the case when a read or write is interrupted.
      if e.errno == errno.EINTR:  # interrupted system call
        self.stop()  # stop the robot
      else:
        raise e  # propagate the exception

    if self._sonars.is_enabled():
      # Update the polled sonar, converting cm to meters
      self._sonars.update(float(self._board.get_sonarVal()) / 100.0)


  ### set methods ###
 
  def set_servo(self, servo_name, percentage):
    """
    Takes the name of the servo as specified by add_servo() (eg, 
    "throttle" or "steering") and a percentage value (between -100 and
    100) and prepares to update the servo to that value.  The servo will 
    be updated on the next call to refresh().
    """
    servo = self._servos[servo_name]
    if not servo:
      raise ValueError("No servo named %s has been added" % servo_name)
    self._board.set_servo(servo.get_id(), servo.get_raw_value(percentage))


  ### get methods ###

  def get_port_name(self):
    """
    Returns the name of the port to which the robot is connected
    (eg, "/dev/ttyACM0").
    """
    return self._board.get_port_name()

  def get_logic_battery(self):
    """
    Returns the level of the logic battery as a percentage.
    """
    return self._logic_batt.get_charge(self._board.get_logic_battery())

  def get_motor_battery(self):
    """
    Returns the level of the motor battery as a percentage.
    """
    return self._motor_batt.get_charge(self._board.get_motor_battery())

  def get_odometer(self):
    """
    Returns the total distance travelled by the robot so far, in meters.
    """
    self._odometer.increment(self._board.get_odometer())
    return self._odometer.get_distance()

  def get_sonar(self, sonar_id):
    """
    Returns the last value (in meters) obtained from the given sonar.
    """
    return self._sonars.get_value(sonar_id)

  def get_analog(self, analog_id):
    """
    Returns the voltage (between 0 and 5.0v) coming from the specified 
    analog port as a float.
    """
    # Analog values from from the board as integers between 0 and 1024.
    # 0 means zero volts; 1024 means 5.0 volts.
    return (self._board.get_analog(analog_id) * 5.0) / 1024

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

###  end FuriousRobot  ###


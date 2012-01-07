"""
This module contains a simulator of the Furious microcontroller board.  
The simulator will listen on a tty port (eg, /dev/pts/1 on Linux) that a
driver (eg, a L{furiousDriver.py} instance or the tbrprobe 
driver) program can connect to.  For more info, see the website of
Ashley McKay, the inventor of the Furious board:

U{http://ashleymckay.com/ashleymckay/cms/?q=node/195}
"""

import os
import pty
import time

# Import the character that is used to mark the end of a Furious
# serial command/response.
from furious.devices import FURIOUS_COMMAND_DELIM

class FuriousSim(object):
  """
  Emulates Ashley McKay's Furious microcontroller board.  For more info
  about the Furious board, see the website of Ashley McKay, the inventor 
  of the Furious board:

  U{http://ashleymckay.com/ashleymckay/cms/?q=node/195}
  """
  
  def __init__(self):
    """ 
    Constructor.
    """
    self._state = [0 for x in range(21)]
    self._command = [0 for x in range(13) ]
    self._dbg = None  # Observer object for debug messages

    # pty terminal device file descriptors (-1 means not opened)
    self._pty_master = -1
    self._pty_slave = -1

  def __del__(self):
    """
    Destructor.  Calls L{disconnect()} in case you forget to.  But don't 
    forget.
    """
    self.disconnect()

  def set_debugger(self, dbg):
    """
    Enables internal debugging output.
    @param dbg: The writable file object (eg, sys.stdout) to which 
      debug statements will be written.
    @type dbg: file
    """
    self._dbg = dbg

  def _debug(self, msg):
    if self._dbg:
      print >>self._dbg, msg

  def open_pty(self):
    """
    Opens a pty terminal device, which Furious drivers can use to 
    communicate with this simulator.
    """
    (self._pty_master, self._pty_slave) = pty.openpty()

  def slave_ttyname(self):
    return os.ttyname(self._pty_slave)
    
  def connect(self):
    """ 
    Awaits for a connection attempt for the Furious driver.
    """
    # Receive the probe packet from the driver and echo it back so the
    # computer knows that a valid "device" (albeit a virutal one) is here.
    cmd = self.receive()
    
    # Wait for rest of the probe packet to arrive, then flush it.  It's
    # there to reset the hardware, which is of no concern to us.
    time.sleep(0.1)
    self.reply(cmd + FURIOUS_COMMAND_DELIM)
  
  def disconnect(self):
    """
    Closes the serial connection to the driver.
    """
    if self._pty_master >= 0:
      os.close(self._pty_master)
      self._pty_master = -1
    if self._pty_slave >= 0:
      os.close(self._pty_slave)
      self._pty_slave  = -1

  def is_ready(self):
    """ 
    @return: True if the connection to the driver is established
    """
    return os.isatty(self._pty_master) and os.isatty(self._pty_slave)

  def _readchar(self):
    return os.read(self._pty_master, 1)
  
  def receive(self):
    """ 
    @return: a command string received from the driver
    """
    self._debug("Awaiting command...")
    cmd = ""
    c = self._readchar()
    while c != FURIOUS_COMMAND_DELIM:
      cmd += c
      c = self._readchar()
    self._debug("Received %s" % cmd)
    return cmd
    
  def decode(self, command):
    """
    Decodes the given command and updates the state of this object.
    @param command: a command string from a Furious driver program
    @type command: string
    """
    values = command.split(",")
    values.pop()  # pop off blank due to trailing comma
    self._command = [ int(x) for x in values ]
  
  def encode(self):
    """ 
    Encodes the state of this object into a Furious-formatted string.
    """
    out = ""
    for val in self._state:
      out += str(val)
      out += ","
    out += FURIOUS_COMMAND_DELIM
    return out 
    
  def reply(self, msg):
    """ 
    Sends the given string back to the driver. 
    @param msg: a string, in the Furious serial protocol format, that
      encodes the state of this simulator
    @type msg: string
    """
    self._debug("Sending %s" % msg)
    os.write(self._pty_master, msg)
  
  def step(self):
    """ 
    Awaits a command, decodes it, prepares the reply, and sends it.
    """
    cmd = self.receive()
    if cmd:
      self.decode(cmd)
      msg = self.encode()
      self.reply(msg)

  def get_sonar_request(self):
    return self._command[0]
  
  def set_analog(self, analog_id, value):
    self._state[analog_id] = value

  def set_logic_battery(self, value):
    self._state[7] = value

  def set_motor_battery(self, value):
    self._state[6] = value

  def set_sonar_value(self, value):
    self._state[8] = value

  def set_odometer(self, value):
    self._state[9] = value
  
###  end FuriousSim  ###


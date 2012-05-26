"""
This module contains a simulator of the Furious microcontroller board.  
The simulator will listen on a tty port (eg, /dev/pts/1 on Linux) that a
driver (eg, furious.devices.FuriousRobot or tbrprobe) can connect to.  For 
more info, see the forum on the inventor's website:

http://ashleymckay.com/ashleymckay/cms/?q=forum
"""

import os
import pty
import time

# Import the character that is used to mark the end of a Furious
# serial command/response.
from furious.devices import FURIOUS_COMMAND_DELIM

class FuriousSim(object):
  """
  Emulates Ashley McKay's Furious microcontroller board.
  """
  
  def __init__(self):
    """ 
    Constructor.  Calls connect() automatically.
    """
    self._state = [0 for x in range(21)]
    self._command = [0 for x in range(13) ]
    self._dbg = None  # Observer object for debug messages

    # pty terminal device file descriptors (-1 means not opened)
    self._pty_master = -1
    self._pty_slave = -1


  def set_debugger(self, dbg):
    """
    Sets the "observer" file object (eg, stdout), to which calls to 
    _debug() will be written.
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
    # Receive the probe packet from tbrprobe and echo it back so tbrprobe knows
    # that a valid "device" (albeit a virutal one) is here.
    cmd = self.receive()
    
    # Wait for rest of the probe packet to arrive, then flush it.  It's
    # there to reset the hardware, which is of no concern to us.
    time.sleep(0.1)
    self.reply(cmd + FURIOUS_COMMAND_DELIM)
  
  def disconnect(self):
    """
    Closes the serial connection to tbrprobe.
    """
    os.close(self._pty_master)
    self._pty_master = -1
    os.close(self._pty_slave)
    self._pty_slave  = -1

  def is_ready(self):
    """ 
    Returns true if the connection to tbrprobe is established.
    """
    return os.isatty(self._pty_master) and os.isatty(self._pty_slave)

  def _readchar(self):
    return os.read(self._pty_master, 1)
  
  def receive(self):
    """ 
    Returns a command string received from the driver. 
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
    """
    values = command.split(",")
    values.pop()  # pop off blank due to trailing comma
    self._command = [ int(x) for x in values ]
  
  def encode(self):
    """ 
    Encodes the state of this object into a tbrprobe-formatted string.
    """
    out = ""
    for val in self._state:
      out += str(val)
      out += ","
    out += FURIOUS_COMMAND_DELIM
    return out 
    
  def reply(self, msg):
    """ 
    Sends the given string back to tbrprobe. 
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
    self._state[6] = value

  def set_motor_battery(self, value):
    self._state[7] = value

  def set_sonar_value(self, value):
    self._state[8] = value

  def set_odometer(self, value):
    self._state[9] = value
  
###  end FuriousSim  ###


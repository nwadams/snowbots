"""
Python package that corresponds to the sb_furious ROS package.  Imports
L{sb_furious.devices.FuriousBoard}, L{sb_furious.devices.Servo}, and 
L{sb_furious.simulator.FuriousSimulator} so that they can be more easily
accessed by the outside world.

@author: Ian Phillips <ianfp@freeshell.org>
"""

from devices import FuriousBoard, Servo
from simulator import FuriousSimulator

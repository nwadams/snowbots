#!/usr/bin/env python
"""
AI for following a "segment map" -- a description of a course that consists
of course segments (eg, straightaway, right turn, etc).
"""

import time, signal, sys
from tbrclient import tbrclient
from robolib import SonarBot, SpeedControl, PidControl, clamp
import maps

MAPFILE = "segmap.txt"
THROTTLE = 33
ESCAPE = 9
SAFE = 6
ESCAPE_THROTTLE = -50
LOOP_PERIOD = 0.1
DIST_CORR = LOOP_PERIOD * 0.001
PV = 0.005
MV = 0.1
KP = 32
KI = 0
FMIX = 0.2
TMIX = 0.25
KILL_OFF = 0.1
SPEED = 0.5

running = False
client = tbrclient()

def main():
  global running, client

  print "Starting up..."
  # Assign signal handlers
  signal.signal(signal.SIGINT, interrupt_handler)

  test = False
  args = sys.argv[1:]
  mapfile = MAPFILE
  while len(args) > 0:
    opt = args.pop(0)
    if opt == "test":
      print "=====  TEST MODE ON  ===="
      test = True
    else:
      mapfile = opt

  map = maps.loadSegmentMap(mapfile)
  seg = 0
  uptime = 0.0
  bot = SonarBot(pv=PV, mv=MV, fmix=FMIX)
  steer_pid = PidControl(KP, KI)
  spd_ctrl = SpeedControl(kp=50, ki=8)
  escape = False
  pthrottle = 0
  prev_l = prev_r = 0.0

  client.initialize(1225, "127.0.0.1")
  time.sleep(1)
  running = client.ready()

  print "mode, segment, distance, left, front, right, dturn, aturn, throttle"
  while running:
    seg_start = cur_dist = client.getOdometerDistance()
    ml = map[seg].length
    mt = map[seg].turn
    prev_turn = 0
    while cur_dist - seg_start < ml:
      # Initialize variables
      turn = mt  # modify the map turn based on sensors
      (f, l, r) = bot.getDangers(client)

      if test:
        print "testing   ",
        throttle = THROTTLE

      # Warm up the sonar filters
      elif uptime < 2.0:
        print "warmup    ",
        turn = 0
        throttle = 0

      # If in the middle of escaping, continue
      elif escape:
        print "escaping  ",
        if prev_turn == 0:
          turn = 0
        else:
          turn = 100 * prev_turn / abs(prev_turn)
        throttle = ESCAPE_THROTTLE
        if f < SAFE:
          escape = False

      # If cornered, back up.
      elif f > ESCAPE:
        print "escape!   ",
        escape = True
        throttle = 0
      
      # Otherwise proceed normally
      else:
        print "driving   ",
        delt_r = r - prev_r
        delt_l = l - prev_l
        turn += steer_pid.getOutput(0, delt_r - delt_l)
        throttle = spd_ctrl.calculateThrottle(SPEED, cur_dist)
        correction = DIST_CORR * (abs(mt) - abs(turn) )
        seg_start += correction

      turn = int( clamp(turn, -100, 100) )
      client.setSteering(turn)

      throttle = (TMIX * throttle) + ((1 - TMIX) * pthrottle)
      throttle = int( clamp( throttle, -100, 100 ) )
      # check the kill switch
      if client.getKillSwitchVal() < KILL_OFF:
        throttle = 0
      client.setThrottle(throttle)

      # Prepare for next iteration
      print ", %i, %.2f,    %.2f, %.2f, %.2f,   %i, %i, %i" % (
          seg, cur_dist, l, f, r, mt, turn, throttle)
      time.sleep(LOOP_PERIOD)
      uptime += LOOP_PERIOD
      cur_dist = client.getOdometerDistance()
      prev_turn = turn
      pthrottle = throttle

    seg += 1

    # Begin next lap
    if seg >= len(map):
      seg = 0

  print "Exiting normally."
  shutdown()


def interrupt_handler(signum, frame):
  print "\nCaught interrupt signal."
  shutdown()


def shutdown(ret_val=0):
  global running, client
  running = False
  client.setThrottle(0)
  client.setSteering(0)
  exit(ret_val)

if __name__ == "__main__":
  main()

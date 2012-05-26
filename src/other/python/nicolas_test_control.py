#!/usr/bin/env python
"""
Nicolas' simple test of obstacle avoidance
Usage:
  %s SPEED
Command line arguments:
  SPEED  the target speed of the robot in meters/second
"""

import sys, time
from tbrclient import tbrclient

## Tunable constants ##
LOOP_PERIOD = 0.05
STOPPING_TIME = 0.8

# What are the limits of the throttle?
THROTTLE_MIN = 24
THROTTLE_MAX = 100

class SonarData:
    limits={}
    # limits with margins of 10 cm
    limits['front']=0#unusable large bumper
    limits['left']=0.30
    limits['right']=0.30
    limits['rear']=0.1#0.20

    def __init__(self, client):
        self.client = client
        self.data={}
        self.data['front'] = [client.getSonarN()]
        self.data['right'] = [client.getSonarNE()]
        self.data['left'] = [client.getSonarNW()]
        self.data['rear'] = [client.getSonarS()]

    def __str__(self):
        return 'n %f nw %f ne %f s %f'%(self.data['front'][-1], 
                                        self.data['left'][-1], 
                                        self.data['right'][-1], 
                                        self.data['rear'][-1])

    def obstacle(self, orientation):
        return self.data[orientation][-1] <= SonarData.limits[orientation]

    def obstacleLeft(self):
        return self.obstacle('left')

    def obstacleRight(self):
        return self.obstacle('right')

    def obstacleFront(self):
        return self.obstacleRight() or self.obstacleLeft()

    def obstacleRear(self):
        return self.obstacle('rear')

    def tooClose(self):
        self.update()
        result = False
        for orientation in SonarData.limits.keys():
            result = result or (self.data[orientation][-1] <= SonarData.limits[orientation])
        return result

    def update(self):
        self.data['front'].append(client.getSonarN())
        self.data['left'].append(client.getSonarNW())
        self.data['right'].append(client.getSonarNE())
        self.data['rear'].append(client.getSonarS())
        
def goForward(client, throttle, timeAmount, sonar=None):
    if (throttle >= THROTTLE_MIN):
        go(client, throttle, timeAmount, sonar, SonarData.obstacleFront)

def goBackward(client, throttle, timeAmount, sonar=None):
    if (throttle >= THROTTLE_MIN):
        go(client, -throttle, timeAmount, sonar, SonarData.obstacleRear)

def go(client, throttle, timeAmount, sonar=None, localobstacle=None):
    '''Goes for timeAmount of time
    take STOPPING_TIME s to stop'''
    t1 = time.time()
    if timeAmount < STOPPING_TIME:
        print 'too short time'
        return
    
    if sonar==None:
        print 'no sonar'
        return

    if not localobstacle(sonar):
        client.setThrottle(throttle)
        while (time.time()-t1<timeAmount) and (not localobstacle(sonar)):#-STOPPING_TIME
            pass
        client.setThrottle(0)
        time.sleep(STOPPING_TIME)
    else:
        print('too close')

client = tbrclient()
print "connecting... "
client.initialize(1225, "127.0.0.1")
print "Done."

if client.ready():
    try:
#         for i in xrange(4):
#             print 'test fwd 2'
#             goForward(client, THROTTLE_MIN, 2)
#             #print 'waiting'
#         #time.sleep(3)
#             print 'test reverse 2'
#             goForward(client, THROTTLE_MIN, 2)
#         #print 'waiting'
#         #time.sleep(1)

        sonarData = SonarData(client)
        while True:
            #sonarData.update(client)
            #print sonarData
            goForward(client, THROTTLE_MIN, 2, sonarData)
            
            left = sonarData.obstacleRight()
            right = sonarData.obstacleLeft()

            if (sonarData.tooClose()):
                goBackward(client, THROTTLE_MIN, 2, sonarData)

#             if (left):
#                 client.setSteer(-100)

#             if (right):
#                 client.setSteer(100)

            #cur_vel = client.getOdometerVelocity()
            #client.setThrottle(throt)
            time.sleep(LOOP_PERIOD)
    except KeyboardInterrupt:
      print " Shutting down..."
      client.setThrottle(0)
else:
    print "Error: unable to connect to tbrprobe."
    client.setThrottle(0)
    client.finalize()

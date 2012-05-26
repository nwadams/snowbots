#!/usr/bin/env python
"""
Kalman filter example demo in Python

A Python implementation of the example given in pages 11-15 of "An
Introduction to the Kalman Filter" by Greg Welch and Gary Bishop,
University of North Carolina at Chapel Hill, Department of Computer
Science, TR 95-041,
http://www.cs.unc.edu/~welch/kalman/kalmanIntro.html

by Andrew D. Straw
modified by Ian Phillips

USAGE:
  %s [-ms STDDEV] [-r MVAR] [-q PVAR]
OPTIONS:
  -ms STDDEV   set the measurement std. dev. to STDDEV
  -r MVAR      set the estimate of measurement variance to MVAR
  -q PVAR      set the process variance to PVAR

REQUIRED PACKAGES:
  python-numpy
  python-scipy
  python-matplotlib
  python-tk
"""

import sys
import numpy
import pylab
from math import sin, radians
from robolib import KalmanFilter

# intial parameters
n_iter = 360
sz = (n_iter,) # size of array
m_stddev = 0.1  # measurement standard deviation
R = 0.1**2 # estimate of measurement variance, change to see effect
Q = 1e-3 # process variance

# allow parameters to be set from the command line
args = sys.argv[1:]
while len(args) > 0:
  opt = args.pop(0)
  if opt == "-ms":
    m_stddev = float(args.pop(0))
  elif opt == "-r":
    R = float(args.pop(0))
  elif opt == "-q":
    Q = float(args.pop(0))
  else:
    print "Invalid option %s" % opt
    print __doc__ % sys.argv[0]

# allocate space for arrays
xhat=numpy.zeros(sz)      # a posteri estimate of x
P=numpy.zeros(sz)         # a posteri error estimate
xhatminus=numpy.zeros(sz) # a priori estimate of x
Pminus=numpy.zeros(sz)    # a priori error estimate
K=numpy.zeros(sz)         # gain or blending factor
x=numpy.zeros(sz)         # true values (a sine wave)
z=numpy.zeros(sz)         # noisy measurements

kf = KalmanFilter(q=Q, r=R)

for k in range(1,n_iter):
  # generate true value (a sine wave)
  x[k] = sin(radians(k))

  # generate a noisy measurement
  z[k] = numpy.random.normal(x[k], m_stddev)

  # use the Kalman filter to estimate the true value
  xhat[k] = kf.getEstimate(z[k])

pylab.figure()
pylab.plot(z,'k+',label='noisy measurements')
pylab.plot(xhat,'b-',label='a posteri estimate')
pylab.plot(x,'g',label='truth value')
pylab.legend()
pylab.xlabel('Iteration')
pylab.ylabel('Value')
pylab.show()


# Define a specific "ranger" object called "snowfury_sonar", 
# which represents the sonar array used on the Snowfury robot. 
# See http://playerstage.sourceforge.net/doc/Stage-2.0.0/group__model__ranger.html
define snowfury_sonar ranger
(
  scount 4

  # define the pose of each transducer [xpos ypos heading]
  spose[0] [  0    0.25    0 ]   # North
  spose[1] [  0.2  0.25   80 ]   # Northeast
  spose[2] [ -0.2  0.25  -80 ]   # Northwest
  spose[3] [  0   -0.25 -180 ]   # South
  
  # define the field of view of each transducer [range_min range_max view_angle]
  sview [.10 3.0 60]

  # define the size of each transducer [xsize ysize] in meters
  ssize [0.20 0.20]
)


#define snowfury_battery snowfury_battery
#(
#  power( pose [-0.14 0 0] capacity 1000 probe_range 0.50  give 0 give_rate 100 take_rate 100 )
#)



define sharp_ir_short_array laser
(
  range_min 0.10
  range_max 0.80
  fov 90
  resolution 1
  samples 3
  color "black"
  size [ 0.05 0.05 0.1 ]
)

define sharp_ir_long_array laser
(
  range_min 0.20
  range_max 1.50
  fov 90
  resolution 1
  samples 3
  color "black"
  size [ 0.05 0.05 0.1 ]
)

define urg_04lx_ug01 laser
(
  range_min 0.02
  range_max 5.60
  fov 120
  resolution 1
  samples 666
  color "black"
  size [ 0.05 0.05 0.07 ]
)

# Define a specific "position" object called "snowfury", which
# represents a Traxxas-type RC card, modified for autonomous 
# navigation.
# See http://playerstage.sourceforge.net/doc/Stage-2.0.0/group__model__position.html
define snowfury position
(
  # actual size
  size [0.50 0.40 0.2]  # length width height (in meters)

  # The robot's center, relative to its pose
  origin [-0.04 0.0 0]  # xpos ypos heading

  # draw a nose on the robot so we can see which way it points
  gui_nose 1

  # estimated mass in KG
  mass 2.0 
  
  # approximate maximum speed (in m/sec)
  max_speed 4.0
  
  # approximate maximum turn rate (in degrees/sec)
  max_turnrate 15

  # Our robot is roughly a rectangle
  polygons 1
  polygon[0].points 4
  polygon[0].point[0] [ 0.0 0.0 ]
  polygon[0].point[1] [ 0.0 0.4 ]
  polygon[0].point[2] [ 0.5 0.4 ]
  polygon[0].point[3] [ 0.5 0.0 ]

  # carlike steering model
  drive "car"

  ###  SENSORS  ###
  # localization model: "gps" is unerring, whereas "odom" is error-
  # prone and relative to the robot's start position
  localization "odom"

  # Uncomment the rangefinder you wish to use
  #snowfury_sonar()  
  #sharp_ir_short_array(pose [0 0 0])
  #sharp_ir_long_array(pose [0 0 0])
  urg_04lx_ug01(pose [0 0 0])

  ###  RETURNS  ###
) 


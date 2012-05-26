#! /usr/bin/env python

from socket import *

# set up connection
host = "127.0.0.1"

send_port = 1225  # tbprobe listens on port 1225
send_addr = (host,send_port)
send_sock = socket(AF_INET, SOCK_DGRAM)


# sensor id lookup
devices = {
  #to test if connection is alive
  00: "dev_noop",
  
  #these raw sensors may be queried individually
  01: "dev_sonar_front_left",
  02: "dev_sonar_front_right",
  03: "dev_sonar_rear_left",
  04: "dev_sonar_rear_right",
  
  #virtual sensor to get approximate distance
  #returns total_distance,current_velocity,current_acceleration,current_jerk
  100: "dev_odometer",
  
  #the auto calculated plane virtual sensor created from ir raw information
  #returns proximity,pitch,yaw,roll
  200: "dev_ir_orientation",
  
  #Jon's code
  #returns proximity,x_hat,y_hat,z_hat
  201: "dev_ir_vector"
}


commands = {
  "as": "auto_steering",
  "at": "auto_throttle",
  "vs": "get_virtual_sensor",
  "ms": "manual_steering",
  "mt": "manual_throttle",
  "mon": "manual_control_on",
  "moff": "manual_control_off"
}


def sendCmd(command):
  send_sock.sendto(command, send_addr)
    
def recv():
  return recv_sock.recvfrom(recv_buf_len)


while True:
  print("Valid commands are:")
  for short_name in commands:
    print("  %5s) %s" % (short_name, commands[short_name]))
  print("      q) quit")
  inputs = raw_input("Choose a command: ").split()
  if len(inputs) > 1:
    (selection, value) = tuple(inputs)
  else:
    selection = inputs[0]
    value = None
  if selection == "q":
    exit(0)
  selection = commands[selection]
  command = ""
  if not value:
    if selection == "get_virtual_sensor":
      print("Valid devices are:")
      for dev_id in devices:
        print("  %4i) %s" % (dev_id, devices[dev_id]))
      value = raw_input("Choose a device id: ")
    else:
      value = raw_input("Enter a value: ")
  command = "%s,%s" % (selection, value)
  print("Command is '%s'" % command)
  sendCmd(command)
  if  selection == "get_virtual_sensor":
    try:
      reply, reply_addr = recv()
    except KeyboardInterrupt:
      print " Cancelled"
      continue
    print("Reply is '%s'" % reply)



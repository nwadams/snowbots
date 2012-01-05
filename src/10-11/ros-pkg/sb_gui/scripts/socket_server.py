#!/usr/bin/env python

# TCP server example
import socket
import threading
import time

class receiveThread(threading.Thread):
	def run(self):
		data = client_socket.recv(512)
		recieved = "recieved"
		if not data == recieved:
			client_socket.send(data)
		print data

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("", 5000))
server_socket.listen(5)

print "TCPServer Waiting for client on port 5000"

while 1:
	client_socket, address = server_socket.accept()
	print "I got a connection from ", address
	myThread = receiveThread()
	myThread.start()
	while 1:
		time.sleep(1)
		if not myThread.isAlive():
			myThread = receiveThread()
			myThread.start()

print "dead out of loop"

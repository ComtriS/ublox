#!/usr/bin/env python

import sys, os, time
import serial
import serial.rfc2217
import threading
from datetime import datetime

from helper import *

# This is a class to handle the serial port comms.
# It supports 2 main API types:
#  - Always on logging to a file
#  - Command / response request from a host (including filtering the received data to look for a specific string)
# It does not support:
#  - random data from the device being received and understood


class SerialPort():
	def __init__( self, uartPort, baudrate, lineEndings, caseSensitive=False, verbose=False ):
		if verbose:
			print "SerialPort: ", uartPort, str(baudrate)
		try:
			self.serial_port = serial.Serial(port=uartPort, baudrate=baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0, xonxoff=False, rtscts=False, writeTimeout=None, dsrdtr=False, interCharTimeout=1 )
			self.dummyFile = False
		except Exception:
			#try as a normal file instead
			self.serial_port = open(uartPort, "rw")
			self.dummyFile = True
			pass
		self.lineEndings = lineEndings
		self.event = threading.Event()
		self.event.clear()
		self.responseFilter = None
		self.readLine = ""
		self.verbose = verbose
		self.caseSensitive = caseSensitive
		self.openLogFiles = {}

		#start the thread automatically
		def read_from_port(self, ser):
			exitThread = False
			readLine = ""
			while not exitThread:
				try:
					# print "logLine:",
					self.readLine = ser.readline()
					logLine = self.readLine
					# print logLine
					
					if len(self.readLine) != 0:
						
						
						if self.caseSensitive:
							self.readLine = self.readLine.upper().lstrip().rstrip()
						self.readLine = self.readLine.lstrip().rstrip()
						lines = self.readLine.splitlines()
						for l in lines:
							if( self.verbose ):
								print datetime.now().strftime("%H:%M:%S.%f"), " R:", l
							if( len(l) ):

								if( self.responseFilter != None and l.find(self.responseFilter) != -1):
									self.response = l

									#trigger the event!
									self.event.set()

								#always write to the output logs
								for key in self.openLogFiles:
									self.openLogFiles[key].write( logLine )

				except Exception, err:
					print Exception, err
					exitThread = True
			#print "SerialPort Thread exit!"

		if self.serial_port != None:
			if self.dummyFile == False:
				self.thread = threading.Thread(target=read_from_port, args=(self, self.serial_port))
				self.thread.start()
			# print "Created thread: ", self.thread, self.serial_port
		else:
			raise Exception("Failed to connect to serial port: ", uartPort)

	def shutdown(self):
		#print "shutDown!: ", self.thread, self.serial_port
		
		#close the logs
		for key in self.openLogFiles.items():
			self.stopLogging(key[0])
		
		if self.serial_port != None:
			self.serial_port.close()
			self.serial_port = None
			if self.dummyFile == False:
				self.thread.join()

	def __del__( self ):
		self.shutdown()

	#Global logging start / stop commands
	def startLogging( self, logFileName ):
		outfile = open(logFileName,"w")
		if outfile != None:
			#print "startLogging:", logFileName, outfile
			self.openLogFiles[logFileName] = outfile

	def stopLogging( self, logFileName ):
		if self.openLogFiles.has_key(logFileName):
			outfile = self.openLogFiles[logFileName]
			del self.openLogFiles[logFileName]
			outfile.close()

	#command / pair
	#responseFilter is a very basic strstr
	#returns the response or throws an exception
	def doCommandResponse( self, cmd, responseFilter, timeout ):
		self.responseFilter = responseFilter
		if self.dummyFile == False:

			self.serial_port.write(cmd)
			self.serial_port.write(self.lineEndings)
			ok = self.event.wait( timeout )
			self.event.clear()
			if ok:
				print datetime.now().strftime("%H:%M:%S.%f"), "ok"
				return self.response
			else:
				print datetime.now().strftime("%H:%M:%S.%f"), "serial timeout"
				return None
		else:
			return None
	
	def waitResponse(self, responseFilter, timeout):
		self.responseFilter = responseFilter
		if self.dummyFile == False:
			ok = self.event.wait( timeout )
			self.event.clear()
			if ok:
				if self.verbose:
					print datetime.now().strftime("%H:%M:%S.%f"), "ok"
				return self.response
			elif self.verbose:
				print datetime.now().strftime("%H:%M:%S.%f"), "serial timeout"
	
	#send a blind command
	def sendCommand(self, cmd):
		if self.dummyFile == False:
			self.serial_port.write(cmd)
			self.serial_port.write(self.lineEndings)

def attach_checksum(data):
	checksum = 0
	for byte in data:
		checksum += byte
	
	checksum_msb = checksum >> 8
	checksum_lsb = checksum & 0xFF
	
	data.append(checksum_msb)
	data.append(checksum_lsb)

def set_standard_datum(serial, datumNum):
	data = bytearray([0xB5, 0x62, 0x06, 0x06, 0x02, datumNum])
	attach_checksum(data)
	serial.sendCommand(data)

def get_standard_datum(serial, datumNum):
	data = bytearray([0xB5, 0x62, 0x06, 0x06, 0x02, datumNum])
	attach_checksum(data)
	serial.sendCommand(data)

def get_lonlat(serial, timeout):
	gll = serial.waitResponse("GNGLL", timeout)
	tokens = gll.split(",")
	
	longitude = tokens[1]
	lon_degrees = longitude[:longitude.index('.')-2]
	lon_minutes = longitude[longitude.index('.')-2:]
	longitude = lon_degrees + " " + lon_minutes
	if tokens[2] == "S":
		longitude = "-" + longitude
	
	latitude = tokens[3]
	lon_degrees = latitude[:latitude.index('.')-2]
	lon_minutes = latitude[latitude.index('.')-2:]
	latitude = lon_degrees + " " + lon_minutes
	if tokens[4] == "W":
		latitude = "-" + latitude
	
	return (longitude, latitude)

if __name__ == "__main__":
	ublox_serial = SerialPort("/dev/ttyACM0", 115200, "\r\n")
	ublox_serial.startLogging("ublox.log")
	
	while True:
		(longitude, latitude) = get_lonlat(ublox_serial, 10000)
		print("LON/LAT: " + longitude + ", " + latitude)
	
	raw_input("Press Enter to shutdown...")
	ublox_serial.shutdown()
	
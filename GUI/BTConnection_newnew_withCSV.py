

##
## Read from a serial port and print received data.
## Set portName to be the name of teh serial port to be used.
##
## Author:  Greg Watkins
## Date:    10 Sep 2021
##

import serial
import time
import sys
import struct
import csv
import os

serialString = ""  # Used to hold data coming over UART
portName = "COM3"          # PC format



# define the serial port.
# specify parameters as needed
serialPort = serial.Serial()
serialPort.port=portName 
serialPort.baudrate=115200
serialPort.bytesize=8
serialPort.timeout=5 # 5 seconds? 
serialPort.stopbits=serial.STOPBITS_ONE


try:
    serialPort.open()
    print("Trying to connect")
except:
    print("Port open failed: " + portName)
    for e in sys.exc_info():
        print("  ",e)


#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! create csv files
csv_filename = 'sensor_data.csv'
if not os.path.isfile(csv_filename):
    with open(csv_filename, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['Sensor Data'])  # Add a header row

csv_filename = 'BPM.csv'
if not os.path.isfile(csv_filename):
    with open(csv_filename, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['BPM Data'])  # Add a header row


# open the port

while True:


    if serialPort.isOpen():


        if serialPort.in_waiting > 0:
            data = serialPort.readline().decode().strip()  # Read a line of data from the Arduino

            if data.startswith("Packet Sequence:"):
                sequence = data[len("Packet Sequence:"):]
                print("Packet Sequence:", sequence)
            elif data.startswith("BPM:"):
                bpm_data = data[len("BPM:"):]
                print("BPM:", bpm_data)
                
                bpm_data = bpm_data.strip("[]").split(" ") # splits the BPM based on spaces
                
                # adds BPM to the csv
                with open('BPM.csv', 'a', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    for ii in bpm_data:
                        csv_writer.writerow([ii.strip()])

            elif data.startswith("RAW:"):
                sensor_data = data[len("RAW:"):]
                print("Raw Sensor Data:", sensor_data)

                sensor_data = sensor_data.strip("[]").split(",") # splits raw value based on ,

                # adds raw data to the csv
                with open('sensor_data.csv', 'a', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    for iii in sensor_data:
                        csv_writer.writerow([iii.strip()])

        else:
            serialPort.close()

    else:

        try:
            serialPort.open()
        except:
            print("Port open failed: " + portName)
            for e in sys.exc_info():
                print("  ",e)

        time.sleep(1)


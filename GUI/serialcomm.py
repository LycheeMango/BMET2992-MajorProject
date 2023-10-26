import serial
import PySimpleGUI as sg
import threading
import time
import csv



class SerialReader:
    def __init__(self, comport, baudrate, pathfilelog, csvfilepath):
        self.comport = comport
        self.baudrate = baudrate
        self.pathfilelog = pathfilelog
        self.csvfilepath = csvfilepath
        self.reading_flag = False
        self.looprun = 0

    def open_serial_port(self):
        serial_port = serial.Serial()
        serial_port.port = self.comport
        serial_port.baudrate = self.baudrate
        serial_port.timeout = 10  # 10 seconds?
        serial_port.stopbits = serial.STOPBITS_ONE

        try:
            serial_port.open()
            return serial_port
        except:
            print("Port open failed: " + self.comport)
            return None

    def read_serial(self):
        self.reading_flag = True
        try:
            file_log = open(self.pathfilelog, "a") #using append methode to file
            print('File Log Ready')

        except Exception as b:
            print ("error saving file: " + str(b))

        try:
            ser = serial.Serial(self.comport,self.baudrate)
            print('Succesful Connected to Serial Port COM:'+self.comport+'  Baudrate:'+self.baudrate)
        except Exception as a:
            print ("error open serial port: " + str(a))

        if ser.isOpen():
            print("**************************************")
            print("** Serial port opened: {}".format(self.comport))
            print("**************************************")
            try:
                ser.flushInput() #flush input buffer, discarding all its contents
                ser.flushOutput()#flush output buffer, aborting current output
                                #and discard all that is in buffer

                while self.looprun == 1 and self.reading_flag:

                    if ser.in_waiting > 0:
                        data = ser.readline().decode().strip()

                        if data.startswith("Packet Sequence:"):
                            sequence = data[len("Packet Sequence:"):]
                            # print("Packet Sequence:", sequence)
                            line = "Packet Sequence: " + sequence
                            print(line)
                        elif data.startswith("BPM:"):
                            bpm_data = data[len("BPM:"):]
                            # print("BPM:", bpm_data)
                            line = "BPM: " + bpm_data
                            print(line)
                        elif data.startswith("RAW:"):
                            sensor_data = data[len("RAW:"):]
                            print("Raw Sensor Data:", sensor_data)
                            line = "Raw Sensor Data: " + sensor_data
                            print(line)

                            # Also save the data into a csv file
                            raw_values = [int(value) for value in sensor_data.split(",")]
                            with open(self.csvfilepath, "w", newline="") as csvfile:
                                csv_writer = csv.writer(csvfile)
                                for raw_value in raw_values:
                                    csv_writer.writerow([sequence, raw_value])


                        # line = data
                        file_log = open(self.pathfilelog, "a")
                        file_log.writelines(line+"\n") # write/append new data from serial port to file

                        # print(line) #write data to output 
                        file_log.close()
                    else:
                        ser.close()

            except Exception as a:
                print ("error open serial port: " + str(a))

        else:
            print("Serial port disconnected. Attempting to reconnect.......")
            try:
                ser.open()
            except Exception as a:
                print ("error open serial port: " + str(a))
            time.sleep(1)


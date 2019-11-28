# Program to control passerelle between Android application
# and micro-controller through USB tty
import time
import argparse
import signal
import sys
import socket
import SocketServer
import serial
import threading

import json
from datetime import timedelta  
import datetime     # for general datetime object handling
import rfc3339      # for date object -> date string
import iso8601      # for date string -> date object
from influxdb import InfluxDBClient


HOST           = "0.0.0.0"
UDP_PORT       = 10000
MICRO_COMMANDS = ["TLH" , "THL" , "LTH" , "LHT" , "HTL" , "HLT"]
FILENAME        = "/home/pi/values.txt"
FILENAME_CONFIG = "/home/pi/config.txt"
LAST_VALUE      = ""

class ThreadedUDPRequestHandler(SocketServer.BaseRequestHandler):

    def handle(self):
        data = self.request[0].strip()
        socket = self.request[1]
        current_thread = threading.current_thread()
        print("{}: client: {}, wrote: {}".format(current_thread.name, self.client_address, data))
        if data != "":
                        if data in MICRO_COMMANDS: # Send message through UART

                                #TO:DO save config in file
                                fconfig= open(FILENAME_CONFIG,"w")
                                fconfig.write(data)
                                fconfig.close()


                                sendUARTMessage(data)
                                socket.sendto("saved", self.client_address) 

                        elif data == "hello": # Send helloBack to tell the client that we exist
                                
				print self.client_address
				socket.sendto("helloBack", self.client_address)      


                        elif data == "getValues()": # Sent last value received from micro-controller
                                socket.sendto(LAST_VALUE, self.client_address) 
                                # TODO: Create last_values_received as global variable   

                        elif data == "getConfig()": # Sent last value received from micro-controller
                                fconfig= open(FILENAME_CONFIG,"r")
                                config = fconfig.read()
                                socket.sendto(config, self.client_address) 
                                fconfig.close()
                                pass
                        else:
                                print("Unknown message: ",data)

class ThreadedUDPServer(SocketServer.ThreadingMixIn, SocketServer.UDPServer):
    pass


# send serial message 
SERIALPORT = "/dev/ttyUSB0"
BAUDRATE = 115200
ser = serial.Serial()

def initUART():        
        # ser = serial.Serial(SERIALPORT, BAUDRATE)
        ser.port=SERIALPORT
        ser.baudrate=BAUDRATE
        ser.bytesize = serial.EIGHTBITS #number of bits per bytes
        ser.parity = serial.PARITY_NONE #set parity check: no parity
        ser.stopbits = serial.STOPBITS_ONE #number of stop bits
        ser.timeout = None          #block read

        # ser.timeout = 0             #non-block read
        # ser.timeout = 2              #timeout block read
        ser.xonxoff = False     #disable software flow control
        ser.rtscts = False     #disable hardware (RTS/CTS) flow control
        ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
        #ser.writeTimeout = 0     #timeout for write
        print ('Starting Up Serial Monitor')
        try:
                ser.open()
        except serial.SerialException:
                print("Serial {} port not available".format(SERIALPORT))
                exit()

def get_date_object(date_string):
  return iso8601.parse_date(date_string)

def get_date_string(date_object):
  return rfc3339.rfc3339(date_object)

def InitDatabaseConnexion(url,port,database):
        client = InfluxDBClient(host=url, port=port, ssl=False, verify_ssl=False)  
        client.switch_database(database)
        # d = datetime.datetime.utcnow() + timedelta(seconds=50)  
        # print d
        # print d.isoformat("T") + "Z"
        # time = str(d.isoformat("T") + "Z")
        # jsonData = [{
        #                 "measurement":"temperature",
        #                 "time" : time,
        #                 "fields":{
        #                         "sensor1" : float(200)
        #                 }

        # }]
        # client.write_points(jsonData)

        # print(client.query('select * from temperature'))

        return client

def InsertDataInfluxDB(client,data):
        d = datetime.datetime.utcnow() + timedelta(seconds=1)  
        time = str(d.isoformat("T") + "Z")

        jsonData = [
        {
                "measurement":"temperature",
                "time" : time,
                "fields":{
                        "sensor1" : float(data["Temp"])
                }

        },
        {
                "measurement":"humidity",
                "time" : time,
                "fields":{
                        "sensor1" : float(data["Humidity"])
                }

        },
        {
                "measurement":"luminosity",
                "time" : time,
                "fields":{
                        "sensor1" : int(data["Lux"])
                }

        },
        ]
        print("date : "+str(time))
        client.write_points(jsonData)
        ## DEBUG
        # print(client.query('select * from temperature'))
        # print(client.query('select * from humidity'))
        # print(client.query('select * from luminosity'))

# def ParseDataReceive(data):
#         raw_data = data
#         #raw_data = '{ "Lux":"030", "Temp":"23.2", "Humidity":"65.5"}'
#         json_data = json.loads(raw_data)

#         ### DEBUG
#         # luxValue = int(json_data["Lux"])
#         # tempValue = float(json_data["Temp"])
#         # humidityValue = float(json_data["Humidity"])
#         # print(luxValue)
#         # print(tempValue)
#         # print(humidityValue)

#         return json_data


def sendUARTMessage(msg):
    ser.write(msg)
    print("Message <" + msg + "> sent to micro-controller." )


# Main program logic follows:
if __name__ == '__main__':
        initUART()
        f= open(FILENAME,"w")
        print ('Press Ctrl-C to quit.')

        client = InitDatabaseConnexion('localhost',8086,'iot')
        # InsertDataInfluxDB(client,ParseDataReceive('{ "Lux":"030", "Temp":"23.2", "Humidity":"65.5"}'))


        

        server = ThreadedUDPServer((HOST, UDP_PORT), ThreadedUDPRequestHandler)

        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.daemon = True

        try:
                server_thread.start()
                print("Server started at {} port {}".format(HOST, UDP_PORT))
                while ser.isOpen() : 
                        #time.sleep(100)
                        if (ser.inWaiting() > 0): # if incoming bytes are waiting 
                                data_str = ser.read(ser.inWaiting()) 
                                f.write(data_str)
                                LAST_VALUE = data_str

                                try:
                                        json_data = json.loads(data_str)
                                        InsertDataInfluxDB(client,json_data)
                                        
                                except:
                                        print("La chaine recu n'est pas en format JSON")


                                #test = ParseDataReceive(data_str)
                                #print(test["Lux"])

                                # InsertDataInfluxDB(client,ParseDataReceive(data_str))
                                print(data_str)


        except (KeyboardInterrupt, SystemExit):
                server.shutdown()
                server.server_close()
                f.close()
                ser.close()
                exit()

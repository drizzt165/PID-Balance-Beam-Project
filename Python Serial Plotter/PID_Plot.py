import serial
import serial.tools.list_ports
import warnings
import matplotlib.pyplot as plt
from drawnow import *
import atexit
import time
import sys

#plotting values
voltageValues = []
adcValues = []
distanceValues = []
kpValues = []
kiValues = []
kdValues = []
timeValues = []

#serial variables
serialCheckStr = ['Arduino','CH340'] #predefined list of possible arduino device names
arduino_ports = []


plt.ion()
maxTime = 10 #x axis time in seconds
period = 50 #period in miliseconds
pointCount = 10/(period/1000) #calculate max time in seconds to run

#find and connect to arduino serial port
for p in serial.tools.list_ports.comports():
    p.device
    if any(x in p.description for x in serialCheckStr): #check for arduino ports using predefined list
        arduino_ports.append(p.device)
print(arduino_ports)
if not arduino_ports:
    raise IOError("No Arduino found")
if len(arduino_ports) > 1:
    warnings.warn('Multiple Arduinos found - using the first')
serialArduino = serial.Serial(arduino_ports[0])  #9600 default baud rate

#initialize first connections
def plotValues():
    plt.title('Distance vs Time')
    plt.grid(True)
    plt.ylabel('Distance [cm]')
    plt.xlabel('Time [s]')
    plt.plot(timeValues,distanceValues)
    plt.legend("Distance",loc='upper right')

def doAtExit():
    serialArduino.close()
    print("Close serial")
    print("serialArduino.isOpen() = " + str(serialArduino.isOpen()))

def addPoints():
    pass

atexit.register(doAtExit)

print("serialArduino.isOpen() = " + str(serialArduino.isOpen()))

#pre-load dummy data
for i in range(0,int(pointCount)):
    voltageValues.append(0)
    adcValues.append(0)
    distanceValues.append(0)
    kpValues.append(0)
    kiValues.append(0)
    kdValues.append(0)
    timeValues.append(0)

while True:
    start = time.time()
    
    while (serialArduino.inWaiting()==0):
        pass
    serialLine = serialArduino.readline(500).strip()
    serialLine = serialLine.decode("utf-8") #decode bytes into string value and strip \r\n off
    serialValues = serialLine.split(',')
    #print(serialValues[0]) # comment out for performance
    #only run loop if the first element is indicating the start of a data row
    if serialValues[0] == 'Start':
        serialValues.pop(0) #remove 'Start' from data since it is only an indicator
        #print(serialValues) #comment out for performance
        try:
            #process string data and assign to variables to plot
            adcValue = float(serialValues[4])
            voltage = float(serialValues[3])
            distance_cm = float(serialValues[5])
            millis = int(serialValues[6]) #current run time for arduino modded with max time

            ## add values to array to plot

            #add new values
            distanceValues.append(distance_cm)
            timeValues.append(millis/1000)

            #remove old values
            distanceValues.pop(0)
            timeValues.pop(0)

            #plot new data
            drawnow(plotValues)
        except ValueError:
            print("Invalid! cannot cast")
        end = time.time()
        executeTime_ms = (end - start)*1000
        
        #clear buffer of serial inputs, to allow python to go as fast as it can
        #This allows for the program to not 'lag' behind arduino output
        #Unfortunately this does reduce the amount of accuracy present by the graph
        #depending on the speed of this python program
        serialArduino.flushInput() 
        #print("Execute time:", round(executeTime_ms,1),"Waiting:",serialArduino.inWaiting()) # comment out for performance
    
from __future__ import absolute_import
from asyncio.windows_events import NULL
import socket
import time
from urllib import request
# from urllib import request
from sympy import Point
from shapely.geometry import Polygon
import keyboard

host, port = "127.0.0.1", 25001
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((host, port))

startPos = [0, 0, 0] #Vector3   x = 0, y = 0, z = 0
receivedData = ""
#get function variable declaration
Steering_Wheel_Angle = 0.0
Throttle_Pedal_Position = 0.0
Brake_Pedal_Position = 0.0
Vehicle_Engine_Torque = 0.0
Vehicle_Brake_Torque = 0.0
Vehicle_RPM = 0.0
Simulation_Time = ""
Absolute_Time = ""
Vehicle_Speed = 0
Vehicle_Position = [0, 0, 0]
Vehicle_Heading = 0.0
Vehicle_Acceleration_Lon = 0.0
Vehicle_Acceleration = [0, 0, 0]

# array for storing position
areaName = []
posTLX = []
posTLZ = []
posTRX = []
posTRZ = []
posBLX = []
posBLZ = []
posBRX = []
posBRZ = []
triggerActive = []

# C# to python message, message used when python sending data to c#, c# needs to take a break before return the message, to prevent the unecessary pause
ctpMessage = ""

# Debug Value
oneTime = True

#GET FUNCTION
###############################################################################################################################################################################################################
def get_Steering_Wheel_Angle():
    request = "getSteer" #send request for values
    sock.sendall(request.encode("UTF-8")) #Converting string to Byte, setnd it to C# as a security code command
    global Steering_Wheel_Angle #announce global right before the variable will be called to overwrite the global variable that has already been defined
    Steering_Wheel_Angle = float(sock.recv(1024).decode("UTF-8")) #receiveing data in Byte fron C#, and converting it to float
    return Steering_Wheel_Angle #converting unicode to float type

def get_Throttle_Pedal_Position():
    request = "getThrottle" #send request for values
    sock.sendall(request.encode("UTF-8")) #Converting string to Byte, setnd it to C# as a security code command
    global Throttle_Pedal_Position #announce global right before the variable will be called to overwrite the global variable that has already been defined
    Throttle_Pedal_Position = float(sock.recv(1024).decode("UTF-8")) #receiveing data in Byte fron C#, and converting it to float
    return Throttle_Pedal_Position

def get_Brake_Pedal_Position():
    request = "getBrake" #send request for values
    sock.sendall(request.encode("UTF-8")) #Converting string to Byte, setnd it to C# as a security code command
    global Brake_Pedal_Position #announce global right before the variable will be called to overwrite the global variable that has already been defined
    Brake_Pedal_Position = float(sock.recv(1024).decode("UTF-8")) #receiveing data in Byte fron C#, and converting it to float
    return Brake_Pedal_Position

def get_Vehicle_Engine_Torque():
    request = "getETorque"
    sock.sendall(request.encode("UTF-8")) #Converting string to Byte, setnd it to C# as a security code command
    global Vehicle_Engine_Torque #announce global right before the variable will be called to overwrite the global variable that has already been defined
    Vehicle_Engine_Torque = float(sock.recv(1024).decode("UTF-8")) #receiveing data in Byte fron C#, and converting it to float
    return Vehicle_Engine_Torque

def get_Vehicle_Brake_Torque():
    request = "getBTorque"
    sock.sendall(request.encode("UTF-8")) #Converting string to Byte, setnd it to C# as a security code command
    global Vehicle_Brake_Torque #announce global right before the variable will be called to overwrite the global variable that has already been defined
    Vehicle_Brake_Torque = float(sock.recv(1024).decode("UTF-8")) #receiveing data in Byte fron C#, and converting it to float
    return Vehicle_Brake_Torque

def get_Vehicle_RPM():
    request = "getRPM"
    sock.sendall(request.encode("UTF-8")) #Converting string to Byte, setnd it to C# as a security code command
    global Vehicle_RPM #announce global right before the variable will be called to overwrite the global variable that has already been defined
    Vehicle_RPM = float(sock.recv(1024).decode("UTF-8")) #receiveing data in Byte fron C#, and converting it to float
    return Vehicle_RPM

def get_Simulation_Time():
    request = "getSimTime"
    sock.sendall(request.encode("UTF-8")) #Converting string to Byte, setnd it to C# as a security code command
    global Simulation_Time #announce global right before the variable will be called to overwrite the global variable that has already been defined
    Simulation_Time = sock.recv(1024).decode("UTF-8") #receiveing data in Byte fron C#, and converting it to String
    return Simulation_Time

def get_Absolute_Time():
    request = "getAbsTime"
    sock.sendall(request.encode("UTF-8")) #Converting string to Byte, setnd it to C# as a security code command
    global Absolute_Time #announce global right before the variable will be called to overwrite the global variable that has already been defined
    Absolute_Time = sock.recv(1024).decode("UTF-8") #receiveing data in Byte fron C#, and converting it to String
    return Absolute_Time

def get_Vehicle_Speed():
    request = "getMPH"
    sock.sendall(request.encode("UTF-8")) #Converting string to Byte, setnd it to C# as a security code command
    global Vehicle_Speed #announce global right before the variable will be called to overwrite the global variable that has already been defined
    Vehicle_Speed = int(sock.recv(1024).decode("UTF-8")) #receiveing data in Byte fron C#, and converting it to int
    return Vehicle_Speed

def get_Vehicle_Position():
    request = "getVehPosX"
    sock.sendall(request.encode("UTF-8")) #Converting string to Byte, setnd it to C# as a security code command
    global Vehicle_Position #announce global right before the variable will be called to overwrite the global variable that has already been defined
    Vehicle_Position[0] = float(sock.recv(1024).decode("UTF-8")) #receiveing data in Byte fron C#, and converting it to float
    request = "getVehPosY"
    sock.sendall(request.encode("UTF-8")) 
    Vehicle_Position[1] = float(sock.recv(1024).decode("UTF-8"))
    request = "getVehPosZ"
    sock.sendall(request.encode("UTF-8")) 
    Vehicle_Position[2] = float(sock.recv(1024).decode("UTF-8"))
    return Vehicle_Position

def get_Vehicle_Heading():
    request = "getVehHeading"
    sock.sendall(request.encode("UTF-8")) #Converting string to Byte, setnd it to C# as a security code command
    global Vehicle_Heading #announce global right before the variable will be called to overwrite the global variable that has already been defined
    Vehicle_Heading = float(sock.recv(1024).decode("UTF-8")) #receiveing data in Byte fron C#, and converting it to int
    return Vehicle_Heading

def get_Vehicle_Acceleration_Lon():
    request = "getVehAccLon"
    sock.sendall(request.encode("UTF-8")) #Converting string to Byte, setnd it to C# as a security code command
    global Vehicle_Acceleration_Lon #announce global right before the variable will be called to overwrite the global variable that has already been defined
    Vehicle_Acceleration_Lon = float(sock.recv(1024).decode("UTF-8")) #receiveing data in Byte fron C#, and converting it to int
    return Vehicle_Acceleration_Lon

def get_Vehicle_Acceleration(): #use vehicle_acceleration[x/y/z] to get acceleration in different axis
    request = "getVehAccX"
    sock.sendall(request.encode("UTF-8")) #Converting string to Byte, setnd it to C# as a security code command
    global Vehicle_Acceleration #announce global right before the variable will be called to overwrite the global variable that has already been defined
    Vehicle_Acceleration[0] = float(sock.recv(1024).decode("UTF-8")) #receiveing data in Byte fron C#, and converting it to float
    request = "getVehAccY"
    sock.sendall(request.encode("UTF-8")) 
    Vehicle_Acceleration[1] = float(sock.recv(1024).decode("UTF-8"))
    request = "getVehAccZ"
    sock.sendall(request.encode("UTF-8")) 
    Vehicle_Acceleration[2] = float(sock.recv(1024).decode("UTF-8"))
    return Vehicle_Acceleration

def get_Trigger_State(triggerAreaName):
    if(triggerAreaName in areaName):  #check if the entered name is in the list of area names
        if triggerActive[areaName.index(triggerAreaName)]:  #if so then check the index of the name, and follow the index, check the status of the trigger area
            return 1
        else:
            return 0

def get_Collision_State():
    request = "getCollisionState"
    sock.sendall(request.encode("UTF-8"))
    isCollided = sock.recv(1024).decode("UTF-8")
    return isCollided

def get_Turn_Signals(): # turn signal 1 = no turn signal, 2 = left turn signal, 3 = right turn signal, 4 = hazard signals
    request = "getTurnSignal"
    sock.sendall(request.encode("UTF-8"))
    turnSignal = int(sock.recv(1024).decode("UTF-8"))
    return turnSignal
    
###############################################################################################################################################################################################################


#SET FUNCTION
###############################################################################################################################################################################################################
def set_Steering_Wheel_Angle(value):
    request = "setSteer " + str(value)
    sock.sendall(request.encode("UTF-8"))
    global ctpMessage
    ctpMessage = sock.recv(1024).decode("UTF-8")
    return

def set_Throttle_Pedal_Position(value):
    request = "setThrot " + str(value)
    sock.sendall(request.encode("UTF-8"))
    global ctpMessage
    ctpMessage = sock.recv(1024).decode("UTF-8")
    return

def set_Brake_Pedal_Position(value):
    request = "setBrake " + str(value)
    sock.sendall(request.encode("UTF-8"))
    global ctpMessage
    ctpMessage = sock.recv(1024).decode("UTF-8")
    return

def set_Vehicle_Speed(value):
    request = "setSpeed " + str(value)
    sock.sendall(request.encode("UTF-8"))
    global ctpMessage
    ctpMessage = sock.recv(1024).decode("UTF-8")
    return

def set_Vehicle_Engine_Toque(value):
    request = "setETorque " + str(value)
    sock.sendall(request.encode("UTF-8"))
    global ctpMessage
    ctpMessage = sock.recv(1024).decode("UTF-8")
    return

def set_Vehicle_Brake_Toque(value):
    request = "setBTorque " + str(value)
    sock.sendall(request.encode("UTF-8"))
    global ctpMessage
    ctpMessage = sock.recv(1024).decode("UTF-8")
    return

def set_Turn_Signal(value):
    request = "setTSignal " + str(value)
    sock.sendall(request.encode("UTF-8"))
    global ctpMessage
    ctpMessage = sock.recv(1024).decode("UTF-8")
    return

def set_Trigger_State(triggerAreaName,triggerState): # first check the name, then trigger state 1 = active, 0 = deactive
    if(triggerAreaName in areaName):  #check if the entered name is in the list of area names
        if(triggerState == 1):
            triggerActive[areaName.index(triggerAreaName)] = True 
        elif(triggerState == 0):
            triggerActive[areaName.index(triggerAreaName)] = False
    return

def add_Trigger(name, pos1X, pos1Z, pos2X, pos2Z, pos3X, pos3Z, pos4X, pos4Z, triggerStatus):

    # add trigger area name, 4 area check points into lists
    global areaName, posTLX, posTLZ, posTRX, posTRZ, posBLX, posBLZ, posBRX, posBRZ, triggerActive
    areaName.append(name)
    posTLX.append(pos1X)
    posTLZ.append(pos1Z)
    posTRX.append(pos2X)
    posTRZ.append(pos2Z)
    posBLX.append(pos3X)
    posBLZ.append(pos3Z)
    posBRX.append(pos4X)
    posBRZ.append(pos4Z)
    triggerActive.append(triggerStatus)

    return

def Delete_Trigger(triggerAreaName):
    # first check if the name is already in the areaName
    if(triggerAreaName in areaName):
        # if so, then check the index of the value in the list, delete every item with the same index in every other lists
        deletePos = areaName.index(triggerAreaName)
        areaName.pop(deletePos)
        posTLX.pop(deletePos)
        posTLZ.pop(deletePos)
        posTRX.pop(deletePos)
        posTRZ.pop(deletePos)
        posBLX.pop(deletePos)
        posBLZ.pop(deletePos)
        posBRX.pop(deletePos)
        posBRZ.pop(deletePos)
        triggerActive.pop(deletePos)
    
    return

def check_if_in_trigger():
    # get the position of 4 of the wheels
    request = "getFLWheelPosX"
    sock.sendall(request.encode("UTF-8"))
    wheelFLX = int(sock.recv(1024).decode("UTF-8"))
    request = "getFLWheelPosZ"
    sock.sendall(request.encode("UTF-8"))
    wheelFLZ = int(sock.recv(1024).decode("UTF-8"))
    request = "getFRWheelPosX"
    sock.sendall(request.encode("UTF-8"))
    wheelFRX = int(sock.recv(1024).decode("UTF-8"))
    request = "getFRWheelPosZ"
    sock.sendall(request.encode("UTF-8"))
    wheelFRZ = int(sock.recv(1024).decode("UTF-8"))
    request = "getRLWheelPosX"
    sock.sendall(request.encode("UTF-8"))
    wheelRLX = int(sock.recv(1024).decode("UTF-8"))
    request = "getRLWheelPosZ"
    sock.sendall(request.encode("UTF-8"))
    wheelRLZ = int(sock.recv(1024).decode("UTF-8"))
    request = "getRRWheelPosX"
    sock.sendall(request.encode("UTF-8"))
    wheelRRX = int(sock.recv(1024).decode("UTF-8"))
    request = "getRRWheelPosZ"
    sock.sendall(request.encode("UTF-8"))
    wheelRRZ = int(sock.recv(1024).decode("UTF-8"))

    # make point for 4 wheels' x&z position
    wheelFL, wheelFR, wheelRL, wheelRR = map(Point, [(wheelFLX, wheelFLZ), 
                                                     (wheelFRX, wheelFRZ), 
                                                     (wheelRLX, wheelRLZ), 
                                                     (wheelRRX, wheelRRZ)])

    global areaName, posTLX, posTLZ, posTRX, posTRZ, posBLX, posBLZ, posBRX, posBRZ

    # loop through for length of array size to run check for all the area
    for i in range(len(posTLX)):

        # make point for check areas
        posTL, posTR, posBL, posBR = map(Point, [(posTLX[i],posTLZ[i]),
                                                 (posTRX[i],posTRZ[i]),
                                                 (posBLX[i],posBLZ[i]),
                                                 (posBRX[i],posBRZ[i])])

        # make two polygon using 4 wheel points and 4 trigger area points
        areaPoly = Polygon([posTL, posTR, posBL, posBR])
        wheelPoly = Polygon([wheelFL, wheelFR, wheelRL, wheelRR])

        # check if wheel position polygon is inside area check polygon, also make sure if the area is active or not
        if areaPoly.contains(wheelPoly) and triggerActive[i] == True:
            print(areaName[i])

    return
    
def set_Vehicle_Position(posX,posY,posZ):
    request = "setVehPosX " + str(posX)
    sock.sendall(request.encode("UTF-8"))
    global ctpMessage
    ctpMessage = sock.recv(1024).decode("UTF-8")
    request = "setVehPosY " + str(posY)
    sock.sendall(request.encode("UTF-8"))
    ctpMessage = sock.recv(1024).decode("UTF-8")
    request = "setVehPosZ " + str(posZ)
    sock.sendall(request.encode("UTF-8"))
    ctpMessage = sock.recv(1024).decode("UTF-8")
    return


###############################################################################################################################################################################################################


#RESET FUNCTION
###############################################################################################################################################################################################################
def reset_Steering_Wheel_Angle():
    request = "reSteer"
    sock.sendall(request.encode("UTF-8"))
    global ctpMessage                               #added so that multiple reset information can be read correctly
    ctpMessage = sock.recv(1024).decode("UTF-8")
    return

def reset_Throttle_Pedal_Position():
    request = "reThrot"
    sock.sendall(request.encode("UTF-8"))
    global ctpMessage
    ctpMessage = sock.recv(1024).decode("UTF-8")
    return

def reset_Brake_Pedal_Position():
    request = "reBrake"
    sock.sendall(request.encode("UTF-8"))
    global ctpMessage
    ctpMessage = sock.recv(1024).decode("UTF-8")
    return

def reset_Vehicle_Speed():
    request = "reSpeed"
    sock.sendall(request.encode("UTF-8"))
    global ctpMessage
    ctpMessage = sock.recv(1024).decode("UTF-8")
    return

def reset_Vehicle_Engine_Toque():
    request = "reETorque"
    sock.sendall(request.encode("UTF-8"))
    global ctpMessage
    ctpMessage = sock.recv(1024).decode("UTF-8")
    return

def reset_Vehicle_Brake_Toque():
    request = "reBTorque"
    sock.sendall(request.encode("UTF-8"))
    global ctpMessage
    ctpMessage = sock.recv(1024).decode("UTF-8")
    return

def reset_Turn_Signal():
    request = "reTSignal"
    sock.sendall(request.encode("UTF-8"))
    global ctpMessage
    ctpMessage = sock.recv(1024).decode("UTF-8")
    return

###############################################################################################################################################################################################################


#RUNNING CONDITION
while True:
    time.sleep(0.02) #sleep 0.5 sec
    # startPos[0] += 0 #increase x by one
    # posString = ','.join(map(str, startPos)) #Converting Vector3 to a string, example "0,0,0"
    #print(posString)

    # sock.sendall(request.encode("UTF-8")) # Converting string to Byte, setnd it to C# as a security code command
    # receivedData = sock.recv(1024).decode("UTF-8") #receiveing data in Byte fron C#, and converting it to String

    # print(receivedData[0:5])  # collect exact character 0 to 5 from a string

    if oneTime == True:
        add_Trigger("apple", 390, -250, 407, -268, 391, -280, 380, -264, True)
        add_Trigger("juice", 430, -214, 446, -231, 431, -243, 417, -228, True)
        add_Trigger("fruit", 1, 2, 3, 4, 5, 6, 7, 8, True)

        oneTime = False


######################################################### SET/RESET TEST
    # if keyboard.is_pressed('q'):
    #     set_Steering_Wheel_Angle(0.5)
    # elif keyboard.is_pressed('w'):
    #     reset_Steering_Wheel_Angle()
    # elif keyboard.is_pressed('e'):
    #     set_Throttle_Pedal_Position(0.7)
    # elif keyboard.is_pressed('r'):
    #     reset_Throttle_Pedal_Position()
    # elif keyboard.is_pressed('t'):
    #     set_Brake_Pedal_Position(0.8)
    # elif keyboard.is_pressed('y'):
    #     reset_Brake_Pedal_Position()
    # elif keyboard.is_pressed('u'):
    #     set_Vehicle_Speed(20)
    # elif keyboard.is_pressed('i'):
    #     reset_Vehicle_Speed()
    # elif keyboard.is_pressed('o'):
    #     set_Vehicle_Engine_Toque(1500)
    # elif keyboard.is_pressed('p'):
    #     reset_Vehicle_Engine_Toque()
    # elif keyboard.is_pressed('a'):
    #     set_Vehicle_Brake_Toque(500)
    # elif keyboard.is_pressed('s'):
    #     reset_Vehicle_Brake_Toque()
    # elif keyboard.is_pressed('d'):
    #     set_Turn_Signal(2)
    # elif keyboard.is_pressed('f'):
    #     reset_Turn_Signal()
    # elif keyboard.is_pressed('g'):
    #     set_Vehicle_Position(430.1,106.48,-214.3)
    # elif keyboard.is_pressed('h'):
    #     check_if_in_trigger()
    # elif keyboard.is_pressed('j'):
    #     Delete_Trigger("juice")

######################################################### GET TEST
    # if keyboard.is_pressed('q'):
    #     print(get_Steering_Wheel_Angle())
    # elif keyboard.is_pressed('w'):
    #     print(get_Throttle_Pedal_Position())
    # elif keyboard.is_pressed('e'):
    #     print(get_Brake_Pedal_Position())
    # elif keyboard.is_pressed('r'):
    #     print(get_Vehicle_Engine_Torque())
    # elif keyboard.is_pressed('t'):
    #     print(get_Vehicle_Brake_Torque())
    # elif keyboard.is_pressed('y'):
    #     print(get_Vehicle_RPM())
    # elif keyboard.is_pressed('u'):
    #     print(get_Simulation_Time())
    # elif keyboard.is_pressed('i'):
    #     print(get_Absolute_Time())
    # elif keyboard.is_pressed('o'):
    #     print(get_Vehicle_Speed())
    # elif keyboard.is_pressed('p'):
    #     print(get_Vehicle_Position())
    # elif keyboard.is_pressed('a'):
    #     print(get_Vehicle_Heading())
    # elif keyboard.is_pressed('s'):
    #     print(get_Vehicle_Acceleration_Lon())
    # elif keyboard.is_pressed('d'):
    #     print(get_Vehicle_Acceleration())
    # elif keyboard.is_pressed('f'):
    #     print(get_Collision_State())
    # elif keyboard.is_pressed('g'):
    #     print(get_Turn_Signals())
    # elif keyboard.is_pressed('h'):
    #     print(get_Trigger_State("apple"))

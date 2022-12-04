from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

import argparse  

import socket
import serial
import array as arr
import RPi.GPIO as GPIO
from multiprocessing import Process
##########################################################################################
led1=11
led2=13
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(led1,GPIO.OUT)
GPIO.setup(led2,GPIO.OUT)
GPIO.output(led1,GPIO.LOW) 
GPIO.output(led2,GPIO.LOW)

ser = serial.Serial('/dev/ttyUSB0')  
ser.write(b'AT')
time.sleep(0.8)
ser.write(b'AT+MODE=TEST')     
time.sleep(0.8)
ser.write(b'AT+TEST=RFCFG,866,SF12,125,12,15,14,ON,OFF,OFF')    
time.sleep(0.8)

index8=0
stopsm=0
stopsm2=0
Lstatus=0
Laltitude=0
LX=0
LY=0

arraystr=[]
arraystrvalue=arr.array('i',[0,0,0,0,0,0,0,0,0,0])

start=0

print("lora ready")
ser.write(b'AT+TEST=RXLRPKT')

##########################################################################################

def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    
    #connection_string = args.connect 
    connection_string ="/dev/ttyS0"
    #connection_string = "127.0.0.1:14550"
    baud_rate = 57600
    
    vehicle = connect(connection_string,baud=baud_rate,wait_ready=True)
    return vehicle

def get_location_metres(original_location, dNorth, dEast):
    """Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`."""
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

def get_distance_metres(aLocation1, aLocation2):
    #Returns the ground distance in metres between two LocationGlobal objects.
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.

def adds_square_mission(aLocation, aSize):

    cmds = vehicle.commands

    print(" Clear any existing commands")
    cmds.clear() 
    
    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_metres(aLocation, aSize, -aSize)
    point2 = get_location_metres(aLocation, aSize, aSize)
    point3 = get_location_metres(aLocation, -aSize, aSize)
    point4 = get_location_metres(aLocation, -aSize, -aSize)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))    

    print(" Upload new commands to vehicle")
    cmds.upload()

def arm():
    while vehicle.is_armable==False:
        print("Waiting for vehicle to become armable..")
        time.sleep(1)
    print("Yooo vehivle is now armable")
    print("")
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed=True
    while vehicle.armed==False:
        print("Waiting for frone to become armed..")
        time.sleep(1)

    print("Vehicle is now armed")
    print("OMG props are spinning.Look out !!!")
#     print ("Airspeed: %s" % vehicle.airspeed)
#     print ("GPS: %s" % vehicle.gps_0)
#     print ("Velocity: %s" % vehicle.velocity)
    return None
    
def takeoff(aTargetAltitude):
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
 
def set_velocity_body(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()    
    
def controlfromL(Lstatus,Laltitude,LX,LY):
    if Lstatus==0: #-- standard keys
        print("pot 0 pressed >> Nothing will be")
        vehicle.mode = VehicleMode("LAND")
        GPIO.output(led2,GPIO.HIGH) 
    #elif Lstatus==1: #-- standard keys
    elif Lstatus==2: #-- standard keys
        print("pot 2 pressed >> Set the vehicle to RTL")
        vehicle.mode = VehicleMode("RTL")
        GPIO.output(led2,GPIO.HIGH) 
            
    else: #-- non standard keys
        #Up
        if LY == 2 : 
            print("Up")
            #set_velocity_body(vehicle, gnd_speed, 0, 0)
        #Down
        elif LY == 1:
            print("Down")
            #set_velocity_body(vehicle,-gnd_speed, 0, 0)
        #Left
        elif LX == 2:
            print("Left")
            #set_velocity_body(vehicle, 0, -gnd_speed, 0)
        #Right
        elif LX == 1:
            print("Right")
            #set_velocity_body(vehicle, 0, gnd_speed, 0)
        LX=0
        LY=0 
    Lstatus=0

    """"targetHeight = Laltitude
    vehicle.simple_takeoff(targetHeight) ##meters
	while True:
		print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
			break
		time.sleep(1)
	print("Target altitude reached!!")""" 

    return None
    
def lora():
    global stopsm
    global stopsm2
    global Lstatus
    global Laltitude
    global LX
    global LY
    global index8
    status=ser.read
    while status:
        processing=str((ser.read(1)),'utf-8')
        arraystr.append(processing)
        processize=len(arraystr)
        index8=processing.rfind('"')
        if index8!=-1:
            if stopsm==0:
                stopsm=processize
            else:
                stopsm2=processize
            #print("stopsm",stopsm)
            if stopsm2-stopsm==5:
                if stopsm2!=0:
                    for i in range(2,6):
                        print(arraystr[processize-i])
                    stopsm=0
                    stopsm2=0
                    Lstatus=int(arraystr[processize-5])
                    Laltitude=int(arraystr[processize-4])
                    LX=int(arraystr[processize-3])
                    LY=int(arraystr[processize-2])
    #                 status=0
        if stopsm2==1:
            stopsm=0
            stopsm2=0
        if processize>1000:
            arraystr.clear()
            processize=0
        controlfromL(Lstatus,Laltitude,LX,LY)  
        
    return None 
    
def dronemiss():   
    print("drone mission started")
    vehicle.mode = VehicleMode("GUIDED")
    print("mode is look:",vehicle.mode.name)
    time.sleep(5)
    arm()
    #takeoff(10)
    time.sleep(5)
    
    print("Starting mission")
    """
    # Reset mission set to first (0) waypoint
    vehicle.commands.next=0

    # Set mode to AUTO to start mission
    vehicle.mode = VehicleMode("AUTO")
    
    while True:
        nextwaypoint=vehicle.commands.next
        print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
      
        if nextwaypoint==5: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print("Exit 'standard' mission when start heading to final waypoint (5)")
            break;
        time.sleep(1)

    print('Return to launch')
    vehicle.mode = VehicleMode("RTL")"""
    #Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.mode = VehicleMode("RTL")
    vehicle.close()
    
    return None
    
####################################--Main_code--############################################################################################################ 

vehicle=connectMyCopter()
print("connected")

status=ser.read
while status:
    print("in loop")
    GPIO.output(led1,GPIO.HIGH) 
    processing=str((ser.read(1)),'utf-8')
    arraystr.append(processing)
    processize=len(arraystr)
    index8=processing.rfind('"')
    if index8!=-1:
        if stopsm==0:
            stopsm=processize
        else:
            stopsm2=processize
        #print("stopsm",stopsm)
        if stopsm2-stopsm==5:
            if stopsm2!=0:
                for i in range(2,6):
                    print(arraystr[processize-i])
                stopsm=0
                stopsm2=0
                Lstatus=int(arraystr[processize-5])
                if Lstatus==1:
                    print("recognized")
                    status=0
                #Laltitude=int(arraystr[processize-4])
                #LX=int(arraystr[processize-3])
                #LY=int(arraystr[processize-2])
#                 status=0
    if stopsm2==1:
        stopsm=0
        stopsm2=0
    if processize>1000:
        arraystr.clear()
        processize=0

    
GPIO.output(led1,GPIO.LOW)    
print("Begin of scripts")
task1=Process(target=lora)
task2=Process(target=dronemiss)
#en azından ilk arm yapılırken llorayı diskalifiye et yada eldivene dikkat et saçmalıyor
task1.start()
task2.start()
task1.join()
task2.join()
print("End of scripts")

print('Return to launch')
vehicle.mode = VehicleMode("RTL")

#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()











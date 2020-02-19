#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
mission_basic.py: Example demonstrating basic mission operations including creating, clearing and monitoring missions.

Full documentation is provided at http://python.dronekit.io/examples/mission_basic.html
"""
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

# Set up option parsing to get connection string
import argparse

parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect',
                    help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl

    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobal(newlat, newlon, original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def distance_to_current_waypoint(point):
    """
    Gets distance in metres to the current waypoint.
    It returns None for the first waypoint (Home location).
    """
    targetWaypointLocation = LocationGlobalRelative(point.lat, point.lon, point.alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()  # wait until download is complete.


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
###########################################
# Get Vehicle Home location - will be `None` until first set by autopilot
while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    if not vehicle.home_location:
        print(" Waiting for home location ...")
# We have a home location, so print it!
print("\n Home location: %s" % vehicle.home_location)


# Set vehicle home_location, mode, and armed attributes (the only settable attributes)

print("\nSet new home location")
# Home location must be within 50km of EKF home location (or setting will fail silently)
# In this case, just set value to current location with an easily recognisable altitude (222)
my_location_alt = vehicle.location.global_frame
my_location_alt.lat=39.781759
my_location_alt.lon=30.514894
vehicle.home_location = my_location_alt
print(" New Home Location (from attribute - altitude should be 222): %s" % vehicle.home_location)

#Confirm current value on vehicle by re-downloading commands
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
print(" New Home Location (from vehicle - altitude should be 222): %s" % vehicle.home_location)


##############################################



print("\n Last time Home location: %s" % vehicle.home_location)

arm_and_takeoff(10)

print("Set default/target airspeed to 3")
vehicle.airspeed = 3

print("Going towards first point for 30 seconds ...")
point1 = get_location_metres(vehicle.location.global_frame, 50, -50)
vehicle.simple_goto(point1, groundspeed=3)

flight_time=0;

while True:
    print('Distance left to point1 :%s'%(distance_to_current_waypoint(point1)))
    print("Set default/target ground speed for point1  :%s" % vehicle.groundspeed)
    if distance_to_current_waypoint(point1) < 2 :
        print('Reached Point1. It takes %s second to reach.'%flight_time)
        break ;
    flight_time=flight_time+2;
    time.sleep(2)

print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
point2 =get_location_metres(vehicle.location.global_frame, 50, 50)
vehicle.simple_goto(point2, airspeed=10, groundspeed=10)

flight_time=0;

while True:
    print('Distance left to point2 :%s' % (distance_to_current_waypoint(point2)))
    print("Set default/target ground speed for point2:%s" % vehicle.groundspeed)
    if distance_to_current_waypoint(point2) < 2:
        print('Reached Point2. It takes %s second to reach.'%flight_time)
        break ;
    flight_time = flight_time + 2;
    time.sleep(2)


print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()


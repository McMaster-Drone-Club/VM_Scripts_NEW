#!/usr/bin/env python

"""
Emergency Return to Home Script
Run this to cancel current mission and return drone to original takeoff position
"""

import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative

# Original takeoff position (update these values if needed)
ORIGINAL_LAT = -35.363253
ORIGINAL_LON = 149.165234
ORIGINAL_ALT = 5.0  # Return altitude in meters

print("="*50)
print("EMERGENCY RETURN TO HOME")
print("="*50)

# Connect to vehicle
print("Connecting to vehicle...")
vehicle = connect('tcp:127.0.0.1:5763', wait_ready=True)
print("Connected!")

# Check if vehicle is armed
if not vehicle.armed:
    print("Vehicle is not armed. Nothing to do.")
    print("Exiting...")
    vehicle.close()
    exit(0)

print("\nCurrent position:")
print("  Lat: %.8f" % vehicle.location.global_relative_frame.lat)
print("  Lon: %.8f" % vehicle.location.global_relative_frame.lon)
print("  Alt: %.2f m" % vehicle.location.global_relative_frame.alt)

print("\nTarget home position:")
print("  Lat: %.8f" % ORIGINAL_LAT)
print("  Lon: %.8f" % ORIGINAL_LON)
print("  Alt: %.2f m" % ORIGINAL_ALT)

# Switch to GUIDED mode
print("\nSwitching to GUIDED mode...")
vehicle.mode = VehicleMode('GUIDED')
while vehicle.mode != 'GUIDED':
    print('Waiting for GUIDED mode...')
    time.sleep(1)
print('In GUIDED mode!')

# Return to original position
print("\nReturning to home position...")
home_location = LocationGlobalRelative(ORIGINAL_LAT, ORIGINAL_LON, ORIGINAL_ALT)
vehicle.simple_goto(home_location)

# Monitor return progress
while True:
    current = vehicle.location.global_relative_frame
    
    # Calculate distance to home
    dlat = current.lat - ORIGINAL_LAT
    dlong = current.lon - ORIGINAL_LON
    distance = math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5  # rough meters
    
    print('Distance to home: %.2f m | Alt: %.2f m' % (distance, current.alt))
    
    if distance < 1.0:  # Within 1 meter
        print('\nReached home position!')
        break
    
    time.sleep(1)

# Hover briefly
print("Hovering for 2 seconds...")
time.sleep(2)

# Land
print("\nLanding...")
vehicle.mode = VehicleMode('LAND')

# Wait for landing
while vehicle.armed:
    alt = vehicle.location.global_relative_frame.alt
    print('Landing... Altitude: %.2f m' % alt)
    time.sleep(1)

print("\n" + "="*50)
print("SUCCESSFULLY LANDED AT HOME")
print("="*50)
print("Mission aborted and drone returned safely.")
print("You can now run your normal script again.")
print("="*50)

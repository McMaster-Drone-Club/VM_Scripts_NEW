#!/usr/bin/env python
# -*- coding: utf-8 -*-

########IMORTS#########

import rospy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import sys
import time
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from array import array

#######VARIABLES########

vehicle = connect('tcp:127.0.0.1:5763',wait_ready=True)
vehicle.parameters['PLND_ENABLED']=1
vehicle.parameters['PLND_TYPE']=1
vehicle.parameters['PLND_EST_TYPE']=0
vehicle.parameters['LAND_SPEED']=30 ##cms/s

velocity=-.3 #m/s (reduced for smoother flight)
takeoff_height=8 #m

# Store original position
original_position = None
########################
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

id_to_find = 72 ##arucoID (not used anymore - detects all IDs)
marker_size = 40 ##CM - CALIBRATED for Gazebo rendering (actual SDF size is 40cm)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = 640
vertical_res = 480

horizontal_fov = 62.2 * (math.pi / 180) ##62.2 for picam V2, 53.5 for V1
vertical_fov = 48.8 * (math.pi / 180) ##48.8 for V2, 41.41 for V1

found_count=0
notfound_count=0
marker_found = False
detected_markers = []  # Store all detected markers as a list (not dict by ID)

#############CAMERA INTRINSICS#######

dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix = [[530.8269276712998, 0.0, 320.5],[0.0, 530.8269276712998, 240.5],[0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

#####
time_last=0
time_to_wait = .1 ##100 ms
################FUNCTIONS###############
def arm_and_takeoff(targetHeight):
    global original_position
    
    while vehicle.is_armable !=True:
        print('Waiting for vehicle to become armable')
        time.sleep(1)
    print('Vehicle is now armable')

    vehicle.mode = VehicleMode('GUIDED')

    while vehicle.mode !='GUIDED':
        print('Waiting for drone to enter GUIDED flight mode')
        time.sleep(1)
    print('Vehicle now in GUIDED mode. Have Fun!')

    vehicle.armed = True
    while vehicle.armed ==False:
        print('Waiting for vehicle to become armed.')
        time.sleep(1)
    print('Look out! Virtual props are spinning!')

    vehicle.simple_takeoff(targetHeight)

    while True:
        print('Current Altitude: %d'%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >=.95*targetHeight:
            break
        time.sleep(1)
    print('Target altitude reached!')
    
    # Store original position after takeoff
    original_position = vehicle.location.global_relative_frame
    print('Original position saved: Lat=%f, Lon=%f, Alt=%f' % 
          (original_position.lat, original_position.lon, original_position.alt))

    return None

##Send velocity command to drone
def send_local_ned_velocity(vx,vy,vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0,
        0,
        0,
        vx,
        vy,
        vz,
        0,0,0,0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def send_velocity_smooth(vx, vy, vz, duration):
    """Send velocity commands continuously for smooth movement"""
    end_time = time.time() + duration
    while time.time() < end_time:
        send_local_ned_velocity(vx, vy, vz)
        time.sleep(0.1)  # Send command every 100ms for smooth control


def calculate_marker_gps(drone_lat, drone_lon, drone_alt, x_offset_cm, y_offset_cm, distance_cm):
    """
    Calculate the actual GPS coordinates of the marker based on drone position and offsets.
    
    Args:
        drone_lat, drone_lon: Drone's GPS position
        drone_alt: Drone's altitude in meters
        x_offset_cm, y_offset_cm: Marker offset from camera center in cm (camera frame)
        distance_cm: Distance from camera to marker in cm (Z-axis)
    
    Returns:
        Dictionary with marker's estimated GPS coordinates
    """
    # Convert cm to meters
    x_offset_m = x_offset_cm / 100.0
    y_offset_m = y_offset_cm / 100.0
    distance_m = distance_cm / 100.0
    
    # In camera frame:
    # X-axis points right, Y-axis points down, Z-axis points forward (into scene)
    # We need to transform to NED (North-East-Down) frame
    
    # Assuming camera is pointing straight down:
    # Camera X -> East (longitude)
    # Camera Y -> South (latitude) 
    # Camera Z -> Down (altitude difference)
    
    # Calculate marker altitude (marker is below drone)
    marker_alt = drone_alt - distance_m
    
    # Calculate horizontal offsets in meters
    # Note: These are approximations assuming camera points straight down
    north_offset_m = -y_offset_m  # Negative Y in camera = North
    east_offset_m = x_offset_m    # Positive X in camera = East
    
    # Convert offsets to GPS coordinate changes
    # 1 degree latitude = 111,320 meters (approximately)
    # 1 degree longitude = 111,320 * cos(latitude) meters (approximately)
    
    dlat = north_offset_m / 111320.0
    dlon = east_offset_m / (111320.0 * math.cos(drone_lat * math.pi / 180.0))
    
    marker_lat = drone_lat + dlat
    marker_lon = drone_lon + dlon
    
    return {
        'lat': marker_lat,
        'lon': marker_lon,
        'alt': marker_alt
    }


def send_land_message(x,y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,0,0
        )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def return_to_original():
    """Return to original takeoff position"""
    global original_position
    
    print("Returning to original position...")
    
    # Switch back to GUIDED mode
    vehicle.mode = VehicleMode('GUIDED')
    while vehicle.mode != 'GUIDED':
        print('Waiting for GUIDED mode...')
        time.sleep(1)
    print('In GUIDED mode, returning to start...')
    
    # Go to original position
    vehicle.simple_goto(original_position)
    
    # Wait until close to original position
    while True:
        current = vehicle.location.global_relative_frame
        # Calculate distance
        dlat = current.lat - original_position.lat
        dlong = current.lon - original_position.lon
        distance = math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5  # rough meters
        
        print('Distance to original position: %.2f m' % distance)
        
        if distance < 1.0:  # Within 1 meter
            print('Reached original position!')
            break
        time.sleep(1)
    
    # Hover for a moment
    time.sleep(2)
    print('Return complete!')



def msg_receiver(message):
    global notfound_count, found_count, time_last, time_to_wait, marker_found, detected_markers

    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(message) ##Deserialize image data into array
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)

        try:
            if ids is not None:
                # Process ALL detected markers
                for i in range(len(ids)):
                    marker_id = int(ids[i][0])
                    
                    ret = aruco.estimatePoseSingleMarkers([corners[i]],marker_size,cameraMatrix=np_camera_matrix,distCoeffs=np_dist_coeff)
                    (rvec, tvec) = (ret[0][0,0,:],ret[1][0,0,:])
                    
                    # Get current drone position
                    current_lat = vehicle.location.global_relative_frame.lat
                    current_lon = vehicle.location.global_relative_frame.lon
                    current_alt = vehicle.location.global_relative_frame.alt
                    
                    # Calculate marker's approximate GPS location
                    # Using drone position and marker offset
                    distance_m = float(tvec[2]) / 100.0  # Convert cm to meters
                    
                    # Calculate marker's actual GPS coordinates
                    marker_gps = calculate_marker_gps(
                        current_lat, current_lon, current_alt,
                        float(tvec[0]), float(tvec[1]), float(tvec[2])
                    )
                    
                    # Check if this is a duplicate detection
                    # Compare against ALL previously detected markers (not just same ID)
                    is_duplicate = False
                    for existing_marker in detected_markers:
                        # Calculate distance between this marker location and existing marker
                        dlat = marker_gps['lat'] - existing_marker['marker_lat']
                        dlon = marker_gps['lon'] - existing_marker['marker_lon']
                        marker_distance = math.sqrt((dlat*dlat) + (dlon*dlon)) * 1.113195e5
                        
                        # If within 3 meters of an existing marker with same ID, it's a duplicate
                        # (Using 3m threshold since markers are small and close together)
                        if marker_distance < 3.0 and marker_id == existing_marker['marker_id']:
                            is_duplicate = True
                            # Update detection count for existing marker
                            existing_marker['detection_count'] += 1
                            break
                    
                    # Only save if it's a new detection
                    if not is_duplicate:
                        new_marker = {
                            'marker_id': marker_id,
                            'drone_lat': current_lat,
                            'drone_lon': current_lon,
                            'drone_alt': current_alt,
                            'marker_lat': marker_gps['lat'],
                            'marker_lon': marker_gps['lon'],
                            'marker_alt': marker_gps['alt'],
                            'distance_cm': float(tvec[2]),
                            'x_offset_cm': float(tvec[0]),
                            'y_offset_cm': float(tvec[1]),
                            'detection_count': 1
                        }
                        detected_markers.append(new_marker)
                        marker_found = True
                        print('NEW MARKER DETECTED: ID %d at location #%d' % (marker_id, len(detected_markers)))
                        print('  Drone position: Lat=%.6f, Lon=%.6f, Alt=%.2f' % 
                              (current_lat, current_lon, current_alt))
                        print('  Marker position: Lat=%.6f, Lon=%.6f, Alt=%.2f' % 
                              (marker_gps['lat'], marker_gps['lon'], marker_gps['alt']))
                        print('  Distance: %.2f cm (%.2f m)' % (new_marker['distance_cm'], new_marker['distance_cm']/100.0))
                        print('  X/Y offsets: %.2f, %.2f cm' % (new_marker['x_offset_cm'], new_marker['y_offset_cm']))
                        print('  Total markers found: %d' % len(detected_markers))
                    
                    # Visual feedback for all markers
                    x = '{:.2f}'.format(tvec[0])
                    y = '{:.2f}'.format(tvec[1])
                    z = '{:.2f}'.format(tvec[2])
                    marker_position = 'ID %d: x=%s y=%s z=%s' % (marker_id, x, y, z)
                    
                    aruco.drawDetectedMarkers(np_data,[corners[i]],[ids[i]])
                    aruco.drawAxis(np_data,np_camera_matrix,np_dist_coeff,rvec,tvec,10)
                    cv2.putText(np_data,marker_position,(10,50+i*30),0,.5,(255,0,0),thickness=2)
                
                found_count = found_count + 1
            else:
                notfound_count=notfound_count+1
        except Exception as e:
            print('Target likely not found')
            print(e)
            notfound_count=notfound_count+1
        
        new_msg = rnp.msgify(Image, np_data,encoding='rgb8')
        newimg_pub.publish(new_msg)
        time_last = time.time()
    else:
        return None


def check_for_marker_quick():
    """Quick check for marker - just 10 frames"""
    global marker_found
    marker_found = False
    
    rospy.init_node('drone_node',anonymous=False)
    sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)
    
    # Check for marker for 1 second (about 10 frames at 10 Hz)
    start_time = time.time()
    while time.time() - start_time < 1.0 and not marker_found:
        time.sleep(0.1)
    
    sub.unregister()
    return marker_found


def search_pattern():
    """Execute lawnmower search pattern over the house roof"""
    
    # House is 6m x 6m, so we'll scan it with overlapping passes
    # Start at corner (-3, -3) relative to house center
    
    scan_speed = 0.3  # m/s
    stripe_width = 2.0  # meters between passes
    scan_duration = 6.0 / scan_speed  # time to cross 6m house
    num_stripes = 4  # number of passes to cover the house
    
    print("Starting lawnmower search pattern over house...")
    
    # Move to starting corner (back-left of house)
    print("Moving to starting corner...")
    send_velocity_smooth(velocity, velocity, 0, 5)  # move back and left
    send_local_ned_velocity(0, 0, 0)
    time.sleep(0.5)
    
    for stripe in range(num_stripes):
        print('Scanning stripe %d/%d...' % (stripe + 1, num_stripes))
        
        # Scan forward across the house
        if stripe % 2 == 0:
            # Even stripes: scan forward
            direction = -scan_speed
            print("Scanning FORWARD...")
        else:
            # Odd stripes: scan backward
            direction = scan_speed
            print("Scanning BACKWARD...")
        
        # Move across while checking frequently
        segments = 6  # Check marker 6 times during each pass
        segment_time = scan_duration / segments
        
        for seg in range(segments):
            send_velocity_smooth(direction, 0, 0, segment_time)
            send_local_ned_velocity(0, 0, 0)
            
            # Quick marker check (but don't stop searching)
            check_for_marker_quick()
        
        # Stop at end of stripe
        send_local_ned_velocity(0, 0, 0)
        time.sleep(0.5)
        
        # Move to next stripe (if not last)
        if stripe < num_stripes - 1:
            print("Moving to next stripe...")
            send_velocity_smooth(0, -stripe_width / scan_speed, 0, stripe_width / scan_speed)
            send_local_ned_velocity(0, 0, 0)
            time.sleep(0.5)
            
            # Check marker at stripe transition
            check_for_marker_quick()
    
    print("Lawnmower search pattern complete.")
    return len(detected_markers) > 0  # Return True if any markers found


def subscriber():
    """Monitor for marker detection but don't land"""
    rospy.init_node('drone_node',anonymous=False)
    sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)
    
    # Just monitor, don't land
    print("Monitoring for marker (not landing)...")
    
    sub.unregister()
    return True


if __name__=='__main__':
    try:
        arm_and_takeoff(takeoff_height)
        time.sleep(1)
        
        # Fly to house center at (10, 10)
        print("Flying to house location...")
        house_location = LocationGlobalRelative(
            vehicle.location.global_relative_frame.lat + (10.0 / 111320.0),  # 10m north
            vehicle.location.global_relative_frame.lon - (10.0 / (111320.0 * math.cos(vehicle.location.global_relative_frame.lat * math.pi / 180))),  # 10m west
            8  # 8m altitude
        )
        vehicle.simple_goto(house_location)
        
        # Wait until close to house
        while True:
            current = vehicle.location.global_relative_frame
            dlat = current.lat - house_location.lat
            dlong = current.lon - house_location.lon
            distance = math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
            print('Distance to house: %.2f m' % distance)
            if distance < 1.0:  # Within 1 meter
                print('Arrived at house!')
                break
            time.sleep(1)
        
        time.sleep(2)  # Stabilize
        
        # Execute search pattern around house
        marker_found = search_pattern()
        
        # Report all detected markers
        if detected_markers:
            print("\n" + "="*60)
            print("ALL MARKERS DETECTED - RETURNING TO BASE")
            print("="*60)
            print("Total markers found: %d" % len(detected_markers))
            
            # Count markers by ID
            id_counts = {}
            for marker in detected_markers:
                mid = marker['marker_id']
                id_counts[mid] = id_counts.get(mid, 0) + 1
            
            print("Marker ID distribution:")
            for mid in sorted(id_counts.keys()):
                print("  ID %d: %d marker(s)" % (mid, id_counts[mid]))
            print("")
            
            for i, marker in enumerate(detected_markers):
                print("Marker #%d - ArUco ID: %d" % (i+1, marker['marker_id']))
                print("  Drone Position (when detected):")
                print("    Latitude:  %.8f" % marker['drone_lat'])
                print("    Longitude: %.8f" % marker['drone_lon'])
                print("    Altitude:  %.2f m" % marker['drone_alt'])
                print("  Marker Estimated Position:")
                print("    Latitude:  %.8f" % marker['marker_lat'])
                print("    Longitude: %.8f" % marker['marker_lon'])
                print("    Altitude:  %.2f m" % marker['marker_alt'])
                print("  Measurement Info:")
                print("    Distance:  %.2f cm" % marker['distance_cm'])
                print("    X Offset:  %.2f cm" % marker['x_offset_cm'])
                print("    Y Offset:  %.2f cm" % marker['y_offset_cm'])
                print("    Detections: %d" % marker['detection_count'])
                print("")
            print("="*60 + "\n")
            
            # Return to original position
            return_to_original()
            
            # Land at original position
            print("Landing at original position...")
            vehicle.mode = VehicleMode('LAND')
            while vehicle.armed:
                time.sleep(1)
            print("Landed at original position.")
            
            # Final report
            print("\n" + "="*60)
            print("MISSION COMPLETE - FINAL REPORT")
            print("="*60)
            print("Reconnaissance mission successful!")
            print("Detected %d total marker(s):" % len(detected_markers))
            for i, marker in enumerate(detected_markers):
                print("  #%d: ID %d at GPS(%.6f, %.6f), altitude %.2fm" % 
                      (i+1, marker['marker_id'], marker['marker_lat'], 
                       marker['marker_lon'], marker['marker_alt']))
            print("="*60 + "\n")
            
        else:
            print("Marker not found in search pattern.")
            # Return to original position before landing
            return_to_original()
            
            # Land at original position
            print("Landing at original position...")
            vehicle.mode = VehicleMode('LAND')
            while vehicle.armed:
                time.sleep(1)
            print("Landed at original position.")
            print("Mission complete - No marker detected.")
            
    except rospy.ROSInterruptException:
        pass

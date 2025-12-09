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
import urllib2
import json

#######VARIABLES########

vehicle = connect('tcp:127.0.0.1:5763',wait_ready=True)
vehicle.parameters['PLND_ENABLED']=1
vehicle.parameters['PLND_TYPE']=1
vehicle.parameters['PLND_EST_TYPE']=0
vehicle.parameters['LAND_SPEED']=30 ##cms/s

velocity=-.3 #m/s (reduced for smoother flight)
takeoff_height=6 #m

# Store original position
original_position = None
########################
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

id_to_find = 72 ##arucoID (not used anymore - detects all IDs)
marker_size = 27 ##CM - CALIBRATED for Gazebo rendering (actual SDF size is 40cm)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = 640
vertical_res = 480

horizontal_fov = 62.2 * (math.pi / 180) ##62.2 for picam V2, 53.5 for V1
vertical_fov = 48.8 * (math.pi / 180) ##48.8 for V2, 41.41 for V1

found_count=0
notfound_count=0
marker_found = False
detected_markers = {}  # Store detected markers by ID (only one per ID) (not dict by ID)

# Precision landing variables
current_landing_id = None
precision_landing_active = False
landing_sub = None

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


def calculate_relative_position(marker, reference_marker):
    """
    Calculate position of marker relative to reference marker in meters.
    Returns dict with x, y, z offsets where:
    - x: East-West (positive = East, negative = West)
    - y: North-South (positive = North, negative = South)  
    - z: Up-Down (positive = Up, negative = Down)
    """
    # Calculate horizontal distance in meters
    dlat = marker['marker_lat'] - reference_marker['marker_lat']
    dlon = marker['marker_lon'] - reference_marker['marker_lon']
    
    # Convert to meters
    y_offset = dlat * 111320.0  # latitude to meters (North-South)
    x_offset = dlon * 111320.0 * math.cos(reference_marker['marker_lat'] * math.pi / 180.0)  # longitude to meters (East-West)
    z_offset = marker['marker_alt'] - reference_marker['marker_alt']  # altitude difference
    
    return {
        'x': x_offset,
        'y': y_offset,
        'z': z_offset
    }


def generate_gemini_summary(detected_markers, reference_id=72):
    """Generate natural language summary using Google Gemini API via HTTP request"""
    
    if reference_id not in detected_markers:
        return "Reference marker ID %d not found." % reference_id
    
    reference_marker = detected_markers[reference_id]
    
    # Build data for Gemini
    fire_locations = []
    for marker_id, marker in detected_markers.items():
        if marker_id != reference_id:
            rel_pos = calculate_relative_position(marker, reference_marker)
            fire_locations.append({
                'id': marker_id,
                'x': rel_pos['x'],
                'y': rel_pos['y'],
                'z': rel_pos['z']
            })
    
    if not fire_locations:
        return "No fires detected relative to reference marker ID %d." % reference_id
    
    # Create prompt for Gemini
    prompt = """You are providing navigation instructions to help a person (located at ArUco marker ID 72) reach multiple fires.

COORDINATE SYSTEM:
- X axis: Positive = East/Right from camera view, Negative = West/Left from camera view
- Y axis: Positive = North/Forward from camera view, Negative = South/Backward from camera view
- Z axis: Positive = Up, Negative = Down

FIRE LOCATIONS (relative to person at marker ID 72):
"""
    
    for fire in fire_locations:
        prompt += "\nFire at ArUco ID %d: X=%.2fm, Y=%.2fm, Z=%.2fm" % (fire['id'], fire['x'], fire['y'], fire['z'])
    
    prompt += """

TASK: Provide clear, natural language navigation instructions to reach each fire from the person's location. Use terms like:
- "left/right" or "west/east" for X direction
- "forward/backward" or "north/south" for Y direction  
- "up/down" for Z direction

Be concise and practical. Format example:
"To reach Fire 1 (ID 226): Go right 3 meters, forward 2 meters, then up 1 meter."

Provide instructions for each fire."""

    try:
        # Gemini API configuration
        api_key = 'AIzaSyBTX8hfnXZaZx2xplwQtCniUukGiuRmm9c'
        
        # Use gemini-2.5-flash model as you mentioned it works
        url = 'https://generativelanguage.googleapis.com/v1beta/models/gemini-2.5-flash:generateContent?key=' + api_key
        
        # Prepare the request data
        request_data = {
            "contents": [{
                "parts": [{
                    "text": prompt
                }]
            }]
        }
        
        # Create HTTP request
        headers = {
            'Content-Type': 'application/json'
        }
        
        req = urllib2.Request(url, json.dumps(request_data), headers)
        response = urllib2.urlopen(req)
        response_data = json.loads(response.read())
        
        # Extract the generated text
        if 'candidates' in response_data and len(response_data['candidates']) > 0:
            candidate = response_data['candidates'][0]
            if 'content' in candidate and 'parts' in candidate['content']:
                parts = candidate['content']['parts']
                if len(parts) > 0 and 'text' in parts[0]:
                    return parts[0]['text']
        
        return "Error: Could not extract response from Gemini API"
        
    except urllib2.HTTPError as e:
        error_msg = "HTTP Error calling Gemini API: " + str(e.code)
        try:
            error_data = json.loads(e.read())
            if 'error' in error_data:
                error_msg += " - " + error_data['error']['message']
        except:
            error_msg += " - " + e.read()
        return error_msg
    except urllib2.URLError as e:
        return "URL Error calling Gemini API: " + str(e.reason)
    except Exception as e:
        return "Error generating AI summary: " + str(e)


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


def precision_landing_msg_receiver(message):
    """Message receiver for precision landing - only detects the target marker"""
    global notfound_count, found_count, time_last, time_to_wait, current_landing_id
    
    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(message) ##Deserialize image data into array
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)

        try:
            if ids is not None:
                marker_detected = False
                for i in range(len(ids)):
                    if int(ids[i][0]) == current_landing_id:
                        marker_detected = True
                        ret = aruco.estimatePoseSingleMarkers([corners[i]],marker_size,cameraMatrix=np_camera_matrix,distCoeffs=np_dist_coeff)
                        (rvec, tvec) = (ret[0][0,0,:],ret[1][0,0,:])
                        
                        # Calculate center of marker in image coordinates
                        y_sum=0
                        x_sum=0
                        x_sum = corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0]
                        y_sum = corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1]
                        x_avg = x_sum / 4
                        y_avg = y_sum / 4

                        # Calculate angular offsets (normalized)
                        x_ang = (x_avg - horizontal_res * 0.5) / (horizontal_res * 0.5)  # -1 to 1
                        y_ang = (y_avg - vertical_res * 0.5) / (vertical_res * 0.5)      # -1 to 1

                        # Send landing target message
                        send_land_message(x_ang, y_ang)

                        # Visual feedback
                        x = '{:.2f}'.format(tvec[0])
                        y = '{:.2f}'.format(tvec[1])
                        z = '{:.2f}'.format(tvec[2])
                        marker_position = 'PRECISION LANDING ON ID %d: x=%s y=%s z=%s' % (current_landing_id, x, y, z)
                        
                        aruco.drawDetectedMarkers(np_data,[corners[i]],[ids[i]])
                        aruco.drawAxis(np_data,np_camera_matrix,np_dist_coeff,rvec,tvec,10)
                        cv2.putText(np_data,marker_position,(10,50),0,.5,(0,255,0),thickness=2)
                        cv2.putText(np_data,"PRECISION LANDING ACTIVE",(10,80),0,.7,(0,0,255),thickness=2)
                        
                        found_count = found_count + 1
                        print('Precision landing - Marker ID %d: X=%s, Y=%s, Z=%s' % (current_landing_id, x, y, z))
                        break
                
                if not marker_detected:
                    notfound_count=notfound_count+1
                    print('Precision landing - Marker ID %d not found in frame' % current_landing_id)
            else:
                notfound_count=notfound_count+1
                print('Precision landing - No markers detected')
        except Exception as e:
            print('Error in precision landing detection: %s' % str(e))
            notfound_count=notfound_count+1
        
        new_msg = rnp.msgify(Image, np_data,encoding='rgb8')
        newimg_pub.publish(new_msg)
        time_last = time.time()


def start_precision_landing_subscriber():
    """Start the subscriber for precision landing"""
    global landing_sub
    if landing_sub is None:
        landing_sub = rospy.Subscriber('/camera/color/image_raw', Image, precision_landing_msg_receiver)
        print("Precision landing subscriber started")


def stop_precision_landing_subscriber():
    """Stop the precision landing subscriber"""
    global landing_sub
    if landing_sub is not None:
        landing_sub.unregister()
        landing_sub = None
        print("Precision landing subscriber stopped")


def precision_land_on_marker(marker_id):
    """Precision land on a specific marker using camera-based landing"""
    global current_landing_id, precision_landing_active
    
    print("="*60)
    print("STARTING PRECISION LANDING ON MARKER ID: %d" % marker_id)
    print("="*60)
    
    current_landing_id = marker_id
    precision_landing_active = True
    
    # First, fly to the marker's approximate GPS location at safe altitude
    marker = detected_markers[marker_id]
    approach_altitude = 4.0  # meters above marker
    
    print("Flying to marker ID %d location at %.1f meters..." % (marker_id, approach_altitude))
    approach_location = LocationGlobalRelative(
        marker['marker_lat'],
        marker['marker_lon'],
        approach_altitude
    )
    vehicle.simple_goto(approach_location)
    
    # Wait until close to marker position
    while True:
        current = vehicle.location.global_relative_frame
        dlat = current.lat - marker['marker_lat']
        dlong = current.lon - marker['marker_lon']
        distance = math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
        
        print('Distance to marker ID %d: %.2f m, Altitude: %.2f m' % (marker_id, distance, current.alt))
        
        if distance < 3.0 and abs(current.alt - approach_altitude) < 1.0:  # Within 3 meters horizontally and 1m vertically
            print('In position for precision landing on marker ID %d' % marker_id)
            break
        time.sleep(1)
    
    # Start precision landing using camera
    print("Starting camera-based precision landing...")
    
    # Start the precision landing subscriber
    start_precision_landing_subscriber()
    
    # Switch to LAND mode for precision landing
    vehicle.mode = VehicleMode('LAND')
    while vehicle.mode != 'LAND':
        time.sleep(0.5)
    print('Vehicle in LAND mode - precision landing active')
    
    # Wait for landing to complete
    landing_start_time = time.time()
    last_altitude = vehicle.location.global_relative_frame.alt
    no_movement_count = 0
    
    while vehicle.armed:
        current_alt = vehicle.location.global_relative_frame.alt
        print('Precision Landing - Altitude: %.2f m' % current_alt)
        
        # Check if we're stuck (no altitude change for 10 seconds)
        if abs(current_alt - last_altitude) < 0.1:  # Less than 10cm change
            no_movement_count += 1
        else:
            no_movement_count = 0
            
        if no_movement_count > 10:  # 10 seconds with no movement
            print("No altitude change detected for 10 seconds, forcing landing completion")
            break
            
        last_altitude = current_alt
        time.sleep(1)
    
    print("Landed successfully on marker ID %d!" % marker_id)
    
    # Stop the precision landing subscriber
    stop_precision_landing_subscriber()
    
    # Spray water simulation
    print("Spraying water on ID %d..." % marker_id)
    time.sleep(5)  # Wait 5 seconds for water spraying
    print("Water spraying complete on ID %d" % marker_id)
    
    precision_landing_active = False
    current_landing_id = None
    
    # Takeoff again to go to next marker (only if there are more fires)
    fire_markers = [m_id for m_id in detected_markers.keys() if m_id != 72]
    if marker_id != fire_markers[-1]:  # If this is not the last fire marker
        print("Taking off to go to next location...")
        arm_and_takeoff(takeoff_height)
    else:
        print("Last fire extinguished, preparing to return home...")


def msg_receiver(message):
    global notfound_count, found_count, time_last, time_to_wait, marker_found, detected_markers

    if precision_landing_active:
        # If precision landing is active, skip the regular detection
        return
        
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
                    
                    # Only store one marker per ID
                    if marker_id not in detected_markers:
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
                        detected_markers[marker_id] = new_marker
                        marker_found = True
                        print('NEW MARKER DETECTED: ID %d' % marker_id)
                        print('  Drone position: Lat=%.6f, Lon=%.6f, Alt=%.2f' % 
                              (current_lat, current_lon, current_alt))
                        print('  Marker position: Lat=%.6f, Lon=%.6f, Alt=%.2f' % 
                              (marker_gps['lat'], marker_gps['lon'], marker_gps['alt']))
                        print('  Distance: %.2f cm (%.2f m)' % (new_marker['distance_cm'], new_marker['distance_cm']/100.0))
                        print('  X/Y offsets: %.2f, %.2f cm' % (new_marker['x_offset_cm'], new_marker['y_offset_cm']))
                        print('  Total unique markers found: %d' % len(detected_markers))
                    else:
                        # Update detection count
                        detected_markers[marker_id]['detection_count'] += 1
                    
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


def perform_fire_extinguishing():
    """Perform precision landing on all fire markers and extinguish fires"""
    global precision_landing_active
    
    # Get all fire markers (excluding ID 72 - the person)
    fire_markers = [marker_id for marker_id in detected_markers.keys() if marker_id != 72]
    
    if not fire_markers:
        print("No fire markers found to extinguish!")
        return
    
    print("\n" + "="*60)
    print("STARTING FIRE EXTINGUISHING OPERATION")
    print("="*60)
    print("Fire markers to extinguish: %s" % fire_markers)
    
    # Land on each fire marker
    for i, marker_id in enumerate(fire_markers):
        print("\n" + "="*50)
        print("EXTINGUISHING FIRE %d/%d - MARKER ID: %d" % (i+1, len(fire_markers), marker_id))
        print("="*50)
        
        # Start precision landing on this marker
        precision_land_on_marker(marker_id)
    
    print("\n" + "="*60)
    print("FIRE EXTINGUISHING COMPLETE!")
    print("All fires have been extinguished and it is now safe for the person.")
    print("="*60)
    
    # Return to original position
    return_to_original()
    
    # Final landing at original position
    print("Final landing at original position...")
    vehicle.mode = VehicleMode('LAND')
    while vehicle.armed:
        time.sleep(1)
    print("Mission complete! Safely landed at original position.")


if __name__=='__main__':
    try:
        # Initialize ROS node
        rospy.init_node('drone_node', anonymous=False)
        
        arm_and_takeoff(takeoff_height)
        time.sleep(1)
        
        # Fly to house center at (10, 10)
        print("Flying to house location...")
        house_location = LocationGlobalRelative(
            vehicle.location.global_relative_frame.lat + (10.0 / 111320.0),  # 10m north
            vehicle.location.global_relative_frame.lon - (10.0 / (111320.0 * math.cos(vehicle.location.global_relative_frame.lat * math.pi / 180))),  # 10m west
            6  # 6m altitude
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
            print("ALL MARKERS DETECTED - GENERATING AI SUMMARY")
            print("="*60)
            print("Total unique markers found: %d" % len(detected_markers))
            print("")
            
            for marker_id in sorted(detected_markers.keys()):
                marker = detected_markers[marker_id]
                print("ArUco Marker ID: %d" % marker_id)
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
            
            # Show relative positions if marker 72 detected
            if 72 in detected_markers:
                print("="*60)
                print("POSITIONS RELATIVE TO REFERENCE (ArUco ID 72 - Person)")
                print("="*60)
                reference = detected_markers[72]
                print("Reference marker (Person) at:")
                print("  GPS: %.6f, %.6f" % (reference['marker_lat'], reference['marker_lon']))
                print("  Altitude: %.2f m" % reference['marker_alt'])
                print("")
                
                for marker_id in sorted(detected_markers.keys()):
                    if marker_id != 72:
                        marker = detected_markers[marker_id]
                        rel_pos = calculate_relative_position(marker, reference)
                        print("Fire ID %d relative position:" % marker_id)
                        print("  X (East-West): %.2f m %s" % (abs(rel_pos['x']), "East/Right" if rel_pos['x'] > 0 else "West/Left"))
                        print("  Y (North-South): %.2f m %s" % (abs(rel_pos['y']), "North/Forward" if rel_pos['y'] > 0 else "South/Backward"))
                        print("  Z (Vertical): %.2f m %s" % (abs(rel_pos['z']), "Up" if rel_pos['z'] > 0 else "Down"))
                        print("")
                
                print("="*60)
                print("AI-GENERATED NAVIGATION SUMMARY")
                print("="*60)
                summary = generate_gemini_summary(detected_markers, reference_id=72)
                print(summary)
                print("="*60 + "\n")
            
            # Now perform fire extinguishing on all fire markers
            perform_fire_extinguishing()
            
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
    except KeyboardInterrupt:
        print("Mission interrupted by user")
        stop_precision_landing_subscriber()
        if vehicle.armed:
            vehicle.mode = VehicleMode('LAND')
            while vehicle.armed:
                time.sleep(1)
        print("Emergency landing complete")

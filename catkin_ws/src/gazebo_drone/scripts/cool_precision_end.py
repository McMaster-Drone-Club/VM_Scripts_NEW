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

vehicle = connect('tcp:127.0.0.1:5763', wait_ready=True)
vehicle.parameters['PLND_ENABLED'] = 1
vehicle.parameters['PLND_TYPE'] = 1
vehicle.parameters['PLND_EST_TYPE'] = 0
vehicle.parameters['LAND_SPEED'] = 30  # cms/s

velocity = -.3  # m/s (reduced for smoother flight)
takeoff_height = 6  # m

# Store original position
original_position = None

########################
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

id_to_find = 72  # arucoID (reference "person"/home pad)
marker_size = 27  # CM - for general markers on roof
marker_size_person = 20  # CM - for final precision landing pad (20x20cm)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = 640
vertical_res = 480

horizontal_fov = 62.2 * (math.pi / 180)  # 62.2 for picam V2, 53.5 for V1
vertical_fov = 48.8 * (math.pi / 180)    # 48.8 for V2, 41.41 for V1

found_count = 0
notfound_count = 0
marker_found = False
detected_markers = {}  # Store detected markers by ID

#############CAMERA INTRINSICS#######

dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix = [
    [530.8269276712998, 0.0, 320.5],
    [0.0, 530.8269276712998, 240.5],
    [0.0, 0.0, 1.0]
]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

#####

time_last = 0
time_to_wait = .1  # 100 ms

# Separate timing for precision landing callback
pl_time_last = 0
pl_time_to_wait = .1

################FUNCTIONS###############

def arm_and_takeoff(targetHeight):
    global original_position

    while vehicle.is_armable != True:
        print('Waiting for vehicle to become armable')
        time.sleep(1)
    print('Vehicle is now armable')

    vehicle.mode = VehicleMode('GUIDED')

    while vehicle.mode != 'GUIDED':
        print('Waiting for drone to enter GUIDED flight mode')
        time.sleep(1)
    print('Vehicle now in GUIDED mode. Have Fun!')

    vehicle.armed = True
    while vehicle.armed == False:
        print('Waiting for vehicle to become armed.')
        time.sleep(1)
    print('Look out! Virtual props are spinning!')

    vehicle.simple_takeoff(targetHeight)

    while True:
        print('Current Altitude: %d' % vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= .95 * targetHeight:
            break
        time.sleep(1)
    print('Target altitude reached!')

    # Store original position after takeoff
    original_position = vehicle.location.global_relative_frame
    print('Original position saved: Lat=%f, Lon=%f, Alt=%f' %
          (original_position.lat, original_position.lon, original_position.alt))

    return None


def send_local_ned_velocity(vx, vy, vz):
    """Send velocity command to drone in body frame (NED offset)."""
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def send_velocity_smooth(vx, vy, vz, duration):
    """Send velocity commands continuously for smooth movement."""
    end_time = time.time() + duration
    while time.time() < end_time:
        send_local_ned_velocity(vx, vy, vz)
        time.sleep(0.1)  # Send command every 100ms


def calculate_marker_gps(drone_lat, drone_lon, drone_alt, x_offset_cm, y_offset_cm, distance_cm):
    """
    Calculate the actual GPS coordinates of the marker based on drone position and offsets.
    """
    # Convert cm to meters
    x_offset_m = x_offset_cm / 100.0
    y_offset_m = y_offset_cm / 100.0
    distance_m = distance_cm / 100.0

    # Assuming camera is pointing straight down:
    # Camera X -> East (longitude)
    # Camera Y -> South (latitude)
    # Camera Z -> Down (altitude difference)

    # Marker altitude (below drone)
    marker_alt = drone_alt - distance_m

    # Horizontal offsets in meters
    north_offset_m = -y_offset_m   # Negative Y in camera = North
    east_offset_m = x_offset_m     # Positive X in camera = East

    # Convert offsets to GPS deltas
    dlat = north_offset_m / 111320.0
    dlon = east_offset_m / (111320.0 * math.cos(drone_lat * math.pi / 180.0))

    marker_lat = drone_lat + dlat
    marker_lon = drone_lon + dlon

    return {
        'lat': marker_lat,
        'lon': marker_lon,
        'alt': marker_alt
    }


def send_land_message(x, y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0, 0, 0
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
    dlat = marker['marker_lat'] - reference_marker['marker_lat']
    dlon = marker['marker_lon'] - reference_marker['marker_lon']

    y_offset = dlat * 111320.0
    x_offset = dlon * 111320.0 * math.cos(reference_marker['marker_lat'] * math.pi / 180.0)
    z_offset = marker['marker_alt'] - reference_marker['marker_alt']

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
        prompt += "\nFire at ArUco ID %d: X=%.2fm, Y=%.2fm, Z=%.2fm" % (
            fire['id'], fire['x'], fire['y'], fire['z']
        )

    prompt += """

TASK: Provide clear, natural language navigation instructions to reach each fire from the person's location. Use terms like:
- "left/right" or "west/east" for X direction
- "forward/backward" or "north/south" for Y direction
- "up/down" for Z direction

Be concise and practical. Format example:
"To reach Fire 1 (ID 226): Go right 3 meters, forward 2 meters, then up 1 meter."

Provide instructions for each fire."""

    try:
        api_key = 'AIzaSyBTX8hfnXZaZx2xplwQtCniUukGiuRmm9c'
        url = 'https://generativelanguage.googleapis.com/v1beta/models/gemini-2.5-flash:generateContent?key=' + api_key

        request_data = {
            "contents": [{
                "parts": [{
                    "text": prompt
                }]
            }]
        }

        headers = {
            'Content-Type': 'application/json'
        }

        req = urllib2.Request(url, json.dumps(request_data), headers)
        response = urllib2.urlopen(req)
        response_data = json.loads(response.read())

        if 'candidates' in response_data and len(response_data['candidates']) > 0:
            candidate = response_data['candidates'][0]
            if 'content' in candidate and 'parts' in candidate['content']:
                parts = candidate['content']['parts']
                if len(parts) > 0 and 'text' in parts[0]:
                    return parts[0]['text']

        return "Error: Could not extract response from Gemini API"

    except urllib2.HTTPError as e:
        return "HTTP Error calling Gemini API: " + str(e.code) + " - " + e.read()
    except urllib2.URLError as e:
        return "URL Error calling Gemini API: " + str(e.reason)
    except Exception as e:
        return "Error generating AI summary: " + str(e)


def visit_fire_targets(hover_alt=6.0, dwell_time=5.0):
    """
    For each detected fire target (all ArUco marker IDs except 72):
    - Fly above target at hover_alt
    - Hover briefly
    - Descend and consider "landed" when altitude < 3m (roof)
    - Stay there for dwell_time seconds
    - Take off again to hover_alt
    """
    global detected_markers

    print("\n" + "=" * 60)
    print("VISITING FIRE TARGETS: Descend + Roof-Land on each fire")
    print("=" * 60)

    # Ensure GUIDED mode
    vehicle.mode = VehicleMode('GUIDED')
    while vehicle.mode != 'GUIDED':
        print("Waiting for GUIDED mode before starting fire visits...")
        time.sleep(1)
    print("In GUIDED mode. Beginning fire visitation sequence.")

    fire_ids = [mid for mid in sorted(detected_markers.keys()) if mid != 72]

    if not fire_ids:
        print("No fire targets (non-72 markers) to visit.\n")
        return

    for idx, mid in enumerate(fire_ids, 1):
        marker = detected_markers[mid]

        print("\n--- FIRE TARGET %d/%d (ID %d) ---" % (idx, len(fire_ids), mid))

        # 1. Fly over fire at hover_alt
        target = LocationGlobalRelative(
            marker['marker_lat'],
            marker['marker_lon'],
            hover_alt
        )
        print("Heading to fire ID %d at %.1fm altitude..." % (mid, hover_alt))
        vehicle.simple_goto(target)

        # Wait until close to target
        while True:
            current = vehicle.location.global_relative_frame
            dlat = current.lat - target.lat
            dlong = current.lon - target.lon
            distance = math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
            alt_error = abs(current.alt - hover_alt)

            print("  Distance to fire ID %d: %.2f m (alt error: %.2f m)" %
                  (mid, distance, alt_error))

            if distance < 1.0 and alt_error < 0.5:
                print("  Reached fire target ID %d. Hovering..." % mid)
                break
            time.sleep(1)

        time.sleep(2)  # brief hover

	# 2. DESCEND to roof WITHOUT LAND MODE
	print("Descending gently to roof for fire ID %d..." % mid)

	vehicle.mode = VehicleMode("GUIDED")
	while vehicle.mode != "GUIDED":
	    print("Waiting for GUIDED mode before roof descent...")
	    time.sleep(1)

	# Controlled descent using NED velocity (positive vz = DOWN)
	while vehicle.location.global_relative_frame.alt > 3.0:
	    print("  Descending... Altitude: %.2f m" %
	          vehicle.location.global_relative_frame.alt)
	    send_local_ned_velocity(0, 0, 0.25)  # slow descent @ 0.25 m/s
	    time.sleep(0.5)

	# Stop movement
	send_local_ned_velocity(0, 0, 0)

	print("  Roof touched (alt < 3m). Considered landed.")
	print("  Waiting %d seconds on the roof..." % dwell_time)
	time.sleep(dwell_time)

	# 3. TAKE OFF AGAIN (no rearming needed)
	print("Taking off again to %.1fm..." % hover_alt)
	vehicle.simple_takeoff(hover_alt)

	# Wait to reach altitude
	while vehicle.location.global_relative_frame.alt < hover_alt * 0.95:
	    print("  Ascending... Current alt: %.2f m" %
	          vehicle.location.global_relative_frame.alt)
	    time.sleep(1)

	print("  Back at %.1fm. Continuing to next fire." % hover_alt)


        print("  Back at %.1fm. Continuing to next fire." % hover_alt)

    print("\nFinished landing on all fire targets.\n" + "=" * 60)


def return_to_original():
    """Return to original takeoff position."""
    global original_position

    print("Returning to original position...")

    vehicle.mode = VehicleMode('GUIDED')
    while vehicle.mode != 'GUIDED':
        print('Waiting for GUIDED mode...')
        time.sleep(1)
    print('In GUIDED mode, returning to start...')

    vehicle.simple_goto(original_position)

    while True:
        current = vehicle.location.global_relative_frame
        dlat = current.lat - original_position.lat
        dlong = current.lon - original_position.lon
        distance = math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

        print('Distance to original position: %.2f m' % distance)

        if distance < 1.0:
            print('Reached original position!')
            break
        time.sleep(1)

    time.sleep(2)
    print('Return complete! (Altitude ~%.2f m)' %
          vehicle.location.global_relative_frame.alt)


def msg_receiver(message):
    global notfound_count, found_count, time_last, time_to_wait, marker_found, detected_markers

    if time.time() - time_last > time_to_wait:
        # Deserialize ROS Image -> numpy
        np_data = rnp.numpify(message)

        # Use grayscale for detection, keep color np_data for overlay
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(
            image=gray_img,
            dictionary=aruco_dict,
            parameters=parameters
        )

        try:
            if ids is not None:
                # Process all detected markers
                for i in range(len(ids)):
                    marker_id = int(ids[i][0])

                    ret = aruco.estimatePoseSingleMarkers(
                        [corners[i]],
                        marker_size,
                        cameraMatrix=np_camera_matrix,
                        distCoeffs=np_dist_coeff
                    )
                    (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])

                    current_lat = vehicle.location.global_relative_frame.lat
                    current_lon = vehicle.location.global_relative_frame.lon
                    current_alt = vehicle.location.global_relative_frame.alt

                    marker_gps = calculate_marker_gps(
                        current_lat, current_lon, current_alt,
                        float(tvec[0]), float(tvec[1]), float(tvec[2])
                    )

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
                        print('  Distance: %.2f cm (%.2f m)' %
                              (new_marker['distance_cm'], new_marker['distance_cm'] / 100.0))
                        print('  X/Y offsets: %.2f, %.2f cm' %
                              (new_marker['x_offset_cm'], new_marker['y_offset_cm']))
                        print('  Total unique markers found: %d' % len(detected_markers))
                    else:
                        detected_markers[marker_id]['detection_count'] += 1

                    # Visual overlay for each marker
                    x = '{:.2f}'.format(tvec[0])
                    y = '{:.2f}'.format(tvec[1])
                    z = '{:.2f}'.format(tvec[2])
                    marker_position = 'ID %d: x=%s y=%s z=%s' % (marker_id, x, y, z)

                    aruco.drawDetectedMarkers(np_data, [corners[i]], [ids[i]])
                    aruco.drawAxis(np_data, np_camera_matrix, np_dist_coeff, rvec, tvec, 10)
                    cv2.putText(np_data, marker_position, (10, 50 + i * 30),
                                0, .5, (255, 0, 0), thickness=2)

                found_count = found_count + 1
            else:
                notfound_count = notfound_count + 1
        except Exception as e:
            print('Target likely not found')
            print(e)
            notfound_count = notfound_count + 1

        # Publish the overlaid image to /camera/color/image_new
        new_msg = rnp.msgify(Image, np_data, encoding='bgr8')
        newimg_pub.publish(new_msg)
        time_last = time.time()
    else:
        return None


def check_for_marker_quick():
    """Quick check for marker - just about 1 second."""
    global marker_found
    marker_found = False

    sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)

    start_time = time.time()
    while time.time() - start_time < 1.0 and not marker_found:
        time.sleep(0.1)

    sub.unregister()
    return marker_found


def search_pattern():
    """Execute lawnmower search pattern over the house roof."""

    # House is ~6m x 6m
    scan_speed = 0.3  # m/s
    stripe_width = 2.0  # meters between passes
    scan_duration = 6.0 / scan_speed  # time to cross house
    num_stripes = 4

    print("Starting lawnmower search pattern over house...")

    # STAGING: move longer towards top-right before forward motion
    print("Staging before recon search: moving further to TOP-RIGHT of house...")
    # -velocity is positive (0.3), so this is forward + right
    send_velocity_smooth(-velocity, -velocity, 0, 8)  # longer staging leg
    send_local_ned_velocity(0, 0, 0)
    time.sleep(1.0)

    for stripe in range(num_stripes):
        print('Scanning stripe %d/%d...' % (stripe + 1, num_stripes))

        if stripe % 2 == 0:
            direction = -scan_speed
            print("Scanning FORWARD towards bottom-left...")
        else:
            direction = scan_speed
            print("Scanning BACKWARD towards top-right...")

        segments = 6
        segment_time = scan_duration / segments

        for seg in range(segments):
            send_velocity_smooth(direction, 0, 0, segment_time)
            send_local_ned_velocity(0, 0, 0)

            check_for_marker_quick()

        send_local_ned_velocity(0, 0, 0)
        time.sleep(0.5)

        if stripe < num_stripes - 1:
            print("Moving to next stripe (side step)...")
            # Negative vy moves left in body frame (to step across roof)
            send_velocity_smooth(0, -stripe_width / scan_speed, 0, stripe_width / scan_speed)
            send_local_ned_velocity(0, 0, 0)
            time.sleep(0.5)

            check_for_marker_quick()

    print("Lawnmower search pattern complete.")
    return len(detected_markers) > 0


########## FINAL PRECISION LANDING ON ARUCO 72 ##########

# Separate callback for precision landing stage
def precision_msg_receiver(message):
    global pl_time_last, pl_time_to_wait, id_to_find

    if time.time() - pl_time_last > pl_time_to_wait:
        np_data = rnp.numpify(message)
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(
            image=gray_img,
            dictionary=aruco_dict,
            parameters=parameters
        )

        try:
            if ids is not None:
                # Keep minimal modification: check first detected marker
                if ids[0] == id_to_find:
                    ret = aruco.estimatePoseSingleMarkers(
                        corners,
                        marker_size_person,
                        cameraMatrix=np_camera_matrix,
                        distCoeffs=np_dist_coeff
                    )
                    (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])

                    x = '{:.2f}'.format(tvec[0])  # cm
                    y = '{:.2f}'.format(tvec[1])  # cm
                    z = '{:.2f}'.format(tvec[2])  # cm

                    x_sum = 0
                    y_sum = 0

                    x_sum = (corners[0][0][0][0] + corners[0][0][1][0] +
                             corners[0][0][2][0] + corners[0][0][3][0])
                    y_sum = (corners[0][0][0][1] + corners[0][0][1][1] +
                             corners[0][0][2][1] + corners[0][0][3][1])

                    x_avg = x_sum / 4.0
                    y_avg = y_sum / 4.0

                    x_ang = (x_avg - horizontal_res * .5) * horizontal_fov / horizontal_res
                    y_ang = (y_avg - vertical_res * .5) * vertical_fov / vertical_res

                    if vehicle.mode != 'LAND':
                        vehicle.mode = VehicleMode('LAND')
                        while vehicle.mode != 'LAND':
                            print('Waiting for LAND mode (precision)...')
                            time.sleep(1)
                        print('Vehicle in LAND mode (precision landing enabled)')
                        send_land_message(x_ang, y_ang)
                    else:
                        send_land_message(x_ang, y_ang)

                    marker_position = 'PL MARKER 72: x=' + x + ' y=' + y + ' z=' + z

                    aruco.drawDetectedMarkers(np_data, corners)
                    aruco.drawAxis(np_data, np_camera_matrix, np_dist_coeff, rvec, tvec, 10)
                    cv2.putText(np_data, marker_position, (10, 50), 0, .7,
                                (0, 255, 0), thickness=2)
                    print(marker_position)
                else:
                    # Other markers, ignore in precision stage
                    pass
            else:
                pass
        except Exception as e:
            print('Precision target likely not found')
            print(e)

        new_msg = rnp.msgify(Image, np_data, encoding='bgr8')
        newimg_pub.publish(new_msg)
        pl_time_last = time.time()
    else:
        return None


def precision_land_on_home():
    """Final precision landing on 20x20cm ArUco72 pad after returning home."""
    print("\n" + "=" * 60)
    print("STARTING FINAL PRECISION LANDING ON ARUCO 72 PAD")
    print("=" * 60)

    # We should already be near original_position and at ~6m (< 8m)
    current_alt = vehicle.location.global_relative_frame.alt
    print("Current altitude before precision landing: %.2f m" % current_alt)

    # Subscribe to camera stream with precision callback
    sub = rospy.Subscriber('/camera/color/image_raw', Image, precision_msg_receiver)

    # Let PLND logic run until drone disarms (i.e., fully landed on pad)
    while vehicle.armed:
        print("Precision landing... Altitude: %.2f m" %
              vehicle.location.global_relative_frame.alt)
        time.sleep(1)

    # Once landed & disarmed, stop listening
    sub.unregister()
    print("Precision landing complete. Motors disarmed on ArUco72 pad.")
    print("=" * 60 + "\n")


######################## MAIN ########################

if __name__ == '__main__':
    try:
        # Init ROS node once
        rospy.init_node('drone_node', anonymous=False)

        arm_and_takeoff(takeoff_height)
        time.sleep(1)

        # Fly to house center at (approx) 10m north, 10m west
        print("Flying to house location...")
        house_location = LocationGlobalRelative(
            vehicle.location.global_relative_frame.lat + (10.0 / 111320.0),  # 10m north
            vehicle.location.global_relative_frame.lon - (
                10.0 / (111320.0 * math.cos(vehicle.location.global_relative_frame.lat * math.pi / 180))
            ),  # 10m west
            6  # 6m altitude
        )
        vehicle.simple_goto(house_location)

        while True:
            current = vehicle.location.global_relative_frame
            dlat = current.lat - house_location.lat
            dlong = current.lon - house_location.lon
            distance = math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
            print('Distance to house: %.2f m' % distance)
            if distance < 1.0:
                print('Arrived at house!')
                break
            time.sleep(1)

        time.sleep(2)  # Stabilize

        # Execute search pattern around house
        marker_found = search_pattern()

        if detected_markers:
            print("\n" + "=" * 60)
            print("ALL MARKERS DETECTED - RECON REPORT")
            print("=" * 60)
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

            if 72 in detected_markers:
                print("=" * 60)
                print("POSITIONS RELATIVE TO REFERENCE (ArUco ID 72 - Person)")
                print("=" * 60)
                reference = detected_markers[72]
                print("Reference marker (Person) at:")
                print("  GPS: %.6f, %.6f" %
                      (reference['marker_lat'], reference['marker_lon']))
                print("  Altitude: %.2f m" % reference['marker_alt'])
                print("")

                for marker_id in sorted(detected_markers.keys()):
                    if marker_id != 72:
                        marker = detected_markers[marker_id]
                        rel_pos = calculate_relative_position(marker, reference)
                        print("Fire ID %d relative position:" % marker_id)
                        print("  X (East-West): %.2f m %s" %
                              (abs(rel_pos['x']), "East/Right" if rel_pos['x'] > 0 else "West/Left"))
                        print("  Y (North-South): %.2f m %s" %
                              (abs(rel_pos['y']), "North/Forward" if rel_pos['y'] > 0 else "South/Backward"))
                        print("  Z (Vertical): %.2f m %s" %
                              (abs(rel_pos['z']), "Up" if rel_pos['z'] > 0 else "Down"))
                        print("")

                print("=" * 60)
                print("AI-GENERATED NAVIGATION SUMMARY")
                print("=" * 60)
                summary = generate_gemini_summary(detected_markers, reference_id=72)
                print(summary)
                print("=" * 60 + "\n")

                print("Starting intermediate step: visiting each detected fire target...")
                visit_fire_targets(hover_alt=takeoff_height, dwell_time=5.0)
                print("Finished visiting all fire targets. Proceeding to return home...")

            print("=" * 60 + "\n")

            # Return to original position (close to ArUco72 pad, at ~6m < 8m)
            return_to_original()

            # FINAL: precision landing on 20x20cm ArUco72 pad
            precision_land_on_home()

            # Final report
            print("\n" + "=" * 60)
            print("MISSION COMPLETE - FINAL REPORT")
            print("=" * 60)
            print("Reconnaissance mission successful!")
            print("Detected %d unique marker(s):" % len(detected_markers))
            for marker_id in sorted(detected_markers.keys()):
                marker = detected_markers[marker_id]
                print("  ID %d: Marker GPS(%.6f, %.6f) at %.2fm altitude" %
                      (marker_id, marker['marker_lat'], marker['marker_lon'], marker['marker_alt']))
            print("=" * 60 + "\n")

        else:
            print("Marker not found in search pattern.")
            return_to_original()
            print("Landing at original position (no precision landing, no marker).")
            vehicle.mode = VehicleMode('LAND')
            while vehicle.armed:
                time.sleep(1)
            print("Landed at original position.")
            print("Mission complete - No marker detected.")

    except rospy.ROSInterruptException:
        pass


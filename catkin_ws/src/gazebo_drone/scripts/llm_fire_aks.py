#!/usr/bin/env python
# -*- coding: utf-8 -*-

######## IMPORTS ########

import rospy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import time
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import urllib2
import json

######## VEHICLE / ARUCO CONFIG ########

# Connect to SITL / Pixhawk
vehicle = connect('tcp:127.0.0.1:5763', wait_ready=True)

# Enable precision landing on ArduPilot
vehicle.parameters['PLND_ENABLED'] = 1
vehicle.parameters['PLND_TYPE'] = 1
vehicle.parameters['PLND_EST_TYPE'] = 0
vehicle.parameters['LAND_SPEED'] = 30  # cm/s

velocity = -0.3        # m/s search speed
takeoff_height = 6.0   # m

original_position = None  # saved after takeoff

# ROS publisher (set after rospy.init_node)
newimg_pub = None

# ArUco configuration
id_person = 72           # person marker (reference)
marker_size = 27.0       # cm, physical size of marker in Gazebo

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = 640
vertical_res = 480
horizontal_fov = 62.2 * (math.pi / 180.0)  # radians
vertical_fov = 48.8 * (math.pi / 180.0)    # radians

# Camera intrinsics
dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix = [
    [530.8269276712998, 0.0, 320.5],
    [0.0, 530.8269276712998, 240.5],
    [0.0, 0.0, 1.0]
]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

# Callback timing
time_last = 0.0
time_to_wait = 0.1  # seconds (10 Hz)

######## GLOBAL STATE ########

# All detected markers, keyed by id
# {
#   'marker_id', 'drone_lat', 'drone_lon', 'drone_alt',
#   'marker_lat', 'marker_lon', 'marker_alt',
#   'distance_cm', 'x_offset_cm', 'y_offset_cm', 'detection_count'
# }
detected_markers = {}

found_count = 0
notfound_count = 0
marker_found_flag = False  # used in quick checks

# Mission phase control
# 'search'  -> just detect & record
# 'landing' -> send LANDING_TARGET for current_landing_id
mission_phase = 'search'
current_landing_id = None


######## BASIC VEHICLE HELPERS ########

def arm_and_takeoff(targetHeight):
    global original_position

    while not vehicle.is_armable:
        print('Waiting for vehicle to become armable')
        time.sleep(1)
    print('Vehicle is now armable')

    vehicle.mode = VehicleMode('GUIDED')
    while vehicle.mode.name != 'GUIDED':
        print('Waiting for GUIDED mode')
        time.sleep(1)
    print('Vehicle in GUIDED mode')

    vehicle.armed = True
    while not vehicle.armed:
        print('Waiting for vehicle to arm')
        time.sleep(1)
    print('Vehicle armed, taking off')

    vehicle.simple_takeoff(targetHeight)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print('Current Altitude: %.2f' % alt)
        if alt >= 0.95 * targetHeight:
            break
        time.sleep(1)
    print('Target altitude reached')

    original_position = vehicle.location.global_relative_frame
    print('Original position saved: lat=%.7f lon=%.7f alt=%.2f' %
          (original_position.lat, original_position.lon, original_position.alt))


def send_local_ned_velocity(vx, vy, vz):
    """
    Send body-frame NED velocity (m/s).
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,  # only velocity fields enabled
        0, 0, 0,             # position
        vx, vy, vz,          # velocity
        0, 0, 0,             # acceleration
        0, 0                 # yaw, yaw_rate
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def send_velocity_smooth(vx, vy, vz, duration):
    """
    Smooth movement for a given duration (s).
    """
    end_time = time.time() + duration
    while time.time() < end_time:
        send_local_ned_velocity(vx, vy, vz)
        time.sleep(0.1)


def send_land_message(x_ang, y_ang):
    """
    MAVLink LANDING_TARGET with angular offsets in radians.
    Matches your working precision_landing.py logic.
    """
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x_ang,
        y_ang,
        0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


######## GEO HELPERS ########

def calculate_marker_gps(drone_lat, drone_lon, drone_alt,
                         x_offset_cm, y_offset_cm, distance_cm):
    """
    Approximate GPS coordinate of marker using drone GPS and camera offsets.
    Camera is pointing DOWN:
      X right -> East, Y down -> South, Z forward -> Down.
    """
    x_offset_m = x_offset_cm / 100.0
    y_offset_m = y_offset_cm / 100.0
    distance_m = distance_cm / 100.0

    marker_alt = drone_alt - distance_m  # marker below drone

    north_offset_m = -y_offset_m
    east_offset_m = x_offset_m

    dlat = north_offset_m / 111320.0
    dlon = east_offset_m / (111320.0 * math.cos(drone_lat * math.pi / 180.0))

    marker_lat = drone_lat + dlat
    marker_lon = drone_lon + dlon

    return {
        'lat': marker_lat,
        'lon': marker_lon,
        'alt': marker_alt
    }


def calculate_relative_position(marker, reference_marker):
    """
    Return marker position relative to reference in meters.
    x: East-West, y: North-South, z: Up-Down.
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


######## AI SUMMARY ########

def generate_gemini_summary(detected_markers_local, reference_id=72):
    if reference_id not in detected_markers_local:
        return "Reference marker ID %d not found." % reference_id

    reference_marker = detected_markers_local[reference_id]
    fire_locations = []

    for marker_id, marker in detected_markers_local.items():
        if marker_id == reference_id:
            continue
        rel = calculate_relative_position(marker, reference_marker)
        fire_locations.append({
            'id': marker_id,
            'x': rel['x'],
            'y': rel['y'],
            'z': rel['z']
        })

    if not fire_locations:
        return "No fires detected relative to marker ID %d." % reference_id

    prompt = """You are providing navigation instructions to help a person (located at ArUco marker ID 72) reach multiple fires.

COORDINATE SYSTEM:
- X axis: Positive = East/Right, Negative = West/Left
- Y axis: Positive = North/Forward, Negative = South/Backward
- Z axis: Positive = Up, Negative = Down

FIRE LOCATIONS (relative to person at marker ID 72):
"""

    for fire in fire_locations:
        prompt += "\nFire at ArUco ID %d: X=%.2fm, Y=%.2fm, Z=%.2fm" % (
            fire['id'], fire['x'], fire['y'], fire['z'])

    prompt += """

TASK: Provide clear, natural language navigation instructions to reach each fire from the person's location. Use terms like left/right, forward/backward, and up/down. Be concise and practical."""

    try:
        api_key = 'AIzaSyBTX8hfnXZaZx2xplwQtCniUukGiuRmm9c'  # your key
        url = 'https://generativelanguage.googleapis.com/v1beta/models/gemini-2.5-flash:generateContent?key=' + api_key

        request_data = {
            "contents": [{
                "parts": [{
                    "text": prompt
                }]
            }]
        }

        headers = {'Content-Type': 'application/json'}
        req = urllib2.Request(url, json.dumps(request_data), headers)
        response = urllib2.urlopen(req)
        response_data = json.loads(response.read())

        if 'candidates' in response_data and len(response_data['candidates']) > 0:
            candidate = response_data['candidates'][0]
            if 'content' in candidate and 'parts' in candidate['content']:
                parts = candidate['content']['parts']
                if len(parts) > 0 and 'text' in parts[0]:
                    return parts[0]['text']

        return "Error: Could not extract response from Gemini API."

    except Exception as e:
        return "Error calling Gemini API: %s" % str(e)


######## MAIN IMAGE CALLBACK ########

def msg_receiver(message):
    """
    Single callback that:
    - Detects all ArUco markers
    - Draws overlays
    - Records marker GPS/offset info
    - In landing phase: sends LANDING_TARGET for current_landing_id
    Always publishes to /camera/color/image_new.
    """
    global time_last, marker_found_flag
    global found_count, notfound_count
    global detected_markers, mission_phase, current_landing_id
    global newimg_pub

    now = time.time()
    if now - time_last < time_to_wait:
        return

    np_data = None

    try:
        # Convert ROS Image to numpy BGR
        np_data = rnp.numpify(message)
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = aruco.detectMarkers(
            gray_img,
            aruco_dict,
            parameters=parameters
        )

        # Normalize ids to numpy array of shape (N,1)
        if ids is not None:
            ids = np.array(ids, dtype=np.int32)
            if ids.ndim == 1:
                ids = ids.reshape(-1, 1)

        if ids is not None and len(ids) > 0:
            found_count += 1
            marker_found_flag = True

            # Drone position at the time of this frame
            current_lat = vehicle.location.global_relative_frame.lat
            current_lon = vehicle.location.global_relative_frame.lon
            current_alt = vehicle.location.global_relative_frame.alt

            for i in range(len(ids)):
                marker_id = int(ids[i])

                # Pose estimate for this marker
                ret = aruco.estimatePoseSingleMarkers(
                    [corners[i]],
                    marker_size,
                    np_camera_matrix,
                    np_dist_coeff
                )
                rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]  # cm

                # Compute image center of marker
                x_sum = (corners[i][0][0][0] +
                         corners[i][0][1][0] +
                         corners[i][0][2][0] +
                         corners[i][0][3][0])
                y_sum = (corners[i][0][0][1] +
                         corners[i][0][1][1] +
                         corners[i][0][2][1] +
                         corners[i][0][3][1])
                x_avg = x_sum / 4.0
                y_avg = y_sum / 4.0

                # Draw marker + axis
                aruco.drawDetectedMarkers(np_data, [corners[i]], [ids[i]])
                aruco.drawAxis(np_data, np_camera_matrix, np_dist_coeff, rvec, tvec, 10)

                # Store/update marker info
                distance_cm = float(tvec[2])
                gps_est = calculate_marker_gps(
                    current_lat, current_lon, current_alt,
                    float(tvec[0]), float(tvec[1]), float(tvec[2])
                )

                if marker_id not in detected_markers:
                    detected_markers[marker_id] = {
                        'marker_id': marker_id,
                        'drone_lat': current_lat,
                        'drone_lon': current_lon,
                        'drone_alt': current_alt,
                        'marker_lat': gps_est['lat'],
                        'marker_lon': gps_est['lon'],
                        'marker_alt': gps_est['alt'],
                        'distance_cm': distance_cm,
                        'x_offset_cm': float(tvec[0]),
                        'y_offset_cm': float(tvec[1]),
                        'detection_count': 1
                    }
                    print('NEW MARKER DETECTED: ID %d' % marker_id)
                    print('  Drone:  lat=%.7f lon=%.7f alt=%.2f' %
                          (current_lat, current_lon, current_alt))
                    print('  Marker: lat=%.7f lon=%.7f alt=%.2f' %
                          (gps_est['lat'], gps_est['lon'], gps_est['alt']))
                    print('  Dist:   %.2f cm (%.2f m)' %
                          (distance_cm, distance_cm / 100.0))
                    print('  Offsets: x=%.2f cm, y=%.2f cm' %
                          (float(tvec[0]), float(tvec[1])))
                    print('  Total markers: %d' % len(detected_markers))
                else:
                    detected_markers[marker_id]['detection_count'] += 1

                # Precision landing phase
                if mission_phase == 'landing' and current_landing_id == marker_id:
                    # Angle offsets (radians) – same as your working script
                    x_ang = (x_avg - horizontal_res * 0.5) * horizontal_fov / horizontal_res
                    y_ang = (y_avg - vertical_res * 0.5) * vertical_fov / vertical_res

                    if vehicle.mode.name != 'LAND':
                        vehicle.mode = VehicleMode('LAND')
                        print('Switching to LAND for precision landing on ID %d' %
                              current_landing_id)

                    send_land_message(x_ang, y_ang)

                    status_text = 'PRECISION LANDING ID %d: x=%.2f y=%.2f z=%.2f' % (
                        marker_id, tvec[0], tvec[1], tvec[2])
                    cv2.putText(np_data, status_text, (10, 50),
                                0, 0.6, (0, 255, 0), 2)
                    cv2.putText(np_data, 'PRECISION LANDING ACTIVE', (10, 80),
                                0, 0.7, (0, 0, 255), 2)
                    print(status_text)
                else:
                    # Normal detection annotation
                    info_text = 'ID %d: x=%.2f y=%.2f z=%.2f cm' % (
                        marker_id, tvec[0], tvec[1], tvec[2])
                    cv2.putText(np_data, info_text,
                                (10, 30 + 25 * i), 0, 0.5, (255, 0, 0), 2)
        else:
            notfound_count += 1

    except Exception as e:
        print('Error in msg_receiver: %s' % str(e))

    # Always try to publish the image (raw or annotated)
    try:
        if newimg_pub is not None and np_data is not None:
            new_msg = rnp.msgify(Image, np_data, encoding='rgb8')
            newimg_pub.publish(new_msg)
    except Exception as e:
        print('Error publishing image_new: %s' % str(e))

    time_last = now


######## SEARCH HELPERS ########

def check_for_marker_quick(duration=1.0):
    """
    Passive wait to let callback see frames; doesn't block detection.
    """
    global marker_found_flag
    marker_found_flag = False
    start = time.time()
    while time.time() - start < duration:
        time.sleep(0.1)
    return marker_found_flag


def search_pattern():
    """
    Lawn-mower search over a ~6x6m area above the house.
    """
    scan_speed = 0.3    # m/s forward/back
    stripe_width = 2.0  # m sideways
    house_span = 6.0    # m along stripe
    scan_duration = house_span / scan_speed
    num_stripes = 4

    print("Starting lawnmower search pattern...")

    # Move to starting corner (back-left relative to house center)
    print("Moving to starting corner...")
    send_velocity_smooth(velocity, velocity, 0, 5)
    send_local_ned_velocity(0, 0, 0)
    time.sleep(1)

    for stripe in range(num_stripes):
        print('Stripe %d/%d' % (stripe + 1, num_stripes))

        direction = -scan_speed if stripe % 2 == 0 else scan_speed
        segments = 6
        segment_time = scan_duration / segments

        for _ in range(segments):
            send_velocity_smooth(direction, 0, 0, segment_time)
            send_local_ned_velocity(0, 0, 0)
            check_for_marker_quick(0.2)

        send_local_ned_velocity(0, 0, 0)
        time.sleep(0.5)

        if stripe < num_stripes - 1:
            print("Shifting sideways to next stripe")
            sideways_duration = stripe_width / abs(velocity)
            send_velocity_smooth(0, -abs(velocity), 0, sideways_duration)
            send_local_ned_velocity(0, 0, 0)
            time.sleep(0.5)

    print("Lawnmower search complete.")
    return len(detected_markers) > 0


######## PRECISION LANDING ########

def precision_land_on_marker(marker_id):
    """
    Use GPS to get above marker; then let vision-based precision landing
    finish the job, driven by msg_receiver.
    """
    global mission_phase, current_landing_id

    if marker_id not in detected_markers:
        print("Marker ID %d not known; cannot precision land." % marker_id)
        return

    marker = detected_markers[marker_id]
    approach_alt = 4.0

    print("=" * 60)
    print("PRECISION LANDING ON MARKER ID %d" % marker_id)
    print("=" * 60)

    target_loc = LocationGlobalRelative(
        marker['marker_lat'],
        marker['marker_lon'],
        approach_alt
    )

    vehicle.mode = VehicleMode('GUIDED')
    while vehicle.mode.name != 'GUIDED':
        time.sleep(0.5)

    vehicle.simple_goto(target_loc)

    while True:
        pos = vehicle.location.global_relative_frame
        dlat = pos.lat - marker['marker_lat']
        dlon = pos.lon - marker['marker_lon']
        dist = math.sqrt(dlat * dlat + dlon * dlon) * 1.113195e5
        print("Distance to marker %d: %.2f m, Alt: %.2f m" %
              (marker_id, dist, pos.alt))

        if dist < 3.0 and abs(pos.alt - approach_alt) < 1.0:
            print("Above marker %d, ready for precision landing" % marker_id)
            break
        time.sleep(1)

    # Activate landing phase
    mission_phase = 'landing'
    current_landing_id = marker_id
    print("Camera-based precision landing ACTIVE for ID %d" % marker_id)

    # Wait for landing to complete
    last_alt = vehicle.location.global_relative_frame.alt
    still_counter = 0

    while vehicle.armed:
        pos = vehicle.location.global_relative_frame
        alt = pos.alt
        print("Landing... Alt: %.2f" % alt)

        if abs(alt - last_alt) < 0.05:
            still_counter += 1
        else:
            still_counter = 0

        if alt < 0.3:
            print("Altitude low (<0.3m). Landing complete.")
            break
        if still_counter > 10:
            print("No significant altitude change; assuming landed.")
            break

        last_alt = alt
        time.sleep(1)

    print("Landed on marker ID %d" % marker_id)

    # Reset state
    mission_phase = 'search'
    current_landing_id = None

    # Simulate water spray
    print("Spraying water on ID %d..." % marker_id)
    time.sleep(5)
    print("Water spraying completed for ID %d." % marker_id)


def perform_fire_extinguishing():
    """
    Land on all 'fire' markers (all IDs except id_person).
    """
    fire_ids = [mid for mid in detected_markers.keys() if mid != id_person]
    fire_ids = sorted(fire_ids)

    if not fire_ids:
        print("No fire markers detected (no non-72 IDs).")
        return

    print("=" * 60)
    print("FIRE EXTINGUISHING PHASE")
    print("Markers to extinguish (non-72): %s" % fire_ids)
    print("=" * 60)

    for i, mid in enumerate(fire_ids):
        print("\n----- FIRE %d/%d: marker ID %d -----" % (i + 1, len(fire_ids), mid))
        precision_land_on_marker(mid)

        if i < len(fire_ids) - 1:
            print("Re-arming to go to next fire...")
            arm_and_takeoff(takeoff_height)
        else:
            print("All fires extinguished.")


######## RETURN HOME ########

def return_to_original_and_land():
    global original_position

    if original_position is None:
        print("No original_position stored; cannot return home accurately.")
        return

    print("Returning to original position...")
    vehicle.mode = VehicleMode('GUIDED')
    while vehicle.mode.name != 'GUIDED':
        time.sleep(0.5)

    vehicle.simple_goto(original_position)

    while True:
        pos = vehicle.location.global_relative_frame
        dlat = pos.lat - original_position.lat
        dlon = pos.lon - original_position.lon
        dist = math.sqrt(dlat * dlat + dlon * dlon) * 1.113195e5
        print("Distance to home: %.2f m" % dist)
        if dist < 1.0:
            print("Close to home.")
            break
        time.sleep(1)

    print("Landing at home...")
    vehicle.mode = VehicleMode('LAND')
    while vehicle.armed:
        time.sleep(1)
    print("Landed at home. Mission complete.")


######## MAIN ########

if __name__ == '__main__':
    try:
        rospy.init_node('drone_node', anonymous=False)

        # Publisher + subscriber
        newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)
        sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)

        # 1) Take off
        arm_and_takeoff(takeoff_height)
        time.sleep(1)

        # 2) Fly to house location (10m north, 10m west)
        print("Flying to house location...")
        start_pos = vehicle.location.global_relative_frame
        house_lat = start_pos.lat + (10.0 / 111320.0)
        house_lon = start_pos.lon - (10.0 / (111320.0 * math.cos(start_pos.lat * math.pi / 180.0)))
        house_loc = LocationGlobalRelative(house_lat, house_lon, takeoff_height)
        vehicle.simple_goto(house_loc)

        while True:
            pos = vehicle.location.global_relative_frame
            dlat = pos.lat - house_lat
            dlon = pos.lon - house_lon
            dist = math.sqrt(dlat * dlat + dlon * dlon) * 1.113195e5
            print("Distance to house: %.2f m" % dist)
            if dist < 1.0:
                print("Arrived at house.")
                break
            time.sleep(1)

        time.sleep(2)

        # 3) Lawn mower search
        search_pattern()

        # 4) Print marker info + relative to 72 + AI summary
        if detected_markers:
            print("\n" + "=" * 60)
            print("ALL DETECTED MARKERS")
            print("=" * 60)
            for mid in sorted(detected_markers.keys()):
                m = detected_markers[mid]
                print("Marker ID %d" % mid)
                print("  Drone:  lat=%.8f lon=%.8f alt=%.2f" %
                      (m['drone_lat'], m['drone_lon'], m['drone_alt']))
                print("  Marker: lat=%.8f lon=%.8f alt=%.2f" %
                      (m['marker_lat'], m['marker_lon'], m['marker_alt']))
                print("  Distance: %.2f cm, X=%.2f, Y=%.2f, seen=%d" %
                      (m['distance_cm'], m['x_offset_cm'],
                       m['y_offset_cm'], m['detection_count']))
                print("")

            if id_person in detected_markers:
                print("=" * 60)
                print("RELATIVE POSITIONS TO PERSON (ID %d)" % id_person)
                print("=" * 60)
                ref = detected_markers[id_person]
                for mid in sorted(detected_markers.keys()):
                    if mid == id_person:
                        continue
                    rel = calculate_relative_position(detected_markers[mid], ref)
                    print("ID %d:" % mid)
                    print("  X (E/W): %.2f m (%s)" %
                          (abs(rel['x']), "East/Right" if rel['x'] > 0 else "West/Left"))
                    print("  Y (N/S): %.2f m (%s)" %
                          (abs(rel['y']), "North/Forward" if rel['y'] > 0 else "South/Backward"))
                    print("  Z (Up/Down): %.2f m (%s)" %
                          (abs(rel['z']), "Up" if rel['z'] > 0 else "Down"))
                    print("")

                print("=" * 60)
                print("AI-GENERATED NAVIGATION SUMMARY")
                print("=" * 60)
                summary = generate_gemini_summary(detected_markers, reference_id=id_person)
                print(summary)
                print("=" * 60)

            # 5) Precision land on each fire marker (non-72)
            perform_fire_extinguishing()

            # 6) Return home and land
            return_to_original_and_land()
        else:
            print("No markers detected during search.")
            return_to_original_and_land()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Keyboard interrupt; forcing LAND...")
        try:
            vehicle.mode = VehicleMode('LAND')
            while vehicle.armed:
                time.sleep(1)
        except:
            pass
        print("Emergency landing complete.")


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
from array import array
import urllib2
import json
from threading import Lock

######## VEHICLE & PARAMETERS ########

vehicle = connect('tcp:127.0.0.1:5763', wait_ready=True)
vehicle.parameters['PLND_ENABLED'] = 1
vehicle.parameters['PLND_TYPE'] = 1
vehicle.parameters['PLND_EST_TYPE'] = 0
vehicle.parameters['LAND_SPEED'] = 30  # cm/s

takeoff_height = 6.0          # m
fire_visit_altitude = 3.0     # m
precision_pre_alt = 5.0       # m
precision_marker_id = 72      # ArUco72
marker_size_search = 27       # cm (roof fires / general markers)
marker_size_precision = 20    # cm (20x20 pad for final landing)

velocity = -0.3               # m/s for search staging

# Publisher for annotated images
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

######## ARUCO / CAMERA PARAMS ########

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = 640
vertical_res = 480
horizontal_fov = 62.2 * (math.pi / 180.0)
vertical_fov = 48.8 * (math.pi / 180.0)

dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix = [[530.8269276712998, 0.0, 320.5],
                 [0.0, 530.8269276712998, 240.5],
                 [0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

######## STATE ########

detected_markers = {}    # marker_id -> dict with GPS + stats
original_position = None

time_last = 0.0          # search callback throttle
time_to_wait = 0.1

pl_time_last = 0.0       # precision callback throttle
pl_time_to_wait = 0.1

# For precision landing logging (match reference)
found_count = 0
notfound_count = 0

aruco_lock = Lock()      # shared lock to avoid overlapping callbacks

######## BASIC DRONE FUNCTIONS ########

def arm_and_takeoff(targetHeight):
    global original_position

    while not vehicle.is_armable:
        print('Waiting for vehicle to become armable')
        time.sleep(1)
    print('Vehicle is now armable')

    vehicle.mode = VehicleMode('GUIDED')
    while vehicle.mode != 'GUIDED':
        print('Waiting for drone to enter GUIDED flight mode')
        time.sleep(1)
    print('Vehicle now in GUIDED mode')

    vehicle.armed = True
    while not vehicle.armed:
        print('Waiting for vehicle to become armed')
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
    print('Original position saved: Lat=%.6f, Lon=%.6f, Alt=%.2f' %
          (original_position.lat, original_position.lon, original_position.alt))


def send_local_ned_velocity(vx, vy, vz):
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
    end_time = time.time() + duration
    while time.time() < end_time:
        send_local_ned_velocity(vx, vy, vz)
        time.sleep(0.1)


def send_land_message(x_ang, y_ang):
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


######## GEOMETRY / LLM ########

def calculate_marker_gps(drone_lat, drone_lon, drone_alt, x_offset_cm, y_offset_cm, distance_cm):
    x_offset_m = x_offset_cm / 100.0
    y_offset_m = y_offset_cm / 100.0
    distance_m = distance_cm / 100.0

    marker_alt = drone_alt - distance_m

    north_offset_m = -y_offset_m
    east_offset_m = x_offset_m

    dlat = north_offset_m / 111320.0
    dlon = east_offset_m / (111320.0 * math.cos(drone_lat * math.pi / 180.0))

    return {
        'lat': drone_lat + dlat,
        'lon': drone_lon + dlon,
        'alt': marker_alt
    }


def calculate_relative_position(marker, reference_marker):
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
    if reference_id not in detected_markers:
        return "Reference marker ID %d not found." % reference_id

    reference_marker = detected_markers[reference_id]

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

        return "Error: Could not extract response from Gemini API"

    except Exception as e:
        return "Error generating AI summary: " + str(e)


######## SEARCH PHASE ARUCO CALLBACK ########

def msg_receiver_search(message):
    global time_last, detected_markers

    if time.time() - time_last < time_to_wait:
        return

    if not aruco_lock.acquire(False):
        return

    try:
        np_data = rnp.numpify(message)
        np_data = np.ascontiguousarray(np_data)
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY).copy()

        try:
            corners, ids, rejected = aruco.detectMarkers(
                image=gray_img, dictionary=aruco_dict, parameters=parameters
            )
        except Exception as e:
            print("Error in aruco.detectMarkers (search):", e)
            return

        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                ret = aruco.estimatePoseSingleMarkers(
                    [corners[i]], marker_size_search,
                    cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff
                )
                (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])

                current = vehicle.location.global_relative_frame
                marker_gps = calculate_marker_gps(
                    current.lat, current.lon, current.alt,
                    float(tvec[0]), float(tvec[1]), float(tvec[2])
                )

                if mid not in detected_markers:
                    detected_markers[mid] = {
                        'marker_id': mid,
                        'drone_lat': current.lat,
                        'drone_lon': current.lon,
                        'drone_alt': current.alt,
                        'marker_lat': marker_gps['lat'],
                        'marker_lon': marker_gps['lon'],
                        'marker_alt': marker_gps['alt']
                    }
                    print("NEW MARKER DETECTED:", mid)

                aruco.drawDetectedMarkers(np_data, [corners[i]], np.array([[mid]]))
                aruco.drawAxis(np_data, np_camera_matrix, np_dist_coeff, rvec, tvec, 10)

        new_msg = rnp.msgify(Image, np_data, encoding='rgb8')
        newimg_pub.publish(new_msg)
        time_last = time.time()
    finally:
        aruco_lock.release()


######## SEARCH PATTERN ########

def search_pattern():
    print("Starting roof search (lawnmower)...")

    print("Staging to top-right...")
    send_velocity_smooth(-velocity, -velocity, 0, 8.0)
    send_local_ned_velocity(0, 0, 0)
    time.sleep(1.0)

    scan_speed = 0.3
    stripe_width = 2.0
    num_stripes = 4

    for stripe in range(num_stripes):
        direction = -scan_speed if stripe % 2 == 0 else scan_speed
        print("Stripe %d/%d, direction=%.2f" % (stripe + 1, num_stripes, direction))

        for _ in range(8):
            send_velocity_smooth(direction, 0, 0, 1.0)
            send_local_ned_velocity(0, 0, 0)
            time.sleep(0.1)

        if stripe < num_stripes - 1:
            print("Side step to next stripe...")
            send_velocity_smooth(0, -stripe_width / scan_speed, 0, stripe_width / scan_speed)
            send_local_ned_velocity(0, 0, 0)
            time.sleep(0.5)

    print("Search finished.")


######## VISIT FIRE TARGETS ########

def visit_fire_targets(visit_alt=3.0, dwell_time=5.0):
    fires = [mid for mid in detected_markers.keys() if mid != precision_marker_id]
    fires.sort()

    if not fires:
        print("No fire markers detected (IDs != 72). Skipping fire visit phase.")
        return

    print("\nVISITING FIRE TARGETS at %.1fm" % visit_alt)

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        print("Waiting for GUIDED before visiting fires...")
        time.sleep(1)

    for mid in fires:
        marker = detected_markers[mid]
        target = LocationGlobalRelative(marker['marker_lat'],
                                        marker['marker_lon'],
                                        visit_alt)
        print("\nGoing to FIRE ID %d at (%.6f, %.6f, %.1f)" %
              (mid, marker['marker_lat'], marker['marker_lon'], visit_alt))
        vehicle.simple_goto(target)

        while True:
            cur = vehicle.location.global_relative_frame
            dlat = cur.lat - target.lat
            dlon = cur.lon - target.lon
            dist = math.sqrt(dlat * dlat + dlon * dlon) * 1.113195e5
            alt_err = abs(cur.alt - visit_alt)
            print("  Distance to fire %d: %.2f m (alt err: %.2f)" %
                  (mid, dist, alt_err))

            if dist < 2.5 and alt_err < 1.0:
                print("  Reached fire ID %d (within 2.5m)." % mid)
                break
            time.sleep(1)

        print("  Simulated roof landing: waiting %d seconds..." % dwell_time)
        time.sleep(dwell_time)

    print("\nAll fire targets visited.\n")


######## RETURN HOME ########

def return_home(home_alt=8.0):
    print("Returning to original position at %.1fm..." % home_alt)

    home = LocationGlobalRelative(original_position.lat,
                                  original_position.lon,
                                  home_alt)
    vehicle.mode = VehicleMode('GUIDED')
    while vehicle.mode != 'GUIDED':
        print("Waiting for GUIDED before return_home...")
        time.sleep(1)

    vehicle.simple_goto(home)

    while True:
        cur = vehicle.location.global_relative_frame
        dlat = cur.lat - home.lat
        dlon = cur.lon - home.lon
        dist = math.sqrt(dlat * dlat + dlon * dlon) * 1.113195e5
        alt_err = abs(cur.alt - home_alt)
        print("  Distance to home: %.2f m (alt err: %.2f)" % (dist, alt_err))
        if dist < 2.0 and alt_err < 0.7:
            print("Reached home staging point.")
            break
        time.sleep(1)


######## PRECISION LANDING CALLBACK (REFERENCE STYLE) ########

def precision_msg_receiver(message):
    global pl_time_last, found_count, notfound_count

    if time.time() - pl_time_last < pl_time_to_wait:
        return

    if not aruco_lock.acquire(False):
        return

    try:
        np_data = rnp.numpify(message)
        np_data = np.ascontiguousarray(np_data)
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY).copy()

        ids = ''
        try:
            corners, ids, rejected = aruco.detectMarkers(
                image=gray_img, dictionary=aruco_dict, parameters=parameters
            )
        except Exception as e:
            print("Error in aruco.detectMarkers (precision):", e)
            return

        try:
            if ids is not None and precision_marker_id in ids.flatten():
                idx = list(ids.flatten()).index(precision_marker_id)

                ret = aruco.estimatePoseSingleMarkers(
                    corners, marker_size_precision,
                    cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff
                )
                (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])

                x = '{:.2f}'.format(tvec[0])
                y = '{:.2f}'.format(tvec[1])
                z = '{:.2f}'.format(tvec[2])

                x_sum = 0
                y_sum = 0

                x_sum = (corners[idx][0][0][0] + corners[idx][0][1][0] +
                         corners[idx][0][2][0] + corners[idx][0][3][0])
                y_sum = (corners[idx][0][0][1] + corners[idx][0][1][1] +
                         corners[idx][0][2][1] + corners[idx][0][3][1])

                x_avg = x_sum / 4.0
                y_avg = y_sum / 4.0

                x_ang = (x_avg - horizontal_res * 0.5) * horizontal_fov / horizontal_res
                y_ang = (y_avg - vertical_res * 0.5) * vertical_fov / vertical_res

                if vehicle.mode != 'LAND':
                    vehicle.mode = VehicleMode('LAND')
                    while vehicle.mode != 'LAND':
                        time.sleep(1)
                    print('Vehicle in LAND mode')
                    send_land_message(x_ang, y_ang)
                else:
                    send_land_message(x_ang, y_ang)

                marker_position = 'MARKER POSITION: x=' + x + ' y=' + y + ' z=' + z

                aruco.drawDetectedMarkers(np_data, corners)
                aruco.drawAxis(np_data, np_camera_matrix, np_dist_coeff, rvec, tvec, 10)
                cv2.putText(np_data, marker_position, (10, 50), 0, 0.7,
                            (255, 0, 0), thickness=2)
                print(marker_position)
                print('FOUND COUNT: ' + str(found_count) +
                      ' NOTFOUND COUNT: ' + str(notfound_count))

                found_count += 1
            else:
                notfound_count += 1
        except Exception as e:
            print('Target likely not found (precision)')
            print(e)
            notfound_count += 1

        new_msg = rnp.msgify(Image, np_data, encoding='rgb8')
        newimg_pub.publish(new_msg)
        pl_time_last = time.time()
    finally:
        aruco_lock.release()


######## FINAL PRECISION LANDING SEQUENCE ########

def precision_land():
    global found_count, notfound_count

    print("\n=== FINAL PRECISION LANDING ON ARUCO 72 (20cm) ===")

    found_count = 0
    notfound_count = 0

    # Pre-descent to ~5m above home
    home5 = LocationGlobalRelative(original_position.lat,
                                   original_position.lon,
                                   precision_pre_alt)
    print("Pre-descent to %.1fm before PLND..." % precision_pre_alt)
    vehicle.mode = VehicleMode('GUIDED')
    while vehicle.mode != 'GUIDED':
        print("Waiting for GUIDED before pre-descent...")
        time.sleep(1)

    vehicle.simple_goto(home5)

    while vehicle.location.global_relative_frame.alt > precision_pre_alt + 0.3:
        print("  descending... alt: %.2f" %
              vehicle.location.global_relative_frame.alt)
        time.sleep(1)

    print("At ~%.1fm, starting precision landing using PLND..." %
          precision_pre_alt)

    pl_sub = rospy.Subscriber('/camera/color/image_raw', Image, precision_msg_receiver)

    while vehicle.armed:
        print("Precision landing... Altitude: %.2f" %
              vehicle.location.global_relative_frame.alt)
        time.sleep(1)

    pl_sub.unregister()
    print("Precision landing complete. Motors disarmed on ArUco72 pad.")


######## MAIN ########

if __name__ == '__main__':
    try:
        rospy.init_node('drone_node', anonymous=False)

        # 1) Takeoff
        arm_and_takeoff(takeoff_height)
        time.sleep(1)

        # 2) Fly to house (~10m north, 10m west)
        print("Flying to house location...")
        base = vehicle.location.global_relative_frame
        house_location = LocationGlobalRelative(
            base.lat + (10.0 / 111320.0),
            base.lon - (10.0 / (111320.0 * math.cos(base.lat * math.pi / 180.0))),
            6.0
        )
        vehicle.simple_goto(house_location)

        while True:
            cur = vehicle.location.global_relative_frame
            dlat = cur.lat - house_location.lat
            dlon = cur.lon - house_location.lon
            dist = math.sqrt(dlat * dlat + dlon * dlon) * 1.113195e5
            print("To house:", dist)
            if dist < 1.0:
                print("Arrived at house.")
                break
            time.sleep(1)

        time.sleep(2)

        # 3) Start search subscriber & pattern
        print("Starting ArUco search subscriber...")
        search_sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver_search)

        search_pattern()

        # 4) Stop search subscriber before PLND to avoid interference
        if search_sub is not None:
            search_sub.unregister()
            print("Search subscriber stopped.")

        # 5) Print detected markers & LLM summary
        if detected_markers:
            print("\n=== DETECTED MARKERS REPORT ===")
            for mid in sorted(detected_markers.keys()):
                m = detected_markers[mid]
                print("ID %d: marker GPS=(%.6f, %.6f, %.2f)" %
                      (mid, m['marker_lat'], m['marker_lon'], m['marker_alt']))

            if precision_marker_id in detected_markers:
                ref = detected_markers[precision_marker_id]
                print("\n=== RELATIVE POSITIONS TO ARUCO 72 (PERSON) ===")
                for mid in sorted(detected_markers.keys()):
                    if mid == precision_marker_id:
                        continue
                    m = detected_markers[mid]
                    rel = calculate_relative_position(m, ref)
                    print("Fire ID %d:" % mid)
                    print("  X (E/W): %.2f m" % rel['x'])
                    print("  Y (N/S): %.2f m" % rel['y'])
                    print("  Z (Up/Down): %.2f m" % rel['z'])

                print("\n=== LLM NAVIGATION SUMMARY ===")
                summary = generate_gemini_summary(detected_markers, reference_id=precision_marker_id)
                print(summary)
                print("=== END SUMMARY ===\n")

        else:
            print("No markers detected during search.")

        # 6) Visit fire targets (IDs != 72) at 3m
        visit_fire_targets(visit_alt=fire_visit_altitude, dwell_time=5.0)

        # 7) Return home to 8m
        return_home(home_alt=8.0)

        # 8) Final precision landing like reference script, from 5m
        precision_land()

        print("\nMISSION COMPLETE.\n")

    except rospy.ROSInterruptException:
        pass


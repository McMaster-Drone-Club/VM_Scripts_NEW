#!/usr/bin/env python

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
takeoff_height=18 #m

# Store original position
original_position = None
########################
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

id_to_find = 72 ##arucoID
marker_size = 20 ##CM

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = 640
vertical_res = 480

horizontal_fov = 62.2 * (math.pi / 180) ##62.2 for picam V2, 53.5 for V1
vertical_fov = 48.8 * (math.pi / 180) ##48.8 for V2, 41.41 for V1

found_count=0
notfound_count=0
marker_found = False

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
    global notfound_count, found_count, time_last, time_to_wait, id_to_find, marker_found

    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(message) ##Deserialize image data into array
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)

        try:
            if ids is not None:
                if ids[0]==id_to_find:
                    ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=np_camera_matrix,distCoeffs=np_dist_coeff)
                    (rvec, tvec) = (ret[0][0,0,:],ret[1][0,0,:])
                    x = '{:.2f}'.format(tvec[0]) ### Xerror/distance between camera and aruco in CM
                    y = '{:.2f}'.format(tvec[1]) ### Yerror/distance between camera and aruco in CM
                    z = '{:.2f}'.format(tvec[2]) ### Zerror/distance between camera and aruco in CM

                    y_sum=0
                    x_sum=0

                    x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] +corners[0][0][3][0]
                    y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] +corners[0][0][3][1]

                    x_avg = x_sum / 4
                    y_avg = y_sum / 4

                    x_ang = (x_avg - horizontal_res*.5)*horizontal_fov/horizontal_res
                    y_ang = (y_avg - vertical_res*.5)*vertical_fov/vertical_res

                    if vehicle.mode !='LAND':
                        vehicle.mode = VehicleMode('LAND')
                        while vehicle.mode !='LAND':
                            time.sleep(1)
                        print('Vehicle in LAND mode')
                        marker_found = True
                        send_land_message(x_ang,y_ang)
                    else:
                        send_land_message(x_ang,y_ang)

                    marker_position = 'MARKER POSITION: x='+x+' y='+y+' z='+z

                    aruco.drawDetectedMarkers(np_data,corners)
                    aruco.drawAxis(np_data,np_camera_matrix,np_dist_coeff,rvec,tvec,10)
                    ##putText(image, text_to_draw,position,font_face,fontScale,color,thickness)
                    cv2.putText(np_data,marker_position,(10,50),0,.7,(255,0,0),thickness=2)
                    print(marker_position)
                    print('FOUND COUNT: '+str(found_count)+ ' NOTFOUND COUNT: '+str(notfound_count))

                    found_count = found_count + 1
                else:
                    notfound_count=notfound_count+1
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
            
            # Quick marker check
            if check_for_marker_quick():
                print('Marker found on stripe %d, segment %d!' % (stripe + 1, seg + 1))
                return True
        
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
            if check_for_marker_quick():
                print('Marker found at stripe transition %d!' % (stripe + 1))
                return True
    
    print("Lawnmower search pattern complete. Marker not found.")
    return False


def subscriber():
    """Monitor for landing and wait on ground"""
    rospy.init_node('drone_node',anonymous=False)
    sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)
    
    # Monitor until landed
    while vehicle.armed:
        time.sleep(0.5)
    
    print("Drone has landed and disarmed!")
    sub.unregister()
    
    # Wait 8 seconds on the ground
    print("Waiting 8 seconds on the ground...")
    time.sleep(8)
    
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
            18  # 18m altitude for better view
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
        
        if marker_found:
            # Continue monitoring and landing
            print("Marker found! Proceeding with precision landing...")
            subscriber()  # This waits until landed + 8 seconds
            
            # Return to original position
            return_to_original()
            
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
            
    except rospy.ROSInterruptException:
        pass

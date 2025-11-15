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

velocity=-.5 #m/s
takeoff_height=9 #m
########################
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

ids_to_find = [129,72]
marker_sizes = [40,20]
marker_heights = [10,4]

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = 640
vertical_res = 480

horizontal_fov = 62.2 * (math.pi / 180) ##62.2 for picam V2, 53.5 for V1
vertical_fov = 48.8 * (math.pi / 180) ##48.8 for V2, 41.41 for V1

found_count=0
notfound_count=0

#############CAMERA INTRINSICS#######

dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix = [[530.8269276712998, 0.0, 320.5],[0.0, 530.8269276712998, 240.5],[0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

#####
time_last=0
time_to_wait = .1 ##100 ms

time_to_sleep=5 ##seconds the drone will wait after dropping off the taco
sub = '' ##initialiaze subscriber variable to make it global
################FUNCTIONS###############
def get_distance_meters(targetLocation,currentLocation):
    dLat=targetLocation.lat - currentLocation.lat
    dLon=targetLocation.lon - currentLocation.lon

    return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5

def goto(targetLocation):
    distanceToTargetLocation = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)

    vehicle.simple_goto(targetLocation)

    while vehicle.mode.name=="GUIDED":
        currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
        if currentDistance<distanceToTargetLocation*.02:
            print("Reached target waypoint")
            time.sleep(2)
            break
        time.sleep(1)
    return None

def arm_and_takeoff(targetHeight):
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






def msg_receiver(message):
    global notfound_count, found_count, time_last, time_to_wait, id_to_find, sub

    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(message) ##Deserialize image data into array
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)

        altitude = vehicle.location.global_relative_frame.alt ##meters

        id_to_find=0
        marker_height=0
        marker_size=0

        if altitude > marker_heights[1]:
            id_to_find=ids_to_find[0]
            marker_height=marker_heights[0]
            marker_size=marker_sizes[0]
        elif altitude < marker_heights[1]:
            id_to_find=ids_to_find[1]
            marker_height=marker_heights[1]
            marker_size=marker_sizes[1]

        ids_array_index=0
        found_id=0
        print("Looking for marker: "+str(id_to_find))

        if vehicle.mode!='LAND':
            vehicle.mode = VehicleMode('LAND')
            while vehicle.mode!='LAND':
                time.sleep(1)
            print("Vehicle is LAND mode.")

        ####We have landed the drone and no longer need to track images
        if vehicle.armed==False:
            sub.unregister()
            return None


        try:
            if ids is not None:
                for id in ids:
                    if id==id_to_find:
                        corners_single=[corners[ids_array_index]]
                        corners_single_np = np.asarray(corners_single)

                        ret = aruco.estimatePoseSingleMarkers(corners_single,marker_size,cameraMatrix=np_camera_matrix,distCoeffs=np_dist_coeff)
                        (rvec, tvec) = (ret[0][0,0,:],ret[1][0,0,:])
                        x = '{:.2f}'.format(tvec[0]) ### Xerror/distance between camera and aruco in CM
                        y = '{:.2f}'.format(tvec[1]) ### Yerror/distance between camera and aruco in CM
                        z = '{:.2f}'.format(tvec[2]) ### Zerror/distance between camera and aruco in CM

                        y_sum=0
                        x_sum=0

                        x_sum = corners_single_np[0][0][0][0] + corners_single_np[0][0][1][0] + corners_single_np[0][0][2][0] +corners_single_np[0][0][3][0]
                        y_sum = corners_single_np[0][0][0][1] + corners_single_np[0][0][1][1] + corners_single_np[0][0][2][1] +corners_single_np[0][0][3][1]

                        x_avg = x_sum / 4
                        y_avg = y_sum / 4

                        x_ang = (x_avg - horizontal_res*.5)*horizontal_fov/horizontal_res
                        y_ang = (y_avg - vertical_res*.5)*vertical_fov/vertical_res

                        if vehicle.mode !='LAND':
                            vehicle.mode = VehicleMode('LAND')
                            while vehicle.mode !='LAND':
                                time.sleep(1)
                            print('Vehicle in LAND mode')
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
                        print("")

                        found_count = found_count + 1
                        found_id=1
                        break
                    ids_array_index=ids_array_index+1
                if found_id==0:
                    notfound_count = notfound_count + 1
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


def lander():
    global sub
    rospy.init_node('drone_node',anonymous=False)
    sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)

    while vehicle.armed==True:
        time.sleep(1)
    return None


if __name__=='__main__':
    try:
        ###Record home coordinates of drone so we know where to fly back to after delivery
        lat_home=vehicle.location.global_relative_frame.lat
        lon_home=vehicle.location.global_relative_frame.lon

        wp_home=LocationGlobalRelative(lat_home,lon_home,takeoff_height)
        wp_taco=LocationGlobalRelative(-35.36303741,149.1652374,takeoff_height) ##Original waypoint +25 meters

        arm_and_takeoff(takeoff_height)
        time.sleep(1)

        ############Fly to taco dropoff waypoint
        goto(wp_taco)

        ############Precision land on taco arucos
        lander()
        print("")
        print("----------------------------------")
        print("Arrived at the taco destination")
        print("Dropping tacos and heading home.")
        print("-----------ENJOY------------------")
        time.sleep(time_to_sleep)

        ###########Fly the drone back to home waypoint
        arm_and_takeoff(takeoff_height)
        goto(wp_home)
        lander()
        print("")
        print("----------------------------------")
        print("Made it home for another delivery")
        print("----------------------------------")

    except rospy.ROSInterruptException:
        pass

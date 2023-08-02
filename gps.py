#!/usr/bin/env python
import rospy
import math
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *


def send_waypoint(lat, lon, alt):
    waypoint = Waypoint()
    waypoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    waypoint.command = 16  # Waypoint command value for NAV_WAYPOINT
    waypoint.is_current = True
    waypoint.autocontinue = True
    waypoint.param1 = 0  # Hold time in seconds (0 for indefinite)
    waypoint.param2 = 0  # Acceptance radius in meters
    waypoint.param3 = 0  # Pass radius in meters
    waypoint.param4 = float('nan')  # Yaw angle (NaN for default)

    waypoint.x_lat = lat
    waypoint.y_long = lon
    waypoint.z_alt = alt

    # Create a waypoint list and add the waypoint
    waypoint_list = WaypointList()
    waypoint_list.waypoints.append(waypoint)

    # Push the waypoint list to ArduPilot using mavros service
    rospy.wait_for_service('/mavros/mission/push')
    push_service = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
    response = push_service(0, waypoint_list.waypoints)

    if response.success:
        rospy.loginfo("Waypoint sent successfully!")
    else:
        rospy.logerr("Failed to send waypoint.")


def setGuidedMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
    except rospy.ServiceException as e:
        print ("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" %e)
        
def setStabilizeMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        #http://wiki.ros.org/mavros/CustomModes for custom modes
        isModeChanged = flightModeService(custom_mode='STABILIZE') #return true or false
    except rospy.ServiceException as e:
        print ("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled"%e)

def setRTLMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='RTL') #return true or false
    except rospy.ServiceException as e:
        print ("Service takeoff call failed: %s"%e)

def setLandMode():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        #http://wiki.ros.org/mavros/CustomModes for custom modes
        isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException as e:
        print ("service land call failed: %s. The vehicle cannot land "%e)
          
def setArm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException as e:
        print ("Service arm call failed: %s"%e)
        
def setDisarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
    except rospy.ServiceException as e:
        print ("Service arm call failed: %s"%e)


def setTakeoffMode():
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
        takeoffService(altitude = 4, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException as e:
        print ("Service takeoff call failed: %s"%e)


def globalPositionCallback(globalPositionCallback):
    global latitude
    global longitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude
    # print ("longitude: %.7f" %longitude)
    #print ("latitude: %.7f" %latitude)

def menu():
    print ("Press")
    print ("1: to set mode to GUIDED")
    print ("2: to set mode to STABILIZE")
    print ("3: to set mode to ARM the drone")
    print ("4: to set mode to DISARM the drone")
    print ("5: to set mode to TAKEOFF")
    print ("6: to set mode to LAND")
    print ("7: Get drone position")
    # print ("8: to set mode to RTL")
    
def myLoop():
    x='1'
    while ((not rospy.is_shutdown())and (x in ['1','2','3','4','5','6','7'])):
        menu()
        x = input("Enter your input: ")
        if (x=='1'):
            setGuidedMode()
        elif(x=='2'):
            setStabilizeMode()
        elif(x=='3'):
            setArm()
        elif(x=='4'):
            setDisarm()
        elif(x=='5'):
            setTakeoffMode()
        elif(x=='6'):
            setLandMode()
        elif(x=='7'):                
            # global latitude
            # global longitude
            print ("longitude: %.7f" %longitude)
            print ("latitude: %.7f" %latitude)
        # elif(x=='8'):  
        #     setRTLMode()
        else: 
            print ("Exit")
        
        
    

if __name__ == '__main__':
    # rospy.init_node('dronemap_node', anonymous=True, gps_listener)
    rospy.init_node('gps_listener')
    # rospy.init_node('waypoint_sender')
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
    # spin() simply keeps python from exiting until this node is stopped
    
    #listener()
    myLoop()
    #rospy.spin()


# import rospy
# from mavros_msgs.msg import Waypoint, WaypointList
# from mavros_msgs.srv import WaypointPush
# from sensor_msgs.msg import NavSatFix

# # Callback function for the global position
# def globalPositionCallback_drone(data):
#     # Get the latitude, longitude, and altitude values from the NavSatFix message
#     latitude = data.latitude
#     longitude = data.longitude
#     altitude = data.altitude

#     # Send the waypoint using the obtained coordinates
#     send_waypoint(latitude, longitude, altitude)

# def send_waypoint(lat, lon, alt):
#     waypoint = Waypoint()
#     waypoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
#     waypoint.command = 16  # Waypoint command value for NAV_WAYPOINT
#     waypoint.is_current = True
#     waypoint.autocontinue = True
#     waypoint.param1 = 0  # Hold time in seconds (0 for indefinite)
#     waypoint.param2 = 0  # Acceptance radius in meters
#     waypoint.param3 = 0  # Pass radius in meters
#     waypoint.param4 = float('nan')  # Yaw angle (NaN for default)

#     waypoint.x_lat = lat
#     waypoint.y_long = lon
#     waypoint.z_alt = alt

#     # Create a waypoint list and add the waypoint
#     waypoint_list = WaypointList()
#     waypoint_list.waypoints.append(waypoint)

#     # Push the waypoint list to ArduPilot using mavros service
#     rospy.wait_for_service('/mavros/mission/push')
#     push_service = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
#     response = push_service(0, waypoint_list.waypoints)

#     if response.success:
#         rospy.loginfo("Waypoint sent successfully!")
#     else:
#         rospy.logerr("Failed to send waypoint.")

# rospy.init_node('waypoint_sender')

# # Subscribe to the global position topic and set the callback function
# rospy.Subscriber('/mavros/global_position/global', NavSatFix, globalPositionCallback_drone)

# # Spin the ROS node
# rospy.spin()









# import rospy
# from sensor_msgs.msg import NavSatFix

# from mavros_msgs.msg import Waypoint, WaypointList
# from mavros_msgs.srv import WaypointPush

# def globalPositionCallback_drone(msg):
#     # Extract the GPS position data from the message
#     latitude = msg.latitude
#     longitude = msg.longitude
#     altitude = msg.altitude

#     send_waypoint(latitude, longitude, altitude)

#     # Print the GPS position
#     print("UGVPosition: Latitude={}, Longitude={}, Altitude={}".format(latitude, longitude, altitude))





# def send_waypoint(lat, lon, alt):
#     waypoint = Waypoint()
#     waypoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
#     waypoint.command = Waypoint.MAV_CMD_NAV_WAYPOINT
#     waypoint.is_current = True
#     waypoint.autocontinue = True
#     waypoint.param1 = 0  # Hold time in seconds (0 for indefinite)
#     waypoint.param2 = 0  # Acceptance radius in meters
#     waypoint.param3 = 0  # Pass radius in meters
#     waypoint.param4 = float('nan')  # Yaw angle (NaN for default)
#     waypoint.x_lat = lat
#     waypoint.y_long = lon
#     waypoint.z_alt = alt

#     # Create a waypoint list and add the waypoint
#     waypoint_list = WaypointList()
#     waypoint_list.waypoints.append(waypoint)

#     # Push the waypoint list to ArduPilot using mavros service
#     rospy.wait_for_service('/mavros/mission/push')
#     push_service = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
#     response = push_service(waypoint_list.waypoints)

#     if response.success:
#         rospy.loginfo("Waypoint sent successfully!")
#     else:
#         rospy.logerr("Failed to send waypoint.")

# # rospy.init_node('gps_listener')
# rospy.init_node('waypoint_sender')
# rospy.Subscriber('/mavros/global_position/global', NavSatFix, globalPositionCallback_drone)


# rospy.spin()



# import rospy
# from mavros_msgs.msg import Waypoint, WaypointList
# from mavros_msgs.srv import WaypointPush

# def send_waypoint(lat, lon, alt):
#     waypoint = Waypoint()
#     waypoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
#     waypoint.command = 16
#     waypoint.is_current = True
#     waypoint.autocontinue = True
#     waypoint.param1 = 0  # Hold time in seconds (0 for indefinite)
#     waypoint.param2 = 0  # Acceptance radius in meters
#     waypoint.param3 = 0  # Pass radius in meters
#     waypoint.param4 = 0  # Yaw angle (0 for north)

#     waypoint.x_lat = lat
#     waypoint.y_long = lon
#     waypoint.z_alt = alt

#     # Create a waypoint list and add the waypoint
#     waypoint_list = WaypointList()
#     waypoint_list.waypoints.append(waypoint)

#     # Push the waypoint list to ArduPilot using mavros service
#     rospy.wait_for_service('/mavros/mission/push')
#     push_service = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
#     response = push_service(0, waypoint_list.waypoints)

#     if response.success:
#         rospy.loginfo("Waypoint sent successfully!")
#     else:
#         rospy.logerr("Failed to send waypoint.")

# if __name__ == '__main__':
#     rospy.init_node('waypoint_sender')

#     # Define the latitude, longitude, and altitude values for the waypoint
#     latitude = 47.123456  # Replace with the desired latitude
#     longitude = -122.987654  # Replace with the desired longitude
#     altitude = 10.0  # Replace with the desired altitude
#     send_waypoint(latitude, longitude, altitude)





# import rospy
# from sensor_msgs.msg import NavSatFix

# def drone_position_callback(msg):
#     # Callback function for drone1 position
#     latitude = msg.latitude
#     longitude = msg.longitude
#     altitude = msg.altitude
#     # Process the position data for drone1 as needed
#     print("Drone 1 Position: Latitude={}, Longitude={}, Altitude={}".format(latitude, longitude, altitude))

# def ugv_position_callback(msg):
#     # Callback function for drone2 position
#     latitude = msg.latitude
#     longitude = msg.longitude
#     altitude = msg.altitude
#     # Process the position data for drone2 as needed
#     print("Drone 2 Position: Latitude={}, Longitude={}, Altitude={}".format(latitude, longitude, altitude))

# def main():
#     rospy.init_node('drone_position_listener')

#     # Subscribe to the position topics for both drones
#     drone1_position_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, drone_position_callback)
#     ugv_position_sub = rospy.Subscriber('/gps_sensor/fix', NavSatFix, ugv_position_callback)

#     # Spin the ROS event loop
#     rospy.spin()

# if __name__ == '__main__':
#     main()


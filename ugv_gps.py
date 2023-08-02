import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import NavSatFix

class GPSPlugin:
    def __init__(self):
        rospy.init_node('gps_plugin')
        self.publisher = rospy.Publisher('/model/gps', NavSatFix, queue_size=10)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

    def model_states_callback(self, msg):
        # Get the index of the model you want to retrieve GPS data from
        model_index = msg.name.index('robovolc_v14')

        # Get the GPS position from the model state
        gps_data = msg.pose[model_index].position

        # Create a NavSatFix message and populate it with GPS data
        gps_msg = NavSatFix()
        gps_msg.latitude = gps_data.x
        gps_msg.longitude = gps_data.y
        gps_msg.altitude = gps_data.z

        # Publish the GPS data to the ROS topic
        self.publisher.publish(gps_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    plugin = GPSPlugin()
    plugin.run()





#!/usr/bin/env python

# import rospy
# from mavros_msgs.msg import GlobalPositionTarget

# def drone_position_callback(msg):
#     # Callback function for drone1 position
#     latitude = msg.latitude
#     longitude = msg.longitude
#     altitude = msg.altitude
#     # Process the position data for drone1 as needed
#     print("Drone Position: Latitude={}, Longitude={}, Altitude={}".format(latitude, longitude, altitude))

# def ugv_position_callback(msg):
#     # Callback function for drone2 position
#     latitude = msg.latitude
#     longitude = msg.longitude
#     altitude = msg.altitude
#     # Process the position data for drone2 as needed
#     print("UGV Position: Latitude={}, Longitude={}, Altitude={}".format(latitude, longitude, altitude))

# def main():
#     rospy.init_node('drone_position_listener')

#     # Subscribe to the position topics for both drones
#     # drone1_position_sub = rospy.Subscriber('/iris_demo/mavros/global_position/global', GlobalPositionTarget, drone_position_callback)
#     drone2_position_sub = rospy.Subscriber('/mavros/global_position/global', GlobalPositionTarget, ugv_position_callback)
    

#     # Spin the ROS event loop
#     rospy.spin()

# if __name__ == '__main__':
#     main()



# import rospy
# from sensor_msgs.msg import NavSatFix

# from mavros_msgs.msg import Waypoint, WaypointList
# from mavros_msgs.srv import WaypointPush

# def globalPositionCallback_drone(msg):
#     # Extract the GPS position data from the message
#     latitude = msg.latitude
#     longitude = msg.longitude
#     altitude = msg.altitude

#     # Print the GPS position
#     print("DronePosition: Latitude={}, Longitude={}, Altitude={}".format(latitude, longitude, altitude))

# rospy.init_node('gps_listener')
# rospy.Subscriber('/mavros/global_position/global', NavSatFix, globalPositionCallback_drone)
# rospy.spin()


# !/usr/bin/env python

# from geographiclib.geodesic import Geodesic
# from geometry_msgs.msg import PoseStamped
# import rospy

# def pose_callback(msg):
#     x = msg.pose.position.x
#     y = msg.pose.position.y
#     z = msg.pose.position.z
    

#     print(x, y)

#     lat0 = -35.363252  # Reference latitude (San Francisco coordinates)
#     lon0 = 149.165243  # Reference longitude (San Francisco coordinates)
#     az = 0.0  # Assuming movement towards the north (adjust as needed)
#     distance = ((x ** 2) + (y ** 2) + (z ** 2)) ** 0.5  # Euclidean distance

#     result = Geodesic.WGS84.Direct(lat0, lon0, az, distance)
#     lat, lon, _, _ = result['lat2'], result['lon2'], result['azi2'], result['azi1']

#     print("Converted Coordinates:")
#     print("Latitude: ", lat)
#     print("Longitude: ", lon)
#     print("Altitude: ", z)

# def main():
#     rospy.init_node('coordinate_conversion')

#     rospy.Subscriber('mavros/local_position/pose', PoseStamped, pose_callback)

#     rospy.spin()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass


#!/usr/bin/env python

# from geographiclib.geodesic import Geodesic
# from gazebo_msgs.msg import ModelStates
# from mavros_msgs.msg import GlobalPositionTarget
# import rospy
# import math

# global_position_pub = None

# def model_state_callback(msg):
#     model_index = msg.name.index('robovolc_v14')  # Replace 'your_model_name' with the name of your model in Gazebo

#     x = msg.pose[model_index].position.x
#     y = -msg.pose[model_index].position.y
#     z = msg.pose[model_index].position.z

#     lat0 = -35.363262  # Set your reference latitude here
#     lon0 = 149.165237  # Set your reference longitude here
#     az = 0.0  # Assuming the model is moving towards the north (you can adjust the heading angle)
#     distance = ((x ** 2) + (y ** 2) + (z ** 2)) ** 0.5  # Euclidean distance

#     result = Geodesic.WGS84.Direct(lat0, lon0, az, distance)
#     lat, lon, _, _ = result['lat2'], result['lon2'], result['azi2'], result['azi1']

#     delta_lon = lon - lon0
#     heading_angle = (360 - delta_lon) % 360
    

#     global_position = GlobalPositionTarget()
#     global_position.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT
#     global_position.latitude = lat
#     global_position.longitude = lon
#     global_position.altitude = z

    # lat = lat0 + (y / (1.0 * 111111.0))  # Approximate conversion based on Earth's circumference
    # lon = lon0 + (x / (1.0 * 111111.0 * math.cos(math.radians(lat0))))
    

#     print(x, y, lat, lon)

#     # Publish the converted global position
#     global_position_pub.publish(global_position)

# def main():
#     global global_position_pub
#     rospy.init_node('gazebo_to_mavros_conversion')

#     rospy.Subscriber('/gazebo/model_states', ModelStates, model_state_callback)
#     global_position_pub = rospy.Publisher('/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=10)

#     rospy.spin()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass





###########################################################################################################################

# import math
# import rospy
# from gazebo_msgs.msg import ModelStates
# from geometry_msgs.msg import PoseStamped
# from sensor_msgs.msg import NavSatFix
# from mavros_msgs.msg import Waypoint, WaypointList
# from mavros_msgs.srv import WaypointPush
# from geographiclib.geodesic import Geodesic
# from geometry_msgs.msg import PoseStamped



# # Callback function for the model states
# def modelStatesCallback(data):
#     # Find the index of the desired model
#     model_index = data.name.index("robovolc_v14")  # Replace "your_model_name" with the actual name of your model

#     # Get the pose of the model
#     model_pose = data.pose[model_index]

#     # Convert the pose to GPS coordinates
#     gps_coords = convert_to_gps(model_pose)

    

#     # Process and use the GPS coordinates as needed
#     # print("UGV: Latitude={}, Longitude={}, Altitude={}".format(gps_coords.latitude, gps_coords.longitude, gps_coords.altitude))

# def convert_to_gps(pose):
#     # Convert the model pose to GPS coordinates
#     # Implement the conversion logic based on your model's reference frame and scale

#     reference_latitude = -35.3632605  # Reference latitude in degrees
#     reference_longitude = 149.1652391  # Reference longitude in degrees
#     reference_altitude = 603.388641432232  # Reference altitude in meters

#     scale = 1.0  # Scaling factor for the GPS coordinates 

#     # Obtain the position of the model
#     # x = pose.position.x
#     # y = pose.position.y
#     # z = pose.position.z

#     x = -2.8146765461255305
#     y = 0.22977237803753975
#     z = 0.6188389655743234

#     # print(x,y,z)

#     # lat0 = -35.363252  # Reference latitude (San Francisco coordinates)
#     # lon0 = 149.165243  # Reference longitude (San Francisco coordinates)
#     # az = 0.0  # Assuming movement towards the north (adjust as needed)
#     # distance = ((x ** 2) + (y ** 2) + (z ** 2)) ** 0.5  # Euclidean distance

#     # result = Geodesic.WGS84.Direct(lat0, lon0, az, distance)
#     # lat, lon, _, _ = result['lat2'], result['lon2'], result['azi2'], result['azi1']

#     # print("Converted Coordinates:")
#     # print("Latitude: ", lat)
#     # print("Longitude: ", lon)
#     # print("Altitude: ", z)
#     # print(" ")


#     # Convert the position to GPS coordinates
#     latitude = reference_latitude + (y / (scale * 111111.0))  # Approximate conversion based on Earth's circumference
#     longitude = reference_longitude + (x / (scale * 111111.0 * math.cos(math.radians(reference_latitude))))
#     altitude = reference_altitude + z

#     # Create a NavSatFix message
#     # gps_msg = NavSatFix()
#     # gps_msg.latitude = latitude
#     # gps_msg.longitude = longitude
#     # gps_msg.altitude = altitude

#     print("UGV: Latitude={}, Longitude={}, Altitude={}".format(latitude, longitude, altitude))
#     # return gps_msg
#     # send_waypoint(latitude, longitude, altitude)

# # Callback function for the global position
# def globalPositionCallback_drone(data):
#     # Get the latitude, longitude, and altitude values from the NavSatFix message
#     latitude = data.latitude
#     longitude = data.longitude
#     altitude = data.altitude

#     # print("UAV: Latitude={}, Longitude={}, Altitude={}".format(latitude, longitude, altitude))




# # def send_waypoint(lat, lon, alt):
# #     waypoint = Waypoint()
# #     waypoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
# #     waypoint.command = 16  # Waypoint command value for NAV_WAYPOINT
# #     waypoint.is_current = True
# #     waypoint.autocontinue = True
# #     waypoint.param1 = 0  # Hold time in seconds (0 for indefinite)
# #     waypoint.param2 = 0  # Acceptance radius in meters
# #     waypoint.param3 = 0  # Pass radius in meters
# #     waypoint.param4 = 0  # Yaw angle (NaN for default)

# #     waypoint.x_lat = lat
# #     waypoint.y_long = lon
# #     waypoint.z_alt = alt

# #     # Create a waypoint list and add the waypoint
# #     waypoint_list = WaypointList()
# #     waypoint_list.waypoints.append(waypoint)

# #     # Push the waypoint list to ArduPilot using mavros service
# #     rospy.wait_for_service('/mavros/mission/push')
# #     push_service = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
# #     response = push_service(0, waypoint_list.waypoints)

# #     if response.success:
# #         rospy.loginfo("Waypoint sent successfully!")
# #     else:
# #         rospy.logerr("Failed to send waypoint.")


# # rospy.init_node('waypoint_sender')
# rospy.init_node('gps_listener')
# # Subscribe to the model states topic
# rospy.Subscriber('/gazebo/model_states', ModelStates, modelStatesCallback)
# rospy.Subscriber('/mavros/global_position/global', NavSatFix, globalPositionCallback_drone)

# # Spin the ROS node
# rospy.spin()

###########################################################################################################################
#!/usr/bin/env python

import rospy
from mavros_msgs.msg import WaypointList, Waypoint

def publish_waypoint_commands():
    # Initialize the ROS node
    rospy.init_node('waypoint_publisher')

    # Create a publisher for waypoint commands
    waypoint_publisher = rospy.Publisher('/mavros/mission/waypoints', WaypointList, queue_size=10)

    # Create two waypoint commands
    waypoint1 = Waypoint()
    waypoint1.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    waypoint1.command = 16
    waypoint1.is_current = True
    waypoint1.autocontinue = True
    waypoint1.param1 = 0.0
    waypoint1.param2 = 0.0
    waypoint1.param3 = 0.0
    waypoint1.param4 = 0.0
    waypoint1.x_lat = 47.123456789
    waypoint1.y_long = -122.987654321
    waypoint1.z_alt = 10.0

    waypoint2 = Waypoint()
    waypoint2.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    waypoint2.command = 16
    waypoint2.is_current = False
    waypoint2.autocontinue = True
    waypoint2.param1 = 0.0
    waypoint2.param2 = 0.0
    waypoint2.param3 = 0.0
    waypoint2.param4 = 0.0
    waypoint2.x_lat = 47.987654321
    waypoint2.y_long = -122.123456789
    waypoint2.z_alt = 20.0

    # Create a waypoint list and add the commands
    waypoint_list = WaypointList()
    waypoint_list.waypoints = [waypoint1, waypoint2]

    # Publish the waypoint list
    waypoint_publisher.publish(waypoint_list)

    rospy.loginfo("Waypoint commands published")

if __name__ == '__main__':
    try:
        publish_waypoint_commands()
    except rospy.ROSInterruptException:
        pass


rosservice call /mavros/cmd/waypoint "frame: 3 latitude: -35.36327653518298 longitude: 149.16526849458987 altitude: 604.0056607722262 command: 16 autocontinue: true"
rosservice call /mavros/cmd/command "command: 16 param1: 0.0 param2: 0.0 param3: 0.0 param4: 0.0 param5: -35.36327653518298 param6: 149.16526849458987 param7: 604.0056607722262"
rostopic pub /mavros/setpoint_raw/global mavros_msgs/GlobalPositionTarget -1 "{'longitude': -35.36327653518298,'latitude': 149.16526849458987, 'altitude': 604.0056607722262}"

rosservice call /mavros/cmd/command "data: '$(cat ~/catkin_ws/src/iq_sim/waypoint.yaml)'"

rostopic pub /mavros/setpoint_position/global geographic_msgs/GeoPoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'home'
pose:
  position:
    x: -35.36337190326276
    y: 149.16519217811833
    z: 603.9996085324448
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"


#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node('setpoint_position')
    rate = rospy.Rate(10)  # Publish at 10 Hz

    # Create a publisher to send setpoint position commands
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

    # Wait for the connection to be established
    while not rospy.is_shutdown() and local_pos_pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    # Create a PoseStamped message with the desired position
    pose = PoseStamped()
    pose.pose.position.x = 1.0  # Set desired x position
    pose.pose.position.y = 2.0  # Set desired y position
    pose.pose.position.z = 3.0  # Set desired z position

    # Publish the setpoint position commands
    for _ in range(100):  # Publish for 10 seconds (100 * 0.1 sec)
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

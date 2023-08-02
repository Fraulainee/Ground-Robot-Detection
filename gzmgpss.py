import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import NavSatFix
import math

# Constants for the transformation (adjust as needed)
REFERENCE_LATITUDE = -35.363262  # Latitude of the Gazebo reference point
REFERENCE_LONGITUDE = 149.165237  # Longitude of the Gazebo reference point
REFERENCE_ALTITUDE = 584.000000  # Altitude of the Gazebo reference point
X_OFFSET = 180.0 # X-axis offset from the reference point
Y_OFFSET = -20.0  # Y-axis offset from the reference point
# -35.363262 149.165237

def model_states_callback(msg):

    model_index = msg.name.index('robovolc_v14')  # name of your model in Gazebo

    gps_position = NavSatFix()

    gps_position.latitude = REFERENCE_LATITUDE + msg.pose[model_index].position.x / (111111.0 * math.cos(REFERENCE_LATITUDE * math.pi / X_OFFSET)) 

    gps_position.longitude = REFERENCE_LONGITUDE + ((-msg.pose[model_index].position.y) / 111111.0) 

    gps_position.altitude = REFERENCE_ALTITUDE + msg.pose[model_index].position.z

    # Do something with the GPS position
    # print("GPS Position: Latitude={}, Longitude={}, Altitude={}".format(gps_position.latitude, gps_position.longitude, gps_position.altitude))
    print("GPS Position: {} {} {}".format(gps_position.latitude, gps_position.longitude, gps_position.altitude))

def main():
    rospy.init_node('gazebo_gps_listener')

    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)

    rospy.spin()

if __name__ == '__main__':
    main()


#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
"""
string[] name
geometry_msgs/Pose[] pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
geometry_msgs/Twist[] twist
  geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z

"""


class GazeboModel(object):
    def __init__(self, gazebo_models_list):

        # We wait for the topic to be available and when it is then retrieve the index of each model
        # This was separated from callbal to avoid doing this in each callback
        self._robots_models_dict = {}
        self._robots_pose_list = []
        self._robots_index_dict = {}
        self._gazebo_models_list = gazebo_models_list

        self.get_robot_index()

        # We now start the suscriber once we have the indexes of each model
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)

    def get_robot_index(self):

        data = None
        found_all_models_names = False
        while not found_all_models_names and not rospy.is_shutdown():
            rospy.loginfo("Retrieveing Model indexes ")
            try:
                data = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=5)
                # Save it in the format {"robot1":4,"robot2":2}
                if data:
                    # Here we have model_states data, but not guarantee that the models we want are there
                    not_found_models_list = []
                    for robot_name in self._gazebo_models_list:
                        robot_name_found = self.update_robot_index(data, robot_name)
                        if not robot_name_found:
                            not_found_models_list.append(robot_name)
                            break
                    found_all_models_names = len(self._robots_index_dict) == len(self._gazebo_models_list)
                else:
                    rospy.loginfo("Topic /gazebo/model_states NOT Ready yet, trying again ")

            except Exception as e:
                s = str(e)
                rospy.loginfo("Error in get_robot_index = " + s)

        assert found_all_models_names, "Models Missing=="+str(not_found_models_list)
        rospy.loginfo("Final robots_index_dict =  %s ", str(self._robots_index_dict))

    def update_robot_index(self, data, robot_name):
        try:
            index = data.name.index(robot_name)
            self._robots_index_dict[robot_name] = index
            found = True
        except ValueError:
            rospy.loginfo("Robot Name=" + str(robot_name) + ", is NOT in model_state, trying again")
            found = False

        return found

    def callback(self, data):

        for robot_name in self._gazebo_models_list:
            # Retrieve the corresponding index
            robot_name_found = self.update_robot_index(data, robot_name)
            if robot_name_found:
                data_index = self._robots_index_dict[robot_name]
                # Get the pose data from theat index
                try:
                    data_pose = data.pose[data_index]
                except IndexError:
                    rospy.logwarn("The model with data index " + str(data_index) + ", something went wrong.")
                    data_pose = None
            else:
                data_pose = None
            # Save the pose inside the dict {"robot1":pose1,"robot2":pose2}
            self._robots_models_dict[robot_name] = data_pose

    def get_model_pose(self, robot_name):

        pose_now = None

        try:
            pose_now = self._robots_models_dict[robot_name]
        except Exception as e:
            s = str(e)
            rospy.loginfo("Error, The _robots_models_dict is not ready = " + s)

        return pose_now

    def get_model_position(self, robot_name):
        pose_now = self.get_model_pose(robot_name)
        if pose_now is not None:
            x_position = pose_now.position.x
            y_position = pose_now.position.y
            z_position = pose_now.position.z
            return x_position, -y_position, z_position
        else:
            return None

    def get_model_x_position(self, robot_name):
        pose_now = self.get_model_pose(robot_name)
        if pose_now is not None:
            x_position = pose_now.position.x
            return x_position
        else:
            return None
    
    def get_model_y_position(self, robot_name):
        pose_now = self.get_model_pose(robot_name)
        if pose_now is not None:
            y_position = pose_now.position.y
            return -y_position
        else:
            return None
    def get_model_z_position(self, robot_name):
        pose_now = self.get_model_pose(robot_name)
        if pose_now is not None:
            z_position = pose_now.position.z
            return z_position
        else:
            return None

    

def robot_position():
    rospy.init_node('listener', anonymous=True)
    gazebo_models_list = ['robovolc_v14', 'iris_demo']
    gz_model = GazeboModel(gazebo_models_list)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        for robot_name in gazebo_models_list:
            model_position = gz_model.get_model_position(robot_name)
            model_position_x = gz_model.get_model_x_position(robot_name)
            model_position_y = gz_model.get_model_y_position(robot_name)
            model_position_z = gz_model.get_model_z_position(robot_name)
            if model_position is not None:
                print(f"x Position of {robot_name}: {model_position_x}")
                print(f"y Position of {robot_name}: {model_position_y}")
                print(f"z Position of {robot_name}: {model_position_z}")
               
                R = 6371 
                lat = np.degrees(np.arcsin(model_position_z/R))
                lon = np.degrees(np.arctan2(model_position_y, model_position_x))
                # print( lon, lat)
            else:
                print(f"Failed to retrieve the position of {robot_name}")


def move_to():
    rospy.init_node('listener', anonymous=True)
    gazebo_models_list = ['robovolc_v14', 'iris_demo']
    gz_model = GazeboModel(gazebo_models_list)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        for robot_name in gazebo_models_list:
            model_position = gz_model.get_model_position(robot_name)
            ugv_position_x = gz_model.get_model_x_position('robovolc_v14')
            ugv_position_y = gz_model.get_model_y_position('robovolc_v14')
            drone_position_x = gz_model.get_model_x_position('iris_demo')
            drone_position_y = gz_model.get_model_y_position('iris_demo')

            if model_position is not None:
                print(f"Moving to x: {ugv_position_x - drone_position_x}, y: {ugv_position_y - drone_position_y}")    
            else:
                None
 

def move_to_location():
    rospy.init_node('move_drone_node')

    # Create a publisher to send the desired position
    local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Create a PoseStamped message with the desired position
    desired_position = PoseStamped()
    desired_position.pose.position.x = 2.0  # Update with desired x-coordinate
    desired_position.pose.position.y = 3.0  # Update with desired y-coordinate
    desired_position.pose.position.z = 1.0  # Update with desired z-coordinate

    # Keep publishing the desired position until ROS is shutdown
    while not rospy.is_shutdown():
        desired_position.header.stamp = rospy.Time.now()
        local_position_pub.publish(desired_position)
        rate.sleep()

if __name__ == '__main__':
    
    robot_position()
    # move_to()
    # try:
    #     move_to_location()
    # except rospy.ROSInterruptException:
    #     pass
 
import moveit_commander
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint,JointConstraint
import geometry_msgs.msg
import copy
from aubo_msgs.srv import SetIO
import rospy
import tf


rospy.init_node('hand2eye_calibration_node')

listener = tf.TransformListener()

group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)
group.set_max_velocity_scaling_factor(0.1)

print "\nwaiting for transform\n"

listener.waitForTransform("world", "object_23", rospy.Time(0), rospy.Duration(4.0))

(robot_trans,robot_rot)= listener.lookupTransform("world", "object_23", rospy.Time(0))

waypoints = []

pose_goal=geometry_msgs.msg.Pose()

pose_goal.position.x= robot_trans[0]
pose_goal.position.y= robot_trans[1]
pose_goal.position.z= robot_trans[2]+.1

pose_goal.orientation.x= robot_rot[0]
pose_goal.orientation.y= robot_rot[2]
pose_goal.orientation.z= robot_rot[1]
pose_goal.orientation.w= robot_rot[3]

waypoints.append(copy.deepcopy(pose_goal))

pose_goal.position.x= robot_trans[0]
pose_goal.position.y= robot_trans[1]
pose_goal.position.z= robot_trans[2]

pose_goal.orientation.x= robot_rot[0]
pose_goal.orientation.y= robot_rot[2]
pose_goal.orientation.z= robot_rot[1]
pose_goal.orientation.w= robot_rot[3]

waypoints.append(copy.deepcopy(pose_goal))

(plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.05,        # eef_step
                                   0.0) 


gripper = rospy.ServiceProxy('/aubo_driver/set_io', SetIO)

print fraction

if fraction==1:
	print "\nPlanning Success. excecuting the plan"
	group.execute(plan, wait=True)
	gripper(1,8,0)

else :
	print "\nOhhh Nooo !!! Planning Failed"
	
waypoints = []

pose_goal=geometry_msgs.msg.Pose()


pose_goal.position.x= robot_trans[0]
pose_goal.position.y= robot_trans[1]
pose_goal.position.z= robot_trans[2]+.1

pose_goal.orientation.x= robot_rot[0]
pose_goal.orientation.y= robot_rot[2]
pose_goal.orientation.z= robot_rot[1]
pose_goal.orientation.w= robot_rot[3]

waypoints.append(copy.deepcopy(pose_goal))

pose_goal.position.x= robot_trans[0]-.1
pose_goal.position.y= robot_trans[1]
pose_goal.position.z= robot_trans[2]

pose_goal.orientation.x= robot_rot[0]
pose_goal.orientation.y= robot_rot[2]
pose_goal.orientation.z= robot_rot[1]
pose_goal.orientation.w= robot_rot[3]

waypoints.append(copy.deepcopy(pose_goal))


(plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.05,        # eef_step
                                   0.0) 


gripper = rospy.ServiceProxy('/aubo_driver/set_io', SetIO)

print fraction

if fraction>=.8:
	print "\nPlanning Success. excecuting the plan"
	group.execute(plan, wait=True)
	gripper(1,8,1)

else :
	print "\nOhhh Nooo !!! Planning Failed"
#gripper(1,8,0)


import moveit_commander
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint,JointConstraint
import geometry_msgs.msg
import copy
from aubo_msgs.srv import SetIO
import rospy

group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)
group.set_max_velocity_scaling_factor(0.1)


xyz1= [0.322, -0.433, 0.829]

xyzw1= [0.961, -0.274, 0.043, 0.006]

xyz=  [0.316, -0.442, 0.733]

xyzw= [0.964, -0.258, 0.050, -0.034]

waypoints = []

pose_goal=geometry_msgs.msg.Pose()

pose_goal.position.x= xyz1[0]
pose_goal.position.y= xyz1[1]
pose_goal.position.z= xyz1[2]

pose_goal.orientation.x= xyzw1[0]
pose_goal.orientation.y= xyzw1[1]
pose_goal.orientation.z= xyzw1[2]
pose_goal.orientation.w= xyzw1[3]

waypoints.append(copy.deepcopy(pose_goal))

pose_goal.position.x= xyz[0]
pose_goal.position.y= xyz[1]
pose_goal.position.z= xyz[2]

pose_goal.orientation.x= xyzw[0]
pose_goal.orientation.y= xyzw[1]
pose_goal.orientation.z= xyzw[2]
pose_goal.orientation.w= xyzw[3]

waypoints.append(copy.deepcopy(pose_goal))

(plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.05,        # eef_step
                                   0.0) 


gripper = rospy.ServiceProxy('/aubo_driver/set_io', SetIO)

print fraction

if fraction==1:
	print "\nPlanning Success. excecuting the plan\n"
	group.execute(plan, wait=True)
	gripper(1,8,1)

else :
	print "\nOhhh Nooo !!! Planning Failed\n"
	

xyz1= [0.394, -0.419, 0.916]

xyzw1= [0.972, -0.233, 0.033, 0.028]

xyz=  [0.395, -0.425, 0.75]

xyzw= [0.969, -0.232, 0.079, -0.028]

waypoints = []

pose_goal=geometry_msgs.msg.Pose()


pose_goal.position.x= xyz1[0]
pose_goal.position.y= xyz1[1]
pose_goal.position.z= xyz1[2]

pose_goal.orientation.x= xyzw1[0]
pose_goal.orientation.y= xyzw1[1]
pose_goal.orientation.z= xyzw1[2]
pose_goal.orientation.w= xyzw1[3]

waypoints.append(copy.deepcopy(pose_goal))

pose_goal.position.x= xyz[0]
pose_goal.position.y= xyz[1]
pose_goal.position.z= xyz[2]

pose_goal.orientation.x= xyzw[0]
pose_goal.orientation.y= xyzw[1]
pose_goal.orientation.z= xyzw[2]
pose_goal.orientation.w= xyzw[3]

waypoints.append(copy.deepcopy(pose_goal))

(plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.05,        # eef_step
                                   0.0) 


gripper = rospy.ServiceProxy('/aubo_driver/set_io', SetIO)

print fraction

if fraction>=.8:
	print "\nPlanning Success. excecuting the plan\n"
	group.execute(plan, wait=True)
	gripper(1,8,0)

else :
	print "\nOhhh Nooo !!! Planning Failed\n"
#gripper(1,8,0)



xyz=  [0.395, -0.425, 0.75]

xyzw= [0.969, -0.232, 0.079, -0.028]

xyz2=  [0.395, -0.425, 0.80]

xyzw2= [0.969, -0.232, 0.079, -0.028]

waypoints = []

pose_goal=geometry_msgs.msg.Pose()

pose_goal.position.x= xyz[0]
pose_goal.position.y= xyz[1]
pose_goal.position.z= xyz[2]

pose_goal.orientation.x= xyzw[0]
pose_goal.orientation.y= xyzw[1]
pose_goal.orientation.z= xyzw[2]
pose_goal.orientation.w= xyzw[3]



waypoints.append(copy.deepcopy(pose_goal))


pose_goal.position.x= xyz2[0]
pose_goal.position.y= xyz2[1]
pose_goal.position.z= xyz2[2]

pose_goal.orientation.x= xyzw2[0]
pose_goal.orientation.y= xyzw2[1]
pose_goal.orientation.z= xyzw2[2]
pose_goal.orientation.w= xyzw2[3]



waypoints.append(copy.deepcopy(pose_goal))



(plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.05,        # eef_step
                                   0.0) 


gripper = rospy.ServiceProxy('/aubo_driver/set_io', SetIO)

print fraction

if fraction>=.8:
	print "\nPlanning Success. excecuting the plan\n"
	group.execute(plan, wait=True)

else :
	print "\nOhhh Nooo !!! Planning Failed\n"
#gripper(1,8,0)


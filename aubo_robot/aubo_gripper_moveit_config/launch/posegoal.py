import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list
from aubo_msgs.srv import SetIO
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint,JointConstraint

rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)
# We can get the joint values from the group and adjust some of the values:
pose_goal=geometry_msgs.msg.Pose()
#Rotation

group.set_max_velocity_scaling_factor(0.1)


#group.set_planner_id("ESTkConfigDefault")

upright_constraints = Constraints()
upright_constraints.name = "upright"

"""
joint_constraint=JointConstraint()
joint_constraint.joint_name='upperArm_joint'
joint_constraint.position=-1.57
joint_constraint.tolerance_above=1.57
joint_constraint.tolerance_below=1.57
joint_constraint.weight=.5
upright_constraints.joint_constraints.append(joint_constraint)


joint_constraint=JointConstraint()
joint_constraint.joint_name='foreArm_joint'
joint_constraint.position=1.57
joint_constraint.tolerance_above=1.57
joint_constraint.tolerance_below=1.57
joint_constraint.weight=.5

upright_constraints.joint_constraints.append(joint_constraint)

joint_constraint=JointConstraint()
joint_constraint.joint_name='shoulder_joint'
joint_constraint.position=1.57
joint_constraint.tolerance_above=1	
joint_constraint.tolerance_below=1.57
joint_constraint.weight=.7

upright_constraints.joint_constraints.append(joint_constraint)

group.set_path_constraints(upright_constraints)
"""
xyz1=[0.325, -0.458, 0.748]

xyzw1= [-0.699, 0.708, -0.094, -0.033]

xyz=[0.321, -0.442, 0.831]

xyzw= [-0.607, 0.781, -0.048, -0.140]



pose_goal.position.x= xyz1[0]
pose_goal.position.y= xyz1[1]
pose_goal.position.z= xyz1[2]

pose_goal.orientation.x= xyzw1[0]
pose_goal.orientation.y= xyzw1[1]
pose_goal.orientation.z= xyzw1[2]
pose_goal.orientation.w= xyzw1[3]


group.set_pose_target(pose_goal)
plan=group.go(wait=True)

gripper = rospy.ServiceProxy('/aubo_driver/set_io', SetIO)

gripper(1,8,1)





pose_goal.position.x= xyz[0]
pose_goal.position.y= xyz[1]
pose_goal.position.z= xyz[2]

pose_goal.orientation.x= xyzw[0]
pose_goal.orientation.y= xyzw[1]
pose_goal.orientation.z= xyzw[2]
pose_goal.orientation.w= xyzw[3]

group.set_pose_target(pose_goal)
plan=group.go(wait=True)


pose_goal.position.x= xyz1[0]
pose_goal.position.y= xyz1[1]
pose_goal.position.z= xyz1[2]

pose_goal.orientation.x= xyzw1[0]
pose_goal.orientation.y= xyzw1[1]
pose_goal.orientation.z= xyzw1[2]
pose_goal.orientation.w= xyzw1[3]
group.set_pose_target(pose_goal)

plan=group.go(wait=True)



gripper = rospy.ServiceProxy('/aubo_driver/set_io', SetIO)

gripper(1,8,0)

pose_goal.position.x= xyz[0]
pose_goal.position.y= xyz[1]
pose_goal.position.z= xyz[2]

pose_goal.orientation.x= xyzw[0]
pose_goal.orientation.y= xyzw[1]
pose_goal.orientation.z= xyzw[2]
pose_goal.orientation.w= xyzw[3]

group.set_pose_target(pose_goal)
plan=group.go(wait=True)

group.clear_pose_targets

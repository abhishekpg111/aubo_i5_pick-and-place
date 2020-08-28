import rospy
import tf2_ros
import tf
import geometry_msgs.msg
from fiducial_msgs.msg import FiducialTransformArray
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np


from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera, reset

rospy.init_node('cali')

world_effector_pub = rospy.Publisher('/moveItController_cmd', JointTrajectoryPoint, queue_size=1)

tfBuffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tfBuffer)

cali_broad = tf2_ros.TransformBroadcaster()

rate = rospy.Rate(10) 
rate.sleep()

cal1= JointTrajectoryPoint()
cal1.time_from_start=rospy.Duration(0.0)

#world_effector_pub.publish(cal1)

cal= JointTrajectoryPoint()
cal.velocities =  [1.0735605072416647, 0.0, 0.0,0.0, 5.153091454187644e-16, 1.0618662495956706]
cal.time_from_start=rospy.Duration(0.0)

rate = rospy.Rate(10) 
rate.sleep()

world_effector_pub.publish(cal)

rate = rospy.Rate(10) 
rate.sleep()

cal1= JointTrajectoryPoint()
cal1.time_from_start=rospy.Duration(0.0)

world_effector_pub.publish(cal1)


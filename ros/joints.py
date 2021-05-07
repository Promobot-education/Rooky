"""
joint_control_sim_test.py

Пример управления манипулятором Rooky в ROS (режим симуляции).

"""

import rospy
from std_msgs.msg import *

from trajectory_msgs.msg import *
from control_msgs.msg import *


class ControlJoints():


	def __init__(self,arm_type):

		self._positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

		self._rate = rospy.get_param('~rate', 20)

		self._joint_names = [str(arm_type) + '_arm_1_joint',str(arm_type) + '_arm_2_joint',str(arm_type) + '_arm_3_joint',str(arm_type) + '_arm_4_joint',str(arm_type) + '_arm_5_joint',str(arm_type) + '_arm_6_joint',str(arm_type) + '_arm_7_joint']

		self._cmd_pub = rospy.Publisher(str(arm_type) + '_arm_controller/command',
										JointTrajectory,
										queue_size=1)

		rospy.sleep(1)

		interval_to_point = 1
		self._positions =  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.move_all_joints(interval_to_point)

		rospy.sleep(2.0)



	def spinOnce(self):
		interval_to_point = 2
		self._positions =  [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
		self.move_all_joints(interval_to_point)
		rospy.sleep(5)


		self._positions =  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.move_all_joints(interval_to_point)
		rospy.sleep(5)


		self.move_joint(str(arm_type) + '_arm_1_joint',1.0,3)
		rospy.sleep(5)



	def rate(self):
		return self._rate
		

	#Функция отправки в желаемую позицию всех суставов Rooky	
	def move_all_joints(self, interval_to_point=0.1):
		traj = JointTrajectory()
		traj.joint_names = self._joint_names
		point = JointTrajectoryPoint()

		point.positions = self._positions

		point.time_from_start = rospy.Duration(interval_to_point)
		traj.points.append(point)

		self._cmd_pub.publish(traj)


	def move_joint(self, joint_name, position, interval_to_point=0.1):
		traj = JointTrajectory()
		traj.joint_names.append(joint_name)
		point = JointTrajectoryPoint()

		point.positions.append(position)

		point.time_from_start = rospy.Duration(interval_to_point)
		traj.points.append(point)

		self._cmd_pub.publish(traj)



rospy.init_node('joint_control_sim_test')

node = ControlJoints("left")
while not rospy.is_shutdown():
	node.spinOnce()
	rospy.Rate(node.rate()).sleep()

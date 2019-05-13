#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8
import math
from tf.transformations import euler_from_quaternion

class Controller_pos(object):
	def __init__(self):
		print "init"
		self.pub = rospy.Publisher('rover_command', Int8, queue_size=100)
		self.subrover = rospy.Subscriber("sequitur_pose", PoseStamped, self.rover_callback)
		self.subdrone = rospy.Subscriber("rover_destination", PoseStamped, self.destination_callback)

		self.pose_actuelle = PoseStamped()
		self.pose_objectif = PoseStamped()

	def rover_callback(self, data):
		self.pose_actuelle.pose = data.pose
		distance_x = self.pose_objectif.pose.position.x - self.pose_actuelle.pose.position.x
		distance_y = self.pose_objectif.pose.position.y - self.pose_actuelle.pose.position.y
		norme = math.sqrt(distance_x * distance_x + distance_y * distance_y)
		#print norme_vecteur, distance_x, distance_y
		teta =  math.atan2(distance_x, distance_y) - math.pi/2
		if teta < 0:
			teta=teta+2*math.pi

                orientation_x=data.pose.orientation.x
                orientation_y=data.pose.orientation.y
                orientation_z=data.pose.orientation.z
                orientation_w=data.pose.orientation.w

                angles=euler_from_quaternion([orientation_w,orientation_x,orientation_y,orientation_z]);

                #Transpose camera angles to get roll, pitch, yaw
                pitch=-angles[1]
                roll=-angles[2]
                yaw=angles[0] + math.pi	#To force yaw zero forward
		print teta, yaw
		if yaw>teta +  0.15:
			self.pub.publish(3)
		elif yaw<teta - 0.15:
			self.pub.publish(4)
		else:
			if norme>0.1:
				self.pub.publish(1)
			else:
				self.pub.publish(0)

		#self.pub.publish(3)

	def destination_callback(self, data):
		self.pose_objectif.pose = data.pose

if __name__ == '__main__':
    try:
        rospy.init_node('pos_control', anonymous=True)
	controller = Controller_pos()
	rospy.spin()
    except rospy.ROSInterruptException:
        pass

#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import sys

def movebase_client(namespace):

	rospy.init_node('vel_publisher')
	pub = rospy.Publisher("/"+namespace+"/cmd_vel",Twist,queue_size=10)
	move = Twist()
	rate = rospy.Rate(1)
	print(namespace +" enters to while to node:"+"/"+namespace+"/cmd_vel")
	while not rospy.is_shutdown():
		move.linear.x=1
		move.angular.z=1
		pub.publish(move)
		rate.sleep()

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        result = movebase_client(sys.argv[1])
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped

def poser():
    pub = rospy.Publisher('mavros/local_position/pose', PoseStamped, queue_size=10)
    rospy.init_node('pozyx', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pose = PoseStamped()
        print(pose)
        rospy.loginfo(pose)
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        poser()
    except rospy.ROSInterruptException:
        pass

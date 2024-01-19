import rospy
from bumperbot_examples.bumperbot_odom import AmrOdom

if __name__ == '__main__':
    rospy.init_node('amr_odom_node')
    amr_odom = AmrOdom()
    rospy.spin()
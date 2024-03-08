import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.init_node("simple_node", anonymous=True)
    pub = rospy.Publisher("chatter", String, queue_size=10)
    r = rospy.Rate(10)
    counter = 0
    while not rospy.is_shutdown():
        hello_msg = "Hello world from python %d" % counter 
        pub.publish(hello_msg)
        r.sleep()
        counter += 1
    
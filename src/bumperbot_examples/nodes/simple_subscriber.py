import rospy 
from std_msgs.msg import String

def msgCallback(msg):
    rospy.loginfo("New mssage received %s", msg.data)

if __name__ == "__main__":
    rospy.init_node("simple_subscriber_node", anonymous=True)
    rospy.Subscriber("chatter", String, msgCallback)
    
    rospy.spin()
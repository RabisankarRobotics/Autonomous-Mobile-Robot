import rospy
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class TfExamples(object):

    def __init__(self):

        # TF Broadcaster
        self.static_broadcaster_ = StaticTransformBroadcaster()
        self.static_transform_stamped_ = TransformStamped()
        
        # TF Listener
        self.static_transform_stamped_.header.stamp = rospy.Time.now()
        self.static_transform_stamped_.header.frame_id = "bumperbot_base"
        self.static_transform_stamped_.child_frame_id = "bumperbot_top"
        self.static_transform_stamped_.transform.translation.x = 0.0
        self.static_transform_stamped_.transform.translation.y = 0.0
        self.static_transform_stamped_.transform.translation.z = 0.3
        self.static_transform_stamped_.transform.rotation.x = 0
        self.static_transform_stamped_.transform.rotation.y = 0
        self.static_transform_stamped_.transform.rotation.z = 0
        self.static_transform_stamped_.transform.rotation.w = 1

        self.static_broadcaster_.sendTransform(self.static_transform_stamped_)
        rospy.loginfo("Publishing static transform between %s and %s", 
                      self.static_transform_stamped_.header.frame_id,
                      self.static_transform_stamped_.child_frame_id)

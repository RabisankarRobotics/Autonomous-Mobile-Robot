#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


class OdomCalculator():
    def __init__(self):
        rospy.init_node('ekf_odom_pub', anonymous=True)

        # Create odometry data publishers
        self.odom_data_pub = rospy.Publisher('odom_data_euler', Odometry, queue_size=100)
        self.odom_data_pub_quat = rospy.Publisher('odom', Odometry, queue_size=100)

        # Initialize odometry messages
        self.odomNew = Odometry()
        self.odomOld = Odometry()


        # Initial pose
        self.initialX = 0.0
        self.initialY = 0.0
        self.initialTheta = 0.00000000001
        self.PI = 3.141592


        # Robot physical constants
        self.TICKS_PER_REVOLUTION = 8123  # For reference purposes.
        self.WHEEL_RADIUS = 0.0485  # Wheel radius in meters
        self.WHEEL_BASE = 0.43  # Center of left tire to center of right tire
        self.TICKS_PER_METER = 26600  # Original was 2800

        

        # self.odomOld.header.stamp = rospy.Time()
        self.odomOld.pose.pose.position.x = self.initialX
        self.odomOld.pose.pose.position.y = self.initialY
        self.odomOld.pose.pose.orientation.z = self.initialTheta

        # self.odomNew.header.frame_id = "odom"
        self.odomNew.pose.pose.position.x = 0.0
        self.odomNew.pose.pose.position.y = 0.0
        self.odomNew.pose.pose.orientation.z = 0.0
        self.odomNew.twist.twist.linear.x = 0.0
        self.odomNew.twist.twist.linear.y = 0.0 
        self.odomNew.twist.twist.linear.z = 0.0
        self.odomNew.twist.twist.angular.x = 0.0
        self.odomNew.twist.twist.angular.y = 0.0
        self.odomNew.twist.twist.angular.z = 0.0

        self.br_ = TransformBroadcaster()
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_link"

        # Flag to see if initial pose has been received
        # self.initialPoseRecieved = False

        # Subscribe to ROS topics
        rospy.Subscriber('front_right_ticks', Int16, self.Calc_front_right, queue_size=100)
        rospy.Subscriber('front_left_ticks', Int16, self.Calc_front_left, queue_size=100)
        rospy.Subscriber('back_right_ticks', Int16, self.Calc_back_right, queue_size=100)
        rospy.Subscriber('back_left_ticks', Int16, self.Calc_back_left, queue_size=100)
        # rospy.Subscriber('initial_2d', PoseStamped, self.set_initial_2d, queue_size=1)

        # Distance both wheels have traveled
        self.distanceRight_front = 0
        self.distanceRight_back = 0
        self.distanceLeft_front = 0
        self.distanceLeft_back = 0
        
        self.lastCountR_front = 0
        self.lastCountR_back = 0
        self.lastCountL_front = 0
        self.lastCountL_back = 0

    def Calc_front_right(self,rightCount):

        if rightCount.data!=0 and self.lastCountR_front != 0:
            rightTicks = (rightCount.data - self.lastCountR_front)
            # print(rightTicks)

            if rightTicks > 10000:
                rightTicks = 0 - (65535 - rightTicks)
            elif rightTicks < -10000:
                rightTicks = 65535 + rightTicks
            else:
                pass
            self.distanceRight_front = rightTicks/self.TICKS_PER_METER
        self.lastCountR_front = rightCount.data

    def Calc_back_right(self,rightCount):
        
        if rightCount.data!=0 and self.lastCountR_back != 0:
            rightTicks = (rightCount.data - self.lastCountR_back)

            if rightTicks > 10000:
                rightTicks = 0 - (65535 - rightTicks)
            elif rightTicks < -10000:
                rightTicks = 65535 + rightTicks
            else:
                pass
            self.distanceRight_back = rightTicks/self.TICKS_PER_METER
        self.lastCountR_back = rightCount.data
        

    def Calc_front_left(self, leftCount):
        
        if leftCount.data!=0 and self.lastCountL_front != 0:
    
            leftTicks = (leftCount.data - self.lastCountL_front)
            print(leftTicks)

            if leftTicks > 10000:
                leftTicks = 0 - (65535 - leftTicks)
            elif leftTicks < -10000:
                leftTicks = 65535 + leftTicks
            else:
                pass
            self.distanceLeft_front = leftTicks/self.TICKS_PER_METER
        self.lastCountL_front = leftCount.data


    def Calc_back_left(self,leftCount):
        
        if leftCount.data!=0 and self.lastCountL_back != 0:
            leftTicks = (leftCount.data - self.lastCountL_back)
            # print(rightTicks)

            if leftTicks > 10000:
                leftTicks = 0 - (65535 - leftTicks)
            elif leftTicks < -10000:
                leftTicks = 65535 + leftTicks
            else:
                pass
            self.distanceLeft_back = leftTicks/self.TICKS_PER_METER
        self.lastCountL_back = leftCount.data
        
        

    def publish_quat(self):
        q = quaternion_from_euler(0, 0, self.odomNew.pose.pose.orientation.z)

        quatOdom = Odometry()
        quatOdom.header.stamp = self.odomNew.header.stamp
        quatOdom.header.frame_id = "odom"
        quatOdom.child_frame_id = "base_link"
        quatOdom.pose.pose.position.x = self.odomNew.pose.pose.position.x
        quatOdom.pose.pose.position.y = self.odomNew.pose.pose.position.y
        quatOdom.pose.pose.orientation.x = q[0]
        quatOdom.pose.pose.orientation.y = q[1]
        quatOdom.pose.pose.orientation.z = q[2]
        quatOdom.pose.pose.orientation.w = q[3]
        quatOdom.twist.twist.linear.x = self.odomNew.twist.twist.linear.x
        quatOdom.twist.twist.linear.y = self.odomNew.twist.twist.linear.y
        quatOdom.twist.twist.linear.z = self.odomNew.twist.twist.linear.z
        quatOdom.twist.twist.angular.x = self.odomNew.twist.twist.angular.x
        quatOdom.twist.twist.angular.y = self.odomNew.twist.twist.angular.y
        quatOdom.twist.twist.angular.z = self.odomNew.twist.twist.angular.z

        #tf
        self.transform_stamped_.transform.translation.x = self.odomNew.pose.pose.position.x
        self.transform_stamped_.transform.translation.y = self.odomNew.pose.pose.position.y

        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.odomNew.header.stamp
        self.br_.sendTransform(self.transform_stamped_)



        for i in range(36):
            if i == 0 or i == 7 or i == 14:
                quatOdom.pose.covariance[i] = 0.01
            elif i == 21 or i == 28 or i == 35:
                quatOdom.pose.covariance[i] += 0.1
            else:
                quatOdom.pose.covariance[i] = 0

        # print(quatOdom)

        self.odom_data_pub_quat.publish(quatOdom)

    def update_odom(self):
        print("All distance: ",[self.distanceLeft_front,self.distanceRight_front,self.distanceLeft_back,self.distanceRight_back])
        cycleDistance = (self.distanceRight_front + self.distanceLeft_front + self.distanceRight_back + self.distanceLeft_back) / 4
        cycleAngle = math.asin((self.distanceRight_front + self.distanceRight_back  - self.distanceLeft_front - self.distanceLeft_back) /2 * self.WHEEL_BASE)
        avgAngle = cycleAngle / 2 + self.odomOld.pose.pose.orientation.z

        if avgAngle > self.PI:
            avgAngle -= 2 * self.PI
        elif avgAngle < -self.PI:
            avgAngle += 2 * self.PI
        else:
            pass

        self.odomNew.pose.pose.position.x = self.odomOld.pose.pose.position.x + math.cos(avgAngle) * cycleDistance
        self.odomNew.pose.pose.position.y = self.odomOld.pose.pose.position.y + math.sin(avgAngle) * cycleDistance
        self.odomNew.pose.pose.orientation.z = cycleAngle + self.odomOld.pose.pose.orientation.z

        if math.isnan(self.odomNew.pose.pose.position.x) or math.isnan(self.odomNew.pose.pose.position.y) or math.isnan(
                self.odomNew.pose.pose.position.z):
            self.odomNew.pose.pose.position.x = self.odomOld.pose.pose.position.x
            self.odomNew.pose.pose.position.y = self.odomOld.pose.pose.position.y
            self.odomNew.pose.pose.orientation.z = self.odomOld.pose.pose.orientation.z

        if self.odomNew.pose.pose.orientation.z > self.PI:
            self.odomNew.pose.pose.orientation.z -= 2 * self.PI
        elif self.odomNew.pose.pose.orientation.z < -self.PI:
            self.odomNew.pose.pose.orientation.z += 2 * self.PI
        else:
            pass

        self.odomNew.header.stamp = rospy.Time.now()
        self.odomNew.twist.twist.linear.x = cycleDistance / (
                self.odomNew.header.stamp.to_sec() - self.odomOld.header.stamp.to_sec())
        self.odomNew.twist.twist.angular.z = cycleAngle / (
                self.odomNew.header.stamp.to_sec() - self.odomOld.header.stamp.to_sec())

        self.odomOld.pose.pose.position.x = self.odomNew.pose.pose.position.x
        self.odomOld.pose.pose.position.y = self.odomNew.pose.pose.position.y
        self.odomOld.pose.pose.orientation.z = self.odomNew.pose.pose.orientation.z
        self.odomOld.header.stamp = self.odomNew.header.stamp

        # self.publish_quat()

        self.odom_data_pub.publish(self.odomNew)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.update_odom()
            self.publish_quat()
            rate.sleep()


if __name__ == '__main__':
    try:
        odom_calculator = OdomCalculator()
        odom_calculator.run()
    except rospy.ROSInterruptException:
        pass



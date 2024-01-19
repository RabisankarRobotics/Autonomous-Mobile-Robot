#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_about_axis


IMU_FRAME = None
imu_pub = None 
received_imu_data = None

def imu_data_callback(msg):
    # Callback function to handle received IMU data
    global received_imu_data
    received_imu_data = msg.data
    rospy.loginfo("IMU data received.")
    
    # rospy.loginfo("Received IMU Data: {}".format(received_imu_data))

# def imu_data_subscriber():
#     # Initialize the ROS node
#     rospy.init_node('imu_data_subscriber', anonymous=True)

#     # Define the topic to subscribe to
#     topic_name = 'imu_data'

#     # Subscribe to the topic using the callback function
#     rospy.Subscriber(topic_name, Float32MultiArray, imu_data_callback)

#     # Spin to keep the script from exiting
#     rospy.spin()


def publish_imu(timer_event):

    global received_imu_data

    # Check if received_imu_data is not None
    if received_imu_data is not None:
        imu_msg = Imu()
        imu_msg.header.frame_id = IMU_FRAME
        
        gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z = received_imu_data
        print("gyro_z is.............................", gyro_z)
        print("accel_x is.............................", accel_x)
        print("accel_y is.............................", accel_y)
        
        # Load up the IMU message
        # multipling 9.8 with linear acceleration getting force respect to gravity.
        imu_msg.linear_acceleration.x = accel_x * 9.8 
        imu_msg.linear_acceleration.y = accel_y * 9.8
        imu_msg.linear_acceleration.z = accel_z * 9.8

        # convert degree to radian by multipling 0.0174
        imu_msg.angular_velocity.x = gyro_x * 0.0174
        imu_msg.angular_velocity.y = gyro_y * 0.0174
        imu_msg.angular_velocity.z = gyro_z * 0.0174

        imu_msg.header.stamp = rospy.Time.now()

        imu_pub.publish(imu_msg)
    else:
        rospy.logwarn("No IMU data received yet.")



if __name__ == '__main__':

    rospy.init_node('imu_node')

    rospy.Subscriber('imu_data', Float32MultiArray, imu_data_callback)

    IMU_FRAME = rospy.get_param('~imu_frame', 'imu_link')

    imu_pub = rospy.Publisher('imu/data_raw', Imu)
    imu_timer = rospy.Timer(rospy.Duration(0.02), publish_imu)
    
    rospy.spin()

#include "amr_config/amr_odom.hpp"
#include <std_msgs/Float32MultiArray.h>

// Forward Kinematics (IK) control
amr_odom::amr_odom()
: NodeHandle("amr_odom")
{
    // Initialize your class, if needed
    odom_pub_ = NodeHandle.advertise<nav_msgs::Odometry>("odom", 10);

    // rpm_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    //                 "motor_rpm", rclcpp::SystemDefaultsQoS(),
    //                 std::bind(&amr_odom::rpmCallback, this, std::placeholders::_1));
}

void amr_odom::rpmCallback(const std_msgs::Float32MultiArray& msg)
{
    // Extract RPM values for left and right motors
    double rpm_right = msg.data[0];
    double rpm_left = msg.data[1];

    // Call the function to update velocities and integrate odometry
    ros::Time now = ros::Time::now();
    getVelocities_DiffDrive(rpm_left, rpm_right, now);

    // Update and publish odometry
    odom_update();
}

void amr_odom::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

int amr_odom::odom_update()
{
    ros::Time now = ros::Time::now();
    nav_msgs::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = get_x();
    odom.pose.pose.position.y = get_y();
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, get_heading());

    odom.pose.pose.orientation.x = quaternion[0];
    odom.pose.pose.orientation.y = quaternion[1];
    odom.pose.pose.orientation.z = quaternion[2];
    odom.pose.pose.orientation.w = quaternion[3];

    odom.twist.twist.linear.x = get_linear_x();
    odom.twist.twist.linear.y = get_linear_y();
    odom.twist.twist.angular.z = get_angular_z();

    // odom_pub_->publish(odom);
    ros::Publisher odom_pub_


    return 0;
}




void amr_odom::getVelocities_DiffDrive(double rpm_l, double rpm_r, ros::Time & time)
{
  // RCLCPP_INFO(get_logger(), "DiffDrive Config");
  double d = Wt/2.0;
  double linear_rpm = (rpm_r + rpm_l)/2.0;
  double angular_rpm = (rpm_r - rpm_l)/(2.0*d);

  double body_linear_vel =  getVel_from_rpm(linear_rpm);
  double body_angular_vel  = getVel_from_rpm(angular_rpm);

  linear_ = body_linear_vel;
  angular_ = body_angular_vel;

  updateOpenLoop(linear_, angular_, time);

}



double amr_odom::getVel_from_rpm(double rpm)
{
  double linear_vel = (2 * M_PI * wheel_radius* rpm)/60.0;
  return linear_vel;
}

std::tuple<double, double> amr_odom::Cartesian_from_polar(double ro, double th)
{
  double x = ro*cos(th);
  double y = ro*sin(th);
  return std::make_tuple(x,y);
}



void amr_odom::updateOpenLoop(double linear, double angular, const ros::Time & time)
{
  /// Save last linear and angular velocity:
  linear_ = linear;
  angular_ = angular;

  /// Integrate odometry:
  const double dt = time.toSec() - timestamp_.toSec();
  timestamp_ = time;
  integrateExact(linear * dt, angular * dt);
}

void amr_odom::integrateXY(double linear_x, double linear_y, double angular)
{
  const double delta_x = linear_x*cos(heading_) - linear_y*sin(heading_);
  const double delta_y = linear_x*sin(heading_) + linear_y*cos(heading_);

  x_ += delta_x;
  y_ += delta_y;
  heading_ += angular;
}

void amr_odom::integrateRungeKutta2(double linear, double angular)
{
  const double direction = heading_ + angular * 0.5;

  /// Runge-Kutta 2nd order integration:
  x_ += linear * cos(direction);
  y_ += linear * sin(direction);
  heading_ += angular;
}

void amr_odom::integrateExact(double linear, double angular)
{
  if (fabs(angular) < 1e-6)
  {
    integrateRungeKutta2(linear, angular);
  }
  else
  {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double r = linear / angular;
    heading_ += angular;
    x_ += r * (sin(heading_) - sin(heading_old));
    y_ += -r * (cos(heading_) - cos(heading_old));
  }
}


amr_odom::~amr_odom()
{
    // Cleanup, if needed
    odom_pub_.reset();
}

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<amr_odom>());
//   rclcpp::shutdown();
//   return 0;
// }
int main(int argc, char **argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "amr_odom_node");

  // Create a ROS node handle
  ros::NodeHandle nh;

  // Create an instance of your amr_odom class
  amr_odom odom_node;

  // Set the loop rate (if your node has a loop)
  ros::Rate loop_rate(10);  // Adjust the rate as needed

  while (ros::ok())
  {
    // Your main node logic here

    // Spin once to process callbacks (if any)
    ros::spinOnce();

    // Sleep to maintain the loop rate
    loop_rate.sleep();
  }

  // Shutdown ROS
  ros::shutdown();

  return 0;
}
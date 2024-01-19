#ifndef AMR_ODOM_HPP
#define AMR_ODOM_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>



#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
class amr_odom : public ros::NodeHandle
{
public:
    amr_odom();
    ~amr_odom();

public:
    double get_x() const { return x_; }  // return x position [m]
    double get_y() const { return y_; }  // return y position [m]
    double get_heading() const { return heading_; } // return heading angle [rad]

    double get_linear_x() const { return linear_; }  // return linear velocity x [m/s]
    double get_linear_y() const { return linear_y_; }  // return linear velocity y [m/s]
    double get_angular_z() const { return angular_; }  // angular velocity [rad/s]

    void resetOdometry();
    
    void updateOpenLoop(double linear, double angular, const ros::Time & time);
    void integrateXY(double linear_x, double linear_y, double angular);
    void integrateRungeKutta2(double linear, double angular);
    void integrateExact(double linear, double angular);

    void getVelocities_DiffDrive(double rpm_l, double rpm_r, ros::Time & time);
    void getVelocities_Four_wheel_drive(double rpm_fl, double rpm_bl, double rpm_fr, double rpm_br, ros::Time & time);

    double getVel_from_rpm(double rpm);
    std::tuple<double, double> Cartesian_from_polar(double ro, double th);

    int odom_update();

private:
    ros::Publisher odom_pub_;

    // Current timestamp:
    ros::Time timestamp_;
    
/* 

Omni config
    | ^
    | L
    | ~
    /\   
   /  \  
  /    \ 
    
Hex Omni
  \   |   /
   \  |  / -
    \ | /  R_hex
     \|/   ~
--------------
     /|\
    / | \
   /  |  \
  /   |   \
 
Other config
\-----------\
     |         
     |
     |  Wb
     |
     |
|-----------| 
          Wt       
*/
    // double R_hex = 0.5;
    // double L = 0.5; 
    // double Wb = 1.1; // m
    double Wt = 0.21; // m
    double wheel_radius = 0.0485;  //r Replace with your robot's wheel radius in meters

    // pose_state && vel_state are the robot body frame informations.

    // pose_state = [pose_x, pose_y, orien_z]; || [pose_x, pose_y, steer_angle];
    // vel_state = [linear_x, linear_y, angular_z];

    double x_ = 0.0;
    double y_ = 0.0;
    double heading_ = 0.0;

    double linear_ = 0.0;
    double linear_y_ = 0.0;
    double angular_ = 0.0;


    double steering_angle;
    double drive_velocity;

    double drive_velocity_left;
    double drive_velocity_right;
    double angular_velocity;

    double steering_angle_r;
    double steering_angle_l;

    std::vector<double> pose_covarience{};

};

#endif  // amr_odom_HPP
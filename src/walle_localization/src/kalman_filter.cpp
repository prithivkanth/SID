#include "walle_localization/kalman_filter.hpp"

using std::placeholders::_1;

KalmanFilter::KalmanFilter(const std::string & name)
    : Node(name)
    , mean_(0.0)
    , variance_(1000.0)
    , imu_angular_z_(0.0)
    , is_first_odom_(true)
    , last_angular_z_(0.0)
    , motion_(0.0)
{
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("walle_controller/odom_noisy", 10, std::bind(&KalmanFilter::odomCallback, this, _1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu/data", 10, std::bind(&KalmanFilter::imuCallback, this, _1));


}

void KalmanFilter::odomCallback(const nav_msgs::msg::Odometry &odom)
{
    kalaman_odom_ = odom;

    if(is_first_odom_)
    {
        mean_ = odom.twist.twist.angular.z;
        last_angular_z_ = odom.twist.twist.angular.z;
        is_first_odom_ = false;
        return ;
    }

    
}

void KalmanFilter::imuCallback(const sensor_msgs::msg::Imu &imu)
{
    imu_angular_z_= imu.angular_velocity.z;
}
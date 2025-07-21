#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>



class KalmanFilter : public rclcpp::Node 
{
public:
    KalmanFilter(const std::string & name);



private:

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;


    double mean_;
    double variance_;
    double imu_angular_z_;
    bool is_first_odom_;
    double last_angular_z_;
    double motion_;

    double motion_variance_;
    double measurement_variance_;

    void measurementUpdate();


    nav_msgs::msg::Odometry kalaman_odom_;


    void odomCallback(const nav_msgs::msg::Odometry & odom);
    void imuCallback(const sensor_msgs::msg::Imu & imu);
};

#endif
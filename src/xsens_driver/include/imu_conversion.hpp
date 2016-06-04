/********************

H file for converting IMU raw to orientation

*******************/
#include <ros/ros.h>
#include <geometry_msgs/quaternion.h>
#include <sensor_msgs/imu.h>


public class imu_conversion {
public:
    imu_conversion;
    geometry_msgs::QuaternionStamped orientation;
    void publish_quat();//function to publish quaternion
    void imu_callback();
    void main_loop();   //prev vals into quats
    void setup_loop(); //15 sec run, to get averages
    bool new_params;
    void quat_convert();
    void imu_callback(const sensor_msgs::imu::ConstPtr& msg);

private:
    ros::NodeHandle nh;
    ros::publisher quat_msg; //quaternion msg publisher
    ros::Rate r
    double[] prev_vals; //x,y,z,r,p,y
    
}

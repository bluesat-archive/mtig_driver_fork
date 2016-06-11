/********************

H file for converting IMU raw to orientation

*******************/
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>


class imu_conversion {
    public:
        imu_conversion();
        geometry_msgs::Quaternion orientation;
        void publish_quat();//function to publish quaternion
        void imu_callback();
        void main_loop();   //prev vals into quats
        void setup_loop(); //15 sec run, to get averages
        bool new_params;
        void quat_convert();
        void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
        

    private:
        ros::NodeHandle nh;
        ros::Publisher quatMsgPub; //quaternion msg publisher
        ros::Publisher correctIMU; //publishes the correct IMU
        ros::Rate r;
        double prev_vals[]; //x,y,z,r,p,y
    
};

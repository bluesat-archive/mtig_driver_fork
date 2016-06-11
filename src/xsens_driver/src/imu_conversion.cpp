/****************************:
Author: Chris Squire

Program to convert IMU accel & angular_rate
2 parts:

1st part, correlate data from IMU,
caclulate averages for stationary platform
determine initial RPY vals

2nd part, given initial RPY values, 
conversion to orientation quat can be achieved
overtime.

****************************/

/*
 * Input:
 *      angular_velocity and linear_acceleration
 *      orientation on the input is (when xsens word is at the front, top):
 *              z is going down (z = -z)
 *              y is positive to the right
 *              x is positive backwards
 * Output:
 *      we want everything in ros's co-ordinate system
 *      which is x forwards, y is left, z is up
 */ 
#include <sensor_msgs/Imu.h>
#include <imu_conversion.hpp>

static sensor_msgs::Imu flipAxies(sensor_msgs::Imu in);

int main(int argc, char **argv) {
    
    //initialise imu_conversion
    imu_conversion imu;

    //we get angular_velocity and linear_acceleration
    //we want orientation
    if(ros::ok()) {
        imu.setup_loop();
        imu.main_loop();
    }
}

imu_conversion::imu_conversion() : r(50) {
    new_params = true;
    //ros::Rate r(50);
    quatMsgPub =  nh.advertise<geometry_msgs::Quaternion>("/orientation",10,false);
    nh.subscribe("mti/sensor/imu", 1, &imu_conversion::imu_callback, this); 
}

void imu_conversion::setup_loop() {
    double setup_vals[6];
    int sampleCount = 0;
    ros::Time setupBegin = ros::Time::now();
    ros::Time cur = setupBegin;
    while(cur.sec - setupBegin.sec < 15) {
        r.sleep();
        if(new_params) {
            new_params = false;
            for(unsigned int i = 0; i < 6; ++i) {
                setup_vals[i] += prev_vals[i]; //angular velocities collected for a bias elimination
                ++sampleCount;
            }
        }
        cur = ros::Time::now();
    }
    for(unsigned int i = 0 ; i < 6; ++i) {
        prev_vals[i] = setup_vals[i]/sampleCount;
    }
    //need to convert to quat
}

void imu_conversion::main_loop() {
    while(ros::ok()) {    
        r.sleep();
        if(new_params) {
            new_params = false;
            //update quat and publish (need to talk to jack)
            quat_convert();
            publish_quat();    
        }
    }
}

void imu_conversion::imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    //flip the axis
    sensor_msgs::Imu correctMsg = flipAxies(*msg);
    
    //getting angular velocity & linear acceleration
    this->prev_vals[0] = correctMsg.linear_acceleration.x;
    this->prev_vals[1] = correctMsg.linear_acceleration.y;
    this->prev_vals[2] = correctMsg.linear_acceleration.z;

    this->prev_vals[3] = correctMsg.angular_velocity.x;
    this->prev_vals[4] = correctMsg.angular_velocity.y;
    this->prev_vals[5] = correctMsg.angular_velocity.z;

    new_params = true;
}

//convert x,y,z,r,p,y velocities accelerations
//into orientation quaternion
//for is orientation.quaternion x,y,z,w 
void imu_conversion::quat_convert() {
    //defualt yaw to be pi/2 (maybe not though because x is facing axis on ros
    
}


void imu_conversion::publish_quat() {
    quatMsgPub.publish(orientation);
}

static sensor_msgs::Imu flipAxies(sensor_msgs::Imu in) {
    sensor_msgs::Imu out;
    out.angular_velocity.x = -in.angular_velocity.x;
    out.angular_velocity.y = -in.angular_velocity.y;
    out.angular_velocity.z = -in.angular_velocity.z;
    out.linear_acceleration.x = -in.linear_acceleration.x;
    out.linear_acceleration.y = -in.linear_acceleration.y;
    out.linear_acceleration.z = -in.linear_acceleration.z;
}

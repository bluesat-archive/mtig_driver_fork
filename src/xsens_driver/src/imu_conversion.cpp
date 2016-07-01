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

#define BIAS_TESTING //remove for actual use


static sensor_msgs::Imu flipAxies(sensor_msgs::Imu in);
static sensor_msgs::Imu addCovar(sensor_msgs::Imu & in);

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_conversion");
    
    //initialise imu_conversion
    imu_conversion imu;

    //calculate initial orientation
    //calculate gyro bias (drift)
    imu.setup_loop();

    ros::spin();//NOTE: this blocks untill the end of the program
    //we get angular_velocity and linear_acceleration
    //we want orientation
    ROS_ERROR("should never reach this part");
}

imu_conversion::imu_conversion() : is_setup_running(true) {
    new_params = true;
    quatMsgPub =  nh.advertise<geometry_msgs::Quaternion>("/orientation",10,false);
    correctIMU = nh.advertise<sensor_msgs::Imu>("/imu",10,false);
    imuSub = nh.subscribe("/mti/sensor/imu", 1, &imu_conversion::imu_callback, this); 
}

void imu_conversion::setup_loop() {
    double setup_vals[6];
    int sampleCount = 0;
    ros::Time setupBegin = ros::Time::now();
    ros::Time cur = setupBegin;
    while(sampleCount <= 400) { //should run for 15sec
//#define DEBUG
#ifdef DEBUG
        std::cout << "setup_loop" << std::endl;
        std::cout << "Z: " << prev_vals[2] << std::endl;
#endif
        ros::spinOnce();
        if(sqrt(prev_vals[2]*prev_vals[2] + prev_vals[1]*prev_vals[1] + prev_vals[0]*prev_vals[0]) <= 8) {
            continue; //ensure "sane" data being transmitted
        }
        if(new_params) {
            new_params = false;
            for(unsigned int i = 0; i < 6; ++i) {
                setup_vals[i] += prev_vals[i];
            }
            ++sampleCount;
        }
        cur = ros::Time::now();
    }
    for(unsigned int i = 0 ; i < 6; ++i) {
        setup_vals[i] = setup_vals[i]/sampleCount;
    }
    bias[0] = setup_vals[3];
    bias[1] = setup_vals[4];
    bias[2] = setup_vals[5];
    is_setup_running = false;
}

/*void imu_conversion::main_loop() {
    while(ros::ok()) {    
        if(new_params) {
            new_params = false;
            //update quat and publish (need to talk to jack)
            quat_convert();
            publish_quat();    
        }
    }
}*/

void imu_conversion::imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    //flip the axis
    

    sensor_msgs::Imu correctMsg = flipAxies(*msg);

    correctMsg.angular_velocity.x -= bias[0];
    correctMsg.angular_velocity.y -= bias[1];
    correctMsg.angular_velocity.z -= bias[2];
#ifdef DEBUG    
    if(!is_setup_running) {        
        std::cout << "roll bias: " << bias[0] << " roll: " << correctMsg.angular_velocity.x << std::endl;
        std::cout << "pitch bias: " << bias[1] << " pitch: " << correctMsg.angular_velocity.y << std::endl;
        std::cout << "yaw bias: " << bias[2] << " yaw: " << correctMsg.angular_velocity.z << std::endl;
    }
#endif
    correctMsg = addCovar(correctMsg);    
    correctMsg.header = msg->header;
    //getting angular velocity & linear acceleration
    if(!is_setup_running) {
#ifdef REMOVE_NOISE
        double noise_param = 0.1;
        if(correctMsg.angular_velocity.x <= noise_param) {
            correctMsg.angular_velocity.x = 0;
        }
        if(correctMsg.angular_velocity.y <= noise_param) {
            correctMsg.angular_velocity.y = 0;
        }
        if(correctMsg.angular_velocity.z <= noise_param) {
            correctMsg.angular_velocity.z = 0;
        }
#endif
        correctIMU.publish<sensor_msgs::Imu>(correctMsg);
    }
    new_params = true;
    if(is_setup_running) {
        prev_vals[0] = msg->linear_acceleration.x;
        prev_vals[1] = msg->linear_acceleration.y;
        prev_vals[2] = msg->linear_acceleration.z;

        prev_vals[3] = msg->angular_velocity.x;
        prev_vals[4] = msg->angular_velocity.y;
        prev_vals[5] = msg->angular_velocity.z;
    }
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


static sensor_msgs::Imu addCovar(sensor_msgs::Imu & in) {
    //see comments on OWRS-195 for explanation
    
    //TODO: these should be ros params
    #define ACCEL_NOISE_DENSITY (88.4*1e-6) //gravity/sqrt(Hz)
    #define RATE_GYRO_NOISE_DENSITY 0.029 //deg/s/sqrt(Hz)
    #define FREQ 400.0 //this should be the frequency we are reciving messages
    #define GRAVITY 9.80665
    
    //TODO: this should me in (m/s^2)^2 should check that the units work
    double accelError = pow((ACCEL_NOISE_DENSITY*GRAVITY),2)*FREQ;
    double gyroError  = pow((RATE_GYRO_NOISE_DENSITY*M_PI),2)/((double)(180*180)) * FREQ;

    in.angular_velocity_covariance[0] = in.angular_velocity_covariance[1] =
        in.angular_velocity_covariance[8] = gyroError;
    
    in.linear_acceleration_covariance[0] = in.linear_acceleration_covariance[1] =
        in.linear_acceleration_covariance[8] = accelError;

    return in;
   

}

static sensor_msgs::Imu flipAxies(sensor_msgs::Imu in) {
    sensor_msgs::Imu out;
    out.angular_velocity.x = in.angular_velocity.x;
    out.angular_velocity.y = in.angular_velocity.y;
    out.angular_velocity.z = in.angular_velocity.z;
    out.linear_acceleration.x = in.linear_acceleration.x;
    out.linear_acceleration.y = in.linear_acceleration.y;
    out.linear_acceleration.z = in.linear_acceleration.z;
    return out;
}

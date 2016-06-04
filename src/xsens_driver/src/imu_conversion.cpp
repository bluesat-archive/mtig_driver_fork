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
int main(int argc, char **argv) {
    
    //initialise imu_conversion
    imu_conversion imu();

    if(ros.ok()) {
        imu.setup_loop();
        imu.main_loop();
    }
}

imu_conversion::imu_conversion() {
    new_params = true;
    ros::Rate r(50);
    nh.advertise();
    nh.subscribe("mti/sensor/imu", 0, /*callback*/);
}

void imu_conversion::setup_loop() {
    double[6] setup_vals;
    int sample_count = 0;
    ros::Time setup_begin = ros::Time::now();
    ros::Time cur = setup_begin;
    while(cur_time.sec - setup_begin.sec < 15) {
        r.sleep();
        if(new_params) {
            new_params = false;
            for(unsigned int i = 0; i < 6; ++i) {
                setup_vals[i] += prev_vals[i]; //angular velocities collected for a bias elimination
                ++sample_count;
            }
        }
        cur = ros::Time::now();
    }
    for(unsigned int i = 0 ; i < 6; ++i) {
        prev_vals[i] = setup_vals[i]/sample_count;
    }
    //need to convert to quat
}

void imu_conversion::main_loop() {
    while(ros.ok()) {    
        r.sleep();
        if(new_params) {
            new_params = false;
            //update quat and publish (need to talk to jack)
            quat_convert();
            publish_quat();    
        }
    }
}

void imu_conversion::imu_callback(const sensor_msgs::imu::ConstPtr& msg) {
    //getting angular velocity & linear acceleration
    this.prev_vals[0] = msg.linear_acceleration.x;
    this.prev_vals[1] = msg.linear_acceleration.y;
    this.prev_vals[2] = msg.linear_acceleration.z;

    this.prev_vals[3] = msg.angular_velocity.x;
    this.prev_vals[4] = msg.angular_velocity.y;
    this.prev_vals[5] = msg.angular_velocity.z;

    new_params = true;
}

//convert x,y,z,r,p,y velocities accelerations
//into orientation quaternion
//for is orientation.quaternion x,y,z,w 
void imu_conversion::quat_convert() {
    //defualt yaw to be pi/2 (maybe not though because x is facing axis on ros
}


void imu_conversion::publish_quat() {
    quat_msg.publish(orientation);
}






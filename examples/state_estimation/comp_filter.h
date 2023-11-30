#include "mbot.h"
#include "msgtypes.h"

class comp_filter : public mbot {
public:
    comp_filter(std::string name, std::string mac, float alpha) 
    : mbot(name, mac) {
        this->alpha = alpha;
        filtered_pose.store({0, 0, 0, 0});
        prev_yaw = 0;
        prev_utime = 0;
    }

    void set_filtered_pose(float x, float y, float theta) {
        filtered_pose.store({0, x, y, theta});
    }

    serial_pose2D_t get_filtered_pose() {
        return filtered_pose.load();
    }

private:
    float alpha;
    std::atomic<serial_pose2D_t> filtered_pose;
    float prev_yaw;
    uint64_t prev_utime; 

    // Override the on_update virtual function to implement the complementary filter
    void on_update() {

        serial_twist2D_t mbot_vel = get_robot_vel();
        serial_pose2D_t odometry = filtered_pose.load();

        float curr_yaw = get_imu().angles_rpy[2];

        // Check if this is the first time we've run
        if (prev_utime == 0) 
            prev_utime = mbot_vel.utime;
        
        // Calculate the time difference between this and the last update in seconds
        float dt = (mbot_vel.utime - prev_utime) / 1e6;

        // Calculate the yaw rate from the IMU
        float dyaw_imu = curr_yaw - prev_yaw;
        
        // Calculate the velocity in the space frame
        float vx_space = mbot_vel.vx * cos(odometry.theta) - mbot_vel.vy * sin(odometry.theta);
        float vy_space = mbot_vel.vx * sin(odometry.theta) + mbot_vel.vy * cos(odometry.theta);
        float dyaw_odom = mbot_vel.wz * dt;

        // Update the odometry
        odometry.x += vx_space * dt;
        odometry.y += vy_space * dt;
        odometry.theta += dyaw_imu * alpha + dyaw_odom * (1 - alpha);

        // Normalize theta to be between -pi and pi
        while (odometry.theta > M_PI) odometry.theta -= 2 * M_PI;
        while (odometry.theta <= -M_PI) odometry.theta += 2 * M_PI;

        filtered_pose.store(odometry);
        prev_yaw = curr_yaw;
        prev_utime = mbot_vel.utime;
    }
    
    // Override the get_functional_pose virtual function to return the filtered pose to the GUI
    serial_pose2D_t get_functional_pose() {
        return filtered_pose.load();
    }
};
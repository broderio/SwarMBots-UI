#include "mbot.h"
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <csignal>
#include <queue>

volatile sig_atomic_t flag = 1;

class path_follower : public mbot {
public:
    path_follower(const std::string &name, const std::string &mac_str, float Kp, float Ki)
     : mbot(name, mac_str), Kp(Kp), Ki(Ki) {}
    
    std::queue<serial_pose2D_t> path;

    void follow_path() {
        std::cout << "Following path ...\n";
        while (!path.empty() && flag) {
            serial_pose2D_t goal = path.front();
            serial_pose2D_t odom = get_odom();
            float error_sum = 0;
            std::cout << "Following path to (" << goal.x << ", " << goal.y << ", " << goal.theta << ")\n";
            while (!at_goal(goal, odom) && flag) {
                float dx = goal.x - odom.x;
                float dy = goal.y - odom.y;

                float lookahead_dist = sqrt(dx*dx + dy*dy);
                float lookahead_theta = atan2(dy, dx) - odom.theta;

                // Ensure the lookahead_theta is between -pi and pi
                while (lookahead_theta > M_PI) lookahead_theta -= 2*M_PI;
                while (lookahead_theta < -M_PI) lookahead_theta += 2*M_PI;

                float curvature = 2*sin(lookahead_theta) / lookahead_dist;

                error_sum += lookahead_dist; // Update error sum

                float v = Kp * lookahead_dist + Ki * error_sum;  // K is a proportional gain
                float w = curvature * v;

                std::cout << "lookahead_dist: " << lookahead_dist << ", lookahead_theta: " << lookahead_theta << "\n";
                std::cout << "v: " << v << ", w: " << w << "\n";

                set_robot_vel_goal(v, 0.0, w);
                usleep(40000);
                odom = get_odom();
            }
            path.pop();
        }
        set_robot_vel_goal(0.0, 0.0, 0.0);
        usleep(1000000);
    }
    private:
        float Kp, Ki;
        float eps_lin = 0.02;
        float eps_ang = 0.1;

        bool at_goal(serial_pose2D_t goal, serial_pose2D_t odom) {
            float dx = goal.x - odom.x;
            float dy = goal.y - odom.y;
            float dtheta = goal.theta - odom.theta;
            return sqrt(dx*dx + dy*dy) < eps_lin && fabs(dtheta) < eps_ang;
        }
};

int main(int argc, char *argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <serial_port> <mac_file_path> <path_file_path>\n";
        return 1;
    }
    std::string port = argv[1];
    std::string mac_file_path = argv[2];
    std::string path_file_path = argv[3];

    // Set serial port
    mbot::port = port;

    // Get mac address
    std::ifstream macs(mac_file_path);
    std::string mac_str;
    std::getline(macs, mac_str);
    std::cout << "Mac address: " << mac_str << "\n";
    macs.close();

    // Create mbot object and start server
    path_follower m("mbot", mac_str, 1.0, 0.06);
    path_follower::start_server();

    // Open path file
    std::ifstream path_file(path_file_path);
    std::signal(SIGINT, [](int signum) {
        flag = 0;
    });

    // Record odometry until user presses enter
    std::cout << "Path Follower!\n";
    std::cout << "Put host into serial mode ...\n";
    std::cout << "Press enter to start driving\n";
    std::cin.get();
    std::cout << "Driving along path ...\n";
    std::cout << "Press ctrl-c to stop\n";
    m.reset_odom();
    m.reset_encoders();
    usleep(1000000); // Wait for odometry to reset

    // Read path file
    for (std::string line; std::getline(path_file, line);) {
        std::istringstream iss(line);
        serial_pose2D_t goal_pose;
        if (!(iss >> goal_pose.utime >> goal_pose.x >> goal_pose.y >> goal_pose.theta)) {
            std::cerr << "Error reading path file\n";
            exit(1);
        }
        m.path.push(goal_pose);
    }
    path_file.close();

    std::cout << "Path size: " << m.path.size() << "\n";

    // Follow path
    m.follow_path();

    std::cout << "Done following!\n";
    exit(0);
}

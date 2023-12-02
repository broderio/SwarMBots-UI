#include "mbot.h"
#include <algorithm>

volatile sig_atomic_t flag = 1;

class path_follower : public mbot {
public:
    path_follower(const std::string &name, const std::string &mac_str, float Kp, float Ki)
     : mbot(name, mac_str), Kp(Kp), Ki(Ki) {}
    
    void load_path(std::string path_file_path) {
        std::ifstream path_file(path_file_path);
        for (std::string line; std::getline(path_file, line);) {
            std::istringstream iss(line);
            serial_pose2D_t goal_pose;

            int64_t utime;
            float x, y, theta;
            if (!(iss >> utime >> x >> y >> theta)) {
                std::cerr << "Error reading path file\n";
                exit(1);
            }

            goal_pose.utime = utime;
            goal_pose.x = x;
            goal_pose.y = y;
            goal_pose.theta = theta;
            
            path.push(goal_pose);
        }
        std::cout << "Path size: " << path.size() << "\n";
    }

    void follow_path() {
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

                error_sum += lookahead_dist;

                float v, w; 

                if (lookahead_dist > 0.3 / 1.5 && fabs(lookahead_theta) > 0.1) {
                    v = 0.0;
                    w = lookahead_theta;
                }
                else {
                    v = Kp * lookahead_dist + Ki * error_sum;
                    w = curvature * v;
                }

                v = std::clamp(v, 0.0f, 0.3f);
                w = std::clamp(w, -1.5f, 1.5f);

                // std::cout << "lookahead_dist: " << lookahead_dist << ", lookahead_theta: " << lookahead_theta << "\n";
                // std::cout << "v: " << v << ", w: " << w << "\n";

                set_robot_vel_goal(v, 0.0, w);
                usleep(100000);
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
        float eps_ang = 0.2;

        std::queue<serial_pose2D_t> path;

        bool at_goal(serial_pose2D_t goal, serial_pose2D_t odom) {
            float dx = goal.x - odom.x;
            float dy = goal.y - odom.y;
            float dtheta = goal.theta - odom.theta;
            return sqrt(dx*dx + dy*dy) < eps_lin && fabs(dtheta) < eps_ang;
        }
};
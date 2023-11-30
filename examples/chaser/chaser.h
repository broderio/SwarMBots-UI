#include "mbot.h"

volatile sig_atomic_t flag = 1;

class chaser : public mbot {
public:
    chaser(const std::string &name, const std::string &mac_str, float Kp, float Ki)
     : mbot(name, mac_str), Kp(Kp), Ki(Ki) {}

    chaser(const chaser &other)
     : mbot(other), Kp(other.Kp), Ki(other.Ki) {
        error_sum.store(other.error_sum.load());
        goal_pose.store(other.goal_pose.load());
        ready.store(other.ready.load());
     }

    void set_goal_pose(serial_pose2D_t goal) {
        goal_pose.store(goal);
        error_sum.store(0);
    }

    void set_ready(bool state) {
        this->ready.store(state);
    }

    void on_update() {
        if (!ready.load()) return;

        serial_pose2D_t goal = goal_pose.load();
        serial_pose2D_t odom = get_odom();
        float e_i = error_sum.load();

        float dx = goal.x - odom.x;
        float dy = goal.y - odom.y;

        float lookahead_dist = sqrt(dx*dx + dy*dy);
        float lookahead_theta = atan2(dy, dx) - odom.theta;

        // Ensure the lookahead_theta is between -pi and pi
        while (lookahead_theta > M_PI) lookahead_theta -= 2*M_PI;
        while (lookahead_theta < -M_PI) lookahead_theta += 2*M_PI;

        float curvature = 2*sin(lookahead_theta) / lookahead_dist;

        e_i += lookahead_dist;

        float v = Kp * lookahead_dist + Ki * e_i;
        float w = curvature * v;

        if (v > 0.1) v = 0.1;
        // std::cout << "lookahead_dist: " << lookahead_dist << ", lookahead_theta: " << lookahead_theta << "\n";
        // std::cout << "v: " << v << ", w: " << w << "\n";

        error_sum.store(e_i);
        set_robot_vel_goal(v, 0.0, w);
    }

    private:
        float Kp, Ki;
        float eps_lin = 0.02;
        float eps_ang = 0.1;
        
        std::atomic<float> error_sum;
        std::atomic<serial_pose2D_t> goal_pose;
        std::atomic<bool> ready;
};
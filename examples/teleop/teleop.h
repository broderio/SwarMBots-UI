#include <iostream>
#include <stdio.h>
#include <curses.h>
#include "mbot.h"

class teleop : public mbot {
public:
    teleop(const std::string &name, const std::string &mac_str)
     : mbot(name, mac_str) {}

     void set_keys(char F, char R, char B, char L, char S) {
            forward = F;
            right = R;
            backward = B;
            left = L;
            stop = S;

            {
                std::lock_guard<std::mutex> lock(key_map_mutex);
                key_map[forward] = [this]() {
                    this->set_robot_vel_goal(0.25, 0, 0);
                };
                key_map[right] = [this]() {
                    this->set_robot_vel_goal(0, 0, 1.5);
                };
                key_map[backward] = [this]() {
                    this->set_robot_vel_goal(-0.25, 0, 0);
                };
                key_map[left] = [this]() {
                    this->set_robot_vel_goal(0, 0, -1.5);
                };
                key_map[stop] = [this]() {
                    this->set_robot_vel_goal(0, 0, 0);
                };
            }
     }

     static void start_teleop() {
        initscr();
        cbreak();
        noecho();
        teleop_th_handle = std::move(std::thread(&teleop::teleop_th));
     }

private:
    char forward, right, backward, left, stop;
    static std::thread teleop_th_handle;
    static std::unordered_map<char, std::function<void(void)>> key_map;
    static std::mutex key_map_mutex;

    static void teleop_th() {
        while (true) {
            char c = getch();
            if (c == 'q') {
                break;
            }

            {
                std::lock_guard<std::mutex> lock(key_map_mutex);
                if (key_map.find(c) != key_map.end()) {
                    key_map[c]();
                }
            }
        }
    }
};

std::thread teleop::teleop_th_handle;
std::unordered_map<char, std::function<void(void)>> teleop::key_map;
std::mutex teleop::key_map_mutex;

#include <iostream>
#include <vector>
#include <unistd.h>
#include <fstream>

#include "mbot.h"

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <serial_port>\n";
        return 1;
    }
    std::string port = argv[1];

    // Create mbot object
    mbot::port = port;
    std::vector<mbot> mbots = mbot::init_from_file();
    mbot::set_verbose(true);
    mbot m = mbots[0];
    
    std::cout << "Measuring round trip latency...\n";
    uint64_t avg = 0;   
    for (int i = 0; i < 100; i++) {
        m.set_odom(i, 0, 0);
        uint64_t start = mbot::get_time_us();
        while (m.get_odom().x != i);
        uint64_t end = mbot::get_time_us();
        avg += end - start;
    }
    std::cout << "Round trip latency: " << avg / 100 / 1e6 << " seconds\n";

    std::cout << "Measuring update rate...\n";
    size_t count = 0;
    mbot::set_on_update([&count](mbot *m) {
        count++;
    });
    sleep(15);
    mbot::set_on_update(nullptr);
    std::cout << "Update rate: " << count / 15 << " Hz\n";

    std::cout << "Measuring jitter...\n";
    std::vector<uint64_t> times;
    mbot::set_on_update([&times](mbot *m) {
        times.push_back(mbot::get_time_us());
    });
    sleep(15);
    mbot::set_on_update(nullptr);

    double avg_diff = 0;
    for (int i = 1; i < times.size(); i++) {
        avg_diff += times[i] - times[i - 1];
    }
    avg_diff /= (times.size() - 1);
    double jitter = 0;
    for (int i = 1; i < times.size(); i++) {
        double diff = times[i] - times[i - 1];
        jitter += (diff - avg_diff) * (diff - avg_diff);
    }
    jitter = sqrt(jitter / (times.size() - 2));
    std::cout << "Jitter: " << jitter << " us\n";

}
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
    mbot::init(port);
    // mbot::set_verbose(true);
    mbot::set_fast(true);
    mbot m("m", "30:30:f9:69:4e:49");
    
    std::cout << "Measuring round trip latency...\n";
    uint64_t avg = 0;   
    for (int i = 0; i < 100; i++) {
        m.set_odom(i, 0, 0);
        std::cout << "Sent " << i << "\r";
        uint64_t start = mbot::get_time_us();
        while (m.get_odom().x != i);
        uint64_t end = mbot::get_time_us();
        avg += end - start;
    }
    std::cout << "\nRound trip latency: " << avg / 100 / 1e6 << " seconds\n";

    std::cout << "Measuring update rate...\n";
    size_t count = 0;
    std::vector<uint64_t> times;
    times.reserve(20000);
    mbot::set_on_update([&count, &times](mbot *m) {
        count++;
        times.push_back(mbot::get_time_us());
    });
    mbot::reset_invalid_msg_count();
    sleep(60);
    mbot::set_on_update(nullptr);
    std::cout << "Received " << count << " packets in 30 seconds.\n"
              << "Received " << mbot::get_invalid_msg_count() << " invalid packets.\n"
              << "Update rate: " << count / 60.0 << " Hz\n";

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
    std::cout << "Average time between packets: " << avg_diff << " us\n";
    std::cout << "Jitter: " << jitter << " us\n";

}
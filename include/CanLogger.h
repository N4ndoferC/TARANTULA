#pragma once
#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <fstream>
#include <cstdint>

struct LogEntry {
    int64_t timestamp;
    bool is_tx;
    uint32_t can_id;
    std::vector<uint8_t> data;
};

class CanLogger {
private:
    std::string filename_;
    std::ofstream file_;
    std::queue<LogEntry> queue_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::thread thread_;
    std::atomic<bool> running_;

    void processQueue();

public:
    CanLogger(const std::string& filename = "can_log.csv");
    ~CanLogger();

    void start();
    void stop();
    void logTx(uint32_t can_id, const std::vector<uint8_t>& data);
    void logRx(uint32_t can_id, const std::vector<uint8_t>& data);
};

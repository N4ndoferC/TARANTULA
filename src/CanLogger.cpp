#include "CanLogger.h"
#include <chrono>
#include <iostream>
#include <iomanip>

CanLogger::CanLogger(const std::string& filename) : filename_(filename), running_(false) {}

CanLogger::~CanLogger() {
    stop();
}

void CanLogger::start() {
    if (running_) return;
    file_.open(filename_, std::ios::out | std::ios::trunc);
    if (file_.is_open()) {
        file_ << "Timestamp_ms,Direction,CAN_ID,Data_Bytes,Hex_Data\n";
    } else {
        std::cerr << "CanLogger error: no se pudo abrir " << filename_ << "\n";
    }
    running_ = true;
    thread_ = std::thread(&CanLogger::processQueue, this);
}

void CanLogger::stop() {
    if (!running_) return;
    running_ = false;
    cv_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
    if (file_.is_open()) {
        file_.close();
    }
}

void CanLogger::logTx(uint32_t can_id, const std::vector<uint8_t>& data) {
    if (!running_) return;
    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.size() < 100000) { // Limitar el búfer para evitar sobrecarga de memoria
        queue_.push({now, true, can_id, data});
        cv_.notify_one();
    }
}

void CanLogger::logRx(uint32_t can_id, const std::vector<uint8_t>& data) {
    if (!running_) return;
    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.size() < 100000) {
        queue_.push({now, false, can_id, data});
        cv_.notify_one();
    }
}

void CanLogger::processQueue() {
    while (running_) {
        std::vector<LogEntry> batch;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this]() { return !queue_.empty() || !running_; });
            
            while (!queue_.empty()) {
                batch.push_back(std::move(queue_.front()));
                queue_.pop();
            }
        }


        if (file_.is_open() && !batch.empty()) {
            for (const auto& entry : batch) {
                file_ << entry.timestamp << ","
                      << (entry.is_tx ? "TX" : "RX") << ","
                      << "0x" << std::hex << std::uppercase << entry.can_id << std::dec << ","
                      << entry.data.size() << ",";
                
                for (size_t i = 0; i < entry.data.size(); ++i) {
                    file_ << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)entry.data[i] << " ";
                }
                file_ << std::dec << "\n";
            }
            file_.flush(); // Asegurar que se guarda en el disco en tiempo real
        }
    }
}

#pragma once
#include "SimpleSerial.h"
#include "CanLogger.h"
#include <vector>
#include <utility>
#include <cstdint>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

class WaveshareInterface {
private:
    SimpleSerial* serial;  //crea su propio puerto serie
    std::string port;
    int baud;
    bool is_connected;
    std::vector<uint8_t> rx_buffer; // Buffer persistente para sincronización

    // Cola de envío asíncrona para espaciar tramas a 1 ms en hardware sin bloquear el hilo de control
    std::queue<std::vector<uint8_t>> tx_queue;
    std::mutex tx_mutex;
    std::condition_variable tx_cv;
    std::thread tx_thread;
    std::atomic<bool> tx_running;

    uint8_t calculate_checksum(const std::vector<uint8_t>& payload);
    void configure_adapter_500k();
    void txLoop();

public:
    CanLogger logger;

    WaveshareInterface(std::string portStr = "COM3", int baudRate = 2000000);
    ~WaveshareInterface();

    bool connect();
    void close();
    bool reconnectWithPort(const std::string& new_port);
    void send_can_frame(uint32_t can_id, const std::vector<uint8_t>& data_bytes);

    // Retorna true si hay trama, y llena can_id y data
    bool receive_can_frame(uint32_t& can_id, std::vector<uint8_t>& data);
    bool connected() const { return is_connected; }
};
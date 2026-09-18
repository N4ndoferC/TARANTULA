#include "WaveshareInterface.h"
#include "Config.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>

#ifdef _MSC_VER
#include <intrin.h>
#endif

WaveshareInterface::WaveshareInterface(std::string portStr, int baudRate)
    : port(portStr), baud(baudRate), is_connected(false), tx_running(false) {
    serial = new SimpleSerial(port, baud);
}

WaveshareInterface::~WaveshareInterface() {
    close();
    delete serial;
}

bool WaveshareInterface::connect() {
    std::cout << "Abriendo puerto " << port << "...\n";
    if (serial->connect()) {
        is_connected = true;
        configure_adapter_500k();
        
        logger.start();

        // Iniciar hilo de transmisión asíncrono
        tx_running = true;
        tx_thread = std::thread(&WaveshareInterface::txLoop, this);

        return true;
    }
    std::cout << "Error critico de conexion.\n";
    return false;
}

void WaveshareInterface::close() {
    if (is_connected) {
        logger.stop();
        
        // Detener hilo de transmisión asíncrono
        tx_running = false;
        tx_cv.notify_all();
        if (tx_thread.joinable()) {
            tx_thread.join();
        }

        // Vaciar la cola restante
        std::queue<std::vector<uint8_t>> empty_q;
        std::swap(tx_queue, empty_q);

        serial->close();
        is_connected = false;
        std::cout << "Puerto cerrado.\n";
    }
}

bool WaveshareInterface::reconnectWithPort(const std::string& new_port) {
    std::cout << "Solicitando cambio de puerto a " << new_port << "...\n";
    close();
    
    // Eliminar el puerto anterior y crear uno nuevo de manera segura
    delete serial;
    port = new_port;
    serial = new SimpleSerial(port, baud);
    
    return connect();
}

uint8_t WaveshareInterface::calculate_checksum(const std::vector<uint8_t>& payload) {
    
    //calcula checksum (suma de verificación final)

    uint32_t sum = 0;
    for (uint8_t b : payload) sum += b;
    return static_cast<uint8_t>(sum & 0xFF);
}

void WaveshareInterface::configure_adapter_500k() {
    std::cout << "Configurando adaptador a 500 Kbps...\n";
    std::vector<uint8_t> frame(20, 0); // Crea una trama vacía de 20 ceros
    frame[0] = Config::FRAME_HEAD_1; // 0xAA (Firma)
    frame[1] = Config::FRAME_HEAD_2; // 0x55 (Firma)
    frame[2] = Config::CMD_TYPE_CONFIG; // 0x02 (Modo Configuración)
    frame[3] = Config::CAN_BAUD_500K; // 0x03 (Pon el motor a 500Kbps)
    frame[4] = 0x01; // Trama estándar

    // Calcula la suma de seguridad y la pone al final (byte 19)
    //Bit de fin
    std::vector<uint8_t> payload(frame.begin() + 2, frame.begin() + 19);
    frame[19] = calculate_checksum(payload);

        serial->writeBytes(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void WaveshareInterface::send_can_frame(uint32_t can_id, const std::vector<uint8_t>& data_bytes) {
    if (!is_connected) return;
    logger.logTx(can_id, data_bytes);
    std::vector<uint8_t> frame(20, 0);
    frame[0] = Config::FRAME_HEAD_1; // 0xAA
    frame[1] = Config::FRAME_HEAD_2; // 0x55
    frame[2] = Config::CMD_TYPE_DATA; // 0x01 (enviar y recibir datos)
    frame[3] = 0x01; //Tipo de Trama: Trama de Datos
    frame[4] = 0x01; //ID de longitud estándar (11 bits)

    // Little Endian ID (El dato que mandamos del ID es ID motor ≪ 5 | ID comando)
    frame[5] = can_id & 0xFF;
    frame[6] = (can_id >> 8) & 0xFF;
    frame[7] = (can_id >> 16) & 0xFF;
    frame[8] = (can_id >> 24) & 0xFF;
    //la longitud de la trama de datos es de 8 bytes para protocolo MIT
    uint8_t data_len = std::min((int)data_bytes.size(), 8);
    frame[9] = data_len;

    for (int i = 0; i < data_len; i++) {
        frame[10 + i] = data_bytes[i];
    }

    //Calculamos el checksum entre los bytes 2 y 18
    std::vector<uint8_t> payload(frame.begin() + 2, frame.begin() + 19);
    frame[19] = calculate_checksum(payload);

    // Encolar de forma asíncrona para que el hilo de control no se bloquee por el sleep
    {
        std::lock_guard<std::mutex> lock(tx_mutex);

        // Evitar Buffer Bloat en caso de latencia de red/USB o retrasos de planificación:
        // Si la cola crece demasiado (>60 tramas), purgamos las tramas antiguas de datos de control
        // pero CONSERVAMOS las 12 más recientes (el último ciclo completo) y cualquier trama de
        // configuración (CMD_TYPE_CONFIG). Esto evita la pérdida de conexión y latigazos, asegurando
        // una respuesta instantánea y continua.
        if (tx_queue.size() > 60) {
            std::queue<std::vector<uint8_t>> config_frames;
            std::vector<std::vector<uint8_t>> data_frames;

            while (!tx_queue.empty()) {
                auto f = std::move(tx_queue.front());
                tx_queue.pop();
                if (f.size() >= 3 && f[2] == Config::CMD_TYPE_CONFIG) {
                    config_frames.push(std::move(f));
                } else {
                    data_frames.push_back(std::move(f));
                }
            }

            size_t start_idx = 0;
            if (data_frames.size() > 12) {
                start_idx = data_frames.size() - 12;
            }

            // Reconstruimos la cola conservando las configuraciones y las 12 tramas de datos más nuevas
            tx_queue = std::move(config_frames);
            for (size_t i = start_idx; i < data_frames.size(); ++i) {
                tx_queue.push(std::move(data_frames[i]));
            }

            std::cout << "[WaveshareInterface] Cola de envio saturada (>60 tramas). Purgadas " << start_idx << " tramas antiguas. Conservando las 12 mas recientes.\n";
        }

        tx_queue.push(std::move(frame));
    }
    tx_cv.notify_one();
}

bool WaveshareInterface::receive_can_frame(uint32_t& can_id, std::vector<uint8_t>& data) {
    if (!is_connected) return false;

    //Tenemos dos buffers: uno temporal (chunk) y uno persistente (rx_buffer)

    // Leemos todos los bytes que están llegando los metemos en el buffer persistente
    int avail = serial->available(); //devuelve el número de bytes que ha recibido el adaptador
    if (avail > 0) { //si hay al menos 1 byte esperando, se ejecuta
        std::vector<uint8_t> chunk;
        if (serial->readBytes(chunk, avail) > 0) { //mete ese número de bytes en el buffer chunk
            rx_buffer.insert(rx_buffer.end(), chunk.begin(), chunk.end()); //insertamos el contenido de chunk en rx_buffer
        }
    }

    // Buscamos una trama completa válida:
    while (rx_buffer.size() >= 20) {
        if (rx_buffer[0] == Config::FRAME_HEAD_1 && rx_buffer[1] == Config::FRAME_HEAD_2 && rx_buffer[2] == Config::CMD_TYPE_DATA) {
            std::vector<uint8_t> payload(rx_buffer.begin() + 2, rx_buffer.begin() + 19);
            uint8_t expected_checksum = calculate_checksum(payload);
            
            //verificamos si el checksum coincide
            if (rx_buffer[19] == expected_checksum) {
                can_id = rx_buffer[5] | (rx_buffer[6] << 8) | (rx_buffer[7] << 16) | (rx_buffer[8] << 24);
                uint8_t length = rx_buffer[9];

                data.clear();
                for (int i = 0; i < length && i < 8; i++) {
                    data.push_back(rx_buffer[10 + i]);
                }
                
                logger.logRx(can_id, data);
                
                // Eliminamos la trama procesada del buffer
                rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + 20);
                return true;
            } else {
                // Checksum erróneo: posible corrupción, borramos 1 byte para resincronizar
                rx_buffer.erase(rx_buffer.begin());
            }
        } else {
            // Falso positivo (ruido o trama desalineada), borramos 1 solo byte para intentar sincronizar el framing
            rx_buffer.erase(rx_buffer.begin());
        }
    }

    // Proteger el buffer para que no crezca infinito por ruido absoluto 
    if (rx_buffer.size() > 1024) {
        rx_buffer.clear(); 
    }

    return false;
}

void WaveshareInterface::txLoop() {
    static constexpr std::chrono::microseconds TARGET_PACING_US(1000);
    static constexpr std::chrono::microseconds MIN_SILENCE_US(500);

    while (tx_running.load(std::memory_order_relaxed)) {
        std::vector<uint8_t> frame;
        {
            std::unique_lock<std::mutex> lock(tx_mutex);
            tx_cv.wait(lock, [this]() {
                return !tx_queue.empty() || !tx_running.load(std::memory_order_relaxed);
            });
            
            if (!tx_running.load(std::memory_order_relaxed) && tx_queue.empty()) {
                break;
            }
            
            frame = std::move(tx_queue.front());
            tx_queue.pop();
        }

        auto start = std::chrono::high_resolution_clock::now();

        if (is_connected) {
            serial->writeBytes(frame);
        }

        auto post_write = std::chrono::high_resolution_clock::now();

        // Calcular el tiempo objetivo considerando 1ms desde el inicio y 0.5ms desde el final de escritura
        auto target_time = std::max(start + TARGET_PACING_US, post_write + MIN_SILENCE_US);
        /*        
        // Usamos un spin-wait compatible con MSVC y MinGW GCC para evitar que el hilo ceda su turno (yield) y sea penalizado con 15ms de retraso.
        while (tx_running.load(std::memory_order_relaxed)) {
            auto elapsed_total = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now() - start).count();
            auto elapsed_silence = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now() - post_write).count();
            if (elapsed_total >= TARGET_PACING_US && elapsed_silence >= MIN_SILENCE_US) {
                break;
            }
            #if defined(_MSC_VER)
                _mm_pause(); // Intrínseco de MSVC
            #elif defined(__GNUC__) && (defined(__x86_64__) || defined(__i386__))
                __asm__ __volatile__("pause"); // Ensamblador en línea de GCC/MinGW
            #endif
        } */

        // Suspender el hilo pasivamente (liberando CPU) hasta la marca de tiempo calculada
        if (tx_running.load(std::memory_order_relaxed)) {
            std::this_thread::sleep_until(target_time);
        }
    }
}
    // feet_captured_ = false;  //esta variable la utilizarÃ© para cuando hago lo de la cinemÃ¡tica inversa// feet_captured_ = false;  //esta variable la utilizarÃ© para cuando hago lo de la cinemÃ¡tica inversa
#include "Tarantula.h"
#include "Config.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

#ifdef _WIN32
#include <windows.h>
#endif

static const std::vector<uint32_t> MOTOR_IDS = {
    11, 12, 13,   // Pata 1
    21, 22, 23,   // Pata 2
    31, 32, 33,   // Pata 3
    41, 42, 43    // Pata 4
};

Tarantula::Tarantula(WaveshareInterface& comm)
    : comm_(comm)
{
    createLegs();
}

Tarantula::~Tarantula()
{
    stop();
    for (Leg* leg : legs_) delete leg;
}

void Tarantula::createLegs()
{
    legs_[0] = new Leg(1,  M_PI / 4.0, this);
    legs_[1] = new Leg(2, -M_PI / 4.0, this);
    legs_[2] = new Leg(3, -3.0 * M_PI / 4.0, this);
    legs_[3] = new Leg(4,  3.0 * M_PI / 4.0, this);
}

void Tarantula::start()
{
    if (running_.load()) return;
    running_.store(true);

    rx_thread_      = std::thread(&Tarantula::rxLoop,      this);
    control_thread_ = std::thread(&Tarantula::controlLoop, this);
}

void Tarantula::stop()
{
    sequence_active_.store(false);
    running_.store(false);

    if (rx_thread_.joinable())      rx_thread_.join();
    if (control_thread_.joinable()) control_thread_.join();

    for (Leg* leg : legs_) {
        if (leg) leg->sendIdleToAllMotors();
    }
}

static const std::vector<uint32_t> ALLOWED_IDS = { 11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43 };

void Tarantula::enableAllLegs()
{
    // Ejecutamos el encendido en un hilo separado para no bloquear la interfaz gráfica
    std::thread boot_thread([this]() {
        for (Leg* leg : legs_) {
            if (leg) leg->enable();
        }
    });
    boot_thread.detach();
}

void Tarantula::disableAllLegs()
{
    abortSequence();
    std::thread shutdown_thread([this]() {
        for (Leg* leg : legs_) {
            if (leg) leg->disable();
        }
    });
    shutdown_thread.detach();
}

bool Tarantula::moveLeg(int leg_id, double x, double y, double z)
{
    if (leg_id < 1 || leg_id > 4) return false;
    return legs_[leg_id - 1]->goToPosition(x, y, z);
}

void Tarantula::moveLegJoint(int leg_id, int joint, float pos_deg, int stiffness)
{
    if (leg_id < 1 || leg_id > 4) return;
    legs_[leg_id - 1]->moveJoint(joint, pos_deg, stiffness);
}

bool Tarantula::setBodyPose(double dx, double dy, double dz,
                            double roll, double pitch, double yaw)
{
    if (!feet_captured_) {
        captureFeetPositions();
    }

    // Comprobar si la postura propuesta es realizable para todas las patas
    std::array<Eigen::Vector3d, 4> targets;
    for (int i = 0; i < 4; ++i) {
        targets[i] = legs_[i]->computeFootTarget(dx, dy, dz, roll, pitch, yaw);
        if (!legs_[i]->isLocalPositionIKValid(targets[i], true)) {
            return false; // Abortar si alguna pata no puede alcanzar la posición o colisiona
        }
    }

    // Aplicar movimiento
    int s1 = 3, s2 = 4, s3 = 4;
    for (int i = 0; i < 4; ++i) {
        legs_[i]->goToPosition(targets[i].x(), targets[i].y(), targets[i].z(), s1, s2, s3);
    }
    return true;
}

void Tarantula::resetBodyPoseReference()
{
    feet_captured_ = false;
}

void Tarantula::captureFeetPositions()
{
    for (int i = 0; i < 4; ++i) {
        legs_[i]->captureInitialFootPosition();
    }
    feet_captured_ = true;
}



void Tarantula::standUp()
{
    abortSequence();
    sequence_active_.store(true);
    std::thread(&Tarantula::runStandUpSequence, this).detach();
}

void Tarantula::sitDown()
{
    abortSequence();
    sequence_active_.store(true);
    std::thread(&Tarantula::runSitDownSequence, this).detach();
}

void Tarantula::abortSequence()
{
    sequence_active_.store(false);
}

// src/Tarantula.cpp

void Tarantula::runStandUpSequence()
{
    // Guardamos si el robot ya estaba levantado antes de empezar
    bool already_standing = feet_captured_;
    feet_captured_ = false; // Desactivamos temporalmente el control de cuerpo (sliders/caminar)

    // Solo hacemos el Paso 1 (bajar al suelo) si venimos de estar acostados o apagados.
    // Si solo estamos "reseteando la pose", nos saltamos esto para no tirarnos al suelo.
    if (!already_standing) {
        // Paso 1: Mover los pies a la huella X,Y de "pie", pero con Z = 0 
        for (Leg* leg : legs_) {
            leg->captureInitialFootPosition();
            Eigen::Vector3d target_body = leg->getInitialFootPosition();
            target_body.z() = 0.0;
            leg->goToBodyPosition(target_body, 3, 5, 4);
        }

        // Esperar de forma síncrona hasta que las patas alcancen esta posición intermedia
        for (Leg* leg : legs_) {
            leg->waitUntilSettled(sequence_active_);
        }
    }

    // Paso 2: Elevar el cuerpo (o simplemente suavizar la vuelta a la postura base)
    for (Leg* leg : legs_) {
        leg->moveJoint(1, 0.0f, 3);
        leg->moveJoint(2, 20.0f, 5);
        leg->moveJoint(3, -100.0f, 4);
    }

    // Esperar de forma síncrona hasta que se complete el movimiento
    for (Leg* leg : legs_) {
        leg->waitUntilSettled(sequence_active_);
    }

    // Confirmamos que el robot está de pie y listo
    if (sequence_active_.load()) {
        captureFeetPositions();
    }

    sequence_active_.store(false);
}

void Tarantula::runSitDownSequence()
{
    feet_captured_ = false; // Indicamos que ya no estamos de pie

    // Paso 1: Bajar el cuerpo hasta el suelo manteniendo los pies en el sitio (Z = 0)
    for (Leg* leg : legs_) {
        leg->captureInitialFootPosition();
        Eigen::Vector3d target_body = leg->getInitialFootPosition();
        target_body.z() = 0.0;
        leg->goToBodyPosition(target_body, 3, 4, 4);
    }

    for (Leg* leg : legs_) {
        leg->waitUntilSettled(sequence_active_);
    }

    // Paso 2: Estirar las patas hasta la posición completamente plana y horizontal
    for (Leg* leg : legs_) {
        leg->moveJoint(1, 0.0f, 3);
        leg->moveJoint(2, 0.0f, 4);
        leg->moveJoint(3, 0.0f, 4);
    }
    
    for (Leg* leg : legs_) {
        leg->waitUntilSettled(sequence_active_);
    }

    sequence_active_.store(false);
}

void Tarantula::sendJointToAllLegsAndWait(int joint, float target_deg, int stiffness)
{
    if (!sequence_active_.load()) return;

    for (Leg* leg : legs_)
        leg->moveJoint(joint, target_deg, stiffness);

    for (Leg* leg : legs_)
        leg->waitUntilSettled(sequence_active_);
}


Leg& Tarantula::getLeg(int leg_id)
{
    return *legs_[leg_id - 1];
}

bool Tarantula::isAnyMotorOnline() const
{
    for (Leg* leg : legs_) {
        if (leg && leg->isAnyMotorOnline()) return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────
//  HILO RX — lectura de tramas CAN
// ─────────────────────────────────────────────────────────────

void Tarantula::rxLoop()
{
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
#endif

    uint32_t               can_id;
    std::vector<uint8_t>   data;

    while (running_.load(std::memory_order_relaxed)) {
        processIncomingFrames(can_id, data);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

void Tarantula::processIncomingFrames(uint32_t& can_id, std::vector<uint8_t>& data)
{
    while (comm_.receive_can_frame(can_id, data)) {
        int leg_id = -1;
        if (can_id >= 1 && can_id <= 4) {
            leg_id = can_id;
        } else {
            uint8_t motor_id = can_id >> 5;
            leg_id = motor_id / 10;
        }

        if (leg_id >= 1 && leg_id <= 4) {
            legs_[leg_id - 1]->handleCanFrame(can_id, data);
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  HILO DE CONTROL — lazo a 100 Hz
// ─────────────────────────────────────────────────────────────

void Tarantula::controlLoop()
{
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
#endif

    auto     last_tick   = std::chrono::steady_clock::now();
    uint64_t cycle_count = 0;

    while (running_.load(std::memory_order_relaxed)) {
        if (isTimeForNextTick(last_tick)) {
            ++cycle_count;
            tickAllLegs(cycle_count);
            last_tick = std::chrono::steady_clock::now();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(3)); // Duerme 1 ms (gracias a la resolución forzada a 1 ms por timeBeginPeriod), evitando el 100% de uso de CPU
    }
}

bool Tarantula::isTimeForNextTick(const std::chrono::steady_clock::time_point& last) const
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - last).count() >= SEND_FREQUENCY;
}

void Tarantula::tickAllLegs(uint64_t cycle_count)
{
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    // Actualizar el control del trote diagonal en tiempo real únicamente si el robot está levantado (pies capturados)
    if (feet_captured_) {
        gait_controller_.tick(legs_, now_ms);
    }

    for (Leg* leg : legs_) {
        if (leg) leg->tick(now_ms, cycle_count);
    }
}

void Tarantula::setGaitVelocity(float vx, float vy)
{
    gait_controller_.setVelocity(vx, vy);
}

void Tarantula::setGaitType(int type)
{
    gait_controller_.setGaitType(static_cast<GaitController::GaitType>(type));
}

void Tarantula::startGait()
{
    abortSequence(); // Abortar cualquier secuencia de levantarse/acostarse activa
    gait_controller_.start();
}

void Tarantula::stopGait()
{
    gait_controller_.stop();
}

bool Tarantula::isGaitActive() const
{
    return gait_controller_.isActive();
}





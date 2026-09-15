#include "MotorController.h"
#include <cstring>
#include <thread>
#include <chrono>
#include <algorithm>
#include <iostream>

WaveshareInterface* g_comm = nullptr;
MotorController* g_controllers[128] = { nullptr };

MotorController::MotorController(uint8_t id, bool reversed) : node_id(id), reversed(reversed) {
    memset(&motorData, 0, sizeof(MW_MOTOR_DATA));

    MW_MOTOR_ACCESS_INFO access_info;
    access_info.busId = 0;
    access_info.nodeId = node_id;
    access_info.motorData = &motorData;

    access_info.sender = [](uint8_t busId, uint32_t canId, uint8_t* data, uint8_t dataSize) {
        if (g_comm) {
            std::vector<uint8_t> vec(data, data + dataSize);
            g_comm->send_can_frame(canId, vec);
        }
    };

    access_info.notifier = [](uint8_t, uint8_t, MW_CMD_ID) {};

    MWRegisterMotor(access_info);
    g_controllers[node_id] = this;
}

MotorController::~MotorController() { g_controllers[node_id] = nullptr; }

// ─────────────────────────────────────────────────────────────
// CONVERSIÓN DE COORDENADAS
// ─────────────────────────────────────────────────────────────

float MotorController::convert_dir(float val) const {
    return reversed ? -val : val;
}

float MotorController::phys_to_logic(float phys_pos) const {
    return reversed ? -phys_pos : phys_pos;
}

float MotorController::logic_to_phys(float log_pos) const {
    return reversed ? -log_pos : log_pos;
}

// ─────────────────────────────────────────────────────────────
// FUNCIONES PARA HILO RX
// ─────────────────────────────────────────────────────────────

void MotorController::update_from_mit_frame() {
    float phys_pos    = motorData.motorMIT.pos    / (float)GEAR_RATIO;
    float phys_vel    = motorData.motorMIT.vel    / (float)GEAR_RATIO;
    float phys_torque = motorData.motorMIT.torque;

    float abs_log_pos = phys_to_logic(phys_pos);
    last_known_pos.store(abs_log_pos - pos_offset.load(std::memory_order_relaxed),
                         std::memory_order_relaxed);
    last_known_vel.store(convert_dir(phys_vel), std::memory_order_relaxed);
    last_known_torque.store(convert_dir(phys_torque), std::memory_order_relaxed);
}

void MotorController::update_from_encoder_frame() {
    float output_revs = motorData.encoderEstimates.encoderPosEstimate / (float)GEAR_RATIO;
    float phys_rads   = output_revs * 2.0f * (float)M_PI;
    float abs_log_pos = phys_to_logic(phys_rads);
    last_known_pos.store(abs_log_pos - pos_offset.load(std::memory_order_relaxed),
                         std::memory_order_relaxed);
}

// ─────────────────────────────────────────────────────────────
// FUNCIONES PARA HILO DE CONTROL
// ─────────────────────────────────────────────────────────────

bool MotorController::check_timeout(int64_t now_ms, int64_t timeout_ms) {
    if (!motor_online.load(std::memory_order_relaxed)) return false;

    uint64_t last_msg = last_msg_time_ms.load(std::memory_order_relaxed);
    if (now_ms > (int64_t)last_msg && (now_ms - (int64_t)last_msg) > timeout_ms) {
        motor_online.store(false, std::memory_order_relaxed);
        active.store(false, std::memory_order_relaxed);
        std::cout << "[MotorController] ¡Timeout! El motor " << (int)node_id
                  << " no ha respondido en " << (now_ms - (int64_t)last_msg) << " ms (limite: " 
                  << timeout_ms << " ms). Desactivando y pasando a IDLE.\n";
        return true;
    }
    return false;
}

// ✅ Firma correcta: recibe now_ms como parámetro
MW_MIT_CTRL MotorController::step_trajectory(int64_t now_ms)
{
    float current_p = current_traj_pos.load(std::memory_order_relaxed);
    float current_v = current_traj_vel.load(std::memory_order_relaxed);
    float target_p  = target_pos.load(std::memory_order_relaxed);
    float max_v     = traj_max_vel.load(std::memory_order_relaxed);
    float max_a     = traj_max_accel.load(std::memory_order_relaxed);

    // ✅ dt real medido, con clamp de seguridad
    int64_t last = last_tick_time_ms.load(std::memory_order_relaxed);  // ← nombre correcto
    float dt = (last == 0)                                              // ← "last" minúscula, coherente
               ? (float)SEND_FREQUENCY
               : std::clamp((float)(now_ms - last) * 0.001f, 0.001f, 0.05f);
    last_tick_time_ms.store(now_ms, std::memory_order_relaxed);

    float error = target_p - current_p;
    float dir   = (error > 0.0f) ? 1.0f : -1.0f;

    if (std::abs(error) < 0.005f && std::abs(current_v) < 0.01f) {
        current_p = target_p;
        current_v = 0.0f;
    } else {
        float v_brake   = std::sqrt(2.0f * max_a * std::abs(error));
        float v_desired = dir * std::min(max_v, v_brake);

        if      (current_v < v_desired) { current_v += max_a * dt; if (current_v > v_desired) current_v = v_desired; }
        else if (current_v > v_desired) { current_v -= max_a * dt; if (current_v < v_desired) current_v = v_desired; }

        current_p += current_v * dt;
    }

    current_traj_pos.store(current_p, std::memory_order_relaxed);
    current_traj_vel.store(current_v, std::memory_order_relaxed);

    float abs_p = current_p + pos_offset.load(std::memory_order_relaxed);

    MW_MIT_CTRL mit;
    mit.pos    = logic_to_phys(abs_p);
    mit.vel    = convert_dir(current_v);
    mit.kp     = target_kp.load(std::memory_order_relaxed);
    mit.kd     = target_kd.load(std::memory_order_relaxed);
    mit.torque = convert_dir(target_torque.load(std::memory_order_relaxed));
    return mit;
}

MW_MIT_CTRL MotorController::build_static_mit() {
    MW_MIT_CTRL mit;
    float abs_p = target_pos.load(std::memory_order_relaxed)
                + pos_offset.load(std::memory_order_relaxed);
    mit.pos    = logic_to_phys(abs_p);
    mit.vel    = convert_dir(target_vel.load(std::memory_order_relaxed));
    mit.kp     = target_kp.load(std::memory_order_relaxed);
    mit.kd     = target_kd.load(std::memory_order_relaxed);
    mit.torque = convert_dir(target_torque.load(std::memory_order_relaxed));
    return mit;
}

void MotorController::enableSafely(int stiffness)
{
    if (!motor_online.load(std::memory_order_relaxed)) {
        std::cout << "[MotorController] Error: Motor " << (int)node_id << " offline. No se puede encender.\n";
        return;
    }

    int max_retries = 3;
    bool success = false;

    for (int attempt = 1; attempt <= max_retries; ++attempt) {
        
        MWClearErrors(0, node_id); //Limpiar errores previos
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        MWSetAxisState(0, node_id, MW_AXIS_STATE_CLOSED_LOOP_CONTROL); //Enviar orden de encendido
        
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); //Esperar a que el motor responda con su Heartbeat

        //Comprobar estado
        if (!motor_online.load(std::memory_order_relaxed)) {
            std::cout << "[MotorController] Error: Motor " << (int)node_id << " se ha desconectado.\n";
            return;
        }

        if (motorData.heartBeat.ErrorStatus.axisError != 0) {
            std::cout << "[MotorController] Error hardware en motor " << (int)node_id 
                      << " (axisError: " << motorData.heartBeat.ErrorStatus.axisError << "). Reintentando...\n";
            continue; // Reintentar limpiar errores y encender
        }

        if (motorData.heartBeat.currentState == 8) { // MW_AXIS_STATE_CLOSED_LOOP_CONTROL
            success = true;
            break;
        } else {
            std::cout << "[MotorController] Advertencia: Trama de encendido perdida para motor " << (int)node_id 
                      << " (estado: " << (int)motorData.heartBeat.currentState << "). Reintentando (" << attempt << "/" << max_retries << ")...\n";
        }
    }

    if (!success) {
        std::cout << "[MotorController] Fallo critico: No se pudo encender el motor " << (int)node_id << " tras " << max_retries << " intentos.\n";
        active.store(false);
        return;
    }

    // Si llegamos aqui, el motor se encendio con exito
    target_kp.store(0.0f);
    target_kd.store(0.0f);
    active.store(true);

    pos_offset.store(last_known_pos.load() + pos_offset.load());
    target_pos.store(0.0f);
    current_traj_pos.store(0.0f);
    current_traj_vel.store(0.0f);                              // ✅ reset velocidad
    last_tick_time_ms.store(0, std::memory_order_relaxed);     // ✅ reset dt
    filtered_pos.store(pos_offset.load(std::memory_order_relaxed), std::memory_order_relaxed); // ✅ Inicializar con el offset real para evitar un salto brusco

    applyStiffness(stiffness);
}

void MotorController::applyStiffness(int stiffness)
{
    auto [kp, kd] = stiffnessToGains(stiffness);
    target_kp.store(kp);
    target_kd.store(kd);
}

std::pair<float, float> MotorController::stiffnessToGains(int stiffness)
{
    switch (stiffness) {

    case 1:  return {  5.0f, 0.5f };
    case 2:  return { 10.0f, 0.8f };
    case 3:  return { 15.0f, 1.2f };
    case 4:  return { 25.0f, 1.6f };
    case 5:  return { 35.0f, 1.8f };
    default: return { 15.0f, 1.2f };   // ✅ fallback equilibrado
    }
}

bool MotorController::isSettled(float tolerance_deg) const
{
    if (!motor_online.load(std::memory_order_relaxed) ||
        !active.load(std::memory_order_relaxed)) {
        return true;
    }
    float current_deg = last_known_pos.load(std::memory_order_relaxed)
                      * static_cast<float>(180.0 / M_PI);
    float target_deg  = target_pos.load(std::memory_order_relaxed)
                      * static_cast<float>(180.0 / M_PI);
    return std::abs(current_deg - target_deg) <= tolerance_deg;
}

void MotorController::setTarget(float pos_rad, int stiffness)
{
    auto [kp, kd] = stiffnessToGains(stiffness);
    target_pos.store(pos_rad);
    target_kp.store(kp);
    target_kd.store(kd);
    is_trap_traj.store(true);
}

void MotorController::setTargetDirect(float pos_rad, int stiffness)
{
    auto [kp, kd] = stiffnessToGains(stiffness);
    target_pos.store(pos_rad);
    current_traj_pos.store(pos_rad);
    current_traj_vel.store(0.0f);
    target_kp.store(kp);
    target_kd.store(kd);
    is_trap_traj.store(false);
}

void MotorController::tick(int64_t now_ms, uint64_t cycle_count)
{
    check_timeout(now_ms);

    bool sent = false;

    if (!motor_online.load(std::memory_order_relaxed)) {
        if (cycle_count % 50 == (node_id % 50)) {
            MWGetEncoderEstimates(0, node_id);
            sent = true;
        }
    } else if (active.load(std::memory_order_relaxed)) {
        MW_MIT_CTRL mit = is_trap_traj.load(std::memory_order_relaxed)
                        ? step_trajectory(now_ms)   // ✅ pasa now_ms
                        : build_static_mit();
        MWMitControl(0, node_id, &mit);
        sent = true;
    } else {
        // Si el motor está online pero inactivo (pierna en OFF), limitamos la consulta 
        // del encoder a 10 Hz (cada 10 ciclos) para liberar el bus y ahorrar sleeps.
        if (cycle_count % 10 == (node_id % 10)) {
            MWGetEncoderEstimates(0, node_id);
            sent = true;
        }
    }

}

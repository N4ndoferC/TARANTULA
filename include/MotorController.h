#pragma once
#include <atomic>
#include <cmath>
#include <chrono>
#include "SteadyWinMotor.h"
#include <vector>
#include "WaveshareInterface.h"

extern const float GEAR_RATIO;
extern const double SEND_FREQUENCY;
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

extern WaveshareInterface* g_comm;
extern class MotorController* g_controllers[128];

class MotorController {
public:
    uint8_t node_id;
    bool reversed;
    MW_MOTOR_DATA motorData;

    std::atomic<float> target_pos{ 0.0f };
    std::atomic<float> target_vel{ 0.0f };
    std::atomic<float> target_kp{ 0.0f };
    std::atomic<float> target_kd{ 0.5f };
    std::atomic<float> target_torque{ 0.0f };

    std::atomic<float> current_traj_pos{ 0.0f };
    std::atomic<float> current_traj_vel{ 0.0f };
    std::atomic<float> traj_max_vel{ 1.0f };
    std::atomic<float> traj_max_accel{ 4.0f };

    std::atomic<bool> active{ false };
    std::atomic<bool> is_trap_traj{ false };
    std::atomic<bool> motor_online{ false };
    std::atomic<uint64_t> last_msg_time_ms{ 0 };
    std::atomic<int64_t>  last_tick_time_ms{ 0 };  

    std::atomic<float> last_known_pos{ 0.0f };
    std::atomic<float> last_known_vel{ 0.0f };
    std::atomic<float> last_known_torque{ 0.0f };
    std::atomic<float> pos_offset{ 0.0f };
    std::atomic<float> filtered_pos{ 0.0f };       

    MotorController(uint8_t id, bool reversed = false);
    ~MotorController();

    float convert_dir(float val) const;
    float phys_to_logic(float phys_pos) const;
    float logic_to_phys(float log_pos) const;

    void update_from_mit_frame();
    void update_from_encoder_frame();

    bool check_timeout(int64_t now_ms, int64_t timeout_ms = 4000);
    MW_MIT_CTRL step_trajectory(int64_t now_ms);    
    void enableSafely(int stiffness);
    void disableSafely();
    void applyStiffness(int stiffness);
    static std::pair<float, float> stiffnessToGains(int stiffness);
    bool isSettled(float tolerance_deg) const;
    void setTarget(float pos_rad, int stiffness);
    void setTargetDirect(float pos_rad, int stiffness);
    MW_MIT_CTRL build_static_mit();
    void tick(int64_t now_ms, uint64_t cycle_count);
};
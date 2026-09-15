#pragma once
#include <array>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include "Leg.h"
#include "WaveshareInterface.h"
#include "GaitController.h"
#include <Eigen/Dense>

extern const double SEND_FREQUENCY;


class Tarantula {
public:
    explicit Tarantula(WaveshareInterface& comm);
    ~Tarantula();

   
    void start();           
    void stop();            
    void enableAllLegs();   
    void disableAllLegs();  

    bool moveLeg(int leg_id, double x, double y, double z); 
    void moveLegJoint(int leg_id, int joint, float pos_deg, int stiffness);

    bool setBodyPose(double dx, double dy, double dz, double roll, double pitch, double yaw = 0.0);
    void resetBodyPoseReference();

    void standUp();
    void sitDown();
    void abortSequence();  

    void setGaitVelocity(float vx, float vy);
    void setGaitType(int type);
    void startGait();
    void stopGait();
    bool isGaitActive() const;

    Leg& getLeg(int leg_id);                 
    bool isAnyMotorOnline() const;
    bool isRunning() const { return running_.load(); }

private:
   
    WaveshareInterface& comm_;
    std::array<Leg*, 4> legs_;  
    bool feet_captured_{ false };
    
    std::atomic<bool> running_{ false };
    std::thread       rx_thread_;
    std::thread       control_thread_;
    std::atomic<bool> sequence_active_{ false };
    GaitController    gait_controller_;

    void rxLoop();
    void controlLoop();

   
    void createLegs();

    // Captura las posiciones actuales de los pies mediante FK
    void captureFeetPositions();

    void runStandUpSequence();
    void runSitDownSequence();
    void sendJointToAllLegsAndWait(int joint, float target_deg, int stiffness);

    // Sub-funciones del hilo RX
    void processIncomingFrames(uint32_t& can_id, std::vector<uint8_t>& data);

    // Sub-funciones del hilo de control
    bool isTimeForNextTick(const std::chrono::steady_clock::time_point& last) const;
    void tickAllLegs(uint64_t cycle_count);
};

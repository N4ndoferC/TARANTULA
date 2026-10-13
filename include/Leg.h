#pragma once
#include <cmath>
#include <utility>
#include <atomic>
#include <optional>
#include <vector>
#include "MotorController.h"

class Tarantula;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <Eigen/Dense>
#include <Eigen/Geometry>

struct JointAngles {
    double q1 = 0.0;   
    double q2 = 0.0;   
    double q3 = 0.0;   
    bool   valid = false;
};

class Leg {
public:
    
    static constexpr double R_HIP = 0.15;   
    static constexpr double L1 = 0.08;      
    static constexpr double L2 = 0.188;      
    static constexpr double L3 = 0.211;    

    Leg(int leg_id, double theta, Tarantula* parent = nullptr);
    ~Leg();

    void enable();          
    void disable();
    bool isEnabled() const;
    void enableJoint(int joint);
    void disableJoint(int joint);
    void tick(int64_t now_ms, uint64_t cycle_count);
    bool isAnyMotorOnline() const;
    void sendIdleToAllMotors();
    void handleCanFrame(uint32_t can_id, const std::vector<uint8_t>& data);

    
    bool goToPosition(double x, double y, double z, int stiffness_q1 = 3, int stiffness_q2 = 4, int stiffness_q3 = 4, bool knee_up = true, bool direct = false); //para cuando tenga la cinemática inversa
    bool goToBodyPosition(const Eigen::Vector3d& p_body, int stiffness_q1 = 3, int stiffness_q2 = 4, int stiffness_q3 = 4, bool knee_up = true, bool direct = false);
    void moveJoint(int joint, float pos_deg, int stiffness);
    void waitUntilSettled(const std::atomic<bool>& sequence_active,
                          float tolerance_deg = 10.0f) const;

    bool isIKPossible(double x, double y, double z, const JointAngles& angles, bool knee_up = true) const;
    bool isLocalPositionIKValid(const Eigen::Vector3d& p_local, bool knee_up = true) const;
    JointAngles getCurrentJointAngles() const;
    std::vector<Eigen::Vector3d> getJointPositionsInBodyFrame(const JointAngles& a) const;

   
    bool  isGrounded()             const;  // para cuando implemente los sensores de efecto Hall
    float getJointAngle(int joint) const;  
    Eigen::Vector3d getCurrentFootPosition() const; // con cinemática directa

    
    int id() const { return leg_id_; }

    
    void captureInitialFootPosition();
    Eigen::Vector3d getInitialFootPosition() const { return initial_foot_position_; }
    Eigen::Vector3d computeFootTarget(double dx, double dy, double dz, double roll, double pitch, double yaw = 0.0) const;

private:
    int              leg_id_;
    double           theta_;
    Eigen::Isometry3d leg_to_body_;
    Eigen::Vector3d   initial_foot_position_; 
    MotorController motor_[3];  // [0]=q1  [1]=q2  [2]=q3
    Tarantula*       parent_;

    std::optional<JointAngles> solveIK(double x, double y, double z, bool knee_up = true)            const;
    bool        isWithinReach(double s, double d)                               const;
    bool        isWithinJointLimits(const JointAngles& angles, bool knee_up = true)                  const;

    bool checkLegCollisions(const JointAngles& proposed_angles) const;
    double ClosestPtSegmentSegment(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                                   const Eigen::Vector3d& q1, const Eigen::Vector3d& q2,
                                   double &s, double &t, Eigen::Vector3d &c1, Eigen::Vector3d &c2) const;

    Eigen::Vector3d forwardKinematics(double q1, double q2, double q3)                     const;

    //aplica los ángulos a los motores
    void applyAngles(const JointAngles& angles, int stiffness_q1 = 3, int stiffness_q2 = 4, int stiffness_q3 = 4, bool direct = false);

    std::atomic<bool> last_command_was_ik_{false};
    std::atomic<float> last_known_hall_z_{0.0f};
};

#include "Leg.h"
#include "Tarantula.h"
#include <cstring>
#include <thread>
#include <chrono>
#include <algorithm>
#include <iostream>


Leg::Leg(int leg_id, double theta, Tarantula* parent)
    : leg_id_(leg_id)
    , theta_(theta)
    , leg_to_body_(Eigen::Isometry3d::Identity())
    , initial_foot_position_(Eigen::Vector3d::Zero())
    , motor_{
        {static_cast<uint8_t>(leg_id * 10 + 1), false},
        {static_cast<uint8_t>(leg_id * 10 + 2), true},  // Motor X2 (fémur) invertido mecánicamente
        {static_cast<uint8_t>(leg_id * 10 + 3), false}
      }
    , parent_(parent)
{
    double x_off = R_HIP * std::cos(theta_);
    double y_off = R_HIP * std::sin(theta_);
    
    // Matriz de transformación de pata a cuerpo:
    // Primero rotamos sobre el eje Z por theta, y luego trasladamos por (x_off, y_off, 0)
    leg_to_body_ = Eigen::Translation3d(x_off, y_off, 0.0) * Eigen::AngleAxisd(theta_, Eigen::Vector3d::UnitZ()); //(6.21)
}  

Leg::~Leg()
{
}




void Leg::enable()
{
    for (int j = 0; j < 3; ++j) {
        motor_[j].enableSafely(3);
    }
}

void Leg::disable()
{
    for (int j = 0; j < 3; ++j) {
        MWSetAxisState(0, motor_[j].node_id, MW_AXIS_STATE_IDLE);
        motor_[j].active.store(false);
        motor_[j].is_trap_traj.store(false);
    }
}

bool Leg::isEnabled() const
{
    // La pata se considera habilitada si al menos un motor está activo
    for (int j = 0; j < 3; ++j)
        if (motor_[j].active.load(std::memory_order_relaxed))
            return true;
    return false;
}

bool Leg::goToPosition(double x, double y, double z, int stiffness_q1, int stiffness_q2, int stiffness_q3, bool knee_up, bool direct)
{
    auto angles_opt = solveIK(x, y, z, knee_up);
    if (!angles_opt.has_value()) {
        std::cout << "⚠️ [Leg " << leg_id_ << "] solveIK fallo para local: (" << x << ", " << y << ", " << z << ")\n";
        return false;
    }
    applyAngles(angles_opt.value(), stiffness_q1, stiffness_q2, stiffness_q3, direct);
    last_command_was_ik_.store(true, std::memory_order_relaxed);
    return true;
}

bool Leg::goToBodyPosition(const Eigen::Vector3d& p_body, int stiffness_q1, int stiffness_q2, int stiffness_q3, bool knee_up, bool direct)
{
    Eigen::Vector3d p_local = leg_to_body_.inverse() * p_body;
    return goToPosition(p_local.x(), p_local.y(), p_local.z(), stiffness_q1, stiffness_q2, stiffness_q3, knee_up, direct);
}

void Leg::moveJoint(int joint, float pos_deg, int stiffness)
{
    if (joint < 1 || joint > 3) return;
    MotorController& m = motor_[joint - 1];

    m.setTarget(pos_deg * static_cast<float>(M_PI / 180.0), stiffness);
    last_command_was_ik_.store(false, std::memory_order_relaxed);
}

void Leg::waitUntilSettled(const std::atomic<bool>& sequence_active,
                           float tolerance_deg) const
{
    float active_tolerance = tolerance_deg;
    if (last_command_was_ik_.load(std::memory_order_relaxed)) {
        active_tolerance = std::min(tolerance_deg, 5.0f);
    }

    bool all_settled = false;
    while (!all_settled && sequence_active.load()) {
        all_settled = true;
        for (int j = 0; j < 3; ++j) {
            if (!motor_[j].isSettled(active_tolerance)) {
                all_settled = false;
                break;
            }
        }
        if (!all_settled)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (sequence_active.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
}


bool Leg::isGrounded() const
{
    if (leg_id_ == 1) {
        return last_known_hall_z_.load(std::memory_order_relaxed) < -6120.0f;
    }
    return false;
}

float Leg::getJointAngle(int joint) const
{
    if (joint < 1 || joint > 3) return 0.0f;
    return motor_[joint - 1].last_known_pos.load(std::memory_order_relaxed)
           * static_cast<float>(180.0 / M_PI);
}


Eigen::Vector3d Leg::getCurrentFootPosition() const
{
    double q1 = motor_[0].last_known_pos.load(std::memory_order_relaxed);
    double q2 = motor_[1].last_known_pos.load(std::memory_order_relaxed);
    double q3 = motor_[2].last_known_pos.load(std::memory_order_relaxed);
    return forwardKinematics(q1, q2, q3);
}

void Leg::captureInitialFootPosition()
{
    // Usamos los ángulos ideales de diseño (q1=0.0, q2=20.0°, q3=-100.0°) de la pose de pie
    // para asegurar que las referencias iniciales de los pies sean perfectamente simétricas y libres de ruido.
    double q1_ideal = 0.0;
    double q2_ideal = 20.0 * M_PI / 180.0;
    double q3_ideal = -100.0 * M_PI / 180.0;
    
    Eigen::Vector3d p_local = forwardKinematics(q1_ideal, q2_ideal, q3_ideal);
    initial_foot_position_ = leg_to_body_ * p_local;
}

Eigen::Vector3d Leg::computeFootTarget(double dx, double dy, double dz, double roll, double pitch, double yaw) const
{
    Eigen::Matrix3d R_body = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
    Eigen::Isometry3d body_transform = Eigen::Translation3d(dx, dy, dz) * Eigen::Isometry3d(R_body);
    Eigen::Vector3d p_body = body_transform.inverse() * initial_foot_position_;
    return leg_to_body_.inverse() * p_body;
}


std::optional<JointAngles> Leg::solveIK(double x, double y, double z, bool knee_up) const
{
    JointAngles angles;
    angles.valid = false;

    // Ángulo de q1 (yaw)
    double q1 = std::atan2(y, x); // (6.5)

    // Proyectamos a plano xy:
    double s = std::sqrt(x * x + y * y); // (6.6)
    double s_femur = s - L1; // (6.7)
    double d = z; //(6.8)

    if (!isWithinReach(s_femur, d)) {
        return std::nullopt;
    }

    double D2 = s_femur * s_femur + d * d; //(6.9)

    double cosq3 = (D2 - L2 * L2 - L3 * L3) / (2.0 * L2 * L3); //(6.11)
    if (cosq3 < -1.0001 || cosq3 > 1.0001) {
        return std::nullopt;
    }
    cosq3 = std::max(-1.0, std::min(1.0, cosq3)); // Clamp de seguridad por precisión de coma flotante
    
    double q3 = std::acos(cosq3); //(6.12)
    if (knee_up) {
        angles.q3 = -q3;
    } else {
        angles.q3 = q3;
    }

    // Obtenemos el ángulo q2 por la ley de los cosenos:
    double alpha1 = std::atan2(d, s_femur); //(6.13)
    double cosAlpha2 = (L2 * L2 + D2 - L3 * L3) / (2.0 * L2 * std::sqrt(D2)); //(6.14)
    if (cosAlpha2 < -1.0001 || cosAlpha2 > 1.0001) {
        return std::nullopt;
    }
    cosAlpha2 = std::max(-1.0, std::min(1.0, cosAlpha2)); // Clamp de seguridad por precisión de coma flotante
    double alpha2 = std::acos(cosAlpha2); //(6.15)

    if (knee_up) {
        angles.q2 = alpha1 + alpha2; //(6.16)
    } else {
        angles.q2 = alpha1 - alpha2;
    }
    angles.q1 = q1;
    
    // Mirar si es viable:
    if (isIKPossible(x, y, z, angles, knee_up)) {
        angles.valid = true;
        return angles;
    }

    return std::nullopt;
}




bool Leg::isWithinReach(double s, double d) const
{
    //(6.17)
    double D = std::sqrt(s * s + d * d);
    double max_reach = L2 + L3;
    double min_reach = std::abs(L2 - L3);
    return (D >= min_reach && D <= max_reach);
}

bool Leg::isWithinJointLimits(const JointAngles& a, bool knee_up) const
{
    //(6.18)
    double q1_deg = a.q1 * 180.0 / M_PI;
    double q2_deg = a.q2 * 180.0 / M_PI;
    double q3_deg = a.q3 * 180.0 / M_PI;

    // límites
    if (q1_deg < -90.0 || q1_deg > 90.0) return false;
    if (q2_deg < -100.0 || q2_deg > 100.0) return false;
    
    if (knee_up) {
        if (q3_deg < -150.0 || q3_deg > 30.0) return false; // rodilla arriba
    } else {
        if (q3_deg < -30.0 || q3_deg > 150.0) return false;  // rodilla abajo
    }

    return true;
}


Eigen::Vector3d Leg::forwardKinematics(double q1, double q2, double q3) const
{

    double r = L1 + L2 * std::cos(q2) + L3 * std::cos(q2 + q3); //(6.2)
    double x = r * std::cos(q1); //(6.4a)
    double y = r * std::sin(q1); //(6.4b)
    double z = L2 * std::sin(q2) + L3 * std::sin(q2 + q3); //(6.4c)

    return Eigen::Vector3d(x, y, z);
}



void Leg::applyAngles(const JointAngles& angles, int stiffness_q1, int stiffness_q2, int stiffness_q3, bool direct)
{
    double qs[3] = { angles.q1, angles.q2, angles.q3 };
    int stiffnesses[3] = { stiffness_q1, stiffness_q2, stiffness_q3 };

    for (int i = 0; i < 3; ++i) {
        if (motor_[i].active.load(std::memory_order_relaxed)) {
            if (direct) {
                motor_[i].setTargetDirect(static_cast<float>(qs[i]), stiffnesses[i]);
            } else {
                motor_[i].setTarget(static_cast<float>(qs[i]), stiffnesses[i]);
            }
        }
    }
}

void Leg::enableJoint(int joint)
{
    if (joint >= 1 && joint <= 3) {
        motor_[joint - 1].enableSafely(3);
    }
}

void Leg::tick(int64_t now_ms, uint64_t cycle_count)
{
    for (int j = 0; j < 3; ++j) {
        motor_[j].tick(now_ms, cycle_count);
    }
}

bool Leg::isAnyMotorOnline() const
{
    for (int j = 0; j < 3; ++j) {
        if (motor_[j].motor_online.load(std::memory_order_relaxed))
            return true;
    }
    return false;
}

void Leg::sendIdleToAllMotors()
{
    for (int j = 0; j < 3; ++j) {
        MWSetAxisState(0, motor_[j].node_id, MW_AXIS_STATE_IDLE);
    }
}

void Leg::handleCanFrame(uint32_t can_id, const std::vector<uint8_t>& data)
{
    if (can_id >= 1 && can_id <= 4) {
        if (data.size() == 4) {
            float z_val = 0.0f;
            std::memcpy(&z_val, data.data(), 4);
            last_known_hall_z_.store(z_val, std::memory_order_relaxed);
        }
        return;
    }

    uint8_t motor_id = can_id >> 5;
    int joint_idx = (motor_id % 10) - 1;
    if (joint_idx >= 0 && joint_idx < 3) {
        MotorController& ctrl = motor_[joint_idx];
        ctrl.motor_online.store(true, std::memory_order_relaxed);
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        ctrl.last_msg_time_ms.store(now_ms, std::memory_order_relaxed);

        if (data.size() == 8) {
            MWReceiver(0, can_id, const_cast<uint8_t*>(data.data()));
            MW_CMD_ID cmd = static_cast<MW_CMD_ID>(can_id & 0x1F);
            if (cmd == MW_MIT_CONTROL_CMD) {
                ctrl.update_from_mit_frame();
            } else if (cmd == MW_GET_ENCODER_ESTIMATES_CMD) {
                ctrl.update_from_encoder_frame();
            }
        }
    }
}

bool Leg::isIKPossible(double x, double y, double z, const JointAngles& angles, bool knee_up) const
{
    double s = std::sqrt(x * x + y * y);
    double s_femur = s - L1;
    double d = z;

    if (!isWithinReach(s_femur, d)) return false;
    if (!isWithinJointLimits(angles, knee_up)) return false;
    if (checkLegCollisions(angles)) return false;

    return true;
}

bool Leg::isLocalPositionIKValid(const Eigen::Vector3d& p_local, bool knee_up) const
{
    return solveIK(p_local.x(), p_local.y(), p_local.z(), knee_up).has_value();
}

JointAngles Leg::getCurrentJointAngles() const
{
    JointAngles a;
    a.q1 = motor_[0].last_known_pos.load(std::memory_order_relaxed);
    a.q2 = motor_[1].last_known_pos.load(std::memory_order_relaxed);
    a.q3 = motor_[2].last_known_pos.load(std::memory_order_relaxed);
    a.valid = true;
    return a;
}

std::vector<Eigen::Vector3d> Leg::getJointPositionsInBodyFrame(const JointAngles& a) const
{
    Eigen::Vector3d j0_local(0.0, 0.0, 0.0);
    Eigen::Vector3d j1_local(L1 * std::cos(a.q1), L1 * std::sin(a.q1), 0.0);
    double r2 = L1 + L2 * std::cos(a.q2);
    Eigen::Vector3d j2_local(r2 * std::cos(a.q1), r2 * std::sin(a.q1), L2 * std::sin(a.q2));
    double r3 = L1 + L2 * std::cos(a.q2) + L3 * std::cos(a.q2 + a.q3);
    Eigen::Vector3d j3_local(r3 * std::cos(a.q1), r3 * std::sin(a.q1), L2 * std::sin(a.q2) + L3 * std::sin(a.q2 + a.q3));

    std::vector<Eigen::Vector3d> J(4);
    J[0] = leg_to_body_ * j0_local;
    J[1] = leg_to_body_ * j1_local;
    J[2] = leg_to_body_ * j2_local;
    J[3] = leg_to_body_ * j3_local;
    return J;
}

bool Leg::checkLegCollisions(const JointAngles& proposed_angles) const
{
    if (!parent_) return false;

    std::vector<Eigen::Vector3d> J_curr = getJointPositionsInBodyFrame(proposed_angles);
    double min_dist_sq = 0.03 * 0.03; // 3 cm threshold

    for (int l = 1; l <= 4; ++l) {
        if (l == leg_id_) continue;

        Leg& other = parent_->getLeg(l);
        if (!other.isAnyMotorOnline()) continue;

        JointAngles other_angles = other.getCurrentJointAngles();
        std::vector<Eigen::Vector3d> J_other = other.getJointPositionsInBodyFrame(other_angles);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                double s, t;
                Eigen::Vector3d c1, c2;
                double dist_sq = ClosestPtSegmentSegment(
                    J_curr[i], J_curr[i+1],
                    J_other[j], J_other[j+1],
                    s, t, c1, c2
                );

                if (dist_sq < min_dist_sq) {
                    return true;
                }
            }
        }
    }
    return false;
}

double Leg::ClosestPtSegmentSegment(
    const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
    const Eigen::Vector3d& q1, const Eigen::Vector3d& q2,
    double &s, double &t, Eigen::Vector3d &c1, Eigen::Vector3d &c2) const
{
    Eigen::Vector3d d1 = p2 - p1;
    Eigen::Vector3d d2 = q2 - q1;
    Eigen::Vector3d r = p1 - q1;
    double a = d1.dot(d1);
    double e = d2.dot(d2);
    double f = d2.dot(r);

    if (a <= 1e-8 && e <= 1e-8) {
        s = 0.0;
        t = 0.0;
        c1 = p1;
        c2 = q1;
        return (c1 - c2).squaredNorm();
    }
    if (a <= 1e-8) {
        s = 0.0;
        t = std::clamp(f / e, 0.0, 1.0);
    } else {
        double c = d1.dot(r);
        if (e <= 1e-8) {
            t = 0.0;
            s = std::clamp(-c / a, 0.0, 1.0);
        } else {
            double b = d1.dot(d2);
            double denom = a * e - b * b;

            if (denom != 0.0) {
                s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
            } else {
                s = 0.0;
            }

            t = (b * s + f) / e;

            if (t < 0.0) {
                t = 0.0;
                s = std::clamp(-c / a, 0.0, 1.0);
            } else if (t > 1.0) {
                t = 1.0;
                s = std::clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }

    c1 = p1 + s * d1;
    c2 = q1 + t * d2;
    return (c1 - c2).squaredNorm();
}


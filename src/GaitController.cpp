#include "GaitController.h"
#include "Leg.h"
#include <cmath>
#include <algorithm>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GaitController::GaitController()
    : active_(false)
    , target_vx_(0.0f)
    , target_vy_(0.0f)
    , phase_(0.0)
    , last_time_ms_(0)
{
}

GaitController::~GaitController()
{
}

void GaitController::start()
{
    active_.store(true);
}

void GaitController::stop()
{
    active_.store(false);
    target_vx_.store(0.0f);
    target_vy_.store(0.0f);
}

bool GaitController::isActive() const
{
    return active_.load();
}

void GaitController::setVelocity(float vx, float vy)
{
    target_vx_.store(vx);
    target_vy_.store(vy);
}

void GaitController::setGaitType(GaitType type)
{
    gait_type_.store(type);
}

GaitController::GaitType GaitController::getGaitType() const
{
    return gait_type_.load();
}

void GaitController::tick(std::array<Leg*, 4>& legs, int64_t now_ms)
{
    if (last_time_ms_ == 0) {
        last_time_ms_ = now_ms;
        return;
    }

    double dt = (now_ms - last_time_ms_) * 0.001;
    last_time_ms_ = now_ms;

    float vx = target_vx_.load();
    float vy = target_vy_.load();

    // Clampear la velocidad de consigna para evitar superar los límites cinemáticos (alcance de las patas)
    // El alcance máximo es ~41 cm. Un paso máximo seguro de 8 cm (velocidad 0.08 m/s con período de 1.0s) es muy adecuado.
    static constexpr float MAX_SPEED = 0.15f; // m/s
    vx = std::clamp(vx, -MAX_SPEED, MAX_SPEED);
    vy = std::clamp(vy, -MAX_SPEED, MAX_SPEED);

    bool walking = (std::abs(vx) > 0.001f || std::abs(vy) > 0.001f);

    // Si no está activo y la velocidad es 0, y ya estamos parados en pose neutra (fase < epsilon), no hacemos nada.
    if (!active_.load() && !walking && std::abs(phase_) < 1e-5) {
        return;
    }



    // Avanzar la fase de la marcha
    double omega = (2.0 * M_PI) / GAIT_PERIOD;
    phase_ += omega * dt;
    if (phase_ >= 2.0 * M_PI) {
        phase_ = std::fmod(phase_, 2.0 * M_PI);
    }

    GaitType current_gait = gait_type_.load();

    // Rampa suave del factor de balanceo (sway_factor_) para evitar sacudidas
    double target_sway = (active_.load() && walking) ? 1.0 : 0.0;
    if (sway_factor_ < target_sway) {
        sway_factor_ = std::min(target_sway, sway_factor_ + dt * SWAY_RAMP_RATE);
    } else if (sway_factor_ > target_sway) {
        sway_factor_ = std::max(target_sway, sway_factor_ - dt * SWAY_RAMP_RATE);
    }

    // Calcular el balanceo activo del cuerpo (Body Sway) en función de la fase (solo para creep)
    double dx_sway = 0.0;
    double dy_sway = 0.0;
    if (current_gait == GaitType::CREEP) {
        dx_sway = -SWAY_AMPLITUDE_X * sway_factor_ * std::sin(2.0 * phase_);
        dy_sway = -SWAY_AMPLITUDE_Y * sway_factor_ * std::cos(phase_);
    }

    // Longitud del paso en X e Y
    double Sx = vx * GAIT_PERIOD;
    double Sy = vy * GAIT_PERIOD;

    // Configuración según el tipo de marcha
    double offsets[4] = {0.0, 0.0, 0.0, 0.0};
    double swing_duration = 0.0;
    double stance_duration = 0.0;

    if (current_gait == GaitType::CREEP) {
        // Secuencia de Creep Gait (Estabilidad Estática):
        // Cada pata pasa el 25% de la fase en vuelo (swing) y el 75% en apoyo (stance).
        // Secuencia de oscilación ordenada para mantener siempre el polígono de sustentación:
        // Pata 0 (RF) -> Pata 2 (BL) -> Pata 1 (FL) -> Pata 3 (BR)
        offsets[0] = 0.0;            // Pata 0 (RF) -> Vuela en [0, 0.5*PI)
        offsets[1] = M_PI;           // Pata 1 (FL) -> Vuela en [PI, 1.5*PI)
        offsets[2] = 1.5 * M_PI;     // Pata 2 (BL) -> Vuela en [0.5*PI, PI)
        offsets[3] = 0.5 * M_PI;     // Pata 3 (BR) -> Vuela en [1.5*PI, 2*PI)
        swing_duration = 0.5 * M_PI;
        stance_duration = 1.5 * M_PI;
    } else {
        // Secuencia de Trote Alterno (Diagonal):
        // Cada diagonal pasa el 50% de la fase en vuelo (swing) y el 50% en apoyo (stance).
        // Diagonal A (Pata 0 y Pata 2) vs Diagonal B (Pata 1 y Pata 3)
        offsets[0] = 0.0;            // Pata 0 (RF)
        offsets[1] = M_PI;           // Pata 1 (FL)
        offsets[2] = 0.0;            // Pata 2 (BL)
        offsets[3] = M_PI;           // Pata 3 (BR)
        swing_duration = M_PI;
        stance_duration = M_PI;
    }

    for (int i = 0; i < 4; ++i) {
        Leg* leg = legs[i];
        if (!leg) continue;

        double leg_phase = std::fmod(phase_ + offsets[i], 2.0 * M_PI); //resto de 2pi
        if (leg_phase < 0.0) leg_phase += 2.0 * M_PI; //Correción de signo por seguridad

        double x_rel = 0.0; //Se inicializan en posicion neutra de reposo para cada pata
        double y_rel = 0.0;
        double z_rel = 0.0;

        if (leg_phase < swing_duration) {
            // ─── FASE DE VUELO (SWING) ───
            double sigma = leg_phase / swing_duration; // [0, 1)
            x_rel = -Sx / 2.0 + Sx * sigma;
            y_rel = -Sy / 2.0 + Sy * sigma;
            z_rel = SWING_HEIGHT * std::sin(M_PI * sigma);
        } else {
            // ─── FASE DE APOYO (STANCE) ───
            double sigma = (leg_phase - swing_duration) / stance_duration; // [0, 1)
            x_rel = Sx / 2.0 - Sx * sigma;
            y_rel = Sy / 2.0 - Sy * sigma;
            z_rel = 0.0;
        }

        // Obtener la posición neutra inicial (pose de levantado) en coordenadas del cuerpo
        Eigen::Vector3d p_neutral = leg->getInitialFootPosition();
        
        // Sumar el desplazamiento relativo en coordenadas de cuerpo y restar el desplazamiento del sway
        // (ya que un movimiento del cuerpo hacia (+dx, +dy) desplaza el pie en (-dx, -dy) relativo al cuerpo)
        Eigen::Vector3d p_body = p_neutral + Eigen::Vector3d(x_rel - dx_sway, y_rel - dy_sway, z_rel);

        // Comandar la pata usando el método encapsulado de cuerpo (con control directo sin filtro)
        leg->goToBodyPosition(p_body, STIFFNESS_Q1, STIFFNESS_Q2, STIFFNESS_Q3, true, true);
    }

    // ─── ALGORITMO DE FRENADO ULTRA-SUAVE (SOFT SETTLE) ───
    // Si se ha pedido detener la marcha, esperamos a que la fase esté en una zona donde
    // la pata en vuelo esté tocando el suelo.
    // Esto garantiza que todas las patas tengan apoyo firme antes de clavar el robot.
    if (!active_.load() && !walking) {
        bool settle = false;
        if (current_gait == GaitType::CREEP) {
            double diff_0 = std::abs(phase_);
            double diff_half_pi = std::abs(phase_ - 0.5 * M_PI);
            double diff_pi = std::abs(phase_ - M_PI);
            double diff_1_5_pi = std::abs(phase_ - 1.5 * M_PI);
            double diff_2pi = std::abs(phase_ - 2.0 * M_PI);

            if (diff_0 < 0.15 || diff_half_pi < 0.15 || diff_pi < 0.15 || diff_1_5_pi < 0.15 || diff_2pi < 0.15) {
                settle = true;
            }
        } else {
            double diff_0 = std::abs(phase_);
            double diff_pi = std::abs(phase_ - M_PI);
            double diff_2pi = std::abs(phase_ - 2.0 * M_PI);

            if (diff_0 < 0.15 || diff_pi < 0.15 || diff_2pi < 0.15) {
                settle = true;
            }
        }

        if (settle) {
            phase_ = 0.0;
            sway_factor_ = 0.0; // Resetear el factor de balanceo al estar parados
            for (int i = 0; i < 4; ++i) {
                if (legs[i]) {
                    legs[i]->goToBodyPosition(legs[i]->getInitialFootPosition(), STIFFNESS_Q1, STIFFNESS_Q2, STIFFNESS_Q3, true, true);
                }
            }
        }
    }
}

#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <Eigen/Dense>

class Leg;

class GaitController {
public:
    enum class GaitType {
        CREEP = 0,
        TROT = 1
    };

    GaitController();
    ~GaitController();

    void start();
    void stop();
    bool isActive() const;

    void setVelocity(float vx, float vy);
    void setGaitType(GaitType type);
    GaitType getGaitType() const;
    void tick(std::array<Leg*, 4>& legs, int64_t now_ms);

private:
    std::atomic<bool> active_{false};
    std::atomic<GaitType> gait_type_{GaitType::CREEP};
    std::atomic<float> target_vx_{0.0f};
    std::atomic<float> target_vy_{0.0f};

    // Parámetros internos de la marcha
    double phase_{0.0};
    int64_t last_time_ms_{0};
    double sway_factor_{0.0};                      // Factor de atenuación dinámico del balanceo

    // Constantes de diseño estables
    static constexpr double GAIT_PERIOD = 4.0;     // Ciclo completo en segundos (4s solicitado por el usuario)
    static constexpr double SWING_HEIGHT = 0.05;    // Altura de vuelo del pie en metros (5 cm)
    
    // Parámetros del balanceo activo (Body Sway)
    static constexpr double SWAY_AMPLITUDE_X = 0.05; // Amplitud del balanceo en X (3.5 cm)
    static constexpr double SWAY_AMPLITUDE_Y = 0.05; // Amplitud del balanceo en Y (3.5 cm)
    static constexpr double SWAY_RAMP_RATE = 2.0;     // Velocidad de transición (completa la rampa en 0.5s)

    // Perfiles de rigidez seguros y sintonizados
    static constexpr int STIFFNESS_Q1 = 5; // Kp = 15.0
    static constexpr int STIFFNESS_Q2 = 5; // Kp = 32.0 (perfil 5 optimizado con Kd = 1.8)
    static constexpr int STIFFNESS_Q3 = 5; // Kp = 32.0 (perfil 5 optimizado con Kd = 1.8)
};

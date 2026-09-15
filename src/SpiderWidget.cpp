#include "SpiderWidget.h"
#include <QPainter>
#include <QPen>
#include <QColor>
#include <cmath>
#include <QPainterPath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

SpiderWidget::SpiderWidget(QWidget *parent) : QWidget(parent){}

void SpiderWidget::updateMotor(uint8_t id, float pos_rad, bool online)
{
    motors[id].pos = pos_rad;
    motors[id].online = online;
    update(); // Forzar repintado
}

void SpiderWidget::updateLegGrounded(int leg_id, bool grounded)
{
    leg_grounded[leg_id] = grounded;
    update(); // Forzar repintado
}

void SpiderWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Rellenar fondo
    painter.fillRect(rect(), QColor("#1e1e2e"));

    int width = this->width();
    // La colocamos en la parte superior, en el hueco encima de los sliders (y=0 a y=260)
    float centerX = width / 2.0f;
    float centerY = 130.0f; 

    // Proyección pseudo-3D: sx = x, sy = y * 0.5 - z
    float tiltFactor = 0.5f;
    float bodyRadius = 70.0f;

    // Dibujar cuerpo de la araña (elipse proyectada)
    QPen bodyPen(QColor("#00ffcc"), 3);
    painter.setPen(bodyPen);
    painter.setBrush(QColor("#2a2a35"));
    
    // Al aplicar el tiltFactor al eje Y, el círculo parece un óvalo inclinado en 3D
    painter.drawEllipse(QPointF(centerX, centerY), bodyRadius, bodyRadius * tiltFactor);

    // Dibujar las 4 patas
    // Pata 10 (Front-Right original) -> Ahora Inferior Derecha (45 grados)
    drawLeg(painter, centerX, centerY, 45.0f, 10, bodyRadius, tiltFactor);
    // Pata 20 (Front-Left original) -> Superior Derecha (-45 grados, sentido antihorario)
    drawLeg(painter, centerX, centerY, -45.0f, 20, bodyRadius, tiltFactor);
    // Pata 30 (Back-Left original) -> Superior Izquierda (-135 grados)
    drawLeg(painter, centerX, centerY, -135.0f, 30, bodyRadius, tiltFactor);
    // Pata 40 (Back-Right original) -> Inferior Izquierda (135 grados)
    drawLeg(painter, centerX, centerY, 135.0f, 40, bodyRadius, tiltFactor);
}

void SpiderWidget::drawLeg(QPainter &painter, float centerX, float centerY, float baseAngleDeg, uint8_t baseId, float bodyRadius, float tiltFactor)
{
    float x1 = motors[baseId + 1].pos; // Radianes (Yaw)
    float x2 = motors[baseId + 2].pos; // Radianes (Pitch fémur)
    float x3 = motors[baseId + 3].pos; // Radianes (Pitch tibia relativo)

    // Longitudes reales en 3D
    float L_FEMUR = 90.0f;
    float L_TIBIA = 110.0f;

    // El ángulo total de la pata en el plano horizontal (XY)
    float theta = (baseAngleDeg * M_PI / 180.0f) + x1;

    // --- CÁLCULO CINEMÁTICO 3D EN EL PLANO DE LA PATA (Radio, Z) ---
    // Origen de la pata (pegada al cuerpo)
    float r_base = bodyRadius;
    float z_base = 0.0f;

    // Rodilla (Fémur asumiendo que positivo es hacia arriba o hacia abajo, lo normal es visualizarlo con - o +)
    float r_knee = r_base + L_FEMUR * std::cos(x2);
    float z_knee = z_base + L_FEMUR * std::sin(x2);

    // Pie (Tibia) - ángulo relativo al fémur
    float r_foot = r_knee + L_TIBIA * std::cos(x2 + x3);
    float z_foot = z_knee + L_TIBIA * std::sin(x2 + x3);

    // --- PROYECCIÓN 3D A PANTALLA 2D ---
    // Base
    float sx_base = centerX + r_base * std::cos(theta);
    float sy_base = centerY + (r_base * std::sin(theta)) * tiltFactor - z_base;

    // Rodilla
    float sx_knee = centerX + r_knee * std::cos(theta);
    float sy_knee = centerY + (r_knee * std::sin(theta)) * tiltFactor - z_knee;

    // Pie
    float sx_foot = centerX + r_foot * std::cos(theta);
    float sy_foot = centerY + (r_foot * std::sin(theta)) * tiltFactor - z_foot;

    // Sombras (Proyección en el suelo, Z=0)
    float shadow_x_knee = centerX + r_knee * std::cos(theta);
    float shadow_y_knee = centerY + (r_knee * std::sin(theta)) * tiltFactor;
    float shadow_x_foot = centerX + r_foot * std::cos(theta);
    float shadow_y_foot = centerY + (r_foot * std::sin(theta)) * tiltFactor;

    // Dibujar sombra para mayor efecto 3D
    QPen shadowPen(QColor(0, 0, 0, 80), 4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    painter.setPen(shadowPen);
    painter.drawLine(QPointF(sx_base, sy_base), QPointF(shadow_x_knee, shadow_y_knee));
    painter.drawLine(QPointF(shadow_x_knee, shadow_y_knee), QPointF(shadow_x_foot, shadow_y_foot));

    // Configurar lápiz de la pata (brillo neón)
    QPen legPen(QColor("#a6e3a1"), 5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    painter.setPen(legPen);

    // Dibujar segmentos reales
    painter.drawLine(QPointF(sx_base, sy_base), QPointF(sx_knee, sy_knee));
    painter.drawLine(QPointF(sx_knee, sy_knee), QPointF(sx_foot, sy_foot));

    // Colores de las articulaciones según conexión
    bool x1_online = motors[baseId + 1].online;
    bool x2_online = motors[baseId + 2].online;
    bool x3_online = motors[baseId + 3].online;

    QColor baseColor;
    if (x1_online && x2_online) baseColor = QColor("#a6e3a1"); // Verde
    else if (!x1_online && !x2_online) baseColor = QColor("#f38ba8"); // Rojo
    else baseColor = QColor("#ffff00"); // Amarillo brillante

    QColor kneeColor = x3_online ? QColor("#a6e3a1") : QColor("#f38ba8");

    // Dibujar articulaciones
    painter.setPen(QPen(QColor("#1e1e2e"), 2));
    
    painter.setBrush(baseColor);
    painter.drawEllipse(QPointF(sx_base, sy_base), 6, 6); // Base (X1 y X2)
    
    painter.setBrush(kneeColor);
    painter.drawEllipse(QPointF(sx_knee, sy_knee), 5, 5); // Rodilla (X3)
    
    int leg_id = baseId / 10;
    bool grounded = leg_grounded[leg_id];
    QColor footColor = grounded ? QColor("#a6e3a1") : QColor("#f38ba8");
    painter.setBrush(footColor);
    painter.drawEllipse(QPointF(sx_foot, sy_foot), 4, 4); // X3 (Extremo del eslabón)
}

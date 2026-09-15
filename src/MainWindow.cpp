#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "Tarantula.h"
#include "MotorController.h"
#include <QTableWidgetItem>
#include <QGroupBox>
#include <QGridLayout>
#include <QString>
#include <QComboBox>
#include <QSlider>
#include <QLabel>
#include <QMessageBox>
#include <thread>
#include <iostream>

static constexpr float RAD_TO_DEG = 180.0f / 3.14159265358979f;
static constexpr float DEG_TO_RAD = 3.14159265358979f / 180.0f;

// Escala de los sliders: 1 unidad = 1 mm para XYZ, 1° para ángulos
//static constexpr double SLIDER_XYZ_SCALE   = 0.001;   // slider int → metros
//static constexpr double SLIDER_ANGLE_SCALE = M_PI / 180.0; // slider int → rad



MainWindow::MainWindow(Tarantula* robot, QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , robot_(robot)
{
    ui->setupUi(this);
    initTable();

    // Recortar la anchura horizontal de los botones principales del panel derecho
    ui->btnStartAll->setMaximumWidth(180);
    ui->btnStopAll->setMaximumWidth(180);
    ui->btnStandUp->setMaximumWidth(180);
    ui->btnSitDown->setMaximumWidth(180);

    ui->btnStartAll->setFocusPolicy(Qt::NoFocus);
    ui->btnStopAll->setFocusPolicy(Qt::NoFocus);
    ui->btnStandUp->setFocusPolicy(Qt::NoFocus);
    ui->btnSitDown->setFocusPolicy(Qt::NoFocus);

    // Crear el GroupBox para el D-Pad de Control de Marcha
    QGroupBox* trotGroup = new QGroupBox("Marcha", this);
    trotGroup->setStyleSheet(
        "QGroupBox { color: #f9e2af; font-weight: bold; border: 2px solid #313244; border-radius: 6px; padding: 10px; }"
    );
    trotGroup->setMaximumWidth(220);

    QGridLayout* gridLayout = new QGridLayout(trotGroup);
    gridLayout->setSpacing(6);
    gridLayout->setContentsMargins(5, 5, 5, 5);

    comboGaitSelect_ = new QComboBox(this);
    comboGaitSelect_->addItems({"Creep Gait", "Trote Alterno"});
    comboGaitSelect_->setStyleSheet(
        "QComboBox { background-color: #313244; color: #cdd6f4; border: 1px solid #45475a; border-radius: 4px; padding: 3px; font-weight: bold; }"
        "QComboBox QAbstractItemView { background-color: #1e1e2e; color: #cdd6f4; selection-background-color: #f9e2af; selection-color: #1e1e2e; }"
    );
    comboGaitSelect_->setFocusPolicy(Qt::NoFocus);

    btnTrotUp_ = new QPushButton("▲", this);
    btnTrotDown_ = new QPushButton("▼", this);
    btnTrotLeft_ = new QPushButton("◀", this);
    btnTrotRight_ = new QPushButton("▶", this);

    btnTrotUp_->setFocusPolicy(Qt::NoFocus);
    btnTrotDown_->setFocusPolicy(Qt::NoFocus);
    btnTrotLeft_->setFocusPolicy(Qt::NoFocus);
    btnTrotRight_->setFocusPolicy(Qt::NoFocus);

    // Estilo elegante adaptado al tema oscuro
    QString btnStyle = "QPushButton { background-color: #313244; color: #cdd6f4; border: 2px solid #45475a; border-radius: 6px; min-width: 45px; min-height: 45px; max-width: 45px; max-height: 45px; font-weight: bold; font-size: 16px; }"
                       "QPushButton:pressed { background-color: #f9e2af; color: #1e1e2e; }"
                       "QPushButton:checked { background-color: #f9e2af; color: #1e1e2e; }";
    btnTrotUp_->setStyleSheet(btnStyle);
    btnTrotDown_->setStyleSheet(btnStyle);
    btnTrotLeft_->setStyleSheet(btnStyle);
    btnTrotRight_->setStyleSheet(btnStyle);

    gridLayout->addWidget(comboGaitSelect_, 0, 0, 1, 3);
    gridLayout->addWidget(btnTrotUp_, 1, 1);
    gridLayout->addWidget(btnTrotLeft_, 2, 0);
    gridLayout->addWidget(btnTrotRight_, 2, 2);
    gridLayout->addWidget(btnTrotDown_, 3, 1);

    // Conectar señales del D-Pad físico
    connect(btnTrotUp_, &QPushButton::pressed, [this]() { keyUpPressed_ = true; updateTrotVelocity(); });
    connect(btnTrotUp_, &QPushButton::released, [this]() { keyUpPressed_ = false; updateTrotVelocity(); });

    connect(btnTrotDown_, &QPushButton::pressed, [this]() { keyDownPressed_ = true; updateTrotVelocity(); });
    connect(btnTrotDown_, &QPushButton::released, [this]() { keyDownPressed_ = false; updateTrotVelocity(); });

    connect(btnTrotLeft_, &QPushButton::pressed, [this]() { keyLeftPressed_ = true; updateTrotVelocity(); });
    connect(btnTrotLeft_, &QPushButton::released, [this]() { keyLeftPressed_ = false; updateTrotVelocity(); });

    connect(btnTrotRight_, &QPushButton::pressed, [this]() { keyRightPressed_ = true; updateTrotVelocity(); });
    connect(btnTrotRight_, &QPushButton::released, [this]() { keyRightPressed_ = false; updateTrotVelocity(); });

    connect(comboGaitSelect_, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
        if (robot_) {
            robot_->setGaitType(index);
        }
    });

    // Extraer los botones de acción del panel derecho y reagruparlos horizontalmente junto con el D-Pad
    ui->verticalLayout_Right->removeWidget(ui->btnStartAll);
    ui->verticalLayout_Right->removeWidget(ui->btnStopAll);
    ui->verticalLayout_Right->removeWidget(ui->btnStandUp);
    ui->verticalLayout_Right->removeWidget(ui->btnSitDown);

    QVBoxLayout* actionButtonsLayout = new QVBoxLayout();
    actionButtonsLayout->setSpacing(6);
    actionButtonsLayout->setContentsMargins(0, 0, 0, 0);
    actionButtonsLayout->addWidget(ui->btnStartAll);
    actionButtonsLayout->addWidget(ui->btnStopAll);
    actionButtonsLayout->addWidget(ui->btnStandUp);
    actionButtonsLayout->addWidget(ui->btnSitDown);

    // Crear el GroupBox para el Control IK de Pata
    QGroupBox* legIkGroup = new QGroupBox("Control IK Pata", this);
    legIkGroup->setStyleSheet(
        "QGroupBox { color: #89b4fa; font-weight: bold; border: 2px solid #313244; border-radius: 6px; padding: 10px; }"
    );
    legIkGroup->setMinimumWidth(280);
    legIkGroup->setMaximumWidth(320);

    QGridLayout* legIkLayout = new QGridLayout(legIkGroup);
    legIkLayout->setSpacing(6);
    legIkLayout->setContentsMargins(5, 5, 5, 5);

    // Selector de Pata
    QLabel* lblLegSel = new QLabel("Pata:", this);
    lblLegSel->setStyleSheet("color: #a6adc8; font-weight: bold;");
    comboLegSelect_ = new QComboBox(this);
    comboLegSelect_->addItems({"Pata 1 (DD)", "Pata 2 (DI)", "Pata 3 (TI)", "Pata 4 (TD)"});
    comboLegSelect_->setStyleSheet(
        "QComboBox { background-color: #313244; color: #cdd6f4; border: 1px solid #45475a; border-radius: 4px; padding: 3px; }"
        "QComboBox QAbstractItemView { background-color: #1e1e2e; color: #cdd6f4; selection-background-color: #89b4fa; selection-color: #1e1e2e; }"
    );
    comboLegSelect_->setFocusPolicy(Qt::NoFocus);

    legIkLayout->addWidget(lblLegSel, 0, 0);
    legIkLayout->addWidget(comboLegSelect_, 0, 1, 1, 2);

    // Slider X
    QLabel* lblLabelX = new QLabel("X:", this);
    lblLabelX->setStyleSheet("color: #a6adc8; font-weight: bold;");
    sliderLegX_ = new QSlider(Qt::Horizontal, this);
    sliderLegX_->setRange(200, 495);
    sliderLegX_->setValue(495);
    sliderLegX_->setStyleSheet(
        "QSlider::groove:horizontal { border: 1px solid #45475a; height: 6px; background: #313244; border-radius: 3px; }"
        "QSlider::handle:horizontal { background: #89b4fa; border: 1px solid #89b4fa; width: 14px; height: 14px; margin: -4px 0; border-radius: 7px; }"
    );
    sliderLegX_->setFocusPolicy(Qt::NoFocus);
    lblLegX_ = new QLabel("495 mm", this);
    lblLegX_->setStyleSheet("color: #cdd6f4; font-weight: bold; min-width: 55px;");
    lblLegX_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    legIkLayout->addWidget(lblLabelX, 1, 0);
    legIkLayout->addWidget(sliderLegX_, 1, 1);
    legIkLayout->addWidget(lblLegX_, 1, 2);

    // Slider Y
    QLabel* lblLabelY = new QLabel("Y:", this);
    lblLabelY->setStyleSheet("color: #a6adc8; font-weight: bold;");
    sliderLegY_ = new QSlider(Qt::Horizontal, this);
    sliderLegY_->setRange(-300, 300);
    sliderLegY_->setValue(0);
    sliderLegY_->setStyleSheet(
        "QSlider::groove:horizontal { border: 1px solid #45475a; height: 6px; background: #313244; border-radius: 3px; }"
        "QSlider::handle:horizontal { background: #89b4fa; border: 1px solid #89b4fa; width: 14px; height: 14px; margin: -4px 0; border-radius: 7px; }"
    );
    sliderLegY_->setFocusPolicy(Qt::NoFocus);
    lblLegY_ = new QLabel("0 mm", this);
    lblLegY_->setStyleSheet("color: #cdd6f4; font-weight: bold; min-width: 55px;");
    lblLegY_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    legIkLayout->addWidget(lblLabelY, 2, 0);
    legIkLayout->addWidget(sliderLegY_, 2, 1);
    legIkLayout->addWidget(lblLegY_, 2, 2);

    // Slider Z
    QLabel* lblLabelZ = new QLabel("Z:", this);
    lblLabelZ->setStyleSheet("color: #a6adc8; font-weight: bold;");
    sliderLegZ_ = new QSlider(Qt::Horizontal, this);
    sliderLegZ_->setRange(-350, 150);
    sliderLegZ_->setValue(0);
    sliderLegZ_->setStyleSheet(
        "QSlider::groove:horizontal { border: 1px solid #45475a; height: 6px; background: #313244; border-radius: 3px; }"
        "QSlider::handle:horizontal { background: #89b4fa; border: 1px solid #89b4fa; width: 14px; height: 14px; margin: -4px 0; border-radius: 7px; }"
    );
    sliderLegZ_->setFocusPolicy(Qt::NoFocus);
    lblLegZ_ = new QLabel("0 mm", this);
    lblLegZ_->setStyleSheet("color: #cdd6f4; font-weight: bold; min-width: 55px;");
    lblLegZ_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    legIkLayout->addWidget(lblLabelZ, 3, 0);
    legIkLayout->addWidget(sliderLegZ_, 3, 1);
    legIkLayout->addWidget(lblLegZ_, 3, 2);

    // Botón Sincronizar / Resetear
    btnResetLeg_ = new QPushButton("Nominal / Sinc", this);
    btnResetLeg_->setStyleSheet(
        "QPushButton { background-color: #313244; color: #a6e3a1; border: 2px solid #45475a; border-radius: 6px; padding: 4px; font-weight: bold; }"
        "QPushButton:pressed { background-color: #a6e3a1; color: #1e1e2e; }"
    );
    btnResetLeg_->setFocusPolicy(Qt::NoFocus);
    legIkLayout->addWidget(btnResetLeg_, 4, 1, 1, 2);

    // Conectar señales del panel de pata individual
    connect(comboLegSelect_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::resetIndividualLegSliders);
    connect(sliderLegX_, &QSlider::valueChanged, this, &MainWindow::sendCurrentIndividualLegTarget);
    connect(sliderLegY_, &QSlider::valueChanged, this, &MainWindow::sendCurrentIndividualLegTarget);
    connect(sliderLegZ_, &QSlider::valueChanged, this, &MainWindow::sendCurrentIndividualLegTarget);
    connect(btnResetLeg_, &QPushButton::clicked, [this]() {
        if (sliderLegX_ && sliderLegY_ && sliderLegZ_) {
            sliderLegX_->blockSignals(true);
            sliderLegY_->blockSignals(true);
            sliderLegZ_->blockSignals(true);

            sliderLegX_->setValue(495);
            sliderLegY_->setValue(0);
            sliderLegZ_->setValue(0);

            sliderLegX_->blockSignals(false);
            sliderLegY_->blockSignals(false);
            sliderLegZ_->blockSignals(false);

            sendCurrentIndividualLegTarget();
        }
    });

    QHBoxLayout* topHorizontalLayout = new QHBoxLayout();
    topHorizontalLayout->setSpacing(15);
    topHorizontalLayout->setContentsMargins(0, 0, 0, 0);
    topHorizontalLayout->addLayout(actionButtonsLayout);
    topHorizontalLayout->addWidget(trotGroup);
    topHorizontalLayout->addWidget(legIkGroup);

    // Añadir un spacer horizontal a la derecha para que todo se agrupe de forma elegante a la izquierda
    topHorizontalLayout->addSpacerItem(new QSpacerItem(0, 0, QSizePolicy::Expanding, QSizePolicy::Minimum));

    // Insertar este layout combinado horizontal justo debajo de las etiquetas de USB/CAN (índice 1 en verticalLayout_Right)
    ui->verticalLayout_Right->insertLayout(1, topHorizontalLayout);

    // --- Configuración del selector de puerto COM ---
    comboComPort_ = new QComboBox(this);
    for (int i = 1; i <= 11; ++i) {
        comboComPort_->addItem(QString("COM%1").arg(i));
    }
    
    // Configurar estilo idéntico a otros combos y tamaño para que sea similar a lblUsbStatus (alto 40)
    comboComPort_->setMinimumSize(85, 40);
    comboComPort_->setMaximumSize(85, 40);
    comboComPort_->setStyleSheet(
        "QComboBox { background-color: #313244; color: #cdd6f4; border: 1px solid #45475a; border-radius: 4px; padding: 3px; font-weight: bold; font-size: 13px; }"
        "QComboBox QAbstractItemView { background-color: #1e1e2e; color: #cdd6f4; selection-background-color: #89b4fa; selection-color: #1e1e2e; }"
    );
    comboComPort_->setFocusPolicy(Qt::NoFocus);

    // Insertar dinámicamente en el horizontalLayout_Status al lado de lblUsbStatus
    int usb_idx = ui->horizontalLayout_Status->indexOf(ui->lblUsbStatus);
    if (usb_idx != -1) {
        ui->horizontalLayout_Status->insertWidget(usb_idx, comboComPort_);
    } else {
        ui->horizontalLayout_Status->addWidget(comboComPort_);
    }

    // Seleccionar COM3 por defecto (índice 2) bloqueando señales temporalmente
    comboComPort_->blockSignals(true);
    comboComPort_->setCurrentIndex(2);
    // Conectar la señal de cambio
    connect(comboComPort_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onComPortChanged);
    comboComPort_->blockSignals(false);

    // Configurar los nuevos límites de movimiento del chasis (X/Y: +/-120mm, Z: +/-140mm)
    ui->sliderBodyX->setRange(-120, 120);
    ui->sliderBodyY->setRange(-120, 120);
    ui->sliderBodyZ->setRange(-140, 140);

    // Inicializar sliders con la posición real de la pata seleccionada por defecto
    resetIndividualLegSliders();

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow::updateUI);
    timer_->start(100);  // 10 Hz de refresco de la UI
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::initTable()
{
    ui->tableMotors->setColumnCount(3);
    ui->tableMotors->setHorizontalHeaderLabels({"Motor", "Posición", "Rigidez"});
    ui->tableMotors->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
}



void MainWindow::updateUI()
{
    updateStatusLeds();
    updateMotorTable();
    updateSpiderWidget();
}

void MainWindow::updateStatusLeds()
{
    bool usb_ok = g_comm && g_comm->connected();
    QString green = "background-color: #a6e3a1; color: #11111b; border-radius: 4px;";
    QString red   = "background-color: #f38ba8; color: #11111b; border-radius: 4px;";

    ui->lblUsbStatus->setStyleSheet(usb_ok ? green : red);
    ui->lblCanStatus->setStyleSheet(robot_->isAnyMotorOnline() ? green : red);
}

void MainWindow::updateMotorTable()
{
    static const std::vector<uint32_t> MOTOR_IDS = {
        11,12,13, 21,22,23, 31,32,33, 41,42,43
    };

    ui->tableMotors->setRowCount(static_cast<int>(MOTOR_IDS.size()));
    int row = 0;

    for (uint32_t id : MOTOR_IDS) {
        MotorController* ctrl = g_controllers[id];
        if (!ctrl) { ++row; continue; }

        bool  online   = ctrl->motor_online.load(std::memory_order_relaxed);
        bool  active   = ctrl->active.load(std::memory_order_relaxed);
        float pos_deg  = ctrl->last_known_pos.load(std::memory_order_relaxed) * RAD_TO_DEG;
        float kp       = ctrl->target_kp.load(std::memory_order_relaxed);

        QString stiffness_label = buildStiffnessLabel(online, active, kp);
        QString color = online ? "#cdd6f4" : "#f38ba8";

        setTableCell(row, 0, QString::number(id),                      color);
        setTableCell(row, 1, online ? QString::number(pos_deg,'f',1)+"°" : "-", color);
        setTableCell(row, 2, stiffness_label, active ? "#a6e3a1" : "#f38ba8");
        ++row;
    }
}

QString MainWindow::buildStiffnessLabel(bool online, bool active, float kp) const
{
    if (!online) return "DESCONECTADO";
    if (!active) return "OFF";
    if (kp <= 0.0f)  return "ON (Suave)";
    if (kp <= 5.0f)  return "Perfil 1";
    if (kp <= 10.0f) return "Perfil 2";
    if (kp <= 20.0f) return "Perfil 3";
    if (kp <= 35.0f) return "Perfil 4";
    return "Perfil 5";
}

void MainWindow::setTableCell(int row, int col, const QString& text, const QString& color)
{
    QTableWidgetItem* item = ui->tableMotors->item(row, col);
    if (!item) {
        item = new QTableWidgetItem();
        item->setTextAlignment(Qt::AlignCenter);
        ui->tableMotors->setItem(row, col, item);
    }
    item->setText(text);
    item->setForeground(QBrush(QColor(color)));
}

void MainWindow::updateSpiderWidget()
{
    static const std::vector<uint32_t> MOTOR_IDS = {
        11,12,13, 21,22,23, 31,32,33, 41,42,43
    };
    for (uint32_t id : MOTOR_IDS) {
        MotorController* ctrl = g_controllers[id];
        if (!ctrl) continue;
        bool  online  = ctrl->motor_online.load(std::memory_order_relaxed);
        float pos_rad = ctrl->last_known_pos.load(std::memory_order_relaxed);
        ui->spiderWidget->updateMotor(id, pos_rad, online);
    }

    for (int leg_id = 1; leg_id <= 4; ++leg_id) {
        bool grounded = robot_->getLeg(leg_id).isGrounded();
        ui->spiderWidget->updateLegGrounded(leg_id, grounded);
    }
}


void MainWindow::on_btnStartAll_clicked()
{
    robot_->enableAllLegs();
}

void MainWindow::on_btnStopAll_clicked()
{
    robot_->disableAllLegs();
}

void MainWindow::on_btnStandUp_clicked()
{
    robot_->standUp();
}

void MainWindow::on_btnSitDown_clicked()
{
    robot_->sitDown();
}


void MainWindow::on_btnSendCommandGrid_clicked()
{
    float p1 = ui->txtPos1->text().toFloat();
    float p2 = ui->txtPos2->text().toFloat();
    float p3 = ui->txtPos3->text().toFloat();
    int   r1 = ui->txtRig1->text().toInt();
    int   r2 = ui->txtRig2->text().toInt();
    int   r3 = ui->txtRig3->text().toInt();

    std::thread([this, p1, p2, p3, r1, r2, r3]() {
        for (int leg = 1; leg <= 4; ++leg) {
            robot_->moveLegJoint(leg, 1, p1, r1);
            robot_->moveLegJoint(leg, 2, p2, r2);
            robot_->moveLegJoint(leg, 3, p3, r3);
        }
    }).detach();
}

void MainWindow::on_btnEnableIndividual_clicked()
{
    bool ok;
    int id = ui->txtMoverID->text().toInt(&ok);
    if (!ok) return;

    int leg_id = id / 10;
    int joint = id % 10;
    if (leg_id >= 1 && leg_id <= 4) {
        robot_->getLeg(leg_id).enableJoint(joint);
    }
}

void MainWindow::on_btnSendIndividual_clicked()
{
    bool ok_id, ok_pos, ok_rig;
    int   id  = ui->txtMoverID->text().toInt(&ok_id);
    float pos = ui->txtMoverPos->text().toFloat(&ok_pos);
    int   rig = ui->txtMoverRigidez->text().toInt(&ok_rig);
    if (!ok_id || !ok_pos || !ok_rig) return;

    int leg_id = id / 10;
    int joint = id % 10;
    if (leg_id >= 1 && leg_id <= 4) {
        std::thread([this, leg_id, joint, pos, rig]() {
            robot_->getLeg(leg_id).moveJoint(joint, pos, rig);
        }).detach();
    }
}


void MainWindow::on_sliderBodyX_valueChanged(int)     { sendCurrentBodyPose(); }
void MainWindow::on_sliderBodyY_valueChanged(int)     { sendCurrentBodyPose(); }
void MainWindow::on_sliderBodyZ_valueChanged(int)     { sendCurrentBodyPose(); }
void MainWindow::on_sliderBodyRoll_valueChanged(int)  { sendCurrentBodyPose(); }
void MainWindow::on_sliderBodyPitch_valueChanged(int) { sendCurrentBodyPose(); }
void MainWindow::on_sliderBodyYaw_valueChanged(int)   { sendCurrentBodyPose(); }

void MainWindow::sendCurrentBodyPose()
{
    static constexpr double SLIDER_XYZ_SCALE   = 0.001; // 1 unit = 1 mm (0.001 m)
    static constexpr double SLIDER_ANGLE_SCALE = M_PI / 180.0;

    int valX = ui->sliderBodyX->value();
    int valY = ui->sliderBodyY->value();
    int valZ = ui->sliderBodyZ->value();
    int valRoll = ui->sliderBodyRoll->value();
    int valPitch = ui->sliderBodyPitch->value();
    int valYaw = ui->sliderBodyYaw->value();

    double dx    = valX * SLIDER_XYZ_SCALE;
    double dy    = valY * SLIDER_XYZ_SCALE;
    double dz    = valZ * SLIDER_XYZ_SCALE;
    double roll  = valRoll * SLIDER_ANGLE_SCALE;
    double pitch = valPitch * SLIDER_ANGLE_SCALE;
    double yaw   = valYaw * SLIDER_ANGLE_SCALE;

    if (robot_->setBodyPose(dx, dy, dz, roll, pitch, yaw)) {
        last_valid_body_x_ = valX;
        last_valid_body_y_ = valY;
        last_valid_body_z_ = valZ;
        last_valid_body_roll_ = valRoll;
        last_valid_body_pitch_ = valPitch;
        last_valid_body_yaw_ = valYaw;

        ui->lblBodyX->setText(QString("%1 mm").arg(valX));
        ui->lblBodyY->setText(QString("%1 mm").arg(valY));
        ui->lblBodyZ->setText(QString("%1 mm").arg(valZ));
        ui->lblBodyRoll->setText(QString("%1°").arg(valRoll));
        ui->lblBodyPitch->setText(QString("%1°").arg(valPitch));
        ui->lblBodyYaw->setText(QString("%1°").arg(valYaw));
    } else {
        ui->sliderBodyX->blockSignals(true);
        ui->sliderBodyY->blockSignals(true);
        ui->sliderBodyZ->blockSignals(true);
        ui->sliderBodyRoll->blockSignals(true);
        ui->sliderBodyPitch->blockSignals(true);
        ui->sliderBodyYaw->blockSignals(true);

        ui->sliderBodyX->setValue(last_valid_body_x_);
        ui->sliderBodyY->setValue(last_valid_body_y_);
        ui->sliderBodyZ->setValue(last_valid_body_z_);
        ui->sliderBodyRoll->setValue(last_valid_body_roll_);
        ui->sliderBodyPitch->setValue(last_valid_body_pitch_);
        ui->sliderBodyYaw->setValue(last_valid_body_yaw_);

        ui->sliderBodyX->blockSignals(false);
        ui->sliderBodyY->blockSignals(false);
        ui->sliderBodyZ->blockSignals(false);
        ui->sliderBodyRoll->blockSignals(false);
        ui->sliderBodyPitch->blockSignals(false);
        ui->sliderBodyYaw->blockSignals(false);

        QMessageBox msg(this);
        msg.setWindowTitle("Postura Inválida");
        msg.setText("La postura solicitada no es factible o causaría una colisión.");
        msg.setIcon(QMessageBox::Warning);
        msg.setStyleSheet(
            "QMessageBox { background-color: #1e1e2e; color: #cdd6f4; }"
            "QLabel { color: #cdd6f4; }"
            "QPushButton { background-color: #313244; color: #cdd6f4; border: 1px solid #45475a; border-radius: 4px; padding: 4px; min-width: 60px; }"
            "QPushButton:pressed { background-color: #f38ba8; color: #1e1e2e; }"
        );
        msg.exec();
    }
}

void MainWindow::on_btnResetPose_clicked()
{
    last_valid_body_x_ = 0;
    last_valid_body_y_ = 0;
    last_valid_body_z_ = 0;
    last_valid_body_roll_ = 0;
    last_valid_body_pitch_ = 0;
    last_valid_body_yaw_ = 0;

    ui->sliderBodyX->blockSignals(true);
    ui->sliderBodyY->blockSignals(true);
    ui->sliderBodyZ->blockSignals(true);
    ui->sliderBodyRoll->blockSignals(true);
    ui->sliderBodyPitch->blockSignals(true);
    ui->sliderBodyYaw->blockSignals(true);

    ui->sliderBodyX->setValue(0);
    ui->sliderBodyY->setValue(0);
    ui->sliderBodyZ->setValue(0);
    ui->sliderBodyRoll->setValue(0);
    ui->sliderBodyPitch->setValue(0);
    ui->sliderBodyYaw->setValue(0);

    ui->sliderBodyX->blockSignals(false);
    ui->sliderBodyY->blockSignals(false);
    ui->sliderBodyZ->blockSignals(false);
    ui->sliderBodyRoll->blockSignals(false);
    ui->sliderBodyPitch->blockSignals(false);
    ui->sliderBodyYaw->blockSignals(false);

    ui->lblBodyX->setText("0 mm");
    ui->lblBodyY->setText("0 mm");
    ui->lblBodyZ->setText("0 mm");
    ui->lblBodyRoll->setText("0°");
    ui->lblBodyPitch->setText("0°");
    ui->lblBodyYaw->setText("0°");

    robot_->standUp();
}

void MainWindow::updateTrotVelocity()
{
    float vx = 0.0f;
    float vy = 0.0f;

    if (keyUpPressed_)    vx += 0.04f; // Adelante +X
    if (keyDownPressed_)  vx -= 0.04f; // Atrás -X
    if (keyLeftPressed_)  vy -= 0.04f; // Izquierda -Y (Y es positivo a la derecha)
    if (keyRightPressed_) vy += 0.04f; // Derecha +Y

    bool walking = (std::abs(vx) > 0.001f || std::abs(vy) > 0.001f);

    if (walking) {
        if (!robot_->isGaitActive()) {
            robot_->startGait();
        }
        robot_->setGaitVelocity(vx, vy);
    } else {
        if (robot_->isGaitActive()) {
            robot_->stopGait();
        }
    }
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    if (event->isAutoRepeat()) return;

    bool handled = false;
    switch (event->key()) {
    case Qt::Key_Up:
        keyUpPressed_ = true;
        if (btnTrotUp_) btnTrotUp_->setDown(true);
        handled = true;
        break;
    case Qt::Key_Down:
        keyDownPressed_ = true;
        if (btnTrotDown_) btnTrotDown_->setDown(true);
        handled = true;
        break;
    case Qt::Key_Left:
        keyLeftPressed_ = true;
        if (btnTrotLeft_) btnTrotLeft_->setDown(true);
        handled = true;
        break;
    case Qt::Key_Right:
        keyRightPressed_ = true;
        if (btnTrotRight_) btnTrotRight_->setDown(true);
        handled = true;
        break;
    default:
        break;
    }

    if (handled) {
        updateTrotVelocity();
    } else {
        QMainWindow::keyPressEvent(event);
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent* event)
{
    if (event->isAutoRepeat()) return;

    bool handled = false;
    switch (event->key()) {
    case Qt::Key_Up:
        keyUpPressed_ = false;
        if (btnTrotUp_) btnTrotUp_->setDown(false);
        handled = true;
        break;
    case Qt::Key_Down:
        keyDownPressed_ = false;
        if (btnTrotDown_) btnTrotDown_->setDown(false);
        handled = true;
        break;
    case Qt::Key_Left:
        keyLeftPressed_ = false;
        if (btnTrotLeft_) btnTrotLeft_->setDown(false);
        handled = true;
        break;
    case Qt::Key_Right:
        keyRightPressed_ = false;
        if (btnTrotRight_) btnTrotRight_->setDown(false);
        handled = true;
        break;
    default:
        break;
    }

    if (handled) {
        updateTrotVelocity();
    } else {
        QMainWindow::keyReleaseEvent(event);
    }
}

void MainWindow::sendCurrentIndividualLegTarget()
{
    if (!comboLegSelect_ || !sliderLegX_ || !sliderLegY_ || !sliderLegZ_ || !lblLegX_ || !lblLegY_ || !lblLegZ_) return;

    int leg_idx = comboLegSelect_->currentIndex();
    int leg_id = leg_idx + 1;

    int valX = sliderLegX_->value();
    int valY = sliderLegY_->value();
    int valZ = sliderLegZ_->value();

    double x = valX * 0.001; // mm to m
    double y = valY * 0.001;
    double z = valZ * 0.001;

    if (robot_->moveLeg(leg_id, x, y, z)) {
        lblLegX_->setText(QString("%1 mm").arg(valX));
        lblLegY_->setText(QString("%1 mm").arg(valY));
        lblLegZ_->setText(QString("%1 mm").arg(valZ));
    } else {
        sliderLegX_->blockSignals(true);
        sliderLegY_->blockSignals(true);
        sliderLegZ_->blockSignals(true);

        resetIndividualLegSliders();

        sliderLegX_->blockSignals(false);
        sliderLegY_->blockSignals(false);
        sliderLegZ_->blockSignals(false);

        QMessageBox msg(this);
        msg.setWindowTitle("Movimiento Inválido");
        msg.setText("El movimiento de la pata no es factible o provocaría una colisión.");
        msg.setIcon(QMessageBox::Warning);
        msg.setStyleSheet(
            "QMessageBox { background-color: #1e1e2e; color: #cdd6f4; }"
            "QLabel { color: #cdd6f4; }"
            "QPushButton { background-color: #313244; color: #cdd6f4; border: 1px solid #45475a; border-radius: 4px; padding: 4px; min-width: 60px; }"
            "QPushButton:pressed { background-color: #f38ba8; color: #1e1e2e; }"
        );
        msg.exec();
    }
}

void MainWindow::resetIndividualLegSliders()
{
    if (!comboLegSelect_ || !sliderLegX_ || !sliderLegY_ || !sliderLegZ_ || !lblLegX_ || !lblLegY_ || !lblLegZ_) return;

    int leg_idx = comboLegSelect_->currentIndex();
    int leg_id = leg_idx + 1;

    Eigen::Vector3d current_pos = robot_->getLeg(leg_id).getCurrentFootPosition();

    int x_mm = static_cast<int>(std::round(current_pos.x() * 1000.0));
    int y_mm = static_cast<int>(std::round(current_pos.y() * 1000.0));
    int z_mm = static_cast<int>(std::round(current_pos.z() * 1000.0));

    // Si la posición leída es descabellada o fuera del alcance de los sliders, forzar nominal
    if (x_mm < 150 || x_mm > 500 || y_mm < -300 || y_mm > 300 || z_mm < -350 || z_mm > 150) {
        x_mm = 495;
        y_mm = 0;
        z_mm = 0;
    }

    sliderLegX_->blockSignals(true);
    sliderLegY_->blockSignals(true);
    sliderLegZ_->blockSignals(true);

    sliderLegX_->setValue(x_mm);
    sliderLegY_->setValue(y_mm);
    sliderLegZ_->setValue(z_mm);

    sliderLegX_->blockSignals(false);
    sliderLegY_->blockSignals(false);
    sliderLegZ_->blockSignals(false);

    lblLegX_->setText(QString("%1 mm").arg(x_mm));
    lblLegY_->setText(QString("%1 mm").arg(y_mm));
    lblLegZ_->setText(QString("%1 mm").arg(z_mm));
}

void MainWindow::onComPortChanged(int index)
{
    QString port_name = QString("\\\\.\\COM%1").arg(index + 1);
    std::cout << "🔌 Solicitando reconexión serial a: " << port_name.toStdString() << "\n";
    if (g_comm) {
        g_comm->reconnectWithPort(port_name.toStdString());
    }
}

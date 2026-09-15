#pragma once
#include <QMainWindow>
#include <QTimer>
#include <QKeyEvent>
#include <QPushButton>

// Declaración adelantada: la GUI solo conoce la interfaz de Tarantula
class Tarantula;
class QComboBox;
class QSlider;
class QLabel;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(Tarantula* robot, QWidget* parent = nullptr);
    ~MainWindow();

protected:
    // Eventos de teclado para control por flechas
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;

private slots:
    // Actualización periódica de la UI (timer 100 ms)
    void updateUI();

    
    void on_btnStartAll_clicked();
    void on_btnStopAll_clicked();
    void on_btnStandUp_clicked();
    void on_btnSitDown_clicked();


    void on_btnSendCommandGrid_clicked();

    void on_btnEnableIndividual_clicked();
    void on_btnSendIndividual_clicked();

    
    void on_sliderBodyX_valueChanged(int value);
    void on_sliderBodyY_valueChanged(int value);
    void on_sliderBodyZ_valueChanged(int value);
    void on_sliderBodyRoll_valueChanged(int value);
    void on_sliderBodyPitch_valueChanged(int value);
    void on_sliderBodyYaw_valueChanged(int value);
    void on_btnResetPose_clicked();

    // Slots para control IK de pata individual
    void sendCurrentIndividualLegTarget();
    void resetIndividualLegSliders();
    void onComPortChanged(int index);

private:
    Ui::MainWindow* ui;
    Tarantula*      robot_;
    QTimer*         timer_;

    // Punteros para la barra de estado superior
    QComboBox*   comboComPort_{nullptr};
    QComboBox*   comboGaitSelect_{nullptr};

    // Punteros para los botones del D-Pad dinámico (Cruz)
    QPushButton* btnTrotUp_{nullptr};
    QPushButton* btnTrotDown_{nullptr};
    QPushButton* btnTrotLeft_{nullptr};
    QPushButton* btnTrotRight_{nullptr};

    // Punteros para el control IK de pata individual
    QComboBox*   comboLegSelect_{nullptr};
    QSlider*     sliderLegX_{nullptr};
    QSlider*     sliderLegY_{nullptr};
    QSlider*     sliderLegZ_{nullptr};
    QLabel*      lblLegX_{nullptr};
    QLabel*      lblLegY_{nullptr};
    QLabel*      lblLegZ_{nullptr};
    QPushButton* btnResetLeg_{nullptr};

    // Flags para rastrear la pulsación de teclas físicas de dirección
    bool keyUpPressed_{false};
    bool keyDownPressed_{false};
    bool keyLeftPressed_{false};
    bool keyRightPressed_{false};

    void updateTrotVelocity();

    
    void initTable();
    void updateStatusLeds();
    void updateMotorTable();
    void updateSpiderWidget();
    QString buildStiffnessLabel(bool online, bool active, float kp) const;
    void    setTableCell(int row, int col, const QString& text, const QString& color);

    
    void sendCurrentBodyPose();

    int last_valid_body_x_{0};
    int last_valid_body_y_{0};
    int last_valid_body_z_{0};
    int last_valid_body_roll_{0};
    int last_valid_body_pitch_{0};
    int last_valid_body_yaw_{0};
};

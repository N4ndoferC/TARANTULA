#include <QApplication>
#include <iostream>
#include "WaveshareInterface.h"
#include "MotorController.h"
#include "Tarantula.h"
#include "MainWindow.h"
#include "Config.h"

#ifdef _WIN32
#include <windows.h>
#include <timeapi.h>
#endif

const float GEAR_RATIO = 8.0f;
const double SEND_FREQUENCY = 0.02; // 20 ms (50 Hz). La cola asíncrona despacha las 12 tramas en 18 ms (pacing de 1.5 ms necesario para evitar latigazos), dejando 2 ms libres por ciclo.

int main(int argc, char* argv[])
{
#ifdef _WIN32
    timeBeginPeriod(1); // Ajustar la resolución del temporizador de Windows a 1 ms
#endif

    QApplication app(argc, argv);

    std::cout << "Iniciando Waveshare USB-CAN adapter...\n";
    WaveshareInterface comm(Config::SERIAL_PORT, Config::SERIAL_BAUDRATE);
    if (!comm.connect())
        std::cerr << "Error: No se pudo conectar al puerto serial. Modo OFFLINE.\n";

    g_comm = &comm;  //en motorcontroler

    Tarantula robot(comm);
    robot.start();

    MainWindow window(&robot);
    window.show();

    int ret = app.exec();

    robot.stop();
    comm.close();

#ifdef _WIN32
    timeEndPeriod(1); // Restaurar la resolución original del temporizador de Windows
#endif

    return ret;
}

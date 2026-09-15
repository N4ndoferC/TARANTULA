#pragma once
#ifndef NOMINMAX
#define NOMINMAX //esto para q windows no interfiera con funciones matemáticas 
#endif
#include <string>
#include <windows.h> //esta función sirve para dar acceso al sistema operativo
#include <vector>
#include <cstdint>

//SimpleSerial es un objeto con atributos connected (bool) , port (nombre), baudrate y hSerial (handle)

class SimpleSerial {
private:
    HANDLE hSerial; //handle del puerto usb
    bool connected;
    std::string port;
    int baudrate;
    OVERLAPPED osReader;
    OVERLAPPED osWriter;

public:
    SimpleSerial(std::string portName, int baud);
    ~SimpleSerial();

    bool connect();
    void close();
    bool writeBytes(const std::vector<uint8_t>& data);
    int readBytes(std::vector<uint8_t>& buffer, int count);
    int available();
    void flush();
    bool isConnected() const { return connected; }
};

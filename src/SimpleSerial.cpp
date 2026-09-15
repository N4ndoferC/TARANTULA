#include "SimpleSerial.h"
#include <iostream>

SimpleSerial::SimpleSerial(std::string portName, int baud)
    : port(portName), baudrate(baud), connected(false), hSerial(INVALID_HANDLE_VALUE) {
    memset(&osReader, 0, sizeof(OVERLAPPED));
    memset(&osWriter, 0, sizeof(OVERLAPPED));
    osReader.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    osWriter.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
}

SimpleSerial::~SimpleSerial() {
    close();
    if (osReader.hEvent) CloseHandle(osReader.hEvent);
    if (osWriter.hEvent) CloseHandle(osWriter.hEvent);
}

bool SimpleSerial::connect() {

    //abrimos el puerto para lectura y escritura creando un fichero con la función CreateFileA de windows.h
    // Usamos FILE_FLAG_OVERLAPPED para habilitar E/S asíncrona
    hSerial = CreateFileA(port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, 0);
    if (hSerial == INVALID_HANDLE_VALUE) return false;

    //configuramos Device Control Block (DCB)
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) { close(); return false; }

    dcbSerialParams.BaudRate = baudrate; 
    dcbSerialParams.ByteSize = 8; // paquetes de 8 bits
    dcbSerialParams.StopBits = ONESTOPBIT; //1 bit de parada al final
    dcbSerialParams.Parity = NOPARITY; //sin bit de comprobación 

    // Deshabilitar control de flujo explícitamente para evitar bloqueos del driver de Windows
    dcbSerialParams.fOutxCtsFlow = FALSE;
    dcbSerialParams.fOutxDsrFlow = FALSE;
    dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
    dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
    dcbSerialParams.fOutX = FALSE;
    dcbSerialParams.fInX = FALSE;

    if (!SetCommState(hSerial, &dcbSerialParams)) { close(); return false; }

    //COMMTIMEOUTS es una estructura de Windows que define el tiempo que se queda congelado el programa esperando a que llegue un dato
    // COMMTIMEOUTS ajustado para modo Overlapped
    // Con MAXDWORD y el resto a 0, ReadFile retorna inmediatamente con lo que haya.
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;

    SetCommTimeouts(hSerial, &timeouts);
    connected = true;
    return true;
}

void SimpleSerial::close() {
    if (connected) {
        // Cancelar operaciones E/S pendientes antes de cerrar
        CancelIo(hSerial);
        CloseHandle(hSerial);
        connected = false;
    }
}

bool SimpleSerial::writeBytes(const std::vector<uint8_t>& data) {
    if (!connected || data.empty()) return false;
    DWORD bytesSend = 0;
    
    // Escribir asincronamente a través del handle del USB
    if (!WriteFile(hSerial, data.data(), data.size(), &bytesSend, &osWriter)) {
        if (GetLastError() != ERROR_IO_PENDING) {
            return false;
        }
        // Esperamos como máximo 2ms para que el driver USB acabe de tragar el paquete
        DWORD dwRes = WaitForSingleObject(osWriter.hEvent, 2);
        switch (dwRes) {
            case WAIT_OBJECT_0:
                if (!GetOverlappedResult(hSerial, &osWriter, &bytesSend, FALSE)) {
                    return false;
                }
                break;
            default:
                // Timeout o error: abortar la escritura pendiente para evitar colapso
                CancelIo(hSerial);
                return false;
        }
    }
    return (bytesSend == data.size());
}

int SimpleSerial::readBytes(std::vector<uint8_t>& buffer, int count) {
    if (!connected) return 0;
    //ensancha el buffer para que quepan count bytes
    buffer.resize(count);
    DWORD bytesRead = 0;
    
    // Al tener el timeout ReadIntervalTimeout=MAXDWORD, ReadFile lee el puerto y si hay datos retorna instantáneamente
    if (!ReadFile(hSerial, buffer.data(), count, &bytesRead, &osReader)) {
        if (GetLastError() == ERROR_IO_PENDING) {
            if (GetOverlappedResult(hSerial, &osReader, &bytesRead, TRUE)) {
                buffer.resize(bytesRead);
                return bytesRead;
            } else {
                buffer.resize(0);
                return 0;
            }
        }
    }
    //recorta el buffer para no leer basura
    buffer.resize(bytesRead);
    return bytesRead;
}

int SimpleSerial::available() {
    if (!connected) return 0;
    COMSTAT status;
    DWORD errors;
    ClearCommError(hSerial, &errors, &status);
    return status.cbInQue;
}

void SimpleSerial::flush() {

    //si vemos que la cabecerra no es 0xAA 0x55 entonces tiramos el mensaje, asumiéndolo como no válido
    if (connected) PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
}

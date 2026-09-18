# Manual de Análisis de Logs CAN (Tarantula)

Este documento es una guía paso a paso para interpretar los datos guardados por `CanLogger` en el archivo `can_log.csv`, así como para utilizar los scripts de diagnóstico en la carpeta `scripts/`.

## 1. Estructura del Archivo `can_log.csv`

El logger guarda un registro continuo de la comunicación entre el PC (Tarantula) y los motores conectados por el adaptador CAN Waveshare. Cada fila representa un paquete enviado o recibido:

| Timestamp_ms | Direction | CAN_ID | Data_Bytes | Hex_Data |
|--------------|-----------|--------|------------|----------|
| 1789726638566 | RX o TX | 0x161 | 8 | 40 00 00 00 01 89 00 7D |

- **Timestamp_ms**: Momento exacto en milisegundos en que se registró el paquete.
- **Direction**: `TX` (Transmitido desde el PC hacia el bus) o `RX` (Recibido desde el bus hacia el PC).
- **CAN_ID**: El identificador de la trama. Define **hacia quién** va dirigido y **qué tipo de comando** es.
- **Data_Bytes**: La longitud del mensaje (siempre 8 bytes para la comunicación estándar del motor).
- **Hex_Data**: Los datos crudos en formato hexadecimal.

## 2. Descifrando el `CAN_ID`

El `CAN_ID` en este sistema (basado en el protocolo MIT/ODrive) contiene dos piezas de información empaquetadas: **El Node ID del motor** y **El ID del comando**.

Para empaquetarlo, la regla es:
`CAN_ID = (Node ID << 5) | Comando`

Por ejemplo, si el motor 11 (`Node ID = 11`) envía un *Heartbeat* (Comando = `1`):
- 11 en binario es `0000 1011`
- Desplazado 5 posiciones a la izquierda: `0001 0110 0000` (que es `352` en decimal, o `0x160` en hex).
- Sumando el Comando 1 (`0x01`): `0x161`.
Por lo tanto, cualquier `CAN_ID` `0x161` es un Heartbeat del motor 11.

**Tabla de correspondencias (Motor ID a Base CAN ID):**

| Pata | Motor | Node ID (Decimal) | Base CAN ID (Hex) | Rango Empaquetado (`0x..._`) |
|------|-------|-------------------|-------------------|------------------------------|
| 1 (DD) | Coxa  | 11 | `0x160` | `0x160` - `0x16F` |
| 1 (DD) | Fémur | 12 | `0x180` | `0x180` - `0x18F` |
| 1 (DD) | Tibia | 13 | `0x1A0` | `0x1A0` - `0x1AF` |
| 2 (DI) | Coxa  | 21 | `0x2A0` | `0x2A0` - `0x2AF` |
| 2 (DI) | Fémur | 22 | `0x2C0` | `0x2C0` - `0x2CF` |
| 2 (DI) | Tibia | 23 | `0x2E0` | `0x2E0` - `0x2EF` |
| 3 (TI) | Coxa  | 31 | `0x3E0` | `0x3E0` - `0x3EF` |
| 3 (TI) | Fémur | 32 | `0x400` | `0x400` - `0x40F` |
| 3 (TI) | Tibia | 33 | `0x420` | `0x420` - `0x42F` |
| 4 (TD) | Coxa  | 41 | `0x520` | `0x520` - `0x52F` |
| 4 (TD) | Fémur | 42 | `0x540` | `0x540` - `0x54F` |
| 4 (TD) | Tibia | 43 | `0x560` | `0x560` - `0x56F` |

*Nota: El último dígito (el guion bajo en `0x16_`) se reemplaza por el ID del comando (ver a continuación).*

**Comandos habituales y sus terminaciones (en hexadecimal):**
- **Terminan en `1` (ej. `0x161`, `0x2A1`)**: `MW_HEARTBEAT_CMD`. Es el latido del motor. Se envía automáticamente de forma periódica e informa sobre el estado y errores.
- **Terminan en `7` (ej. `0x167`, `0x2A7`)**: `MW_SET_AXIS_STATE_CMD`. Sirve para mandar a encender (`0x08`) o a apagar/IDLE (`0x01`) el motor.
- **Terminan en `8` (ej. `0x168`, `0x2A8`)**: `MW_MIT_CONTROL_CMD`. Es la orden de control. Indica la posición objetivo, velocidad, y las ganancias Kp/Kd.
- **Terminan en `9` (ej. `0x169`, `0x2A9`)**: `MW_GET_ENCODER_ESTIMATES_CMD`. Se usa para preguntar la posición al motor cuando este está inactivo (IDLE).

## 3. Comprendiendo la Trama de Heartbeat (`0x...1`)

El Heartbeat es la trama más útil para diagnosticar apagones repentinos porque contiene los **códigos de error y el estado actual**. La trama RX tiene 8 bytes de datos (`Hex_Data`):

`[Byte 0] [Byte 1] [Byte 2] [Byte 3] [Byte 4] [Byte 5] [Byte 6] [Byte 7]`

- **Bytes 0 a 3 (`axisError`)**: Es un número entero de 32 bits (en formato Little Endian). Un valor de `00 00 00 00` significa que no hay errores. Un valor como `40 00 00 00` se lee como `0x00000040` (`0x40`) e indica que hubo un fallo interno en el motor.
- **Byte 4 (`currentState`)**: Indica el estado de la máquina de estados. `08` significa operando (*Closed Loop Control*). `01` significa apagado/suelto (*IDLE*).
- **Byte 5 (`motorFlags`)**: Banderas específicas de error del hardware del motor (p.ej. `89`).
- **Byte 6 (`encoderFlags`)**: Banderas de error del encoder magnético.
- **Byte 7 (`controllerFlags`)**: Banderas de control de lazo.

*Si notas que un motor pasa de estado `08` a estado `01` por su cuenta y muestra un `axisError != 0`, el motor se ha protegido contra un fallo de hardware (como exceso de corriente, voltaje bajo o pérdida de pasos).*

## 4. Cómo usar el Script de Python (`analizador_can.py`)

He creado un script automático que hace el trabajo pesado de analizar este archivo gigante. Abre tu terminal de PowerShell, navega a la carpeta de tu proyecto y ejecuta el script con Python.

### Modo "heartbeats": Detectar Errores y Apagados
Este modo escanea los millones de líneas y localiza únicamente cuándo los motores lanzaron un error de Hardware (`axisError`).

```powershell
python scripts\analizador_can.py build\Desktop_Qt_6_11_2_MinGW_64_bit_Debug\can_log.csv heartbeats
```
**Qué buscar en la salida**: 
Revisa el "Resumen de estados". Si ves muchas tramas con "IDLE (Apagado)" mientras el robot debería estar andando, hay problemas. Revisa la tabla de "ERRORES DETECTADOS": allí te dirá en qué milisegundo saltó el fallo, en qué motor, y cuál es el código hexadecimal del fallo.

### Modo "trafico": Detectar Cuellos de Botella y Tiempos Muertos
Este modo comprueba si la comunicación se ha "congelado" o si hay pérdidas severas de paquetes entre el PC y los motores.

```powershell
python scripts\analizador_can.py build\Desktop_Qt_6_11_2_MinGW_64_bit_Debug\can_log.csv trafico
```
**Qué buscar en la salida**:
- Fíjate en la columna `Max Gap TX` (Tiempo máximo sin transmitir) y `Max Gap RX` (Tiempo máximo sin recibir). Si mientras el robot opera normalmente estos números suben por encima de `2000 ms`, indica que o bien el hilo del PC se bloqueó, o bien el motor se ha quedado colgado y no responde.
- Si observas gaps gigantescos (~11000 ms a 14000 ms) en los comandos terminados en `8` (como `0x168`), significa que el bucle de control (`MotorController::tick()`) dejó de enviar tramas de movimiento. Esto ocurre normalmente cuando el motor se marca como inactivo (`active = false`).



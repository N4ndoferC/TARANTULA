import serial
import time
from collections import defaultdict

PORT = "\\\\.\\COM3"
BAUDRATE = 2000000

def monitorear():
    print(f"Abriendo {PORT} con alineacion de tramas Waveshare...")
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=0.05)
    except Exception as e:
        print(f"Error abriendo puerto: {e}")
        return

    ser.reset_input_buffer()
    counts = defaultdict(int)
    t_end = time.time() + 3.0
    buffer = bytearray()

    while time.time() < t_end:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buffer.extend(chunk)

        # Procesar tramas de 20 bytes con alineación estricta
        while len(buffer) >= 20:
            if buffer[0] == 0xAA and buffer[1] == 0x55 and buffer[2] == 0x01:
                # Verificar checksum de bytes 2 a 18
                checksum = sum(buffer[2:19]) & 0xFF
                if buffer[19] == checksum:
                    can_id = buffer[5] | (buffer[6] << 8) | (buffer[7] << 16) | (buffer[8] << 24)
                    cmd = can_id & 0x1F
                    node_id = can_id >> 5
                    counts[(node_id, cmd, can_id)] += 1
                    buffer = buffer[20:]
                else:
                    buffer.pop(0)
            else:
                buffer.pop(0)

    ser.close()

    print("\n" + "=" * 70)
    print("DETALLE EXACTO DE TRAFICO EN EL BUS (3 SEGUNDOS):")
    print("=" * 70)
    print(f"{'Node ID':<8} | {'Hex CAN ID':<12} | {'Tipo':<25} | {'Tramas'}")
    print("-" * 70)

    encoder_senders = []
    total_encoders = 0
    total_heartbeats = 0

    for (node_id, cmd, can_id), cnt in sorted(counts.items()):
        if cmd == 1:
            cmd_name = "Heartbeat (0x1)"
            total_heartbeats += cnt
        elif cmd == 9:
            cmd_name = "Encoder Estimate (0x9)"
            encoder_senders.append(node_id)
            total_encoders += cnt
        else:
            cmd_name = f"Comando 0x{cmd:X}"

        print(f"{node_id:<8} | 0x{can_id:03X}       | {cmd_name:<25} | {cnt}")

    print("=" * 70)
    print(f"Total Heartbeats: {total_heartbeats}")
    print(f"Total Tramas Encoder (0x...9): {total_encoders}")
    if encoder_senders:
        print(f"\nMotores que todavia emiten encoder 0x...9: {sorted(list(set(encoder_senders)))}")
    else:
        print("\n¡PERFECTO! Ningún motor está emitiendo tramas de encoder 0x...9.")
    print("=" * 70 + "\n")

if __name__ == '__main__':
    monitorear()

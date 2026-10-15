import csv
import sys
import os

MOTOR_NAMES = {
    11: "Pata 1 (DD) - Coxa",
    12: "Pata 1 (DD) - Fémur",
    13: "Pata 1 (DD) - Tibia",
    21: "Pata 2 (DI) - Coxa",
    22: "Pata 2 (DI) - Fémur",
    23: "Pata 2 (DI) - Tibia",
    31: "Pata 3 (TI) - Coxa",
    32: "Pata 3 (TI) - Fémur",
    33: "Pata 3 (TI) - Tibia",
    41: "Pata 4 (TD) - Coxa",
    42: "Pata 4 (TD) - Fémur",
    43: "Pata 4 (TD) - Tibia",
}

def analizar_saltos(log_path):
    if not os.path.exists(log_path):
        print(f"Error: No se encuentra el archivo {log_path}")
        return

    prev_pos = {}
    prev_time = {}
    max_jumps = {}
    all_large_jumps = []
    first_timestamp = None

    with open(log_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['Direction'] != 'TX':
                continue
            
            try:
                cid_int = int(row['CAN_ID'], 16)
            except ValueError:
                continue

            # Filtrar estrictamente solo comandos MIT (cmd = 0x08)
            cmd = cid_int & 0x1F
            if cmd != 8:
                continue

            node_id = cid_int >> 5
            if node_id not in MOTOR_NAMES:
                continue

            t = int(row['Timestamp_ms'])
            if first_timestamp is None:
                first_timestamp = t
            rel_t_sec = (t - first_timestamp) / 1000.0

            data = [int(b, 16) for b in row['Hex_Data'].split()]
            if len(data) >= 8:
                # Desempaquetar los 16 bits de posición MIT
                pos_int = (data[0] << 8) | data[1]
                pos = (pos_int * 25.0 / 65535.0) - 12.5

                if node_id in prev_pos:
                    dp = abs(pos - prev_pos[node_id])
                    dt = t - prev_time[node_id]
                    
                    if node_id not in max_jumps or dp > max_jumps[node_id][0]:
                        max_jumps[node_id] = (dp, dt, rel_t_sec, prev_pos[node_id], pos)

                    # Si el salto supera 0.1 rad (~5.7 grados en un solo ciclo de 20-30 ms)
                    if dp > 0.10:
                        all_large_jumps.append({
                            't_sec': rel_t_sec,
                            'node_id': node_id,
                            'motor': MOTOR_NAMES[node_id],
                            'dp_rad': dp,
                            'dp_deg': dp * 57.2958,
                            'dt_ms': dt,
                            'from': prev_pos[node_id],
                            'to': pos
                        })

                prev_pos[node_id] = pos
                prev_time[node_id] = t

    print("\n" + "=" * 90)
    print("ANALISIS DE DISCONTINUIDADES EN CONSIGNAS MIT DEL PC (TX)")
    print("=" * 90)
    print(f"{'Motor':<25} | {'Node':<5} | {'Max Salto (rad)':<16} | {'Salto (deg)':<12} | {'En segundo':<10}")
    print("-" * 90)

    for node_id, (dp, dt, t_sec, p0, p1) in sorted(max_jumps.items()):
        deg = dp * 57.2958
        name = MOTOR_NAMES.get(node_id, f"Node {node_id}")
        print(f"{name:<25} | {node_id:<5} | {dp:<16.4f} | {deg:<12.1f} | {t_sec:<10.2f} s")

    print("-" * 90)
    print(f"\nTOTAL DE SALTOS BRUSCOS (> 5.7 grados / 0.10 rad) REGISTRADOS: {len(all_large_jumps)}")
    if all_large_jumps:
        print("\nPrimeros 15 saltos bruscos detectados cronológicamente:")
        print(f"{'Tiempo (s)':<12} | {'Motor':<25} | {'Salto (grados)':<15} | {'dt (ms)':<8} | {'Transición (rad)'}")
        print("-" * 90)
        for j in all_large_jumps[:15]:
            trans = f"{j['from']:.3f} -> {j['to']:.3f}"
            print(f"{j['t_sec']:<12.3f} | {j['motor']:<25} | {j['dp_deg']:<15.1f} | {j['dt_ms']:<8} | {trans}")

    print("\n" + "=" * 90)

if __name__ == '__main__':
    default_log = os.path.join("build", "Desktop_Qt_6_11_2_MinGW_64_bit_Debug", "can_log.csv")
    path = sys.argv[1] if len(sys.argv) > 1 else default_log
    analizar_saltos(path)

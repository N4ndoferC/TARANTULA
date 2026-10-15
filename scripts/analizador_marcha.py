import csv
import sys
import os

MOTOR_NAMES = {
    11: "Pata 1 Coxa", 12: "Pata 1 Fémur", 13: "Pata 1 Tibia",
    21: "Pata 2 Coxa", 22: "Pata 2 Fémur", 23: "Pata 2 Tibia",
    31: "Pata 3 Coxa", 32: "Pata 3 Fémur", 33: "Pata 3 Tibia",
    41: "Pata 4 Coxa", 42: "Pata 4 Fémur", 43: "Pata 4 Tibia",
}

def analizar_intervalo_marcha(log_path, t_start=39.5, t_end=55.5):
    if not os.path.exists(log_path):
        print(f"Error: {log_path} no existe")
        return

    first_t = None
    tx_times = {}
    rx_times = {}
    gaps_tx = {nid: [] for nid in MOTOR_NAMES}
    gaps_rx = {nid: [] for nid in MOTOR_NAMES}
    encoder_rx_count = 0

    with open(log_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = int(row['Timestamp_ms'])
            if first_t is None:
                first_t = t
            t_sec = (t - first_t) / 1000.0

            if t_sec < t_start or t_sec > t_end:
                continue

            try:
                cid = int(row['CAN_ID'], 16)
            except ValueError:
                continue

            node_id = cid >> 5
            cmd = cid & 0x1F

            if row['Direction'] == 'TX' and cmd == 8 and node_id in MOTOR_NAMES:
                if node_id in tx_times:
                    dt = t - tx_times[node_id]
                    if dt > 40: # Gap superior a lo normal (20 ms)
                        gaps_tx[node_id].append((dt, t_sec))
                tx_times[node_id] = t

            elif row['Direction'] == 'RX' and cmd == 8 and node_id in MOTOR_NAMES:
                if node_id in rx_times:
                    dt = t - rx_times[node_id]
                    if dt > 50: # Silencio de respuesta superior a 50 ms
                        gaps_rx[node_id].append((dt, t_sec))
                rx_times[node_id] = t

            elif row['Direction'] == 'RX' and cmd == 9:
                encoder_rx_count += 1

    print("\n" + "=" * 80)
    print(f"ANALISIS DE SALUD DEL BUS CAN DURANTE LA MARCHA ({t_start:.1f}s a {t_end:.1f}s = {t_end - t_start:.1f}s)")
    print("=" * 80)
    print(f"Total tramas parásitas de encoder (0x...9) en este intervalo: {encoder_rx_count} tramas")
    print(f"Tasa de bombardeo en el bus: ~{encoder_rx_count / (t_end - t_start):.0f} tramas/segundo\n")

    print(f"{'Motor':<18} | {'Gaps TX > 40ms':<16} | {'Max Gap TX':<12} | {'Silencios RX > 50ms':<22} | {'Max Gap RX':<12}")
    print("-" * 80)

    for nid, name in sorted(MOTOR_NAMES.items()):
        gtx = gaps_tx[nid]
        grx = gaps_rx[nid]
        max_gtx = max([g[0] for g in gtx]) if gtx else 20
        max_grx = max([g[0] for g in grx]) if grx else 20
        print(f"{name:<18} | {len(gtx):<16} | {max_gtx:<12} ms | {len(grx):<22} | {max_grx:<12} ms")

    print("=" * 80)

if __name__ == '__main__':
    default_log = os.path.join("build", "Desktop_Qt_6_11_2_MinGW_64_bit_Debug", "can_log.csv")
    path = sys.argv[1] if len(sys.argv) > 1 else default_log
    analizar_intervalo_marcha(path)

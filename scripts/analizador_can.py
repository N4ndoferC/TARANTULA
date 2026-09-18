import csv
import sys
from collections import defaultdict

def print_help():
    print("Uso: python analizador_can.py <archivo.csv> <modo>")
    print("Modos disponibles:")
    print("  trafico    - Muestra estadísticas de TX/RX, pérdidas y latencias máximas.")
    print("  heartbeats - Analiza las tramas Heartbeat (Comando 1) en busca de errores y cambios de estado.")

def analizar_trafico(filepath):
    tx_counts = defaultdict(int)
    rx_counts = defaultdict(int)
    last_tx_time = {}
    max_tx_gap = defaultdict(int)
    last_rx_time = {}
    max_rx_gap = defaultdict(int)
    
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                timestamp = int(row['Timestamp_ms'])
                direction = row['Direction']
                can_id = row['CAN_ID']
                
                if direction == 'TX':
                    tx_counts[can_id] += 1
                    if can_id in last_tx_time:
                        gap = timestamp - last_tx_time[can_id]
                        if gap > max_tx_gap[can_id]: max_tx_gap[can_id] = gap
                    last_tx_time[can_id] = timestamp
                elif direction == 'RX':
                    rx_counts[can_id] += 1
                    if can_id in last_rx_time:
                        gap = timestamp - last_rx_time[can_id]
                        if gap > max_rx_gap[can_id]: max_rx_gap[can_id] = gap
                    last_rx_time[can_id] = timestamp
            except ValueError:
                pass
                
    print(f"\n{'CAN ID':<10} | {'TX Count':<10} | {'Max Gap TX (ms)':<16} | {'RX Count':<10} | {'Max Gap RX (ms)':<16}")
    print("-" * 75)
    all_ids = sorted(set(tx_counts.keys()).union(set(rx_counts.keys())))
    for cid in all_ids:
        print(f"{cid:<10} | {tx_counts[cid]:<10} | {max_tx_gap[cid]:<16} | {rx_counts[cid]:<10} | {max_rx_gap[cid]:<16}")

def analizar_heartbeats(filepath):
    errores = []
    estados = defaultdict(int)
    
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                if row['Direction'] == 'RX' and row['CAN_ID'].endswith('1'):
                    hex_data = row['Hex_Data'].strip().split()
                    if len(hex_data) == 8:
                        # Extraer información del Heartbeat (Little Endian)
                        axis_error = int(hex_data[3], 16) << 24 | int(hex_data[2], 16) << 16 | int(hex_data[1], 16) << 8 | int(hex_data[0], 16)
                        current_state = int(hex_data[4], 16)
                        
                        estados[current_state] += 1
                        
                        if axis_error != 0:
                            errores.append({
                                'tiempo': row['Timestamp_ms'],
                                'motor': row['CAN_ID'],
                                'error_hex': hex(axis_error),
                                'estado': current_state,
                                'motor_flag': hex_data[5],
                                'encoder_flag': hex_data[6],
                                'control_flag': hex_data[7]
                            })
            except ValueError:
                pass
                
    print("\n=== RESUMEN DE ESTADOS DE LOS MOTORES ===")
    for est, count in estados.items():
        nom_est = "IDLE (Apagado)" if est == 1 else ("Closed Loop Control" if est == 8 else f"Estado {est}")
        print(f"{nom_est}: {count} tramas")
        
    print(f"\n=== ERRORES DETECTADOS: {len(errores)} ===")
    if errores:
        print("Muestra de los primeros 20 errores:")
        print(f"{'Tiempo (ms)':<15} | {'Motor':<10} | {'axisError':<12} | {'Estado':<10} | {'Banderas (M/E/C)'}")
        print("-" * 75)
        for e in errores[:20]:
            flags = f"{e['motor_flag']} / {e['encoder_flag']} / {e['control_flag']}"
            print(f"{e['tiempo']:<15} | {e['motor']:<10} | {e['error_hex']:<12} | {e['estado']:<10} | {flags}")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print_help()
        sys.exit(1)
        
    archivo = sys.argv[1]
    modo = sys.argv[2].lower()
    
    if modo == 'trafico':
        analizar_trafico(archivo)
    elif modo == 'heartbeats':
        analizar_heartbeats(archivo)
    else:
        print_help()

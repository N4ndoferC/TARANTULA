import sys
import time

try:
    import odrive
except ImportError:
    print("\n[AVISO] La libreria 'odrive' no esta instalada.")
    print("Ejecuta en tu terminal: pip install odrive\n")
    sys.exit(1)

def main():
    print("=" * 65)
    print("CONFIGURACION DE MOTOR STEADYWIN GIM6010-8 POR USB")
    print("Silenciar broadcast ciclico de encoder (encoder_rate_ms = 0)")
    print("=" * 65)
    print("Buscando motor conectado por USB Type-C...")
    print("(Asegurate de que el motor tiene alimentacion de 24V y el USB conectado)")
    
    try:
        odrv = odrive.find_any(timeout=10)
    except Exception as e:
        print(f"\n[ERROR] No se pudo encontrar el motor por USB: {e}")
        return

    print(f"\n[OK] Motor detectado con exito!")
    
    # 1. Leer parametros actuales
    try:
        node_id = odrv.axis0.config.can.node_id
        hb_rate = odrv.axis0.config.can.heartbeat_rate_ms
        enc_rate = odrv.axis0.config.can.encoder_rate_ms
        print(f" - CAN Node ID actual:        {node_id}")
        print(f" - Heartbeat rate actual:     {hb_rate} ms")
        print(f" - Encoder rate actual:       {enc_rate} ms (telemetria ciclica)")
    except Exception as e:
        print(f"[AVISO] No se pudieron leer algunos parametros: {e}")

    # 2. Modificar encoder_rate_ms a 0
    print("\nConfigurando encoder_rate_ms = 0...")
    try:
        odrv.axis0.config.can.encoder_rate_ms = 0
        print("[OK] encoder_rate_ms establecido a 0 (desactivado).")
    except Exception as e:
        print(f"[ERROR] Fallo al escribir encoder_rate_ms: {e}")
        return

    # 3. Guardar en memoria Flash permanente
    print("\nGuardando configuracion permanente en memoria Flash...")
    try:
        odrv.save_configuration()
    except Exception:
        # El comando save_configuration() reinicia el motor, desconectando el USB temporalmente.
        pass

    print("[OK] Configuracion guardada con exito en Flash.")
    print("El motor se ha reiniciado y ya NO emitira tramas de posicion sin solicitarse.")
    print("=" * 65 + "\n")

if __name__ == '__main__':
    main()

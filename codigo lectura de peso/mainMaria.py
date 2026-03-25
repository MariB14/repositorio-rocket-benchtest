#!/usr/bin/python3
# -*- coding:utf-8 -*-

import spidev, RPi.GPIO as GPIO, time, sys, termios, tty, select, csv, os
from collections import deque
import math
import json

# ==============================
# CONFIGURACIÓN DE PINES / SPI
# ==============================
RST_PIN, CS_PIN, DRDY_PIN = 18, 22, 17
RELAY_PIN = 23  # Pin para control de relay (ignición)

SPI = spidev.SpiDev(0, 0)

def digital_write(pin, val): GPIO.output(pin, val)
def digital_read(pin): return GPIO.input(pin)
def delay_ms(ms): time.sleep(ms/1000.0)
def spi_writebyte(data): SPI.writebytes(data)
def spi_readbytes(n): return SPI.readbytes(n)

def module_init():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(RST_PIN, GPIO.OUT)
    GPIO.setup(CS_PIN, GPIO.OUT)
    GPIO.setup(DRDY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RELAY_PIN, GPIO.OUT, initial=GPIO.LOW)
    print(f"🔧 RELAY_PIN {RELAY_PIN} configurado como OUTPUT")
    SPI.max_speed_hz, SPI.mode = 20000, 0b01
    return 0

# ==============================
# ADS1256
# ==============================
ADS1256_GAIN_E = {1:0, 2:1, 4:2, 8:3, 16:4, 32:5, 64:6}
ADS1256_DRATE_E = {'30SPS':0xF0, '60SPS':0xF1, '100SPS':0x82, '500SPS':0x85}
REG_E = {'REG_MUX':1,'REG_ADCON':2,'REG_DRATE':3}
CMD = {'CMD_WREG':0x50,'CMD_RDATA':0x01}

class ADS1256:
    def __init__(self): 
        self.rst_pin, self.cs_pin, self.drdy_pin = RST_PIN, CS_PIN, DRDY_PIN
    
    def reset(self):
        digital_write(self.rst_pin,1); delay_ms(50)
        digital_write(self.rst_pin,0); delay_ms(50)
        digital_write(self.rst_pin,1); delay_ms(50)
    
    def write_reg(self, reg, data):
        digital_write(self.cs_pin,0)
        spi_writebyte([CMD['CMD_WREG']|reg,0,data])
        digital_write(self.cs_pin,1)
    
    def wait_drdy(self): 
        timeout = time.time() + 1.0
        while digital_read(self.drdy_pin)==1:
            if time.time() > timeout:
                raise Exception("DRDY timeout")
            pass
    
    def config(self, gain, drate):
        self.wait_drdy()
        digital_write(self.cs_pin,0)
        spi_writebyte([CMD['CMD_WREG']|0,0x03])
        spi_writebyte([0x01,0x08,ADS1256_GAIN_E[gain],drate])
        digital_write(self.cs_pin,1)
        delay_ms(1)
    
    def set_diff_ch(self): 
        self.write_reg(REG_E['REG_MUX'], (0<<4)|1)  # AIN0-AIN1
    
    def init(self, gain, drate):
        if module_init()!=0: return -1
        self.reset()
        self.config(gain, drate)
        return 0
    
    def read_data(self):
        self.wait_drdy()
        digital_write(self.cs_pin,0)
        spi_writebyte([CMD['CMD_RDATA']])
        b=spi_readbytes(3)
        digital_write(self.cs_pin,1)
        raw=(b[0]<<16)|(b[1]<<8)|b[2]
        if raw & 0x800000: raw -= 0x1000000
        return raw

# ==============================
# CONFIGURACIÓN Y CALIBRACIÓN
# ==============================
CONFIG_FILE = "thrust_config.json"

def load_config():
    """Carga configuración guardada o usa defaults"""
    default = {
        "GAIN": 16,
        "DRATE": "100SPS",
        "VREF": 5.0,
        "calibration_factor": 1.0,  # kgf = mV * factor
        "raw_zero": 0
    }
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r') as f:
                return json.load(f)
        except:
            return default
    return default

def save_config(config):
    """Guarda configuración actual"""
    with open(CONFIG_FILE, 'w') as f:
        json.dump(config, f, indent=2)

# ==============================
# FILTROS
# ==============================
def calc_alpha(sps, tau=0.03):
    """Calcula alpha para filtro EMA (tau=30ms default)"""
    Ts = 1.0/sps
    return 1 - math.exp(-Ts/tau)

def median3(buf):
    """Filtro mediana de 3 muestras"""
    if len(buf) < 3: return buf[-1]
    a,b,c = list(buf)[-3:]
    return sorted([a,b,c])[1]

def code_to_mV(delta, gain, vref, code_fs=0x7fffff):
    """Convierte código ADC a milivoltios"""
    return (delta * (vref/gain) / code_fs) * 1000.0

# ==============================
# UTILIDADES TECLADO
# ==============================
def set_cbreak():
    fd=sys.stdin.fileno()
    st=termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return fd,st

def restore_term(fd,st): 
    termios.tcsetattr(fd,termios.TCSADRAIN,st)

def kbhit(): 
    dr,_,_=select.select([sys.stdin],[],[],0)
    return dr!=[]

def getch(): 
    return sys.stdin.read(1)

def input_with_default(prompt, default):
    """Input con valor por defecto"""
    val = input(f"{prompt} [default: {default}]: ").strip()
    return val if val else str(default)

# ==============================
# MODO 1: CALIBRACIÓN
# ==============================
def modo_calibracion():
    """Menú de calibración"""
    print("\n" + "="*60)
    print("🔧 MODO CALIBRACIÓN")
    print("="*60)
    print("1. Calibración manual (pesos verticales)")
    print("2. Configurar parámetros (GAIN, SPS)")
    print("3. Calibración automática con pesos conocidos")
    print("4. Volver al menú principal")
    
    opcion = input("\nSelecciona opción: ").strip()
    
    if opcion == "1":
        calibracion_manual()
    elif opcion == "2":
        configurar_parametros()
    elif opcion == "3":
        calibracion_automatica()
    elif opcion == "4":
        return
    else:
        print("❌ Opción inválida")

def calibracion_manual():
    """Calibración manual con pesos conocidos"""
    config = load_config()
    
    print("\n" + "="*60)
    print("📏 CALIBRACIÓN MANUAL")
    print("="*60)
    print("Instrucciones:")
    print("1. Coloca el banco en posición VERTICAL")
    print("2. Coloca un peso conocido")
    print("3. El sistema calculará el factor de calibración")
    print("="*60)
    
    # Inicializar ADC
    adc = ADS1256()
    gain = config["GAIN"]
    drate = config["DRATE"]
    
    if adc.init(gain, ADS1256_DRATE_E[drate]) != 0:
        print("❌ Error inicializando ADC")
        return
    
    adc.set_diff_ch()
    
    # Tare inicial
    print("\n🔄 Realizando tare (sin peso)...")
    input("Presiona ENTER cuando esté sin peso...")
    N = 20
    raw_zero = sum(adc.read_data() for _ in range(N)) // N
    config["raw_zero"] = raw_zero
    print(f"✅ Tare establecido: raw = {raw_zero}")
    
    # Lectura con peso conocido
    print("\n⚖️  Coloca el peso conocido en posición vertical")
    peso_kg = float(input("Peso conocido (kg): "))
    input("Presiona ENTER cuando esté el peso colocado...")
    
    # Promediar lecturas
    print("📊 Promediando 50 lecturas...")
    readings = []
    for _ in range(50):
        adc.set_diff_ch()
        raw = adc.read_data()
        delta = raw - raw_zero
        mv = code_to_mV(delta, gain, config["VREF"])
        readings.append(mv)
        time.sleep(0.01)
    
    avg_mv = sum(readings) / len(readings)
    
    # Calcular factor
    if avg_mv != 0:
        calibration_factor = peso_kg / avg_mv
        config["calibration_factor"] = calibration_factor
        
        print(f"\n✅ Calibración completada:")
        print(f"   Voltaje promedio: {avg_mv:.3f} mV")
        print(f"   Peso conocido: {peso_kg} kg")
        print(f"   Factor calibración: {calibration_factor:.6f} kg/mV")
        
        save_config(config)
        print("\n💾 Configuración guardada")
    else:
        print("❌ Error: voltaje cero, verifica conexiones")
    
    input("\nPresiona ENTER para continuar...")

def configurar_parametros():
    """Configurar GAIN y SPS"""
    config = load_config()
    
    print("\n" + "="*60)
    print("⚙️  CONFIGURAR PARÁMETROS")
    print("="*60)
    print(f"Configuración actual:")
    print(f"  GAIN: {config['GAIN']}")
    print(f"  SPS: {config['DRATE']}")
    print(f"  Factor calibración: {config['calibration_factor']:.6f}")
    print("="*60)
    
    # GAIN
    print("\nGAIN disponibles: 1, 2, 4, 8, 16, 32, 64")
    gain_input = input_with_default("Nuevo GAIN", config["GAIN"])
    try:
        new_gain = int(gain_input)
        if new_gain in ADS1256_GAIN_E:
            config["GAIN"] = new_gain
        else:
            print("⚠️  GAIN inválido, manteniendo anterior")
    except:
        print("⚠️  Entrada inválida, manteniendo anterior")
    
    # SPS
    print("\nSPS disponibles: 30SPS, 60SPS, 100SPS, 500SPS")
    drate_input = input_with_default("Nuevo SPS", config["DRATE"])
    if drate_input in ADS1256_DRATE_E:
        config["DRATE"] = drate_input
    else:
        print("⚠️  SPS inválido, manteniendo anterior")
    
    save_config(config)
    print("\n✅ Parámetros guardados")
    input("\nPresiona ENTER para continuar...")

def calibracion_automatica():
    """Calibración automática con múltiples pesos"""
    config = load_config()
    
    print("\n" + "="*60)
    print("🤖 CALIBRACIÓN AUTOMÁTICA")
    print("="*60)
    print("Este proceso usará 3 pesos conocidos: 1kg, 10kg, 20kg")
    print("Coloca el banco en posición VERTICAL")
    print("="*60)
    
    pesos_conocidos = [1.0, 10.0, 20.0]  # kg
    
    # Inicializar ADC
    adc = ADS1256()
    gain = config["GAIN"]
    drate = config["DRATE"]
    
    if adc.init(gain, ADS1256_DRATE_E[drate]) != 0:
        print("❌ Error inicializando ADC")
        return
    
    adc.set_diff_ch()
    
    # Tare
    print("\n🔄 Realizando tare (sin peso)...")
    input("Presiona ENTER cuando esté sin peso...")
    N = 20
    raw_zero = sum(adc.read_data() for _ in range(N)) // N
    config["raw_zero"] = raw_zero
    print(f"✅ Tare: {raw_zero}")
    
    # Recolectar datos de calibración
    voltajes = []
    for peso in pesos_conocidos:
        print(f"\n⚖️  Coloca peso de {peso} kg")
        input("Presiona ENTER cuando esté colocado...")
        
        # Promediar
        readings = []
        for _ in range(30):
            adc.set_diff_ch()
            raw = adc.read_data()
            delta = raw - raw_zero
            mv = code_to_mV(delta, gain, config["VREF"])
            readings.append(mv)
            time.sleep(0.01)
        
        avg_mv = sum(readings) / len(readings)
        voltajes.append(avg_mv)
        print(f"   Voltaje promedio: {avg_mv:.3f} mV")
    
    # Regresión lineal simple (y = mx)
    # m = sum(x*y) / sum(x*x)
    sum_xy = sum(p * v for p, v in zip(pesos_conocidos, voltajes))
    sum_xx = sum(v * v for v in voltajes)
    
    if sum_xx != 0:
        calibration_factor = sum_xy / sum_xx
        config["calibration_factor"] = calibration_factor
        
        print(f"\n✅ Calibración completada:")
        print(f"   Factor: {calibration_factor:.6f} kg/mV")
        print(f"\n   Verificación:")
        for peso, voltaje in zip(pesos_conocidos, voltajes):
            estimado = voltaje * calibration_factor
            error = abs(estimado - peso) / peso * 100
            print(f"   {peso}kg → {voltaje:.2f}mV → {estimado:.2f}kg (error: {error:.1f}%)")
        
        save_config(config)
        print("\n💾 Configuración guardada")
    else:
        print("❌ Error en calibración")
    
    input("\nPresiona ENTER para continuar...")

# ==============================
# MODO 2: PRUEBA CON IGNICIÓN
# ==============================
def modo_prueba():
    """Modo de prueba con countdown e ignición"""
    config = load_config()
    
    print("\n" + "="*60)
    print("🚀 MODO PRUEBA - BANCO DE EMPUJE")
    print("="*60)
    
    # Parámetros de prueba
    print("\n⏱️  CONFIGURACIÓN DE TIEMPOS:")
    timer_ignicion = int(input_with_default("Tiempo ANTES de ignición (segundos)", 60))
    duracion_relay = int(input_with_default("Duración activación relay (segundos)", 30))
    duracion_total = int(input_with_default("Duración TOTAL de grabación (segundos)", 120))
    
    if timer_ignicion >= duracion_total:
        print("❌ Error: timer de ignición debe ser menor que duración total")
        input("\nPresiona ENTER para continuar...")
        return
    
    print(f"\n📋 RESUMEN:")
    print(f"   t=0s → Inicia grabación + countdown")
    print(f"   t={timer_ignicion}s → IGNICIÓN (relay ON por {duracion_relay}s)")
    print(f"   t={timer_ignicion + duracion_relay}s → RELAY OFF")
    print(f"   t={duracion_total}s → Fin de prueba")
    print(f"\n   GAIN: {config['GAIN']}")
    print(f"   SPS: {config['DRATE']}")
    print(f"   Factor calibración: {config['calibration_factor']:.6f} kg/mV")
    
    confirm = input("\n¿Iniciar prueba? (s/n): ").strip().lower()
    if confirm != 's':
        print("❌ Prueba cancelada")
        return
    
    # Inicializar ADC
    adc = ADS1256()
    gain = config["GAIN"]
    drate = config["DRATE"]
    vref = config["VREF"]
    calibration_factor = config["calibration_factor"]
    
    if adc.init(gain, ADS1256_DRATE_E[drate]) != 0:
        print("❌ Error inicializando ADC")
        return
    
    adc.set_diff_ch()
    
    # TEST MANUAL DEL RELAY
    print("\n🔧 TEST DEL RELAY:")
    print("Activando relay por 3 segundos...")
    GPIO.output(RELAY_PIN, GPIO.HIGH)
    print(f"Estado GPIO después de HIGH: {GPIO.input(RELAY_PIN)}")
    time.sleep(3)
    GPIO.output(RELAY_PIN, GPIO.LOW)
    print(f"Estado GPIO después de LOW: {GPIO.input(RELAY_PIN)}")
    test_ok = input("¿Se activó el relay correctamente? (s/n): ").strip().lower()
    
    if test_ok != 's':
        print("⚠️  ADVERTENCIA: El relay no funcionó en el test.")
        print("Verifica las conexiones antes de continuar.")
        continuar = input("¿Continuar de todas formas? (s/n): ").strip().lower()
        if continuar != 's':
            return
    
    # Usar tare guardado o hacer uno nuevo
    usar_tare = input("\n¿Usar tare guardado? (s/n): ").strip().lower()
    if usar_tare == 's':
        raw_zero = config["raw_zero"]
        print(f"✅ Usando tare guardado: {raw_zero}")
    else:
        print("🔄 Realizando nuevo tare...")
        N = 20
        raw_zero = sum(adc.read_data() for _ in range(N)) // N
        config["raw_zero"] = raw_zero
        save_config(config)
        print(f"✅ Nuevo tare: {raw_zero}")
    
    # Setup filtros
    sps_val = int(drate.replace("SPS", ""))
    alpha = calc_alpha(sps_val)
    history = deque(maxlen=3)
    ema_val = 0
    
    # Iniciar prueba
    print("\n" + "="*60)
    print("🔥 INICIANDO PRUEBA EN 3 SEGUNDOS...")
    print("="*60)
    time.sleep(3)
    
    relay_activado = False
    relay_apagado = False  # Nueva bandera para evitar múltiples prints
    
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"thrust_test_{timestamp}.csv"
    
    try:
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["tiempo_s", "raw", "delta", "voltaje_mV", 
                           "fuerza_kgf", "relay_status", "GAIN", "SPS"])
            
            start_time = time.time()
            
            print("\n⏺️  GRABANDO DATOS...")
            print("Presiona Ctrl+C para detener manualmente\n")
            
            while True:
                t = time.time() - start_time
                
                # Verificar si terminar
                if t >= duracion_total:
                    break
                
                # CONTROL DEL RELAY - VERSIÓN CORREGIDA
                tiempo_fin_relay = timer_ignicion + duracion_relay
                
                if t >= timer_ignicion and t < tiempo_fin_relay:
                    # Periodo de activación
                    if not relay_activado:
                        print(f"\n{'🔥'*20}")
                        print(f"⚡ IGNICIÓN ACTIVADA - t={t:.2f}s")
                        print(f"⚡ Relay permanecerá activo hasta t={tiempo_fin_relay:.2f}s")
                        print(f"{'🔥'*20}\n")
                        relay_activado = True
                    GPIO.output(RELAY_PIN, GPIO.HIGH)
                else:
                    # Fuera del periodo de activación
                    if relay_activado and not relay_apagado:
                        GPIO.output(RELAY_PIN, GPIO.LOW)
                        print(f"\n{'⚠️ '*20}")
                        print(f"⚠️  RELAY APAGADO - t={t:.2f}s")
                        print(f"{'⚠️ '*20}\n")
                        relay_apagado = True
                    elif not relay_activado:
                        GPIO.output(RELAY_PIN, GPIO.LOW)
                
                # Leer datos
                adc.set_diff_ch()
                raw = adc.read_data()
                delta = raw - raw_zero
                mv = code_to_mV(delta, gain, vref)
                
                # Filtros
                history.append(mv)
                mv_med = median3(history)
                ema_val = alpha * mv_med + (1 - alpha) * ema_val
                
                # Convertir a fuerza
                fuerza_kgf = ema_val * calibration_factor
                
                # Estado relay
                relay_status = "ON" if GPIO.input(RELAY_PIN) == GPIO.HIGH else "OFF"
                
                # Guardar y mostrar
                writer.writerow([f"{t:.3f}", raw, delta, f"{ema_val:.6f}", 
                               f"{fuerza_kgf:.6f}", relay_status, gain, drate])
                
                # Display con countdown visual
                if t < timer_ignicion:
                    countdown = timer_ignicion - t
                    print(f"⏳ t={t:6.2f}s | COUNTDOWN: {countdown:5.1f}s | "
                          f"V={ema_val:8.3f}mV | F={fuerza_kgf:9.2f}kgf | "
                          f"Relay={relay_status}", end='\r')
                else:
                    print(f"🚀 t={t:6.2f}s | Raw={raw:7d} | Δ={delta:7d} | "
                          f"V={ema_val:8.3f}mV | F={fuerza_kgf:9.2f}kgf | "
                          f"Relay={relay_status}", end='\r')
                
                time.sleep(0.01)
        
        # Asegurar relay apagado
        GPIO.output(RELAY_PIN, GPIO.LOW)
        
        print(f"\n\n✅ PRUEBA COMPLETADA")
        print(f"📁 Datos guardados en: {filename}")
        print(f"⏱️  Duración: {duracion_total}s")
        
    except KeyboardInterrupt:
        GPIO.output(RELAY_PIN, GPIO.LOW)
        print(f"\n\n⚠️  PRUEBA INTERRUMPIDA (Ctrl+C)")
        print(f"📁 Datos parciales en: {filename}")
    except Exception as e:
        GPIO.output(RELAY_PIN, GPIO.LOW)
        print(f"\n\n❌ ERROR: {e}")
        print(f"📁 Datos parciales en: {filename}")
    
    input("\nPresiona ENTER para continuar...")

# ==============================
# MENÚ PRINCIPAL
# ==============================
def menu_principal():
    """Menú principal del sistema"""
    while True:
        print("\n" + "="*60)
        print("🔬 SISTEMA DE BANCO DE PRUEBAS - THRUST TEST")
        print("="*60)
        
        # Mostrar config actual
        config = load_config()
        print(f"\n📊 Configuración actual:")
        print(f"   GAIN: {config['GAIN']}")
        print(f"   SPS: {config['DRATE']}")
        print(f"   Factor calibración: {config['calibration_factor']:.6f} kg/mV")
        print(f"   Tare: {config['raw_zero']}")
        
        print("\n" + "="*60)
        print("1. 🔧 MODO CALIBRACIÓN")
        print("2. 🚀 MODO PRUEBA (con ignición)")
        print("3. ❌ SALIR")
        print("="*60)
        
        opcion = input("\nSelecciona opción: ").strip()
        
        if opcion == "1":
            modo_calibracion()
        elif opcion == "2":
            modo_prueba()
        elif opcion == "3":
            print("\n👋 Saliendo del sistema...")
            break
        else:
            print("❌ Opción inválida")

# ==============================
# MAIN
# ==============================
if __name__ == "__main__":
    try:
        menu_principal()
    except KeyboardInterrupt:
        print("\n\n⚠️  Sistema interrumpido")
    except Exception as e:
        print(f"\n\n❌ ERROR FATAL: {e}")
    finally:
        try:
            GPIO.output(RELAY_PIN, GPIO.LOW)  # Seguridad
            print("🔒 Relay asegurado en OFF")
        except:
            pass
        GPIO.cleanup()
        print("✅ GPIO liberado - Sistema cerrado de forma segura")
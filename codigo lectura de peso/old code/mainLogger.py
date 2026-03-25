#!/usr/bin/python3
# -*- coding:utf-8 -*-

import spidev, RPi.GPIO as GPIO, time, sys, termios, tty, select, csv
from collections import deque
import math

# ==============================
# CONFIGURACIÓN DE PINES / SPI
# ==============================
RST_PIN, CS_PIN, DRDY_PIN = 18, 22, 17
SPI = spidev.SpiDev(0, 0)

def digital_write(pin, val): GPIO.output(pin, val)
def digital_read(pin): return GPIO.input(DRDY_PIN)
def delay_ms(ms): time.sleep(ms/1000.0)
def spi_writebyte(data): SPI.writebytes(data)
def spi_readbytes(n): return SPI.readbytes(n)

def module_init():
    GPIO.setmode(GPIO.BCM); GPIO.setwarnings(False)
    GPIO.setup(RST_PIN, GPIO.OUT); GPIO.setup(CS_PIN, GPIO.OUT)
    GPIO.setup(DRDY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
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
    def __init__(self): self.rst_pin, self.cs_pin, self.drdy_pin = RST_PIN, CS_PIN, DRDY_PIN
    def reset(self):
        digital_write(self.rst_pin,1); delay_ms(50)
        digital_write(self.rst_pin,0); delay_ms(50)
        digital_write(self.rst_pin,1); delay_ms(50)
    def write_reg(self, reg, data):
        digital_write(self.cs_pin,0); spi_writebyte([CMD['CMD_WREG']|reg,0,data]); digital_write(self.cs_pin,1)
    def wait_drdy(self): 
        while digital_read(self.drdy_pin)==1: pass
    def config(self, gain, drate):
        self.wait_drdy()
        digital_write(self.cs_pin,0)
        spi_writebyte([CMD['CMD_WREG']|0,0x03])
        spi_writebyte([0x01,0x08,ADS1256_GAIN_E[gain],drate])
        digital_write(self.cs_pin,1); delay_ms(1)
    def set_diff_ch(self): self.write_reg(REG_E['REG_MUX'], (0<<4)|1) # AIN0-AIN1
    def init(self, gain, drate):
        if module_init()!=0: return -1
        self.reset(); self.config(gain, drate); return 0
    def read_data(self):
        self.wait_drdy(); digital_write(self.cs_pin,0)
        spi_writebyte([CMD['CMD_RDATA']]); b=spi_readbytes(3); digital_write(self.cs_pin,1)
        raw=(b[0]<<16)|(b[1]<<8)|b[2]
        if raw & 0x800000: raw -= 0x1000000
        return raw

# ==============================
# TECLADO
# ==============================
def set_cbreak():
    fd=sys.stdin.fileno(); st=termios.tcgetattr(fd); tty.setcbreak(fd); return fd,st
def restore_term(fd,st): termios.tcsetattr(fd,termios.TCSADRAIN,st)
def kbhit(): dr,_,_=select.select([sys.stdin],[],[],0); return dr!=[]
def getch(): return sys.stdin.read(1)

# ==============================
# MAIN CONFIG
# ==============================
VREF = 5.0
GAIN = 16
DRATE = '100SPS'
CODE_FS = 0x7fffff
FS_mV = 18.75   # calibración

# Filtro EMA adaptativo
def calc_alpha(sps, tau=0.03):   # tau=30 ms
    Ts = 1.0/sps
    return 1 - math.exp(-Ts/tau)

# Mediana 3
def median3(buf):
    if len(buf) < 3: return buf[-1]
    a,b,c = list(buf)[-3:]
    return sorted([a,b,c])[1]

def code_to_mV(delta, gain): return (delta * (VREF/gain) / CODE_FS) * 1000.0

# ==============================
# MAIN
# ==============================
try:
    duracion = int(input("⏱️ ¿Cuánto tiempo quieres muestrear (segundos)? "))

    adc=ADS1256(); adc.init(GAIN, ADS1256_DRATE_E[DRATE]); adc.set_diff_ch()
    N=20; raw_zero=sum(adc.read_data() for _ in range(N))//N
    fd,old=set_cbreak()
    print("Tare inicial raw=%d\nPresiona: t=tare | g=ganancia | s=sampling | q=salir\n"%raw_zero)

    # Setup filtros
    history = deque(maxlen=3)
    alpha = calc_alpha(100)  # valor inicial para 100 SPS
    ema_val = 0

    with open("thrust-curve.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["tiempo_s","raw","delta","voltaje_mV","fuerza_kgf","GAIN","SPS"])

        start=time.time()
        while (time.time()-start)<duracion:
            adc.set_diff_ch(); raw=adc.read_data()
            delta=raw-raw_zero
            mv=code_to_mV(delta, GAIN)

            # Filtro mediana + EMA
            history.append(mv)
            mv_med = median3(history)
            ema_val = alpha*mv_med + (1-alpha)*ema_val

            fuerza=(ema_val*FS_mV)
            t=time.time()-start

            writer.writerow([f"{t:.3f}", raw, delta, f"{ema_val:.6f}", f"{fuerza:.6f}", GAIN, DRATE])
            print("t=%6.2fs GAIN=%2d SPS=%6s Raw=%7d Δ=%7d Voltaje=%8.3f mV Fuerza=%9.2f kgf"%
                  (t,GAIN,DRATE,raw,delta,ema_val,fuerza))
            time.sleep(0.01)

            if kbhit():
                ch=getch().lower()
                if ch=='t':
                    raw_zero=sum(adc.read_data() for _ in range(N))//N
                    print("\n>>> Nuevo tare raw=%d\n"%raw_zero)
                elif ch=='g':
                    # Ciclar ganancias
                    GAIN = {4:8, 8:16, 16:64, 64:4}[GAIN]
                    adc.config(GAIN, ADS1256_DRATE_E[DRATE])
                    print("\n>>> GAIN cambiado a %d\n"%GAIN)
                elif ch=='s':
                    # Ciclar SPS
                    DRATE = {'30SPS':'60SPS','60SPS':'100SPS','100SPS':'500SPS','500SPS':'30SPS'}[DRATE]
                    adc.config(GAIN, ADS1256_DRATE_E[DRATE])
                    sps_val = int(DRATE.replace("SPS",""))
                    alpha = calc_alpha(sps_val)
                    print("\n>>> SPS cambiado a %s (alpha=%.3f)\n"%(DRATE,alpha))
                elif ch=='q':
                    print("\n>>> Muestreo terminado por usuario\n")
                    break

    print("\n✅ Muestreo terminado. Datos guardados en thrust-curve.csv")

except KeyboardInterrupt:
    print("\n>>> Interrumpido con Ctrl+C")
finally:
    try: restore_term(fd,old)
    except: pass
    GPIO.cleanup(); print("GPIO liberado")

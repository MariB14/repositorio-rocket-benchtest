# main ADS1256.py
# Lee la celda de carga en AIN0-AIN1, imprime en mV y Newtons

import time
import ADS1256
import RPi.GPIO as GPIO

# Parámetros de la celda
VREF = 5.0        # Voltaje de referencia configurado en la placa
FS_N = 2943       # 300 kg ≈ 2943 N
FS_mV = 9.0       # 1 mV/V * 9 V excitación = 9 mV salida máxima

try:
    ADC = ADS1256.ADS1256()
    ADC.ADS1256_init()

    while True:
        # Seleccionamos el canal diferencial AIN0-AIN1
        ADC.ADS1256_SetDiffChannal(0)
        raw = ADC.ADS1256_Read_ADC_Data()

        # Convertir a voltaje (mV)
        voltage = (raw * VREF / 0x7fffff) * 1000
        # Convertir a Newtons
        fuerza = (voltage / FS_mV) * FS_N

        print("Celda = %.3f mV   (%.2f N)" % (voltage, fuerza))
        time.sleep(0.01)  # ~100 muestras/segundo

except KeyboardInterrupt:
    GPIO.cleanup()
    print("\r\nPrograma terminado")
    exit()

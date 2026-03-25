# ADS1256.py
# Controlador del ADC ADS1256 para Raspberry Pi

import config
import RPi.GPIO as GPIO

# Ganancias internas del ADS1256 (PGA)
ADS1256_GAIN_E = {
    'ADS1256_GAIN_1'  : 0,
    'ADS1256_GAIN_2'  : 1,
    'ADS1256_GAIN_4'  : 2,
    'ADS1256_GAIN_8'  : 3,
    'ADS1256_GAIN_16' : 4,
    'ADS1256_GAIN_32' : 5,
    'ADS1256_GAIN_64' : 6,
}

# Tasa de muestreo
ADS1256_DRATE_E = {
    'ADS1256_100SPS' : 0x82,  # Usaremos esta (100 samples/segundo)
}

# Registros internos del chip
REG_E = {
    'REG_STATUS' : 0,
    'REG_MUX'    : 1,
    'REG_ADCON'  : 2,
    'REG_DRATE'  : 3,
}

# Comandos del chip
CMD = {
    'CMD_WREG'  : 0x50,
    'CMD_RREG'  : 0x10,
    'CMD_SYNC'  : 0xFC,
    'CMD_WAKEUP': 0x00,
    'CMD_RDATA' : 0x01,
}

class ADS1256:
    def __init__(self):
        self.rst_pin  = config.RST_PIN
        self.cs_pin   = config.CS_PIN
        self.drdy_pin = config.DRDY_PIN

    def ADS1256_reset(self):
        config.digital_write(self.rst_pin, GPIO.HIGH)
        config.delay_ms(200)
        config.digital_write(self.rst_pin, GPIO.LOW)
        config.delay_ms(200)
        config.digital_write(self.rst_pin, GPIO.HIGH)

    def ADS1256_WriteCmd(self, reg):
        config.digital_write(self.cs_pin, GPIO.LOW)
        config.spi_writebyte([reg])
        config.digital_write(self.cs_pin, GPIO.HIGH)

    def ADS1256_WriteReg(self, reg, data):
        config.digital_write(self.cs_pin, GPIO.LOW)
        config.spi_writebyte([CMD['CMD_WREG'] | reg, 0x00, data])
        config.digital_write(self.cs_pin, GPIO.HIGH)

    def ADS1256_Read_data(self, reg):
        config.digital_write(self.cs_pin, GPIO.LOW)
        config.spi_writebyte([CMD['CMD_RREG'] | reg, 0x00])
        data = config.spi_readbytes(1)
        config.digital_write(self.cs_pin, GPIO.HIGH)
        return data

    def ADS1256_WaitDRDY(self):
        while(config.digital_read(self.drdy_pin) == 1):
            pass

    def ADS1256_ConfigADC(self, gain, drate):
        self.ADS1256_WaitDRDY()
        buf = [0, 0, 0, 0]
        buf[0] = 0x01  # STATUS: Auto-cal OFF
        buf[1] = 0x08  # MUX default
        buf[2] = gain  # ADCON: configurar ganancia
        buf[3] = drate # DRATE: frecuencia de muestreo
        config.digital_write(self.cs_pin, GPIO.LOW)
        config.spi_writebyte([CMD['CMD_WREG'] | 0, 0x03])
        config.spi_writebyte(buf)
        config.digital_write(self.cs_pin, GPIO.HIGH)
        config.delay_ms(1)

    def ADS1256_SetDiffChannal(self, Channal):
        # Canal diferencial 0 = AIN0-AIN1 (ideal para tu celda)
        if Channal == 0:
            self.ADS1256_WriteReg(REG_E['REG_MUX'], (0 << 4) | 1)

    def ADS1256_init(self):
        if (config.module_init() != 0):
            return -1
        self.ADS1256_reset()
        # Configuración: GAIN=64, DRATE=100SPS
        self.ADS1256_ConfigADC(ADS1256_GAIN_E['ADS1256_GAIN_64'],
                               ADS1256_DRATE_E['ADS1256_100SPS'])
        return 0

    def ADS1256_Read_ADC_Data(self):
        self.ADS1256_WaitDRDY()
        config.digital_write(self.cs_pin, GPIO.LOW)
        config.spi_writebyte([CMD['CMD_RDATA']])
        buf = config.spi_readbytes(3)
        config.digital_write(self.cs_pin, GPIO.HIGH)
        read = (buf[0] << 16) | (buf[1] << 8) | buf[2]
        if (read & 0x800000):  # Conversión a signed
            read -= 0x1000000
        return read

"""ADS1220 example (print configuration register settings)."""
from machine import Pin, SPI  # type: ignore
from ads1220 import ADC

cs = 15  # Chip select pin
drdy = 27  # Data ready pin
spi = SPI(1,
          baudrate=10000000,  # 10 MHz (try lower speed to troubleshoot)
          sck=Pin(14),
          mosi=Pin(13),
          miso=Pin(12),
          phase=1)  # ADS1220 uses SPI mode 1
adc = ADC(spi, cs, drdy)


def test():
    """Test code."""
    config_registers = adc.get_config_dict()

    for key, value in sorted(config_registers.items()):
        print("{0}: {1}".format(key, value))

    adc.power_down()
    spi.deinit()


test()

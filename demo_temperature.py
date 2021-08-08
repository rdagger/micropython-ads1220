"""ADS1220 example (polling temperature)"""
from time import sleep
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
    adc.conversion_continuous()  # Set continuous conversion mode
    adc.start_conversion()  # Start conversions
    adc.temperature_mode(enable=True)  # Enable temperature mode
    sleep(.1)  # Ensure temperature mode is set

    try:
        while True:
            t = adc.read_temperature()
            tf = t * 1.8 + 32
            print("Temperature: {0:.2f}\xb0 C,  {1:.2f}\xb0 F".format(t, tf))
            sleep(3)
    except KeyboardInterrupt:
        print("\nCtrl-C pressed to exit.")
    finally:
        adc.power_down()
        spi.deinit()


test()

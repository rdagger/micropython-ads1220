"""ADS1220 example (polling ADC).
   Uses single shot mode and wait for data ready."""
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
    vref = 2.048  # Internal voltage reference
    res = 8388607  # ADC resolution 23 bit (2^23, assumes 1 bit polarity)

    adc.conversion_single_shot()  # Set single shot conversion mode
    adc.select_channel(0)  # Select channel 0 (0 to 3 ADC channels)
    sleep(.1)  # Ensure ADC ready

    try:
        while True:
            adc.start_conversion()  # Conversion must be started each shot
            reading = adc.read_wait()
            v = reading * vref / res
            print("raw: {0}, volts: {1}".format(reading, v))
            sleep(3)
    except KeyboardInterrupt:
        print("\nCtrl-C pressed to exit.")
    finally:
        adc.power_down()
        spi.deinit()


test()

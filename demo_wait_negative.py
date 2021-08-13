"""ADS1220 example (monitor for negative voltage)."""
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
    adc.pga_off()  # Disable gain
    adc.fir_filter(1)  # Simultaneous 50-Hz and 60-Hz rejection
    adc.operating_mode(2)  # Turbo mode
    adc.data_rate(2)  # 180 SPS
    adc.start_conversion()  # Start conversions
    adc.select_channel(0)  # Select channel 0 (0 to 3 ADC channels)
    sleep(.1)  # Ensure ADC ready

    try:
        while True:
            result = adc.read_wait_negative(timeout=1000)

            if result:
                print("Negative voltage acquired.")
            else:
                print("Timeout.")
            sleep(3)

    except KeyboardInterrupt:
        print("\nCtrl-C pressed to exit.")
    finally:
        adc.power_down()
        spi.deinit()


test()

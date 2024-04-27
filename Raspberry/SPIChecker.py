import pigpio
import time

# Configure the SPI
SPI_CHANNEL = 0
SPI_SPEED = 500000
SPI_FLAGS = 0

# Initialize the PiGPIO library
pi = pigpio.pi()

# Open SPI connection
handle = pi.spi_open(SPI_CHANNEL, SPI_SPEED, SPI_FLAGS)

# Data to send
tx_data = bytes([i for i in range(256)])

# Buffer to receive data
rx_data = [0] * len(tx_data)

while True:
    try:
        # Transfer data over SPI
        (count, rx_data) = pi.spi_xfer(handle, tx_data)
        if rx_data != tx_data:
            print("fail")

        # Delay
        time.sleep(0.001)

    except:
        # Close SPI connection
        pi.spi_close(handle)

        # Cleanup
        pi.stop()

        break

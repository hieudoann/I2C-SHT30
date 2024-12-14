# Raspberry Pi 4 with SHT30 Sensor (I2C Communication)

This project demonstrates how to use a **Raspberry Pi 4** to read data from an **SHT30 temperature and humidity sensor** via the I2C communication protocol. The SHT30 is a high-accuracy digital sensor that outputs temperature and relative humidity values, making it ideal for environmental monitoring projects.

---

## Table of Contents

- [Requirements](#requirements)
- [Hardware Setup](#hardware-setup)
- [Software Setup](#software-setup)
- [Wiring Diagram](#wiring-diagram)
- [Usage](#usage)
- [Python Code Example](#python-code-example)
- [C Code Example](#c-code-example)
- [References](#references)

---

## Requirements

### Hardware
- Raspberry Pi 4 (any model with GPIO pins)
- SHT30 Sensor Module (I2C version)
- Breadboard and jumper wires

### Software
- Raspbian OS (Bullseye or newer)
- Python 3
- Required Python Libraries:
  - `smbus2`
  - `time`
- GCC Compiler (for C code)

---

## Hardware Setup

### Steps:
1. **Enable I2C on Raspberry Pi**:
   - Open a terminal and run:
     ```bash
     sudo raspi-config
     ```
   - Navigate to `Interfacing Options` > `I2C` and enable it.
   - Reboot the Raspberry Pi if necessary.

2. **Connect the SHT30 sensor to the Raspberry Pi**:
   - Use jumper wires to connect the sensor's pins to the Raspberry Pi GPIO pins as per the wiring diagram below.

---

## Wiring Diagram

| **SHT30 Pin** | **Raspberry Pi Pin** |
|---------------|----------------------|
| VCC           | 3.3V (Pin 1)         |
| GND           | GND (Pin 6)          |
| SDA           | SDA1 (Pin 3, GPIO 2) |
| SCL           | SCL1 (Pin 5, GPIO 3) |

---

## Software Setup

### 1. Update and Install Required Packages
Run the following commands to ensure your system is up to date and to install the required libraries:
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install python3-smbus python3-pip gcc make -y
pip3 install smbus2
```

### 2. Test I2C Connection
Use the following command to check if the Raspberry Pi detects the SHT30 sensor:
```bash
i2cdetect -y 1
```
The SHT30's default address (`0x44` or `0x45`) should appear on the I2C bus.

---

## Usage

### Python Code Example
Save the following Python script (e.g., `sht30_reader.py`) to read the sensor data:

```python
import smbus2
import time

# Define I2C address and bus
SHT30_I2C_ADDRESS = 0x44
bus = smbus2.SMBus(1)

def read_sht30():
    # Send measurement command to the sensor
    bus.write_i2c_block_data(SHT30_I2C_ADDRESS, 0x2C, [0x06])
    time.sleep(0.5)  # Wait for the measurement

    # Read 6 bytes of data from the sensor
    data = bus.read_i2c_block_data(SHT30_I2C_ADDRESS, 0x00, 6)

    # Convert temperature and humidity values
    temp = -45 + (175 * ((data[0] << 8) + data[1]) / 65535.0)
    humidity = 100 * ((data[3] << 8) + data[4]) / 65535.0

    return temp, humidity

try:
    while True:
        temperature, humidity = read_sht30()
        print(f"Temperature: {temperature:.2f} °C, Humidity: {humidity:.2f}%")
        time.sleep(2)  # Delay between readings
except KeyboardInterrupt:
    print("Program terminated.")
```

### Run the Script
Execute the script with the following command:
```bash
python3 sht30_reader.py
```
You should see real-time temperature and humidity readings displayed in the terminal.

---

### C Code Example
Save the following C code to a file (e.g., `sht30_reader.c`) and compile it using GCC:

```c
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#define SHT30_ADDR 0x44

int main() {
    int fd;
    char *filename = (char *)"/dev/i2c-1";
    char buf[6];

    if ((fd = open(filename, O_RDWR)) < 0) {
        perror("Failed to open the i2c bus");
        exit(1);
    }

    if (ioctl(fd, I2C_SLAVE, SHT30_ADDR) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        close(fd);
        exit(1);
    }

    while (1) {
        // Send measurement command
        char cmd[2] = {0x2C, 0x06};
        if (write(fd, cmd, 2) != 2) {
            perror("Failed to write to the i2c device");
            close(fd);
            exit(1);
        }

        // Wait for the measurement to complete
        usleep(500000); // 500 ms

        // Read 6 bytes of data
        if (read(fd, buf, 6) != 6) {
            perror("Failed to read from the i2c device");
            close(fd);
            exit(1);
        }

        // Convert the data
        int temp_raw = (buf[0] << 8) | buf[1];
        int hum_raw = (buf[3] << 8) | buf[4];

        double temperature = -45 + (175.0 * temp_raw / 65535.0);
        double humidity = 100.0 * hum_raw / 65535.0;

        printf("Temperature: %.2f°C, Humidity: %.2f%%\n", temperature, humidity);

        sleep(1); // Wait for 1 second
    }

    close(fd);
    return 0;
}
```

### Compile and Run the C Program
To compile the code:
```bash
gcc -o sht30_reader sht30_reader.c
```

To run the program:
```bash
sudo ./sht30_reader
```

You should see the temperature and humidity readings printed to the terminal in real time.

---

## References
- [SHT30 Datasheet](https://www.sensirion.com/file/datasheet_sht3x)
- [Raspberry Pi Documentation](https://www.raspberrypi.org/documentation/)

Feel free to modify or expand the project to suit your specific needs!

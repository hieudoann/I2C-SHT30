# Raspberry Pi 4 with SHT30 Sensor (I2C Communication)

This project demonstrates how to use a **Raspberry Pi 4** to read data from an **SHT30 temperature and humidity sensor** via the I2C communication protocol. The SHT30 is a high-accuracy digital sensor that outputs temperature and relative humidity values, making it ideal for environmental monitoring projects.

---

## Table of Contents

- [Requirements](#requirements)
- [Hardware Setup](#hardware-setup)
- [Software Setup](#software-setup)
- [Wiring Diagram](#wiring-diagram)
- [Usage](#usage)
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
sudo apt install python3-smbus python3-pip -y
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

## References
- [SHT30 Datasheet](https://www.sensirion.com/file/datasheet_sht3x)
- [Raspberry Pi Documentation](https://www.raspberrypi.org/documentation/)

Feel free to modify or expand the project to suit your specific needs!

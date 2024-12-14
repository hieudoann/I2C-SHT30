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
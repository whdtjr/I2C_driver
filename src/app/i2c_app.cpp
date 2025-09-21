#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>

#define I2C_DEVICE   "/dev/i2c-1"
#define I2C_ADDR     0x0A

static void reset_i2c_bus(int fd)
{
    printf("Attempting I2C bus reset...\n");
    ioctl(fd, I2C_SLAVE, I2C_ADDR);
}

static void safe_die(int fd, const char *msg)
{
    perror(msg);
    reset_i2c_bus(fd);
    printf("Bus reset completed\n");
}

int main(void)
{
    int fd = open(I2C_DEVICE, O_RDWR);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    if (ioctl(fd, I2C_SLAVE, I2C_ADDR) < 0) {
        perror("ioctl(I2C_SLAVE)");
        close(fd);
        return 1;
    }

    uint8_t tx = 0xAA;
    uint8_t rx = 0x00;

    puts("STM32 I2C loop-back test with recovery (Ctrl-C to quit)");

    while (1)
    {
        /* 1) 쓰기 */
        if (write(fd, &tx, 1) != 1) {
            continue;
        }

        /* 2) 읽기 */
        if (read(fd, &rx, 1) != 1) {
            continue;
        }

        printf("TX 0x%02X → RX 0x%02X\n", tx, rx);
        
        tx++;
    }

    close(fd);
    return 0;
}
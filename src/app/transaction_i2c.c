#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>

#define I2C_DEVICE "/dev/stm32_i2c0"
#define I2C_ADDR   0x0a
#define MAX_RETRIES 5

static int xfer_wr_rd(int fd, uint8_t w, uint8_t *r)
{
    struct i2c_rdwr_ioctl_data pkt;
    struct i2c_msg msgs[2];

    msgs[0].addr  = I2C_ADDR;
    msgs[0].flags = 0;          // write
    msgs[0].len   = 1;
    msgs[0].buf   = &w;

    msgs[1].addr  = I2C_ADDR;
    msgs[1].flags = I2C_M_RD;   // read
    msgs[1].len   = 1;
    msgs[1].buf   = r;

    pkt.msgs  = msgs;
    pkt.nmsgs = 2;

    // 한 번의 호출로 write -> (Repeated START) -> read -> STOP
    return ioctl(fd, I2C_RDWR, &pkt);
}

int main(void)
{
    int fd = open(I2C_DEVICE, O_RDWR);
    if (fd < 0) { perror("open"); return 1; }

    // if (ioctl(fd, I2C_SLAVE, I2C_ADDR) < 0) {
    //     perror("ioctl(I2C_SLAVE)");
    //     close(fd);
    //     return 1;
    // }

    uint8_t tx = 0xAA, rx = 0x00;
    int retry = 0;

    puts("STM32 I2C combined-transfer loop (Ctrl-C to quit)");

    for (;;)
    {
        if (xfer_wr_rd(fd, tx, &rx) < 0) {
            int err = errno;
            perror("I2C_RDWR");
            // 주소 NACK: ENXIO, 데이터 중 NACK/버스오류: EREMOTEIO 가 흔함
            if (++retry >= MAX_RETRIES) break;

            // 어댑터가 Busy/EAGAIN이면 아주 살짝 쉬었다 재시도
            if (err == EAGAIN || err == EBUSY) usleep(1000);
            continue;
        }

        printf("TX 0x%02X → RX 0x%02X\n", tx, rx);
        retry = 0;
        tx++;
    }

    close(fd);
    return 0;
}

# BSP 2주차 - I2C

# I2C 통신 방식

<img width="500" height="500" alt="Image" src="https://github.com/user-attachments/assets/b0c4e770-c5a8-4db2-af66-d793c726ab8a" />

I2C는 SDA, SCL 그리고 GND를 연결하여 서로 통신하는 프로토콜이다. SDA는 데이터를 주고 받는 통신선이고 SCL은 클럭신호를 보내는 통신선이다.

주소가 선을 꽂을 수 있는 환경만 있다면 2^7개의 slave를 1개의 master와 연결시킬 수 있다. 왜냐하면 7비트 주소를 이용하여 address를 사용하기 때문이다. 1 bit는 read, write 비트 

그리고 I2C는 master와 slave 간의 전압이 같아야 통신할 수 있다. 왜냐하면 Master와 Slave는 데이터 전송이 성공했는지에 대한 ACK 신호를 보내기 때문에 전압이 같아야 HIGH, LOW를 제대로 판단할 수 있기 때문이다.

# **Linux에서 I2C device가 연결되었는지 확인하는 방법**

### **STM32의 통신 interrupt 통신 준비**

```c
sudo i2cdetect -y 1

// 결과
// STM의 Device address 10(dex) 0a(hex) 으로 설정
0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- 0a -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- -- 
```

```c
//main.c
if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
{
  Error_Handler();
}
//i2c_slave.c
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c,
                          uint8_t TransferDirection,
                          uint16_t AddrMatchCode)

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)

void HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxBuffer, 1, I2C_FIRST_AND_LAST_FRAME);

void HAL_I2C_Slave_Seq_Transmit_IT(hi2c, TxBuffer, 1, I2C_FIRST_AND_LAST_FRAME);
```

Master에서 read 호출 시 Slave에서는 write Tx 명령 마스터는 슬레이브로부터 데이터를 읽음

Master에서 write 호출 시 Slave에서는 read Rx 명령 마스터는 슬레이브로부터 데이터를 씀

**Raspberry pi code**

Raspberry pi kernel에는 i2c 디바이스 드라이버가 내장되어 insmod 되어 있어서 application 코드가 있다면 바로 이용할 수 있다.

**확인 명령어**

```c
cat /proc/devices 
```

# application 작성 할 때 참고

**ioctl 관련 정의 목록**

```c
#define I2C_RETRIES	0x0701	/* number of times a device address should
				   be polled when not acknowledging */
#define I2C_TIMEOUT	0x0702	/* set timeout in units of 10 ms */

/* NOTE: Slave address is 7 or 10 bits, but 10-bit addresses
 * are NOT supported! (due to code brokenness)
 */
#define I2C_SLAVE	0x0703	/* Use this slave address */
#define I2C_SLAVE_FORCE	0x0706	/* Use this slave address, even if it
				   is already in use by a driver! */
#define I2C_TENBIT	0x0704	/* 0 for 7 bit addrs, != 0 for 10 bit */

#define I2C_FUNCS	0x0705	/* Get the adapter functionality mask */

#define I2C_RDWR	0x0707	/* Combined R/W transfer (one STOP only) */

#define I2C_PEC		0x0708	/* != 0 to use PEC with SMBus */
#define I2C_SMBUS	0x0720	/* SMBus transfer */
```

**application 작성 시** 

```c
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
```

**DMA 설정 시 I2C 이용** 

Raspberry pi에서는 따로 처리해줄 것은 없다.

```c
// 기존 (인터럽트)
HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxBuffer, 1, I2C_FIRST_AND_LAST_FRAME);
HAL_I2C_Slave_Seq_Transmit_IT(hi2c, TxBuffer, 1, I2C_FIRST_AND_LAST_FRAME);

// 변경 (DMA)
HAL_I2C_Slave_Receive_DMA(hi2c, dma_rxBuffer, sizeof(dma_rxBuffer));
HAL_I2C_Slave_Transmit_DMA(hi2c, dma_txBuffer, sizeof(dma_txBuffer));

```

그런데 위 코드의 문제는 raspberry pi에서는 stm이 rx, tx 처리를 하고 다시 listen을 준비하는 과정에 대한 interrupt처리를 하지 않는 것이 문제이다. 

그런데 I2C에서는 Master에서는 Slave의 신호를 받아서 처리하지 못하고 Slave만이 Master의 신호를 받아서 interrupt가 가능하게 된다고 한다.

그러면 **STM에서 여러 인터럽트에 대한 지연 때문에 I2C의 listen 준비가 늦어질 수 있게 되는 문제**가 생긴다.

그래서 application 에서는 delay를 이용하게 되는데 이것을 완화하기 위한 방법 및 예방책이 있다.

# 한계정리

**통신 프로토콜**

I2C 통신에서는 SCL이  오픈 드레인 상태에 의해 high로 유지되는데 Master에서 SCL 신호를 보내고 High-Z 상태로 두게 된다. 이때 Slave가 강제로 Low 신호를 보낼 때까지 대기하게 됨 ( 수신 완료 ) -> 그리고 Master에서 High로 다시 전환하여 데이터 전송

Slave에서 데이터 처리 시간이 필요할 때, SCL을 Low로 유지하여 클럭 스트레칭: 마스터는 대기상태

그래서 delay를 따로 줄 필요가 없이 delay가 내장되어 있음

**소프트웨어적인 한계**

Slave에서는 Tx, Rx 처리 후에 HAL_I2C_EnableListen_IT 함수를 이용해서 다시 통신을 준비하게 되는데 해당 Master에서 대기하거나 신호를 받을 수 없다면 Listen 준비까지 시간을 보장할 수 없다.

**한계 요약**

Master – Slave 선 연결 필요

Master – Slave 의 구조에 따른 신호 한계 : Master의 일방적인 clock 신호 전달

Clock은 항상 Master에서만 SCL을 통해 생성되게 되고 Slave는 거기에 맞게 동작

## 대안: 추가 GPIO 인터럽트 라인 사용

device tree에서는 raspberry pi 같이 master만 이용할 수 있는 보드 및 i2c에 대한 제한을 완화하기 위해서

gpio를 이용한 interrupt pin을 device tree에서 따로 설정해줄 수 있다.

```c
/dts-v1/;
/plugin/;

/ {
    compatible = "brcm,bcm2711";

    fragment@0 {
        target = <&i2c1>;
        __overlay__ {
            status = "okay";
            
            stm32_comm: stm32@0a {
                compatible = "st,stm32-i2c-slave";
                reg = <0x0a>;
                status = "okay";
                
                // STM32 특화 속성들
                slave-mode;
                clock-frequency = <100000>;
                
                // 인터럽트 핀 (선택사항)
                interrupt-parent = <&gpio>;
                interrupts = <25 IRQ_TYPE_EDGE_FALLING>;
                
                // 리셋 핀 (선택사항)
                reset-gpios = <&gpio 24 GPIO_ACTIVE_LOW>;
            };
        };
    };
}
```

해당 핀을 쓴다면 stm에서 listen 하게 될 때 따로 GPIO를 toggle하여 raspberry pi에서는 poll 및 select로  interrupt를 이용할 수 있을 것이라 생각

# SMBus

SMBus는 Intel이 I2C를 기반으로 개발한 시스템 관리 전용 프로토콜로, 배터리 관리, 전원 모니터링, 온도 센서 등에 특화되어 있음.

**SMBus vs I2C 차이점**:

| 특징 | I2C | SMBus |
| --- | --- | --- |
| 클럭 속도 | 100kHz~3.4MHz | 10kHz~100kHz (더 느림) |
| 타임아웃 | 없음 | 35ms 타임아웃 필수 |
| 최소 HIGH/LOW | 없음 | 엄격한 타이밍 요구사항 |
| 프로토콜 | 자유로운 형태 | 정의된 명령 구조 |

**SMBus timeout**

```c
text클럭 스트레칭 제한: 최대 35ms
- 슬레이브가 SCL을 35ms 이상 LOW로 유지하면 에러
- 모든 장치가 버스 리셋 수행
- I2C와 달리 무한 대기 불가능
```

**Raspberry pi code**

```c
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>        // SMBus 전용 헤더

// SMBus Write Byte
i2c_smbus_write_byte(fd, 0xAA);

// SMBus Read Byte  
int result = i2c_smbus_read_byte(fd);

// 또는 SMBus Write Byte Data
i2c_smbus_write_byte_data(fd, command, data);
int result = i2c_smbus_read_byte_data(fd, command);
```

- `write_byte` / `read_byte` → **command 없이 1바이트만** 주고받음.
- `write_byte_data` / `read_byte_data` → **command + 데이터** 주고받음.

command는 Master에서 알아서 정해서 값을 넘김

command가 생기므로 STM에서는 따로 처리해주어야함

이것을 flag로 이용하거나, 레지스터를 넘기는 용도로 이용가능

**command 이용 예시**

```c
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c,
                          uint8_t TransferDirection,
                          uint16_t AddrMatchCode)
{
    if (TransferDirection == I2C_DIRECTION_TRANSMIT) { // transfer direction으로 확인
		    //Master가 transmit 전달한 것 stm에서는 읽어야함
        // 커맨드만 받고, 데이터 수신은 조건부로 처리
        smbus_state = SMB_IDLE;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, dma_rx_cmd, 1, I2C_NEXT_FRAME);
    } else { 
		    // Master가 receive 받는 것 : stm에서는 써야함
        // Read phase에서는 바로 응답
        i2c_prepare_tx_from_regmap();
        dma_tx_data[0] = 3;
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, dma_tx_data, 1, I2C_LAST_FRAME);
    }
}
```

**맨 처음 전달되는 주소 매칭 callback에서 command를 이용하여 상태를 관리할 수 있다.**

```c
typedef enum {
    SMB_IDLE = 0,
    SMB_GOT_CMD_WAIT_DATA,  // 커맨드 1B 수신 완료, write_byte_data의 Data 1B를 읽는다
} smbus_state_t;
```

**STM 32 코드** 

**처리 후 상태를 IDLE로 초기화 하여 관리한다** 

```c
    if (smbus_state == SMB_IDLE) {
        /* 1) 방금 커맨드 1B를 받았음 */
        last_cmd = dma_rx_cmd[0];
        smbus_state = SMB_GOT_CMD_WAIT_DATA;

        HAL_I2C_Slave_Seq_Receive_IT(hi2c, dma_rx_data, 1, I2C_LAST_FRAME);

    } else if (smbus_state == SMB_GOT_CMD_WAIT_DATA) {
        /* write_byte_data의 Data 1B가 정상 수신된 경우 */
        uint8_t data = dma_rx_data[0];
        regmap[last_cmd] = data;

        smbus_state = SMB_IDLE;
        rx_done = 1;  /* 필요하면 유지 */
        HAL_I2C_EnableListen_IT(hi2c);
```

**Error callback을 이용한 통신 유지**

```c
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    uint32_t ec = hi2c->ErrorCode;

    printf("I2C Error: 0x%08X, State: %d\n", ec, hi2c->State);

    // 모든 DMA 강제 중단
    if (hi2c->hdmarx && hi2c->hdmarx->State != HAL_DMA_STATE_READY) {
        HAL_DMA_Abort(hi2c->hdmarx);
    }
    if (hi2c->hdmatx && hi2c->hdmatx->State != HAL_DMA_STATE_READY) {
        HAL_DMA_Abort(hi2c->hdmatx);
    }

    // I2C 상태 강제 리셋
    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    // 상태 머신 초기화
    smbus_state = SMB_IDLE;

    // 리스닝 재시작
    if (HAL_I2C_EnableListen_IT(hi2c) != HAL_OK) {
        printf("Failed to restart listening, will retry...\n");
        // 하드웨어 리셋 후 재시도
        HAL_I2C_DeInit(hi2c);
        HAL_Delay(10);
        HAL_I2C_Init(hi2c);
        HAL_I2C_EnableListen_IT(hi2c);
    }
}
```

# Combine Transaction 처리

만약 read, write의 과정이 한번에 필요할 때, 외부의 간섭 없이 read-write가 완료되면 stop하게 되는 방법

**필요한 설정 : raspberry pi**

```c
struct i2c_rdwr_ioctl_data pkt;
struct i2c_msg msgs[2];
    
 // 구조체 내용   
struct i2c_rdwr_ioctl_data {
	struct i2c_msg *msgs;	/* pointers to i2c_msgs */
	__u32 nmsgs;			/* number of i2c_msgs */
};

struct i2c_msg {
	__u16 addr;
	__u16 flags;
#define I2C_M_RD		0x0001	/* guaranteed to be 0x0001! */
#define I2C_M_TEN		0x0010	/* use only if I2C_FUNC_10BIT_ADDR */
#define I2C_M_DMA_SAFE		0x0200	/* use only in kernel space */
#define I2C_M_RECV_LEN		0x0400	/* use only if I2C_FUNC_SMBUS_READ_BLOCK_DATA */
#define I2C_M_NO_RD_ACK		0x0800	/* use only if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK	0x1000	/* use only if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_REV_DIR_ADDR	0x2000	/* use only if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NOSTART		0x4000	/* use only if I2C_FUNC_NOSTART */
#define I2C_M_STOP		0x8000	/* use only if I2C_FUNC_PROTOCOL_MANGLING */
	__u16 len;
	__u8 *buf;
};
```

**Application code**

```c
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
```

write 인지 read인지 정하고 write시 전달할 버퍼, read 시 읽어올 버퍼 지정 및 개수 설정을 하고 ioctl로 한번에 넘긴다.

# **device tree 작성 예시**

```jsx
/dts-v1/;
/plugin/;
/ {
	compatible = "brcm,bcm2711";

	fragment@0 {
		target = <0xffffffff>;

		__overlay__ {
			#address-cells = <0x01>;
			#size-cells = <0x00>;

			stm32@a {
				compatible = "st,stm32-i2c-slave";
				reg = <0x0a>;
				status = "okay";
				slave-mode = <0x01>;
				clock-frequency = <100000>;
				phandle = <0x01>;
			};
		};
	};

	__symbols__ {
		stm32_comm = "/fragment@0/__overlay__/stm32@a";
	};

	__fixups__ {
		i2c1 = "/fragment@0:target:0";
	};
};

```

**dts 파일 컴파일**

```jsx
 dtc -@ -I dts -O dtb -o test.dtbo test.dts
 
// 아래 경로로 들어가서 작성됐는지 확인
ls /proc/device-tree/

# I2C 노드의 상태 확인
ls /proc/device-tree/soc/i2c@XXXXXXX/

# overlay로 추가/수정된 노드 확인
cat /proc/device-tree/soc/i2c@XXXXXXX/node-a/status
```

# **Driver code**

```c
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/i2c-dev.h>
#include <linux/ioctl.h>
#include <linux/slab.h>

#define I2C_BUFSIZE 32
#define DRIVER_NAME "stm32_i2c_client"
#define DEVICE_NAME "stm32_i2c"
#define I2C_RDWR_IOCTL_MAX_MSGS 42  // 추가 필요

static dev_t dev_number;
static struct class *stm32_class;
static struct cdev stm32_cdev;
static struct stm32_i2c_data *global_data;

struct stm32_i2c_data {
    struct i2c_client *client;
    u8 buf[I2C_BUFSIZE];
    struct device *dev;
    __u16 slave_addr;
};

// I2C id table
static const struct i2c_device_id stm32_i2c_id[] = {
    { "stm32-i2c-slave" },  // 커널 6.x에서는 두 번째 인자 생략 가능
    { }
};
MODULE_DEVICE_TABLE(i2c, stm32_i2c_id);

// Device Tree OF match table
static const struct of_device_id stm32_of_match[] = {
    { .compatible = "st,stm32-i2c-slave" },
    { }
};
MODULE_DEVICE_TABLE(of, stm32_of_match);

/* open 함수 -----------------------------------------------------------------------------------*/
// probe에서 stm32_i2c_data구조체를 초기화 한 값을
// file->private_data 구조체 변수에 저장해서 다른 함수에서도 공유.
static int stm32_i2c_open(struct inode *inode, struct file *file)
{
    if (!global_data) 
        return -ENODEV;
    file->private_data = global_data;
    return 0;
}

/* read 함수 -----------------------------------------------------------------------------------*/
// i2c_master_recv함수는
// master가 슬레이브 장치에 read 트랜잭션을 보냄
static ssize_t stm32_i2c_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    struct stm32_i2c_data *data = file->private_data;
    int ret;
    
    if (!data || !data->client)
        return -ENODEV;
    if (count > I2C_BUFSIZE)
        count = I2C_BUFSIZE;

    ret = i2c_master_recv(data->client, data->buf, count);
    if (ret < 0)
        return ret;

    if (copy_to_user(buf, data->buf, ret))
        return -EFAULT;
    return ret;
}

/* write 함수 -----------------------------------------------------------------------------------*/
// 어플리케이션에서 stm32으로 write할 데이터를
// copy_from_user 함수를 통해서 가져옴.
// 그다음 i2c_master_send함수로 stm32에 데이터를 송신함.
static ssize_t stm32_i2c_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
    struct stm32_i2c_data *data = file->private_data;
    int ret;
    
    if (!data || !data->client)
        return -ENODEV;
    if (count > I2C_BUFSIZE)
        count = I2C_BUFSIZE;

    if (copy_from_user(data->buf, buf, count))
        return -EFAULT;

    ret = i2c_master_send(data->client, data->buf, count);
    if (ret < 0)
        return ret;
    return ret;
}

/* ioctl 함수 -----------------------------------------------------------------------------------*/
// i2c-dev.h에 정의 되어 있는 ioctl 함수 사용.

static long stm32_i2c_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct stm32_i2c_data *data = file->private_data;
    struct i2c_rdwr_ioctl_data rdwr_data;
    struct i2c_msg *msgs = NULL;
    // --- 수정 1: 원래 사용자 버퍼 주소를 저장할 포인터 배열 추가 ---
    void __user **user_bufs = NULL; 
    int ret = 0;
    int i;

    if (!data)
        return -ENODEV;

    switch (cmd) {
    
    /* 어플리케이션에서 슬레이브 주소를 변경하고 싶을때 사용함. */
    case I2C_SLAVE:
        // 7비트 주소는 0 ~ 127
        // 10비트 주소는 0 ~ 1023
        // 슬레이브 장치의 주소가 1023보다 크면 에러 처리
        if (arg > 0x3ff) 
            return -EINVAL;
        data->client->addr = arg;
        data->slave_addr = (__u16)arg;
        return 0;

    /* 연속 read/write 트랜잭션 */
    // 인터럽트 기능을 구현하지 못해서 트랜젝션 방식을 사용함
    case I2C_RDWR:

        
        // msgs : 유저 공간의 i2c_rdwr_ioctl_data구조체의 주소만 가져옴.
        // nmsgs : read / write를 몇번 할건지 횟수 정보
        if (copy_from_user(&rdwr_data, (struct i2c_rdwr_ioctl_data __user *)arg, sizeof(rdwr_data)))
            return -EFAULT;

        // 통신 횟수 제한. (설정값 - 42번이 최대)
        // #define I2C_RDWR_IOCTL_MAX_MSGS 42
        if (rdwr_data.nmsgs > I2C_RDWR_IOCTL_MAX_MSGS)
            return -EINVAL;

        // copy_from_user로 가져온 msg는 데이터 전송에 필요한
        // 구조체 배열의 시작주소가 들어있음. 
        // 그래서 아직 사용자 영역의 데이터이므로
        // memdup_user 함수를 이용해서 사용자 영역의 데이터를 가져오는거임.

        // msgs[0] = write의 정보가 담긴 구조체 주소
        // msgs[1] = read의 정보가 담긴 구조체 주소
        // msgs[0].len 이런식으로 구조체 멤버에 접근 가능.
        msgs = memdup_user(rdwr_data.msgs, rdwr_data.nmsgs * sizeof(struct i2c_msg));
        if (IS_ERR(msgs))
            return PTR_ERR(msgs);

        // 유저 공간에 데이터를 덮어쓸때 사용할 수 있는 포인터
        user_bufs = kmalloc_array(rdwr_data.nmsgs, sizeof(void __user *), GFP_KERNEL);
        if (!user_bufs) {
            ret = -ENOMEM;
            goto cleanup_msgs;
        }

        ret = 0;
        for (i = 0; i < rdwr_data.nmsgs; i++) {
            // 이 루프에서 에러 발생 시, 할당된 버퍼들을 해제하기 위한 준비
            if (msgs[i].len > I2C_BUFSIZE) {
                ret = -EINVAL;
                goto cleanup_buffers;
            }

            // --- 수정 3: 덮어쓰기 전에 원래 사용자 공간 버퍼 포인터를 저장 ---
            user_bufs[i] = msgs[i].buf;

            if (msgs[i].flags & I2C_M_RD) {
                msgs[i].buf = kmalloc(msgs[i].len, GFP_KERNEL);
                if (!msgs[i].buf) {
                    ret = -ENOMEM;
                    goto cleanup_buffers;
                }
            } else {
                msgs[i].buf = memdup_user(user_bufs[i], msgs[i].len);
                if (IS_ERR(msgs[i].buf)) {
                    ret = PTR_ERR(msgs[i].buf);
                    msgs[i].buf = NULL; // 이중 해제 방지
                    goto cleanup_buffers;
                }
            }
        }

        ret = i2c_transfer(data->client->adapter, msgs, rdwr_data.nmsgs);

        if (ret >= 0) {
            for (i = 0; i < rdwr_data.nmsgs; i++) {
                if (msgs[i].flags & I2C_M_RD) {
                    // --- 수정 4: 저장해두었던 사용자 공간 포인터를 목적지로 사용 ---
                    if (copy_to_user(user_bufs[i], msgs[i].buf, msgs[i].len)) {
                        ret = -EFAULT;
                        break;
                    }
                }
            }
        }

cleanup_buffers:
        // 루프를 돌며 할당된 모든 커널 버퍼를 안전하게 해제
        for (i = 0; i < rdwr_data.nmsgs; i++) {
            if (msgs[i].buf && !IS_ERR(msgs[i].buf))
                kfree(msgs[i].buf);
        }
        // --- 수정 5: 임시로 사용했던 포인터 배열 해제 ---
        kfree(user_bufs);

cleanup_msgs:
        kfree(msgs);
        return ret;

    /* 위에 정의하지 않은 cmd가 오면 에러 처리 */
    default:
        return -ENOTTY;
    }
}

/* file_operations 구조체 ------------------------------------------------------------------------*/
static const struct file_operations stm32_i2c_fops = {
    .owner           = THIS_MODULE,
    .read            = stm32_i2c_read,
    .write           = stm32_i2c_write,
    .open            = stm32_i2c_open,
    .release         = stm32_i2c_release,
    .unlocked_ioctl  = stm32_i2c_ioctl,  // ioctl 함수 추가
};

/* probe 함수 -----------------------------------------------------------------------------------*/
/* 기능 */

// 1. 프라이빗 데이터 동적 할당
    // 커널 메모리, 자동 해제 관리

// 2. I2C 클라이언트와 데이터 연결       // (= 클라이언트는 I2C 디바이스를 나타내는 커널 내부 구조체)
    // 각 디바이스마다 자신만의 정보 관리

// 3. 캐릭터 디바이스 번호 할당
    // 주/부 번호 자동 할당(alloc_chrdev_region)

/* goto 문으로 에러 발생시 역순으로 해제*/
    // 4. cdev 구조체 등록 및 오퍼레이션 연결
        //(open/read/write/ioctl 등 연결)

    // 5. 커널 클래스 생성
        // /sys/class/에 노출(udev 연동 가능)

    // 6. 디바이스 파일 생성
        // /dev/stm32_i2c0 유저가 접근 가능해짐

// 7. 성공 메시지 로그

// 8. 각 단계별 에러 발생 시
    // 이전에 할당한 자원을 역순으로 해제
static int stm32_i2c_probe(struct i2c_client *client)
{
    struct stm32_i2c_data *data;
    int ret;

    /* 구조체 동적 할당*/
    // devm__ 시리즈는 디바이스 해제되면 자동으로 리소스를 반환해줌.
    // 해당 디바이스가 unregister될 때,
    // 또는 드라이버의 remove() 함수가 끝나면
    // 커널이 자동으로 kfree를 해줌.
    data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    /* I2C 클라이언트 포인터를 저장*/
    // struct i2c_client *client 이 구조체는 드라이버와 매칭된 I2C 디바이스 노드를 표현하는 구조체
    data->client = client; // 디바이스 정보를 저장.
    i2c_set_clientdata(client, data); // I2C 디바이스에 연결된 data 구조체를 쉽게 찾게 해줌.
    global_data = data;

    /* 문자 디바이스 번호 할당 */
    // 커널에서 /dev/xxx를 만들기 위해 주번호/ 부번호를 자동으로 할당받음.
    ret = alloc_chrdev_region(&dev_number, 0, 1, DRIVER_NAME); 
    if (ret < 0)  
        return ret;

    /* 문자 디바이스 구조체 등록 */
    // register_chrdev( LEDKEY_DEV_MAJOR, LEDKEY_DEV_NAME, &ledkey_fops);랑 같은 역할
    // read, write 등을 연결함.
    cdev_init(&stm32_cdev, &stm32_i2c_fops);
    stm32_cdev.owner = THIS_MODULE;
    ret = cdev_add(&stm32_cdev, dev_number, 1);
    if (ret < 0)
        goto err_chrdev;

    

    /* 커널 디바이스 클래스 생성 */
    // /sys/class/DEVICE_NAME/에 해당하는 커널 클래스 생성
    stm32_class = class_create(DEVICE_NAME);
    if (IS_ERR(stm32_class)) {
        ret = PTR_ERR(stm32_class);
        goto err_cdev;
    }

    /* 디바이스 파일 /dev/stm32_i2c0 생성 */
    // 위에서 생성한 클래스와 디바이스 번호를 이용해서
    // 유저 공간에서 사용할 수 있는 파일(/dev/stm32_i2c0)을 자동 생성
    // 여기까지 진행해야 어플리케이션에서 read, write등의 함수 사용 가능.
    data->dev = device_create(stm32_class, NULL, dev_number, NULL, DEVICE_NAME "0");
    if (IS_ERR(data->dev)) {
        ret = PTR_ERR(data->dev);
        goto err_class;
    }

    /* 성공 메시지 출력 */
    dev_info(&client->dev, "STM32 I2C client driver loaded\n");
    return 0;

err_class:
    class_destroy(stm32_class);
err_cdev:
    cdev_del(&stm32_cdev);
err_chrdev:
    unregister_chrdev_region(dev_number, 1);
    return ret;
}

/* remove 함수 */
static void stm32_i2c_remove(struct i2c_client *client)
{
    struct stm32_i2c_data *data = i2c_get_clientdata(client);

    if (data && data->dev) {
        device_destroy(stm32_class, dev_number); // /dev/에 생성된 디바이스 파일 삭제
        class_destroy(stm32_class); // sys/class/에 생성된 커널 디바이스 클래스 삭제
        cdev_del(&stm32_cdev); // 문자 디바이스를 커널에서 제거, read,write 등록된 함수 제거
        unregister_chrdev_region(dev_number, 1); // 문자 디바이스 주/부 번호 반환
    }
    global_data = NULL;
}

/* insmod */
// 1. module_i2c_driver 매크로에 있는 init 함수가 호출됨.
// 2. init 함수 에서 i2c_add_driver 함수를 호출함.
// 3. name과 of_match_table를 이용해서 디바이스 트리와 매칭이되면 probe를 호출!
//
// 3-1. name에 적힌 드라이버가 커널에 등록됨.
// 3-2. match_table에 적힌 노드를 디바이스 트리에서 찾음.
static struct i2c_driver stm32_i2c_driver = {
    .driver = {
        .name           = DRIVER_NAME,                  // 디바이스 드라이버 이름
        .of_match_table = of_match_ptr(stm32_of_match), // 어떤 디바이스 트리 노드와 연결될지.
    },
    .probe      = stm32_i2c_probe,  // insmod로 드라이버를 삽입할때 실행함.
    .remove     = stm32_i2c_remove, // rmmod로 드라이버를 내릴때 실행함.
    .id_table   = stm32_i2c_id,     // 구버전 커널용 어떤 ID와 매치되는지.
};
module_i2c_driver(stm32_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("YJS");
MODULE_DESCRIPTION("STM32 I2C Client Driver Example");

```

## Transaction code

```jsx
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

```

<img width="800" height="600" alt="Image" src="https://github.com/user-attachments/assets/c0f962cb-1638-4676-84b9-0f8532913372" />

1. 통신을 시작하기 위해 SDA(Master)가 LOW로 신호를 보내고 SCL도 SDA의 신호를 확인하고 LOW로 신호를 보냄.
2. 통신은 8Bit 단위로 진행됨.
3. 7비트의 주소 + Read/Write(1/0)
4. ACK, NACK (오류를 검사하는 비트로 ACK : 정상, NACK : 비정상)
5. 데이터 8Bit를 보냄.
6. 통신을 종료하기 위해 SCL은 HIGH를 유지함. 그리고 SDA가 LOW를 보냈는데 SCL에서 HIGH를 유지하면 STOP 종료됨.

## SCL이 HIGH일때만 통신이 일어남. 
아래는 READ, 위에는 WRITE
데이터를 누가 보내는지, ACK가 NACK로 끝나는 차이 있음.

<img width="800" height="600" alt="Image" src="https://github.com/user-attachments/assets/c0f962cb-1638-4676-84b9-0f8532913372" />

## Fast Mode Duty Cycle은 400khz로 Clock Speed를 설정하면
1Hz의 시간은 2.5us가 나오는데, high의 시간은 1.25us, low의 시간은 1.25us가 된다.

<img width="800" height="600" alt="Image" src="https://github.com/user-attachments/assets/6d4fa534-000d-42d3-b38c-8afeceeb9e77" />

## I2C를 만든 NXP 회사에서 Fast모드를 사용할때 low의 시간을 적어도 1.3us로 사용하라고 나와있음. 
그래서 low의 시간을 늘려주기위해 (1 : 2), (1:1.777…) 비율을 사용한다.
※자세한 low의 시간은 사용하는 장치의 Slave 설정을 확인하기

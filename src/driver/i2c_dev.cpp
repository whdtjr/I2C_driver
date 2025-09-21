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

/* release 함수 -----------------------------------------------------------------------------------*/
// 사용 X
static int stm32_i2c_release(struct inode *inode, struct file *file)
{
    return 0;
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

/*
 * 1-wire-uart.c - 1-wire-uart driver for linux.
 *
 * Copyright (C) 2014
 *
 * Author: gdyshi <gdyshi@126.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/delay.h>

#define DEVNAME		 	"1-wire-uart"

#define GPIO_TO_PIN(bank, gpio)     (32 * (bank) + (gpio))
#define UART_GPIO             GPIO_TO_PIN(0, 6)

#define Debug(fmt, arg...)          printk("%s.%d ", __FILE__, __LINE__);\
                                    printk("(%s %s):", __DATE__, __TIME__);\
					                printk(fmt, ##arg);\
					                printk("\n")

#define US_TO_NS(var)               (var * 1000)

#define LED_DRV_MAGICNUM            'z'
#define LED_IOCTL_CMD_GET(cmd)      (_IOC_NR(cmd))
#define LED_IOCTL_CMD_IS_VALID(cmd) ((_IOC_TYPE(cmd) == LED_DRV_MAGICNUM) ? 1 : 0)

#define CMD_BASE                    0x80
#define CMD_LIGHT 		            CMD_BASE+0x01
#define CMD_FLASH                   CMD_BASE+0x02
#define CMD_CLOSE                   CMD_BASE+0x03

typedef struct _uart_elem
{
    unsigned char direction;
    unsigned char value;
    unsigned short delay_time_us;// hz
}uart_elem_t;

struct _uart_dev
{
    struct cdev cdev;
    struct hrtimer timer;
    struct class *uart_class;
    dev_t uart_dev_number;
    int gpio;
    
    unsigned short delay_time_us;
    
    uart_elem_t * arg;//执行序列
    unsigned short arg_len;//执行序列长度
    unsigned short arg_idx;//当前执行的下标
	struct completion	complete_request;
    spinlock_t lock;
}*uart_dev;

//防止被多用户同时打开
static atomic_t dev_is_open = ATOMIC_INIT(1);
static int set_gpio_in(int gpio)
{
    return gpio_direction_input(gpio);
}
static int set_gpio_out(int gpio)
{
    return gpio_direction_output(gpio,1);
}
static int set_gpio_val(int gpio,int val)
{
    gpio_set_value(gpio, val);
    printk("set_gpio_val : %d\n",val);
    return 0;
}
static int get_gpio_val(int gpio)
{
    //printk("get_gpio_val\n");
    //return 1;
    return gpio_get_value(gpio);
}

static int process_next_elem(struct _uart_dev *dev)
{
    unsigned long _ns = 0;
    unsigned long hz = 0;
    uart_elem_t * elem = NULL;
    ktime_t ntime;

    if(dev->arg_len <= dev->arg_idx)
    {
        Debug("process_next_elem: out of len.%d----%d",dev->arg_len,dev->arg_idx);
        goto out;
    }
    elem = &dev->arg[dev->arg_idx++];
    if(elem->direction)
    {
        set_gpio_val(dev->gpio, elem->value);
    }
    else
    {
        elem->value=get_gpio_val(dev->gpio);
    }

    _ns = US_TO_NS(hz);

    Debug("timer:%ld ns",_ns);
    ntime = ktime_set(0, _ns);
    hrtimer_start(&dev->timer, ntime, HRTIMER_MODE_REL);
    return 0;

out:
    if (!completion_done(&(dev->complete_request)))
    {
        printk("complete 1\n");
        complete(&(dev->complete_request));
    }
    return -1;

}

/********************************************************************************************/
/*	函数名称：	static enum hrtimer_restart elem_timeout(struct hrtimer *t)				*/
/*	入口参数：	struct hrtimer *t		hrtimer 超时中断默认参数							*/
/*	返回值：	enum hrtimer_restart    hrtimer 超时中断处理完成结果						*/
/*	函数功能：	hrtimer 超时中断响应函数。处理gpio 的状态翻转，产生PWM方波					*/
/********************************************************************************************/
static enum hrtimer_restart elem_timeout(struct hrtimer *t)
{
    /*struct timespec uptime;  	*/
    //ktime_t ntime;
    struct _uart_dev *dev = uart_dev;

    Debug("elem_timeout\n");
    process_next_elem(dev);
    
    //ntime = ktime_set(0, uart_dev->arg[uart_dev->arg_idx++].delay_time_us);
    //hrtimer_start(&uart_dev->timer, ntime, HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

static ssize_t uart_write (struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
    struct _uart_dev *dev = filp->private_data;
    uart_elem_t * uart_array = dev->arg;
    unsigned short array_len = 0;
    unsigned char bytee = *(unsigned char *)buf;
    int i=0;
    printk("uart_write size: %d\n", size);

    //if(1 != size){
    //    return 0;
    //}
    set_gpio_out(dev->gpio);

    printk("uart_write: 0x%x\n", bytee);

    i=0;
    uart_array[i].delay_time_us=dev->delay_time_us;
    uart_array[i].direction=1;
    uart_array[i].value=0;
    for(i=1;i<9;i++)
    {
        uart_array[i].delay_time_us=dev->delay_time_us;
        uart_array[i].direction=1;
        uart_array[i].value=(bytee & (1<< (i-1)))?1:0;
    }
    array_len = 9;

    dev->arg = uart_array;
    dev->arg_len = array_len;
    dev->arg_idx = 0;    
    process_next_elem(dev);
    
    wait_for_completion(&(dev->complete_request));

    return 1;
}
ssize_t uart_read (struct file *filp, char *buff, size_t size, loff_t *offp)
{
    struct _uart_dev *dev = filp->private_data;
    uart_elem_t * uart_array = dev->arg;
    unsigned short array_len = 0;
    unsigned char bytee = *(unsigned char *)buff;
    int i=0;
    int delay_count=0;
    ssize_t result = 0;
    //if(1 != size){
    //    return 0;
    //}
    set_gpio_in(dev->gpio);

    
    uart_array[i].delay_time_us=dev->delay_time_us;
    uart_array[i].direction=0;
    uart_array[i].value=0;
    for(i=1;i<9;i++)
    {
        uart_array[i].delay_time_us=dev->delay_time_us;
        uart_array[i].direction=0;
        uart_array[i].value=0;
    }
    array_len = 9;

    dev->arg = uart_array;
    dev->arg_len = array_len;
    dev->arg_idx = 0;    
    // 等待起始位
    printk("wait\n");
    while(get_gpio_val(dev->gpio))
    {
        delay_count++;
        udelay(1000);
        if(1000 < delay_count) break;
    }
    printk("wait:%d\n",delay_count);
    process_next_elem(dev);
    
    wait_for_completion(&(dev->complete_request));
    bytee=0;
    for(i=1;i<9;i++)
    {
        bytee |= (uart_array[i].value)?(1<< (i-1)):0;
    }
    printk("uart_read: 0x%x\n", bytee);
    
    if (copy_to_user (buff, &bytee, 1))
    {
        result = -EFAULT;
    }
    else
    {
        result = 1;
    }
    return result;
}

/********************************************************************************************/
/*	函数名称：	int uart_open(struct inode *inode, struct file *filp)						*/
/*	入口参数：	struct inode *inode, 用户空间传递的设备节点 								*/
/*				struct file *filp	  文件描述符											*/
/*	返回值：	int  0 成功，<0 失败														*/
/*	函数功能：	设备节点的标准打开函数														*/
/********************************************************************************************/
int uart_open(struct inode *inode, struct file *filp)
{
    struct _uart_dev *dev = NULL;
    
    if(!atomic_dec_and_test(&dev_is_open))
    {
        atomic_inc(&dev_is_open);
        Debug("Bus busy \n");
        return -EBUSY;//already open
    }

    //将私有数据关联到 private_data
    dev = container_of(inode->i_cdev, struct _uart_dev, cdev);
    if(!dev)
    {
        Debug("get private data error!\n");
        return -EFAULT;
    }


    filp->private_data = dev;
    Debug("uart: OK devices ok \n");
    return 0;
}

/********************************************************************************************/
/*	函数名称： static int uart_release(struct indoe *ndoe, struct file *file)				*/
/*	入口参数： struct indoe *ndoe	  设备节点												*/
/*			   struct file *file	  文件描述符											*/
/*	返回值：	int  0 成功，<0 失败														*/
/*	函数功能：	设备关闭函数																*/
/********************************************************************************************/
static int uart_release(struct inode *indoe, struct file *file)
{
    Debug("close \n");
    if (!completion_done(&(uart_dev->complete_request)))
    {
        Debug("complete 2\n");
        complete(&(uart_dev->complete_request));
    }

    //释放被打开的文件
    atomic_inc(&dev_is_open);
    //销毁设备
    Debug("close done \n");

    return 0;
}

struct file_operations uart_fops =
{
    .owner = THIS_MODULE,
    .open = uart_open,
    //.unlocked_ioctl = uart_ioctl,
    .write = uart_write,
    .read = uart_read,
    .release=uart_release
};

/********************************************************************************************/
/*	函数名称： static int __init uart_init(void)												*/
/*	入口参数： 无																			*/
/*	返回值：	int  0 标识成功，<0 设备失败												*/
/*	函数功能：	驱动初始化入口函数，注册设备节点											*/
/********************************************************************************************/
static int __init uart_init(void)
{
    int result = -1;

    Debug("uart_init get in\n");

    uart_dev = kzalloc(sizeof(struct _uart_dev), GFP_KERNEL);
    if(!uart_dev)
    {
        result = ~ENOMEM;
        Debug("uart_init: malloc memory error");
        goto err_dev;
    }
    Debug("uart_init malloc memory ok\n");
    uart_dev->arg = kzalloc(13*sizeof(uart_elem_t), GFP_KERNEL);
    if(!uart_dev->arg)
    {
        Debug("uart_init: malloc memory error");
        goto err_arg;
    }

    if(alloc_chrdev_region(&uart_dev->uart_dev_number, 0, 1, DEVNAME))
    {
        Debug("Cant't register devices \n");
        goto err_region;
    }
    Debug("uart_init register ok in\n");

    cdev_init(&uart_dev->cdev, &uart_fops);
    uart_dev->cdev.owner = THIS_MODULE;
    uart_dev->cdev.ops = &uart_fops;
    result = cdev_add(&uart_dev->cdev, uart_dev->uart_dev_number, 1);
    if(result)
    {
        if(!uart_dev)
            kfree(uart_dev);

        Debug(KERN_NOTICE "Error adding LED");
        goto err_add;
    }

    uart_dev->uart_class = class_create(THIS_MODULE, "1-wire-uart_class");
    device_create(uart_dev->uart_class, NULL, MKDEV(MAJOR(uart_dev->uart_dev_number),
                  0), "LED",DEVNAME);
    Debug("uart_init create device ok\n");

    uart_dev->gpio = UART_GPIO;
    gpio_request(uart_dev->gpio,"uart");
    spin_lock_init(&uart_dev->lock);
    hrtimer_init(&uart_dev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    uart_dev->timer.function = elem_timeout;
    Debug("uart_init set uart polarity ok!\n");
    
    init_completion(&(uart_dev->complete_request));
    return 0;

err_add:
    unregister_chrdev_region(uart_dev->uart_dev_number, 1);
err_region:
    if(!uart_dev->arg)
        kfree(uart_dev->arg);
err_arg:
    if(!uart_dev)
        kfree(uart_dev);
err_dev:
    return -1;
}

/********************************************************************************************/
/*	函数名称： static void __exit uart_exit(void)											*/
/*	入口参数： 无																			*/
/*	返回值：   无																			*/
/*	函数功能：	驱动卸载函数																*/
/********************************************************************************************/
static void __exit uart_exit(void)
{
    Debug("uart_exit get in\n\n\n");

    if(uart_dev)
    {
        hrtimer_cancel(&uart_dev->timer);
        //cancel_work_sync(&uart_dev->work);
        Debug("hrtime and work cancel done !\n");
    }
    gpio_free(uart_dev->gpio);
    //cancel_work_sync(&uart_dev->work);
    unregister_chrdev_region(uart_dev->uart_dev_number, 1);

    //�����豸
    device_destroy(uart_dev->uart_class, uart_dev->uart_dev_number);
    cdev_del(&uart_dev->cdev);

    class_destroy(uart_dev->uart_class);
    if(!uart_dev)
    {
        if(!uart_dev->arg)
            kfree(uart_dev->arg);
        kfree(uart_dev);
    }

    Debug("uart_exit unregister_chrdev_region\n\n\n");
}

module_init(uart_init);
module_exit(uart_exit);

MODULE_AUTHOR("gdyshi <gdyshi@126.com>");
MODULE_DESCRIPTION("1-wire-uart driver");
MODULE_LICENSE("GPL");


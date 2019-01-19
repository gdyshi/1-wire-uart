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
#include <linux/device.h>

#define DEVNAME		 	"1-wire"

#define UART_GPIO             22

#define Debug(fmt, arg...)          printk(fmt, ##arg);\
					                printk("\n")

#define DEF_BAUD              (4800)
#define US_TO_NS(var)               (var * 1000)
#define BAUD_TO_NS(_baud_)          (1000000000/_baud_)
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
    unsigned short delay_time_ns;// hz
}uart_elem_t;
typedef enum{
       idle=0,
	recv,
	send
}status_e;
status_e status;

struct _uart_dev
{
    struct cdev cdev;
    struct hrtimer timer;
    struct class *uart_class;
    dev_t uart_dev_number;
    int gpio;

    unsigned long delay_time_ns;
    status_e status;

    unsigned char send_char;
    unsigned char recv_char;
    
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

/********************************************************************************************/
/*	函数名称：	static enum hrtimer_restart elem_timeout(struct hrtimer *t)				*/
/*	入口参数：	struct hrtimer *t		hrtimer 超时中断默认参数							*/
/*	返回值：	enum hrtimer_restart    hrtimer 超时中断处理完成结果						*/
/*	函数功能：	hrtimer 超时中断响应函数。处理gpio 的状态翻转，产生PWM方波					*/
/********************************************************************************************/
static enum hrtimer_restart elem_timeout(struct hrtimer *t)
{
    /*struct timespec uptime;  	*/
    ktime_t ntime;
    struct _uart_dev *dev = uart_dev;
    int in_val=0;
    static int recving=0;
    static int recv_bitnum=0;
    static int recv_bits=0;
    static int sending=0;
    static int send_bitnum=0;
    static int send_bits=0;
    Debug("elem_timeout\n");
    Debug("timer:%ld ns",dev->delay_time_ns);
    ntime = ktime_set(0, dev->delay_time_ns);
    hrtimer_start(&dev->timer, ntime, HRTIMER_MODE_REL);


    switch(dev->status)
    {
    	case recv:
    		in_val=get_gpio_val(dev->gpio);
    		if(recving)//已经接收到起始位
    		{
    			if(--recv_bitnum==0)
    			{
    				dev->recv_char=recv_bits;	//save the data to RBUF
    				recving=0;		//stop receive
                            spin_lock(dev->lock);
                            dev->status=idle;
                            spin_unlock(dev->lock);
                            printk("complete\n");
                            complete(&(dev->complete_request));
    			}else
    			{
                            printk("got bit %d\n",in_val);
    				recv_bits >>=1;
    				if(in_val)	recv_bits|=0x80;//shift RX data to RX buffer
    			}
    		}else if(0==in_val)//接收到起始位
    		{
                     printk("got start\n");
    			recving=1;
    			recv_bitnum=9;
    		}
    		break;
    	case send:
    		if(send_bitnum==0)
    		{
                     printk("put start\n");
    			set_gpio_val(dev->gpio,0);//send start bit
    			send_bits=dev->send_char;//load data from buf to send_bits
    			send_bitnum=9;//inital send bit number(8 data bits + 1 stop bit)
    		}else
    		{
    			if(--send_bitnum==0)
    			{
    				set_gpio_val(dev->gpio,1);//stop send
                            spin_lock(dev->lock);
                            dev->status=idle;
                            spin_unlock(dev->lock);
                            printk("complete\n");
                            complete(&(dev->complete_request));

    			}else
    			{
                            printk("put bit %d\n",send_bits&0x01);
    				set_gpio_val(dev->gpio,send_bits&0x01);
    				send_bits >>=1;
    			}
    		}
    		break;
    }






    
    return HRTIMER_NORESTART;
}

static ssize_t uart_write (struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
    struct _uart_dev *dev = filp->private_data;
    uart_elem_t * uart_array = dev->arg;
    unsigned short array_len = 0;
    int i=0;
    printk("uart_write size: %d\n", size);
    
    if(recv==dev->status) 
    {
        printk("uart_write err: bus busy\n");
        return -EFAULT;
    }

    set_gpio_out(dev->gpio);

    printk("uart_write: 0x%x\n", bytee);
    
    spin_lock(dev->lock);
    dev->status=send;
    dev->send_char=*(unsigned char *)buf;
    spin_unlock(dev->lock);

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

    if(send==dev->status) 
    {
        printk("uart_read err: bus busy\n");
        return -EFAULT;
    }

    set_gpio_in(dev->gpio);
    spin_lock(dev->lock);
    dev->status=recv;
    dev->recv_char=0;
    spin_unlock(dev->lock);

    wait_for_completion(&(dev->complete_request));
    printk("uart_read: 0x%x\n", dev->recv_char);

    if (copy_to_user (buff,dev->recv_char, 1))
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
    set_gpio_out(uart_dev->gpio);

    if(!atomic_dec_and_test(&dev_is_open))
    {
        atomic_inc(&dev_is_open);
        Debug("Bus busy \n");
        return -EBUSY;//already open
    }
    spin_lock(dev->lock);
    dev->status=idle;
    spin_unlock(dev->lock);

    //将私有数据关联到 private_data
    dev = container_of(inode->i_cdev, struct _uart_dev, cdev);
    if(!dev)
    {
        Debug("get private data error!\n");
        return -EFAULT;
    }
    init_completion(&(uart_dev->complete_request));

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
    set_gpio_out(uart_dev->gpio);
    Debug("close \n");
    //if (!completion_done(&(uart_dev->complete_request)))
    {
        //Debug("complete 2\n");
        complete_all(&(uart_dev->complete_request));
    }
    spin_lock(dev->lock);
    dev->status=idle;
    spin_unlock(dev->lock);

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
    ktime_t ntime;

    Debug("uart_init get in\n");

    uart_dev = kzalloc(sizeof(struct _uart_dev), GFP_KERNEL);
    if(!uart_dev)
    {
        result = ~ENOMEM;
        Debug("uart_init: malloc memory error");
        goto err_dev;
    }
    Debug("uart_init malloc memory ok\n");
    uart_dev->delay_time_ns=BAUD_TO_NS(DEF_BAUD);
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
        Debug(KERN_NOTICE "Error adding cdev");
        goto err_add;
    }

    uart_dev->uart_class = class_create(THIS_MODULE, "1-wire_class");
    device_create(uart_dev->uart_class, NULL, MKDEV(MAJOR(uart_dev->uart_dev_number),
                  0), "LED",DEVNAME);
    Debug("uart_init create device ok\n");

    uart_dev->gpio = UART_GPIO;
    result = gpio_request(uart_dev->gpio,"uart");
    if(result)
    {
        printk("Error request gpio");
        //goto err_add;
    }
    set_gpio_out(uart_dev->gpio);

    spin_lock_init(&uart_dev->lock);
    hrtimer_init(&uart_dev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    uart_dev->timer.function = elem_timeout;
    Debug("uart_init set uart polarity ok!\n");
    ntime = ktime_set(0, uart_dev->delay_time_ns);
    hrtimer_start(&uart_dev->timer, ntime, HRTIMER_MODE_REL);

    init_completion(&(uart_dev->complete_request));
    return 0;

err_add:
    unregister_chrdev_region(uart_dev->uart_dev_number, 1);
err_region:
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

    //?????豸
    device_destroy(uart_dev->uart_class, uart_dev->uart_dev_number);
    cdev_del(&uart_dev->cdev);

    class_destroy(uart_dev->uart_class);
    if(!uart_dev)
    {
        kfree(uart_dev);
    }

    Debug("uart_exit unregister_chrdev_region\n\n\n");
}

module_init(uart_init);
module_exit(uart_exit);

MODULE_AUTHOR("gdyshi <gdyshi@126.com>");
MODULE_DESCRIPTION("1-wire driver");
MODULE_LICENSE("GPL");

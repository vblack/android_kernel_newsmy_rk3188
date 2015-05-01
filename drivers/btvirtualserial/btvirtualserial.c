#include <linux/init.h>   
#include <linux/module.h>   

////////////////////////////////////
#include <linux/fs.h>            
#include <linux/mm.h>            
#include <linux/errno.h>         
#include <linux/types.h>         
#include <linux/fcntl.h>         
#include <linux/cdev.h>         
#include <linux/device.h>         
#include <linux/major.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/delay.h>

#include <asm/mach-types.h>
#include <asm/uaccess.h>  
#include <asm/io.h>  
/////////////////////////////////////////////

MODULE_LICENSE("Dual BSD/GPL");   


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define HANDLE_DEV_NAME      "bonovo_tty"   // device name

#define BT_CTRL_DEV_MAJOR		331
#define BT_CTRL_DEV_MINOR		0

static int      handle_major = BT_CTRL_DEV_MAJOR; 
static dev_t    dev;

//
#define IOCTL_HANDLE_BT_POWER_ON      		_IO(BT_CTRL_DEV_MAJOR, 20)
#define IOCTL_HANDLE_BT_POWER_OFF		_IO(BT_CTRL_DEV_MAJOR, 21)
#define IOCTL_HANDLE_BT_CLEAR_BUF			_IO(BT_CTRL_DEV_MAJOR, 23)
#define IOCTL_HANDLE_BT_UPDATE_FW		_IO(BT_CTRL_DEV_MAJOR, 24)

//
static struct   cdev handle_cdev;
static struct   class *handle_class;

//
#define BLUETOOTH_BUF_LEN  2046
struct bluetooth_buff
{
	spinlock_t bluetooth_lock;
	wait_queue_head_t bt_r_wait;
	wait_queue_head_t bt_w_wait;
	unsigned int buf_head_idx;
	unsigned int buf_tail_idx;
	char buff[BLUETOOTH_BUF_LEN];
};
static struct bluetooth_buff *bt_dev;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////


typedef void (*callback_t)(char* data, int size);
extern  void register_callback_func(callback_t func);
extern  void unregister_callback_func();
extern unsigned int calculateSum(unsigned char* cmdBuf, int size);
extern int serial_send_ack(char * data, int len);


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int isBTBuffEmpty(void)
{
    return (bt_dev->buf_head_idx == bt_dev->buf_tail_idx);
}

static int isBTBuffFull(void)
{
    return ((bt_dev->buf_head_idx + 1)%BLUETOOTH_BUF_LEN == bt_dev->buf_tail_idx);
}


static int clearBTBuff(void)
{
       spin_lock(&bt_dev->bluetooth_lock);
	memset(bt_dev->buff, 0, BLUETOOTH_BUF_LEN);
	bt_dev->buf_head_idx = bt_dev->buf_tail_idx = 0;
	spin_unlock(&bt_dev->bluetooth_lock);
	return 0;
}

static int btPowerCtl(int offOn)
{
    char btCmd[9] = {0xFA, 0xFA, 0x09, 0x00, 0x83, 0x01};
	unsigned int sum, ret;

	if(offOn){
		btCmd[6] = 0x01;
	}else{
		btCmd[6] = 0x00;
	}

	sum = calculateSum((unsigned char*)btCmd, 7);
	btCmd[7] = sum & 0x00FF;
	btCmd[8] = (sum >> 8) & 0x00FF;

	//
	ret = serial_send_ack(btCmd, 9);
	return ret;
}

static void update_bt_fw(int offOn)
{
	char btCmd[9] = {0xFA, 0xFA, 0x09, 0x00, 0x83, 0x0B};	

	unsigned int sum, ret;

	if(offOn){
		btCmd[6] = 0x01;
	}else{
		btCmd[6] = 0x00;
	}

	sum = calculateSum((unsigned char*)btCmd, 7);
	btCmd[7] = sum & 0x00FF;
	btCmd[8] = (sum >> 8) & 0x00FF;

	//
	ret = serial_send_ack(btCmd, 9);
	
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static unsigned char uart3_frame_buf[512+8] =
{
	0xFA, 0xFA, 0x0C, 0x00, 0xA2, 0x00
};

#define BT_CMD          0xA2


static ssize_t virtualserial_write(struct file *fp, const char __user *buf,	size_t count, loff_t *ppos)
{	

	unsigned int checksum;
	ssize_t ret=0;
	
	if (count > 290)
	{		
		printk("=====virtualserial_write=====, data is too large!\r\n");
		return -1;	
	}
	
	if (copy_from_user(&uart3_frame_buf[5], buf, count))	
	{		
		return -EFAULT;	
	}	

	int cmdLen = count+7;

	uart3_frame_buf[2] = cmdLen & 0x00FF;
	uart3_frame_buf[3] = (cmdLen >> 8) & 0x00FF;
	uart3_frame_buf[4]=BT_CMD;
	
	checksum = calculateSum(uart3_frame_buf, cmdLen - 2);
	
	uart3_frame_buf[cmdLen - 2] = checksum&0xFF;
	uart3_frame_buf[cmdLen - 1] = (checksum>>8)&0xFF;

	ret = serial_send_ack(uart3_frame_buf, cmdLen);

	//printk("virtualserial_write  ret=[%d],count=[%d]\n",ret,count);
	
	return ret;
}

static int bt_fill_buff(char* data, int size)
{
	int i = 0, res;
	
	if(isBTBuffFull())
	{
        	//printk("bluetooth buffer is full when be writted!\n");
		res = -ENOMEM;
		goto fail_buff_full;
	}

	spin_lock(&bt_dev->bluetooth_lock);
	for(i=0; (i<size)&&(!isBTBuffFull()); i++)
	{
		bt_dev->buff[bt_dev->buf_head_idx] = data[i];
		bt_dev->buf_head_idx = (bt_dev->buf_head_idx + 1)%BLUETOOTH_BUF_LEN;
	}

	spin_unlock(&bt_dev->bluetooth_lock);
	wake_up_interruptible(&bt_dev->bt_r_wait);
	
	return 0;
fail_buff_full:
	return res;
	
}

static ssize_t virtualserial_read(struct file *fp, char __user *buff, size_t count, loff_t *offset)
{
	
	int i, res, size;
	char *temp_buf;
	
	DECLARE_WAITQUEUE(wait, current);   // 定义等待队列
	add_wait_queue(&bt_dev->bt_r_wait, &wait);

	while(isBTBuffEmpty())
	{
		if(fp->f_flags & O_NONBLOCK)
		{
			res = -EAGAIN;
			goto fail_buff_empty;
		}

		__set_current_state(TASK_INTERRUPTIBLE);
		schedule();

		if(signal_pending(current))
		{
			res = -ERESTARTSYS;
			goto fail_buff_empty;
		}
	}


	temp_buf = kmalloc((size_t)BLUETOOTH_BUF_LEN, GFP_KERNEL);
	if(!temp_buf)
	{
		return -ENOMEM;
	}
	memset(temp_buf, 0, BLUETOOTH_BUF_LEN);
	size = 0;
	spin_lock(&bt_dev->bluetooth_lock);

	for(i=0; (i < count) && (!isBTBuffEmpty()); i++)
	{
		temp_buf[i] = bt_dev->buff[bt_dev->buf_tail_idx];
		bt_dev->buf_tail_idx = (bt_dev->buf_tail_idx+1)%BLUETOOTH_BUF_LEN;
		size++;
	}
	spin_unlock(&bt_dev->bluetooth_lock);

	if(copy_to_user(buff, temp_buf, size)){
		res = -EFAULT;
	}else{
		res = size;
	}
	kfree(temp_buf);

fail_buff_empty:
	remove_wait_queue(&bt_dev->bt_r_wait, &wait);
	set_current_state(TASK_RUNNING);
	return res;
}


static long virtualserial_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    switch( cmd )
    {
   	case IOCTL_HANDLE_BT_CLEAR_BUF:
		printk("virtualserial_ioctl IOCTL_HANDLE_BT_CLEAR_BUF\n");
		clearBTBuff();
		break;
	case IOCTL_HANDLE_BT_POWER_ON:
		printk("virtualserial_ioctl IOCTL_HANDLE_BT_POWER_ON\n");
		return btPowerCtl(1);
		break;
	case IOCTL_HANDLE_BT_POWER_OFF:
		printk("virtualserial_ioctl IOCTL_HANDLE_BT_POWER_OFF\n");
		return btPowerCtl(0);
		break;
	case IOCTL_HANDLE_BT_UPDATE_FW:
		printk("virtualserial_ioctl IOCTL_HANDLE_BT_UPDATE_FW\n");
		update_bt_fw(1);
		break;
	default:
		return -EINVAL;
		break;
    }
    return 0;
}


static int virtualserial_open (struct inode *inode, struct file *filp)  
{
	printk("virtualserial virtualserial_open\n");
	register_callback_func(bt_fill_buff);
	return 0;  
}  

static int virtualserial_release (struct inode *inode, struct file *filp)  
{  
	printk("virtualserial virtualserial_release\n");
	unregister_callback_func();
	return 0;  
}

static struct file_operations handle_fops =  
{  
	.owner			= THIS_MODULE,
	.unlocked_ioctl 	= virtualserial_ioctl,
	.open     			= virtualserial_open,
	.release  			= virtualserial_release,
	.read     			= virtualserial_read,
	.write    			= virtualserial_write,
};  

static int __init virtualserial_init(void)   
{   

	int result; 
	
	printk("virtualserial_init BLUETOOTH_BUF_LEN=[%d]\n",BLUETOOTH_BUF_LEN);  
	
	
	if (0 == handle_major)
	{
		/* auto select a major */
		result = alloc_chrdev_region(&dev, 0, 1, HANDLE_DEV_NAME);
		handle_major = MAJOR(dev);
	}
	else
	{
		/* use load time defined major number */
		dev = MKDEV(handle_major, 0);
		result = register_chrdev_region(dev, 1, HANDLE_DEV_NAME);
	}

	if (result)
	{
		printk("virtualserial_init  register_chrdev_region error!\n");
		return result;
	}


	//
	bt_dev = kmalloc(sizeof(struct bluetooth_buff), GFP_KERNEL);
	if(!bt_dev)
	{
		printk("virtualserial_init Can't allocate memory for BT of bonovo_bt\n");
		result = -ENOMEM;
		goto fail_bt_mem;
	}

	memset(bt_dev, 0, sizeof(struct bluetooth_buff));

	//
	spin_lock_init(&bt_dev->bluetooth_lock);

	init_waitqueue_head(&bt_dev->bt_r_wait);
	init_waitqueue_head(&bt_dev->bt_w_wait);

	//
	memset(&handle_cdev, 0, sizeof(handle_cdev));

	/* initialize our char dev data */
	cdev_init(&handle_cdev, &handle_fops);

	/* register char dev with the kernel */
	result = cdev_add(&handle_cdev, dev, 1);
    
	if (0 != result)
	{
		unregister_chrdev_region(dev, 1);
		printk("virtualserial_init Error registrating mali device object with the kernel\n");
	}


	//
	handle_class = class_create(THIS_MODULE, HANDLE_DEV_NAME);
   	device_create(handle_class, NULL, MKDEV(handle_major, MINOR(dev)), NULL,HANDLE_DEV_NAME);

	//register_callback_func(bt_fill_buff);
	

    return 0;  

fail_bt_mem:
	return result;
}  


static void __exit virtualserial_exit(void)   
{   
	kfree(bt_dev);
	
	device_destroy(handle_class, MKDEV(handle_major, 0));
	class_destroy(handle_class);

	cdev_del(&handle_cdev);
	unregister_chrdev_region(dev, 1);
	
       printk("virtualserial_exit\n");   
}   
module_init(virtualserial_init);   
module_exit(virtualserial_exit);

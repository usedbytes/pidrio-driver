/*
 *  pidrio.c
 *
 *  Copyright (C) 2012, 2014 Brian Starkey <stark3y@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Driver for the low-level interface board on the Pidrive. LED Control
 *  and input (button) handling
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/bcd.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/reboot.h>
#include <linux/leds.h>

#define PIDRIO_NAME "pidrio"

#define PIDRIO_MAX_DEVICES 1
#define CONFIG_PIDRIO_RATE 20

#define RED 0
#define GREEN 1
#define BLUE 2
#define SPEED 3

static int pidrio_major;
static struct class * pidrio_class = NULL;
static unsigned int pidrio_num_devices = 0;
static unsigned int pidrio_rate = CONFIG_PIDRIO_RATE;
module_param(pidrio_rate, uint, S_IRUGO);
MODULE_PARM_DESC(pidrio_rate, "Sample rate (hertz)");

/* Device data structure */

struct mled {
    struct i2c_client * client;
    struct led_classdev led_cdev;
};

struct pidrio {
    int major;
    struct mutex update_lock;
    
    unsigned char led_vals[3];
    
    unsigned char led_speed;
    
    unsigned char auto_update;

    unsigned char pressed_keymap[8];
    unsigned char held_keymap[8];

    struct i2c_client * client;
    struct input_dev * input_dev;

    struct delayed_work work_item;
    struct cdev cdev;
    
    struct mled mled[3];
    
};

#define cdev_to_led(c)  container_of(c, struct mled, led_cdev)

/* LED Control */
static int pidrio_write_leds(struct pidrio * pidrio)
{
    struct i2c_msg msg = {
        .addr = pidrio->client->addr,
        .flags = I2C_M_IGNORE_NAK,
        .buf = pidrio->led_vals,
        .len = 4
    };

    return i2c_transfer(pidrio->client->adapter, &msg, 1) == 1 ? 0 : -1;
}

static void pidrio_set_led(struct led_classdev *led_cdev, uint8_t value, int colour) {
    struct mled * led = cdev_to_led(led_cdev);
    struct pidrio * pidrio = i2c_get_clientdata(led->client);
    
    if ((colour < 0) || (colour > 2)) {
        return;
    }
    mutex_lock(&pidrio->update_lock);
    pidrio->led_vals[colour] = value;
    mutex_unlock(&pidrio->update_lock);        
    pidrio_write_leds(pidrio);                
}

#define show(value, i) \
static ssize_t \
show_##value(struct device *dev, struct device_attribute *attr, char *buf)\
{                                                    \
    struct i2c_client *client = to_i2c_client(dev);  \
    struct pidrio *pidrio =                          \
        i2c_get_clientdata(client);                  \
                                                     \
    return sprintf(buf, "%d\n", pidrio->led_vals[i]);\
}
show(red, RED);
show(green, GREEN);
show(blue, BLUE);
show(speed, SPEED);

#define set(value, i) \
static ssize_t \
set_##value(struct device *dev, struct device_attribute *attr, \
			  const char *buf, size_t count)        \
{                                                   \
    struct i2c_client *client = to_i2c_client(dev); \
    struct pidrio *pidrio =                         \
        i2c_get_clientdata(client);                 \
    mutex_lock(&pidrio->update_lock);               \
    pidrio->led_vals[i] =                           \
                simple_strtoul(buf, NULL, 10);      \
    mutex_unlock(&pidrio->update_lock);             \
    pidrio_write_leds(pidrio);                      \
    return count;                                   \
}
set(red, RED);
set(green, GREEN);
set(blue, BLUE);
set(speed, SPEED);

static DEVICE_ATTR(red, S_IRUGO | S_IWUGO, show_red, set_red);
static DEVICE_ATTR(green, S_IRUGO | S_IWUGO, show_green, set_green);
static DEVICE_ATTR(blue, S_IRUGO | S_IWUGO, show_blue, set_blue);
static DEVICE_ATTR(speed, S_IRUGO | S_IWUGO, show_speed, set_speed);

static struct attribute *pidrio_attrs[] = {
	&dev_attr_red.attr,
	&dev_attr_green.attr,
	&dev_attr_blue.attr,
	&dev_attr_speed.attr,
	NULL,
};

static struct attribute_group pidrio_group = {
	.name = "pidrio-leds",
	.attrs = pidrio_attrs,
};

static void pidrio_led_red_set(struct led_classdev *led_cdev,
			       enum led_brightness value) {
	pidrio_set_led(led_cdev, (uint8_t)value, RED);
}

static void pidrio_led_green_set(struct led_classdev *led_cdev,
			       enum led_brightness value) {
	pidrio_set_led(led_cdev, (uint8_t)value, GREEN);
}

static void pidrio_led_blue_set(struct led_classdev *led_cdev,
			       enum led_brightness value) {
	pidrio_set_led(led_cdev, (uint8_t)value, BLUE);
}

/* Read Buttons Work */

static struct workqueue_struct *pidrio_workqueue;

static void pidrio_queue(struct pidrio * pidrio)
{
	queue_delayed_work(pidrio_workqueue, &pidrio->work_item,
		HZ / pidrio_rate);
}

void pidrio_read_buttons (struct work_struct *work)
{
    struct pidrio * pidrio = container_of((struct delayed_work *)work, 
                                            struct pidrio, work_item);
    unsigned char i, mask;
    unsigned char read_buffer[2];
    struct i2c_msg msg = {
        .addr = pidrio->client->addr,
        .flags = I2C_M_IGNORE_NAK | I2C_M_RD,
        .buf = read_buffer,
        .len = 2
    };


    i2c_transfer(pidrio->client->adapter, &msg, 1);

    //printk(KERN_ALERT "Reading buttons from pidrio: %x %x\n", read_buffer[0], read_buffer[1]);
    
    for (i = 0, mask = 1; i < 8; i++, mask <<= 1) {
        input_report_key(pidrio->input_dev, pidrio->pressed_keymap[i],
            (read_buffer[0] & mask) ? 1 : 0);
        input_sync(pidrio->input_dev);
    }

    for (i = 0, mask = 1; i < 8; i++, mask <<= 1) {
        input_report_key(pidrio->input_dev, pidrio->held_keymap[i],
            (read_buffer[1] & mask) ? 1 : 0);
        input_sync(pidrio->input_dev);
    }
    
    /*
    // Read the damn buttons. Queue it up on the file descriptor
    input_report_key(pidrio->input_dev, KEY_A, 1);
    input_sync(pidrio->input_dev);
    input_report_key(pidrio->input_dev, KEY_A, 0);
    input_sync(pidrio->input_dev);
    */
    
    // Power button held
    if (read_buffer[1] & 1) {
        printk(KERN_ALERT "Pidrio: System Powering OFF!\n");
        pidrio->auto_update = 0;
        orderly_poweroff(true);
    }

    if (pidrio->auto_update) pidrio_queue(pidrio);
    
}

int pidrio_open(struct input_dev * dev) {
    printk(KERN_ALERT "Pidrio Opened!\n");
    return 0;
}

void pidrio_close(struct input_dev * dev) {
    printk(KERN_ALERT "Pidrio Closed!\n");
}
/* i2c Client */

static int pidrio_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    struct pidrio *pidrio;
    char led_name[15];
    int ret, err, i;
    char path[11];
    if (pidrio_num_devices >= PIDRIO_MAX_DEVICES) {
        printk(KERN_ERR PIDRIO_NAME ": ERROR: "
	        "Maximum number of devices reached\n");
        return -ENODEV;
    }
    
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C 
                /*| I2C_FUNC_PROTOCOL_MANGLING*/))
        return -ENODEV;
    
    pidrio = kzalloc(sizeof(struct pidrio), GFP_KERNEL);
	if (!pidrio)
		return -ENOMEM;

    pidrio->major = pidrio_major;
    mutex_init(&pidrio->update_lock);    
    pidrio->auto_update = 1;
    i2c_set_clientdata(client, pidrio);
    pidrio->client = client;
    
    INIT_DELAYED_WORK(&pidrio->work_item, pidrio_read_buttons);
    
    /* LEDs */
	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &pidrio_group);
	if (err < 0) {
		dev_err(&client->dev, "couldn't register sysfs group\n");
		goto exit;
	}

    pidrio->led_speed = 5;
    for (i = 0; i < 3; i++) {
        
        pidrio->mled[i].led_cdev.max_brightness = 255;
        pidrio->mled[i].client = client;
        switch (i) {
            case 0:
            	snprintf(led_name, sizeof(led_name),
					 "pidrio:red:%d", i);
			pidrio->mled[i].led_cdev.name = led_name;
			pidrio->mled[i].led_cdev.brightness_set = pidrio_led_red_set;
			err = led_classdev_register(&client->dev,
						    &pidrio->mled[i].led_cdev);
			if (err < 0) {
				dev_err(&client->dev,
					"couldn't register LED %s\n",
					pidrio->mled[i].led_cdev.name);
				goto failred;
			}
                break;
            case 1:
            	snprintf(led_name, sizeof(led_name),
					 "pidrio:green:%d", i);
                pidrio->mled[i].led_cdev.name = led_name;
                pidrio->mled[i].led_cdev.brightness_set = pidrio_led_green_set;
                err = led_classdev_register(&client->dev,
                                &pidrio->mled[i].led_cdev);
                if (err < 0) {
                    dev_err(&client->dev,
                        "couldn't register LED %s\n",
                        pidrio->mled[i].led_cdev.name);
                    goto failgreen;
                }            
                break;
            case 2:
            	snprintf(led_name, sizeof(led_name),
					 "pidrio:blue:%d", i);
                pidrio->mled[i].led_cdev.name = led_name;
                pidrio->mled[i].led_cdev.brightness_set = pidrio_led_blue_set;
                err = led_classdev_register(&client->dev,
                                &pidrio->mled[i].led_cdev);
                if (err < 0) {
                    dev_err(&client->dev,
                        "couldn't register LED %s\n",
                        pidrio->mled[i].led_cdev.name);
                    goto failblue;
                }                            
                break;
        }
    }
            

    /* Input device */
    pidrio->input_dev = input_allocate_device();
    if (!pidrio->input_dev)
        return -ENOMEM;

    /* set up descriptive labels */
    pidrio->input_dev->name = "Pidrive Front-Panel Buttons";
    pidrio->input_dev->open = pidrio_open;
    pidrio->input_dev->close = pidrio_close;

    /* phys is unique on a running system */
    sprintf(path, "i2c-%1i/0x%02X", pidrio->client->adapter->nr, 
            pidrio->client->addr);
    pidrio->input_dev->phys = path;
    pidrio->input_dev->id.bustype = BUS_I2C;
    pidrio->input_dev->id.vendor = 0xDEAD;
    pidrio->input_dev->id.product = 0xBEEF;
    pidrio->input_dev->id.version = 0x0100;
                               
    set_bit(EV_KEY, pidrio->input_dev->evbit);

    // Reverse button press/hold
    set_bit(KEY_PREVIOUSSONG, pidrio->input_dev->keybit);
    set_bit(KEY_REWIND, pidrio->input_dev->keybit);
    pidrio->pressed_keymap[1] = KEY_PREVIOUSSONG;
    pidrio->held_keymap[1] = KEY_REWIND;

    set_bit(KEY_PLAYPAUSE, pidrio->input_dev->keybit);
    pidrio->pressed_keymap[2] = KEY_PLAYPAUSE;
    pidrio->held_keymap[2] = KEY_PLAYPAUSE;
    
    // Forward button press/hold
    set_bit(KEY_NEXTSONG, pidrio->input_dev->keybit);
    set_bit(KEY_FASTFORWARD, pidrio->input_dev->keybit);
    pidrio->pressed_keymap[3] = KEY_NEXTSONG;
    pidrio->held_keymap[3] = KEY_FASTFORWARD;

    set_bit(KEY_BACK, pidrio->input_dev->keybit);
    pidrio->pressed_keymap[4] = KEY_BACK;
    pidrio->held_keymap[4] = KEY_HOME;
    
    // Scrollwheel
    set_bit(KEY_SCROLLUP, pidrio->input_dev->keybit);
    set_bit(KEY_SCROLLDOWN, pidrio->input_dev->keybit);
    pidrio->pressed_keymap[7] = KEY_SCROLLUP;
    pidrio->held_keymap[7] = KEY_SCROLLUP;
    pidrio->pressed_keymap[6] = KEY_SCROLLDOWN;
    pidrio->held_keymap[6] = KEY_SCROLLDOWN;
    
    // Scrollwheel click/hold
    set_bit(KEY_ENTER, pidrio->input_dev->keybit);
    set_bit(KEY_MENU, pidrio->input_dev->keybit);
    pidrio->pressed_keymap[5] = KEY_ENTER;
    pidrio->held_keymap[5] = KEY_MENU;

    // Power button press/hold
    set_bit(KEY_STOP, pidrio->input_dev->keybit);
    set_bit(KEY_POWER, pidrio->input_dev->keybit);
    pidrio->pressed_keymap[0] = KEY_STOP;
    pidrio->held_keymap[0] = KEY_POWER;

    set_bit(KEY_A, pidrio->input_dev->keybit);

    pidrio_queue(pidrio);

    /* and finally register with the input core */
    input_register_device(pidrio->input_dev);
    pidrio_num_devices++;
    return 0;
    
failblue:
	led_classdev_unregister(&pidrio->mled[GREEN].led_cdev);

failgreen:
	led_classdev_unregister(&pidrio->mled[RED].led_cdev);

failred:
	sysfs_remove_group(&client->dev.kobj, &pidrio_group);

exit:    
    return err;
}

static int __devexit pidrio_remove(struct i2c_client *client)
{
    int i;
    //printk(KERN_ALERT "pidrio_remove\n");
	
    struct pidrio * pidrio = i2c_get_clientdata(client);

    pidrio->auto_update = 0;
    
    if (!cancel_delayed_work(&pidrio->work_item)) {
        flush_workqueue(pidrio_workqueue);
    }

	for (i = 0; i < 3; i++) {
		led_classdev_unregister(&pidrio->mled[i].led_cdev);
	}    

    del_timer_sync(&pidrio->input_dev->timer);
    input_unregister_device(pidrio->input_dev);
    
    sysfs_remove_group(&client->dev.kobj, &pidrio_group);
    
    kfree(pidrio);
    
    return 0;
}

static const struct i2c_device_id pidrio_id[] = {
    { "pidrio", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, pidrio_id);

static struct i2c_driver pidrio_driver = {
    .class = I2C_CLASS_HWMON,
    .driver = {
        .name = "pidrio",
        .owner = THIS_MODULE,
    },
    .probe = pidrio_probe,
    .remove = __devexit_p(pidrio_remove),
    .id_table = pidrio_id,
};

/* Init and Exit */

static void pidrio_cleanup( void ) {
    
    destroy_workqueue(pidrio_workqueue);   

    return;
}

static __init int pidrio_init(void)
{
    int err = -EFAULT;
    dev_t dev = 0;
    
    pidrio_workqueue = create_workqueue("pidrio_queue");
    if (pidrio_workqueue == NULL)
		goto exit;

    return i2c_add_driver(&pidrio_driver);

  exit:
    pidrio_cleanup();
    return -EFAULT;
}

static __exit void pidrio_exit(void)
{
    i2c_del_driver(&pidrio_driver);
    pidrio_cleanup();
}

module_init(pidrio_init);
module_exit(pidrio_exit);

MODULE_AUTHOR("Brian Starkey");
MODULE_DESCRIPTION("Pidrive low-level IO driver");
MODULE_LICENSE("GPL");

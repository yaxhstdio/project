// mario_controller.c
// Universal Low-Latency GPIO Game Controller Driver
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>

#define DRIVER_NAME "mario_controller"
#define DEVICE_NAME "mario_controller"
#define CLASS_NAME  "mario_ctrl"
#define NUM_BUTTONS 8
#define BUFFER_SIZE 64
#define DEBOUNCE_MS 5

/* Optional base offset if using virtual gpio-sim/gpio-mockup chip */
static int gpio_base = 0;
module_param(gpio_base, int, 0444);
MODULE_PARM_DESC(gpio_base, "Base GPIO offset for simulation (0 for native BCM GPIO)");

/* Simulation mode flag: if 1, skips physical GPIO claim and allows software IRQ injection via write() */
static int sim_mode = 0;
module_param(sim_mode, int, 0444);
MODULE_PARM_DESC(sim_mode, "Enable pure software simulation mode (1 = no GPIO hardware required)");

/* Button Configuration matching Fig. 3 Circuit Diagram */
struct button_config {
    int gpio_offset;
    int keycode;
    const char *label;
};

static const struct button_config button_map[NUM_BUTTONS] = {
    { 17, KEY_UP,    "BTN_UP"    }, // Physical Pin 11
    { 27, KEY_DOWN,  "BTN_DOWN"  }, // Physical Pin 13
    { 22, KEY_LEFT,  "BTN_LEFT"  }, // Physical Pin 15
    {  4, KEY_RIGHT, "BTN_RIGHT" }, // Physical Pin 7
    {  5, BTN_SOUTH, "BTN_A"     }, // Physical Pin 29
    {  6, BTN_EAST,  "BTN_B"     }, // Physical Pin 31
    { 13, BTN_NORTH, "BTN_X"     }, // Physical Pin 33
    { 19, BTN_WEST,  "BTN_Y"     }  // Physical Pin 35
};

/* Event record structure for /dev/mario_controller character device */
struct controller_event {
    __u8  button_id;      // 0 to 7
    __u8  state;          // 1 = Pressed, 0 = Released
    __u64 timestamp_ns;   // High-resolution ktime_get_ns() for latency measurement
};

/* Per-button runtime state */
struct button_priv {
    int id;
    int gpio_num;
    struct gpio_desc *desc;
    int irq;
    int last_state;
    unsigned long last_jiffies;
};

/* Global Driver State */
struct mario_drv_data {
    dev_t dev_num;
    struct cdev cdev;
    struct class *dev_class;
    struct device *dev_device;
    struct input_dev *input_dev;

    struct button_priv buttons[NUM_BUTTONS];
    u8 button_bitmap;

    /* Circular Buffer for Character Device */
    struct controller_event event_buf[BUFFER_SIZE];
    int head;
    int tail;
    int count;

    spinlock_t lock;
    wait_queue_head_t wait_q;
};

static struct mario_drv_data *mario_data;

/* Core event processing logic (shared by Hardware ISR and Simulation Trigger) */
static void process_button_event(int btn_id, int raw_pin_value)
{
    unsigned long flags;
    unsigned long now_jiffies = jiffies;
    u64 now_ns = ktime_get_ns();
    int pressed;
    struct button_priv *btn = &mario_data->buttons[btn_id];

    /* Active-LOW configuration (10k pull-up): 0V (LOW) = Pressed (1), 3.3V (HIGH) = Released (0) */
    pressed = (raw_pin_value == 0) ? 1 : 0;

    spin_lock_irqsave(&mario_data->lock, flags);

    /* Software Debounce Check using jiffies */
    if (time_before(now_jiffies, btn->last_jiffies + msecs_to_jiffies(DEBOUNCE_MS))) {
        spin_unlock_irqrestore(&mario_data->lock, flags);
        return;
    }

    /* Filter duplicate mechanical bounce states */
    if (pressed == btn->last_state) {
        spin_unlock_irqrestore(&mario_data->lock, flags);
        return;
    }

    /* Update state & bitmap */
    btn->last_state = pressed;
    btn->last_jiffies = now_jiffies;
    if (pressed)
        mario_data->button_bitmap |= (1 << btn_id);
    else
        mario_data->button_bitmap &= ~(1 << btn_id);

    /* Enqueue into Circular Buffer for /dev/mario_controller */
    if (mario_data->count < BUFFER_SIZE) {
        mario_data->event_buf[mario_data->head].button_id = btn_id;
        mario_data->event_buf[mario_data->head].state = pressed;
        mario_data->event_buf[mario_data->head].timestamp_ns = now_ns;
        mario_data->head = (mario_data->head + 1) % BUFFER_SIZE;
        mario_data->count++;
    }

    spin_unlock_irqrestore(&mario_data->lock, flags);

    /* 1. Report to Linux Input Subsystem (/dev/input/eventX) */
    input_report_key(mario_data->input_dev, button_map[btn_id].keycode, pressed);
    input_sync(mario_data->input_dev);

    /* 2. Wake sleeping user-space processes on Character Device wait queue */
    wake_up_interruptible(&mario_data->wait_q);
}

/* Hardware Interrupt Service Routine (ISR) */
static irqreturn_t mario_gpio_isr(int irq, void *dev_id)
{
    struct button_priv *btn = (struct button_priv *)dev_id;
    int val = gpiod_get_value(btn->desc);
    
    if (val < 0)
        return IRQ_NONE;

    process_button_event(btn->id, val);
    return IRQ_HANDLED;
}

/* Character Device: Blocking read() implementation */
static ssize_t mario_cdev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    unsigned long flags;
    struct controller_event evt;
    int ret;

    if (count < sizeof(struct controller_event))
        return -EINVAL;

    /* Sleep in wait queue (0% CPU usage) until an event is available */
    if (filp->f_flags & O_NONBLOCK) {
        if (mario_data->count == 0)
            return -EAGAIN;
    } else {
        ret = wait_event_interruptible(mario_data->wait_q, mario_data->count > 0);
        if (ret)
            return -ERESTARTSYS;
    }

    /* Pop one event from Circular Buffer under spinlock protection */
    spin_lock_irqsave(&mario_data->lock, flags);
    if (mario_data->count == 0) {
        spin_unlock_irqrestore(&mario_data->lock, flags);
        return 0;
    }
    evt = mario_data->event_buf[mario_data->tail];
    mario_data->tail = (mario_data->tail + 1) % BUFFER_SIZE;
    mario_data->count--;
    spin_unlock_irqrestore(&mario_data->lock, flags);

    if (copy_to_user(buf, &evt, sizeof(struct controller_event)))
        return -EFAULT;

    return sizeof(struct controller_event);
}

/* 
 * Character Device: write() hook for Worst-Case Simulation!
 * Writing "0 0" simulates Button 0 pulled LOW (Pressed).
 * Writing "0 1" simulates Button 0 pulled HIGH (Released).
 */
static ssize_t mario_cdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    char kbuf[16];
    int btn_id, pin_val;

    if (count >= sizeof(kbuf))
        return -EINVAL;
    if (copy_from_user(kbuf, buf, count))
        return -EFAULT;
    kbuf[count] = '\0';

    if (sscanf(kbuf, "%d %d", &btn_id, &pin_val) == 2) {
        if (btn_id >= 0 && btn_id < NUM_BUTTONS) {
            process_button_event(btn_id, pin_val);
            return count;
        }
    }
    return -EINVAL;
}

static const struct file_operations mario_fops = {
    .owner = THIS_MODULE,
    .read  = mario_cdev_read,
    .write = mario_cdev_write,
};

static int __init mario_driver_init(void)
{
    int i, ret;

    mario_data = kzalloc(sizeof(struct mario_drv_data), GFP_KERNEL);
    if (!mario_data)
        return -ENOMEM;

    spin_lock_init(&mario_data->lock);
    init_waitqueue_head(&mario_data->wait_q);

    /* 1. Allocate & Register Character Device (/dev/mario_controller) */
    ret = alloc_chrdev_region(&mario_data->dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) goto err_free_data;

    cdev_init(&mario_data->cdev, &mario_fops);
    ret = cdev_add(&mario_data->cdev, mario_data->dev_num, 1);
    if (ret < 0) goto err_unreg_chrdev;

    mario_data->dev_class = class_create(CLASS_NAME);   
    if (IS_ERR(mario_data->dev_class)) {
        ret = PTR_ERR(mario_data->dev_class);
        goto err_del_cdev;
    }

    mario_data->dev_device = device_create(mario_data->dev_class, NULL,
                                           mario_data->dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(mario_data->dev_device)) {
        ret = PTR_ERR(mario_data->dev_device);
        goto err_destroy_class;
    }

    /* 2. Allocate & Register Linux Input Subsystem Device */
    mario_data->input_dev = input_allocate_device();
    if (!mario_data->input_dev) {
        ret = -ENOMEM;
        goto err_destroy_device;
    }

    mario_data->input_dev->name = "Mario GPIO Controller";
    mario_data->input_dev->phys = "gpio/input0";
    mario_data->input_dev->id.bustype = BUS_HOST;
    mario_data->input_dev->evbit[0] = BIT_MASK(EV_KEY);

    for (i = 0; i < NUM_BUTTONS; i++)
        set_bit(button_map[i].keycode, mario_data->input_dev->keybit);

    ret = input_register_device(mario_data->input_dev);
    if (ret) {
        input_free_device(mario_data->input_dev);
        goto err_destroy_device;
    }

    /* 3. Configure GPIOs and Edge-Triggered Interrupts (unless in pure software sim_mode) */
    if (!sim_mode) {
        for (i = 0; i < NUM_BUTTONS; i++) {
            struct button_priv *btn = &mario_data->buttons[i];
            btn->id = i;
            btn->gpio_num = gpio_base + button_map[i].gpio_offset;
            btn->last_state = 0;
            btn->last_jiffies = 0;

            btn->desc = gpio_to_desc(btn->gpio_num);
            if (!btn->desc) {
                pr_err("%s: Failed to get GPIO descriptor for pin %d\n", DRIVER_NAME, btn->gpio_num);
                ret = -ENODEV;
                goto err_free_irqs;
            }

            gpiod_direction_input(btn->desc);
            btn->irq = gpiod_to_irq(btn->desc);
            if (btn->irq < 0) {
                pr_err("%s: Failed to map IRQ for GPIO %d\n", DRIVER_NAME, btn->gpio_num);
                ret = btn->irq;
                goto err_free_irqs;
            }

            ret = request_irq(btn->irq, mario_gpio_isr,
                              IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                              button_map[i].label, btn);
            if (ret) {
                pr_err("%s: Failed to request IRQ %d\n", DRIVER_NAME, btn->irq);
                goto err_free_irqs;
            }
        }
    } else {
        pr_info("%s: Loaded in SIMULATION MODE (no hardware GPIOs claimed).\n", DRIVER_NAME);
    }

    pr_info("%s: Driver initialized successfully.\n", DRIVER_NAME);
    return 0;

err_free_irqs:
    while (--i >= 0)
        free_irq(mario_data->buttons[i].irq, &mario_data->buttons[i]);
    input_unregister_device(mario_data->input_dev);
err_destroy_device:
    device_destroy(mario_data->dev_class, mario_data->dev_num);
err_destroy_class:
    class_destroy(mario_data->dev_class);
err_del_cdev:
    cdev_del(&mario_data->cdev);
err_unreg_chrdev:
    unregister_chrdev_region(mario_data->dev_num, 1);
err_free_data:
    kfree(mario_data);
    return ret;
}

static void __exit mario_driver_exit(void)
{
    int i;
    if (!sim_mode) {
        for (i = 0; i < NUM_BUTTONS; i++)
            free_irq(mario_data->buttons[i].irq, &mario_data->buttons[i]);
    }
    input_unregister_device(mario_data->input_dev);
    device_destroy(mario_data->dev_class, mario_data->dev_num);
    class_destroy(mario_data->dev_class);
    cdev_del(&mario_data->cdev);
    unregister_chrdev_region(mario_data->dev_num, 1);
    kfree(mario_data);
    pr_info("%s: Driver unloaded.\n", DRIVER_NAME);
}

module_init(mario_driver_init);
module_exit(mario_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yash G. Patil");
MODULE_DESCRIPTION("Universal Low-Latency GPIO Game Controller Driver");
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/delay.h> // for usleep_range
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <asm/uaccess.h>
#include <linux/kobject.h>
#include <linux/sched.h>

//#define DISABLE_KERNEL_PREEMPTION 1
// uncomment #define DISABLE_KERNEL_PREEMPTION if you wish to disable kernel preemption

#define MAXLOOP 10000

// ***************  GPIO ACCESS FUNCTIONS AND MACROS ************
// Raspberry Pi has mapped the base GPIO address here
// see https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
// Section 6 on General Purposes I/O (GPIO)
// And see section 1.2.2 ARM virtual addresses to see the mapping to
// 0xF2nnnnnn from 0x7Ennnnnnnn
// For all of the gpio functions defined here, the info for creating them
// was gleaned from Section 6
// WARNING!!!  This is the Raspberry Pi 1!
// Since we are on a Raspberry Pi 3, the mapping is actually mapping to:
// 0x3F2nnnnn from 0x7Ennnnnn

#define GPIO_REG_PHYS_ADDR 0x3f200000
#define GPIO_SIZE 0x40
#define GPIO_NAME "Direct GPIO Access"
#define GPIO_INPUT 0b000
#define GPIO_OUTPUT 0b001

#define GPIO_GPFSEL0 0x0
#define GPIO_ALTFUNC0 0b100
#define GPIO_ALTFUNC1 0b101
#define GPIO_ALTFUNC2 0b110
#define GPIO_ALTFUNC3 0b111
#define GPIO_ALTFUNC4 0b011
#define GPIO_ALTFUNC5 0b010

#define GPIO_GPSET0 0x1C
#define GPIO_GPCLR0 0x28
#define GPIO_ON true
#define GPIO_OFF false
#define GPIO_DRIVE_HI

#define GPIO_MAX_PIN 53

static uint8_t *Gpio_addr;

// set the gpio pin - input, output or alt function
static void set_gpio_altfunc(int gpio, int altfunc)
{
    int reg_index;
    int bit;
    unsigned int old_value, new_value;
    unsigned int mask;
    uint32_t *gpfsel_regaddr;

    reg_index = gpio / 10;
    bit = (gpio % 10) * 3;
    mask = 0b111 << bit;
    gpfsel_regaddr = (uint32_t *)(Gpio_addr + (GPIO_GPFSEL0 + (reg_index * 4)));
    old_value = ioread32(gpfsel_regaddr);
    printk("Changing function of GPIO%d from %x to %x\n",
           gpio,
           (old_value >> bit) & 0b111,
           altfunc);

    new_value = (old_value & ~mask) | ((altfunc << bit) & mask);
    iowrite32(new_value, gpfsel_regaddr);
}

static void set_gpio_output_value(int gpio,  bool value)
{
    uint32_t gpio_setreg, gpio_clrreg;
    int reg_index;
    int bit_to_use;

    if (gpio > GPIO_MAX_PIN)
    {
        printk(KERN_ERR "%s: gpio pin of %d requested, max is %d\n",
               __func__, gpio, GPIO_MAX_PIN);
        return;
    }
    reg_index = gpio / 32; // register 0 or 1
    bit_to_use = 1 << (gpio % 32);

    if (value == 0)
    {
        gpio_clrreg = ((uint32_t) Gpio_addr) + (GPIO_GPCLR0 + (reg_index * 4));
        iowrite32(bit_to_use, (uint32_t *) gpio_clrreg);
    }
    else
    {
        gpio_setreg = ((uint32_t) Gpio_addr) + (GPIO_GPSET0 + (reg_index * 4));
        iowrite32(bit_to_use, (uint32_t *) gpio_setreg);
    }
}

static void run_pulse_test(int pulse_width, int gpio_pin);

// objects/vars accessible to user
static int Gpio_pin_to_drive = 4;  // default to 4, J8 Pin 7
// note that the Shortest and Longest pulse_widths are stored in nsec,
static int Shortest_pulse_width = 0x7FFFFFFF;  // biggest signed value
static int Longest_pulse_width = 0;
// but User_pulse_width is stored in usec
static int User_pulse_width = 10 * 1000; // default to 10 ms
// accumulated values to compute average
static long long Run_time_average = 1;
static int Average_divider = 1;

static uint8_t Stop_loop = 0;

enum user_delay_enum_type {mdelay_type, usleep_type, end_delay_types};
// busy wait or kernel sleep for delay?
static enum user_delay_enum_type User_delay_type = mdelay_type;

static int get_average_pulse_width(void)
{
    long hold_numerator;
    int average;

    if (Average_divider == 1)
    {
        return 0;
    }

    hold_numerator = Run_time_average;
    average = (int) div64_s64(Run_time_average, Average_divider);
    printk("%s: average=%d, divider=%d\n",
           __func__, average, Average_divider);
    return average;
}

static void do_the_delay_or_sleep(int usec_min, int usec_max)
{
    if (User_delay_type == mdelay_type)
    {
        if (usec_min > 1000)
        {
            mdelay(usec_min/1000); // convert usec to msec
        }
        else
        {
            udelay(usec_min);
        }

    }
    else if (User_delay_type == usleep_type)
    {
        usleep_range(usec_min, usec_max);
    }
}

ssize_t show_pulse_width(struct device *dev,
                         struct device_attribute *attr, char *buf)
{
    printk("%s, %d usec\n", __func__, User_pulse_width);
    return sprintf(buf, "User set pulse width of %d usec\n",
                   User_pulse_width);
}

ssize_t show_pulse_results(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
    printk("%s, L=%dnsec, S=%dnsec\n", __func__,
           Longest_pulse_width, Shortest_pulse_width);
    return sprintf(buf, "Results: longest=%dnsec, shortest=%dnsec, ave=%dnsec\n",
                   Longest_pulse_width, Shortest_pulse_width,
                   get_average_pulse_width());
}

ssize_t show_gpio_pin(struct device *dev,
                      struct device_attribute *attr, char *buf)
{
    printk("%s, Driving gpio pin %d\n", __func__, Gpio_pin_to_drive);
    return sprintf(buf, "Driving gpio pin %d\n", Gpio_pin_to_drive);
}

ssize_t show_user_delay_type(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    if (User_delay_type == mdelay_type)
    {
        return sprintf(buf, "mdelay - busy/wait (val=%d)\n", User_delay_type);
    }
    else if (User_delay_type == usleep_type)
    {
        return sprintf(buf, "usleep_range, letting the kernel sleep (val=%d)\n",
                       User_delay_type);
    }
    return 0; // should never reach here
}

// NOTE: user needs to set pulse width in usec
ssize_t set_pulse_width(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t len)
{
    int r;

    r = kstrtoint(buf, 10, &User_pulse_width);
    printk("%s: setting pulse width to %dusec\n", __func__, User_pulse_width);
    return r ? 0 : len;
}

ssize_t start_pulse_run(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t len)
{
    printk("%s: starting, width of %dusec\n", __func__, User_pulse_width);


    run_pulse_test(User_pulse_width, Gpio_pin_to_drive);
    return len;
}

// The user can change which gpio pin to be driving
ssize_t set_gpio_pin(struct device *dev, struct device_attribute *attr,
                     const char *buf, size_t len)
{
    int r;

    printk("%s\n", __func__);
    r = kstrtoint(buf, 10, &Gpio_pin_to_drive);
    set_gpio_altfunc(Gpio_pin_to_drive, GPIO_OUTPUT);

    return (r ? 0 : len);
}

// The user can request to do a busy wait delay or a sleep see
// enum user_delay_enum_type for values
ssize_t set_user_delay_type(struct device *dev, struct device_attribute *attr,
                            const char *buf, size_t len)
{
    int r, hold_delay_type;

    printk("%s\n", __func__);
    r = kstrtoint(buf, 10, &hold_delay_type);
    if (hold_delay_type < end_delay_types)
    {
        User_delay_type = (enum user_delay_enum_type) hold_delay_type;
    }
    return (r ? 0 : len);
}

// The user can request to stop the test
ssize_t set_stop_loop(struct device *dev, struct device_attribute *attr,
                 const char *buf, size_t len)
{
    printk("%s\n", __func__);

    Stop_loop = 1;
    return (len);
}

static void run_pulse_test(int pulse_width, int gpio_pin)
{
    int i, loopsize, x, sched_ret;
    struct timespec starttime, endtime, deltatime;
    const int cpu = 3; // /boot/cmdline.txt has isolcpus=3, so only we are on it
    // highest real time priority
    const struct sched_param sched_param = {.sched_priority = 99};
    cpumask_t save_allowed;

    Stop_loop = 0;
    Run_time_average = 0;
    if (User_delay_type == 0)
    {
        printk(KERN_INFO "doing mdelay - busy/wait\n");
    }
    else
    {
        printk(KERN_INFO "doing usleep_range, letting the kernel sleep\n");
    }
    printk(KERN_INFO "running pulse test - my pid is %d\n", current->pid);

    save_allowed = current->cpus_allowed;
    // set the cpu affinity of this process
    if (set_cpus_allowed_ptr(current, cpumask_of(cpu)) != 0)
    {
        printk(KERN_ERR "cpu affinity returned error\n");
    }
    // set scheduler to RT SCHED_FIFO
    sched_ret = sched_setscheduler(current, SCHED_FIFO, &sched_param);
    if (sched_ret != 0)
    {
        printk(KERN_ERR "Setting scheduling to SCHED_FIFO returned error %d\n", sched_ret);
    }
#ifdef DISABLE_KERNEL_PREEMPTION
    get_cpu();
#endif

    // initialize values
    Shortest_pulse_width = 0x7FFFFFFF;  // biggest signed value
    Longest_pulse_width = 0;


    // start with value low
    set_gpio_output_value(gpio_pin, GPIO_OFF);
    loopsize = MAXLOOP;  // to make a shorter test, set loopsize to a smaller value
    x = MAXLOOP/pulse_width;
    // make the loopsize longer for short pulse widths
    loopsize *= x;

    printk("loopsize is %d\n",loopsize);
    for (i=0; i < loopsize; i++)
    {
        if (Stop_loop != 0)
        {
            break;
            // get out of here
        }
        getnstimeofday(&starttime);
        set_gpio_output_value(gpio_pin, GPIO_ON);

        do_the_delay_or_sleep(pulse_width, pulse_width);
        set_gpio_output_value(gpio_pin, GPIO_OFF);
        getnstimeofday(&endtime);

        // sleep to make the pulse obvious on oscope or logic analyzer
        do_the_delay_or_sleep(pulse_width * 2, pulse_width * 2);
        deltatime = timespec_sub(endtime, starttime);
        if (deltatime.tv_sec != 0)
        {
            printk(KERN_ERR "we had %lu second delay!!!!!  ERROR\n",
                   deltatime.tv_sec);
        }

        if (deltatime.tv_nsec < Shortest_pulse_width)
        {
            Shortest_pulse_width = deltatime.tv_nsec;
            printk(KERN_INFO "shortest time is now %d\n",
                   Shortest_pulse_width);
        }
        if (deltatime.tv_nsec > Longest_pulse_width)
        {
            Longest_pulse_width = deltatime.tv_nsec;
            printk(KERN_INFO "longest time is now %d\n",
                   Longest_pulse_width);
        }
        // keep an average
        Run_time_average += deltatime.tv_nsec;
        Average_divider = i+1;
    }
#ifdef DISABLE_KERNEL_PREEMPTION
    put_cpu();
#endif
    // put this task back on the un-isolated CPU core (restore the cpu affinity)
    set_cpus_allowed_ptr(current, &save_allowed);
    printk(KERN_INFO "longest %dnsec, shortest time was %dnsec, ave=%dnsec\n",
           Longest_pulse_width, Shortest_pulse_width,
           get_average_pulse_width());
}

#undef VERIFY_OCTAL_PERMISSIONS
#define VERIFY_OCTAL_PERMISSIONS(perms) (perms)

static DEVICE_ATTR(pulse_run, S_IWUSR | S_IRUGO, show_pulse_results, start_pulse_run);
static DEVICE_ATTR(gpio_pin, S_IWUSR | S_IRUGO, show_gpio_pin, set_gpio_pin);
static DEVICE_ATTR(pulse_width, S_IWUSR | S_IRUGO, show_pulse_width, set_pulse_width);
static DEVICE_ATTR(user_delay_type, S_IWUSR | S_IRUGO, show_user_delay_type, set_user_delay_type);
static DEVICE_ATTR(stop_run, S_IWUSR, NULL, set_stop_loop);

static struct kobject *kobj;

static int __init gpio_pulse_init(void)
{
    int ret;

    ret = 0;

    kobj = kobject_create_and_add("gpio_control", NULL);
    if (kobj == NULL)
    {
        printk(KERN_ERR "kobj could not be created!!\n");
        return -1;
    }

    if (sysfs_create_file(kobj, &dev_attr_pulse_run.attr) != 0)
    {
        printk(KERN_ERR "sysfs_create:pulse_run attr returned non 0 value\n");
    }
    if (sysfs_create_file(kobj, &dev_attr_gpio_pin.attr) != 0)
    {
        printk(KERN_ERR "sysfs_create: gpio_pin attr returned non 0 value\n");
    }
    if (sysfs_create_file(kobj, &dev_attr_pulse_width.attr) != 0)
    {
        printk(KERN_ERR "sysfs_create:pulse_width attr returned non 0 value\n");
    }
    if (sysfs_create_file(kobj, &dev_attr_user_delay_type.attr) != 0)
    {
        printk(KERN_ERR "sysfs_create:user_delay_type attr returned non 0 value\n");
    }
    if (sysfs_create_file(kobj, &dev_attr_stop_run.attr) != 0)
    {
        printk(KERN_ERR "sysfs_create: stop_run attr returned non 0 value\n");
    }

    // set up gpio access
    if (!request_mem_region(GPIO_REG_PHYS_ADDR, GPIO_SIZE, GPIO_NAME))
    {
        printk(KERN_INFO "Request_mem_region failed!!\n");
        printk("Tried to request addr 0x%X, size 0x%X, name %s\n",
               GPIO_REG_PHYS_ADDR, GPIO_SIZE, GPIO_NAME);
#if 0
        // WARNING - we allow ioremap to continue on, it just won't
        // have exclusive access.  So be sure not to use any of the
        // standard gpio functions, or there will be conflict/instability
        // and all manner of evilness
        ret = -EBUSY;
        goto out;
#endif
    }

    Gpio_addr = ioremap(GPIO_REG_PHYS_ADDR, GPIO_SIZE);
    printk(KERN_INFO "Gpio_addr = 0x%p\n", Gpio_addr);

    // set up the default gpio pin as output
    if (Gpio_addr == 0)
    {
        printk(KERN_ERR "Uh oh, ioremap returned 0\n");
        ret = -1;
        goto out;
    }
    set_gpio_altfunc(Gpio_pin_to_drive, GPIO_OUTPUT);

    return ret;

out:
    // remove the sysfs files
    sysfs_remove_file(kobj, &dev_attr_pulse_width.attr);
    sysfs_remove_file(kobj, &dev_attr_gpio_pin.attr);
    sysfs_remove_file(kobj, &dev_attr_pulse_run.attr);
    sysfs_remove_file(kobj, &dev_attr_user_delay_type.attr);
    // remove the kernel object
    kobject_put(kobj);
    return ret;
}

static void __exit gpio_pulse_exit(void)
{
    printk(KERN_INFO "Exiting: cleanup\n");

    // release GPIO
    iounmap(Gpio_addr);
    release_mem_region(GPIO_REG_PHYS_ADDR, GPIO_SIZE);

    // remove the sysfs files
    sysfs_remove_file(kobj, &dev_attr_pulse_width.attr);
    sysfs_remove_file(kobj, &dev_attr_gpio_pin.attr);
    sysfs_remove_file(kobj, &dev_attr_pulse_run.attr);
    sysfs_remove_file(kobj, &dev_attr_user_delay_type.attr);
    // remove the kernel object
    kobject_put(kobj);
}

module_init(gpio_pulse_init);
module_exit(gpio_pulse_exit);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Control gpio pulse with < 1msec accuracy");
MODULE_VERSION("Jan 3 2017");


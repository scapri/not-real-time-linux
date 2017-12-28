#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>

#include <linux/delay.h> // for usleep_range

#include <asm/io.h> // for talking to gpio
#include <mach/platform.h> // for talkint to gpio

// SANDRA, STOP dereferencing variable use the iowrite32 function!!!!!
// utility functions

static inline unsigned ccnt_read (void)
{
  unsigned cc;
  asm volatile ("mrc p15, 0, %0, c15, c12, 1" : "=r" (cc));
  return cc;
}

// GPIO - structure reflects registers as defined in the Broadcom h/w spec
// https://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
struct gpio_registers
{
    uint32_t GPFSEL[6];
    uint32_t Reserved1;
    uint32_t GPSET[2];
    uint32_t Reserved2;
    uint32_t GPCLR[2];
    uint32_t Reserved3;
    uint32_t GPLEV0[2];
    uint32_t Reserved4;
};

struct gpio_registers *gpio_regs;
#define GPIO_OUT 0b001
#define GPIO_ON true
#define GPIO_OFF false

// set the gpio pin - input, output or alt function
static void set_gpio_altfunc(int gpio, int altfunc)
{
    int reg_index = gpio / 10;
    int bit = (gpio % 10) * 3;

    unsigned old_value = gpio_regs->GPFSEL[reg_index];
    unsigned mask = 0b111 << bit;
    printk("Changing function of GPIO%d from %x to %x\n",
           gpio,
           (old_value >> bit) & 0b111,
           altfunc);

    gpio_regs->GPFSEL[reg_index] =
        (old_value & ~mask) | ((altfunc << bit) & mask);
}

static void set_gpio_output_value(int gpio,  bool value)
{
    if (value)
        gpio_regs->GPSET[gpio/32] = (1 << (gpio % 32));
    else
        gpio_regs->GPCLR[gpio/32] = (1 << (gpio % 32));
}


// probe - ideas - do setup, get device tree information
// (e.g. pulse width parms), and set up the ISR
// then start a pthread that can wait for a sysfs write
// (poll or function sysfs) to tell it to do the first
// pulse, and it will continue to do the pulses, incrementing
// length based on the pulse width parms.  When the interrupt
// comes, it will imeediately drop the pulse, and stop any future
// pulses until the next sysfs request

//SANDRA, need to add protected variable for current gpio state
// When the interrupt happens, only set low if state is hi
// when the hrtimer finishes the sleep, only set low if state is hi
// Also another one for xray_complete, and when set (only in the int), the
// main thread won't do any new pulses if xray_complete is set.


#define dothesleep(sleep1, sleep2)  usleep_range(sleep1, sleep2)
#if 1
#define do_the_important_sleep(sleep1, sleep2)  mdelay(sleep1/1000)
#define printthis "doing mdelay - busy wait\n"
#else
#define do_the_important_sleep(sleep1, sleep2)  usleep_range(sleep1, sleep2)
#define printthis "doing usleep_range, letting the kernel do the sleep\n"
#endif

// in the real code, read this from the dtb, or from a sysfs (security?)
int pulse_length[] = {10};

static const int xray_gpio_pin = 4;  // J8 Pin 7 - set in DTB, FIXME

static int __init xray_pulse_init(void)
{
    int cpu = 3; // /boot/cmdline.txt has isolcpus=3, so only we are on it
    struct sched_param sched_param = {.sched_priority = 99}; // highest rt priority
    int i, loopsize;
    struct timespec starttime, endtime, deltatime;
    long shortest, longest;

    printk(KERN_INFO "Entry: init - move most of this to probe later\n");
    printk(KERN_INFO printthis);
    shortest = 0x7FFFFFFF;
    longest = 0;

    // set up gpio access
    gpio_regs = (struct gpio_registers *)__io_address(GPIO_BASE);
    set_gpio_altfunc(xray_gpio_pin, GPIO_OUT);

    // set the cpu affinity of this process
    set_cpus_allowed_ptr(current, cpumask_of(cpu));
    sched_setscheduler(current, SCHED_FIFO, &sched_param);
    // do we need to try to lock in memory to avoid paging - in kernel? (mlockall() user space)

//    printk(KERN_ERR "Sandra, setting GPIO LOW FIRST (with x) \n");
//    set_gpio_output_value(xray_gpio_pin, GPIO_OFF);
//    msleep(1000 * 20);
//    printk(KERN_ERR "Sandra, setting GPIO HI NOW\n");
//    set_gpio_output_value(xray_gpio_pin, GPIO_ON);
//    msleep(1000 * 20);
//    printk(KERN_ERR "Sandra, again setting GPIO LOW \n");
//    set_gpio_output_value(xray_gpio_pin, GPIO_OFF);
//    msleep(1000 * 20);
//    printk(KERN_ERR "Sandra, setting GPIO HI NOW\n");
//    set_gpio_output_value(xray_gpio_pin, GPIO_ON);
//    printk(KERN_ERR "Sandra, done with GPIO - hack quit\n");
//    return(0);

    // start with value low
    set_gpio_output_value(xray_gpio_pin, GPIO_OFF);
    loopsize = 10000;
//    loopsize = 2000;
    printk("loopsize is %d\n",loopsize);
    for (i=0; i<loopsize; i++)
    {
        getnstimeofday(&starttime);
        set_gpio_output_value(xray_gpio_pin, GPIO_ON);

        // will we have a problem with interrupts since this is uninterruptible?  or will the isr fire fine?
        do_the_important_sleep(1000 * pulse_length[0], 1000 * pulse_length[0]);
        set_gpio_output_value(xray_gpio_pin, GPIO_OFF);
        getnstimeofday(&endtime);

        // sleep to make the pulse obvious on oscope
        dothesleep(1000 * pulse_length[0]*3, 1000 * pulse_length[0]*3);
        deltatime = timespec_sub(endtime, starttime);
        if (deltatime.tv_sec != 0)
            printk(KERN_ERR "we had %lu second delay!!!!!  ERROR\n", deltatime.tv_sec);

        if (deltatime.tv_nsec < shortest)
            shortest = deltatime.tv_nsec;
        if (deltatime.tv_nsec > longest)
            longest = deltatime.tv_nsec;
        if (i%1000 == 0)
            printk(KERN_CRIT "i=%d, shortest time was %lu longest time was %lu (curr=%lu!)\n",
                   i, shortest, longest, deltatime.tv_nsec);
    }
    printk(KERN_CRIT "shortest time was %lu longest time was %lu\n",
           shortest, longest);
    return 0;
}

static void __exit xray_pulse_exit(void)
{
   printk(KERN_INFO "Exiting: cleanup\n");
}

module_init(xray_pulse_init);
module_exit(xray_pulse_exit);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Control x-ray pulse with 1msec accuracy");
MODULE_VERSION("Dec 18 2016");

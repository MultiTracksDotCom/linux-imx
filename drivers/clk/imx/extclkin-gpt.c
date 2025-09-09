/*
 * AUDINATE EXAMPLE CODE LICENSE
 *
 * PERMISSION NOTICE
 *
 * Copyright © 2020 Audinate Pty Ltd.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * extclkin-gpt.c
 * External clock input driver - i.MX 8M GPT version
 *
 * This is a character device driver which returns a pair of times in nanoseconds
 * when read from - an external clock time and the linux monotonic raw time.
 * This version uses an external clock input pin into a i.MX 8M General Purpose Timer
 * (GPT) to act as a counter and determine an accurate and tracking (to external
 * clocking circuitry) time, with an arbitrary offset. The "host" (linux) timestamp
 * provides a reference to determine this offset and track differences.
 *
 * See the README.md file for more information.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/timekeeping.h>
#include <linux/mutex.h>
#include <linux/preempt.h>
#include <linux/uaccess.h>


/* Requires GPL compatible license for module */
#define DRIVER_LICENSE "GPL"

#define DRIVER_AUTHOR "James Stuart <opensource@audinate.com>"
#define DRIVER_DESC "External clock input driver using i.MX general purpose timer (GPT)"
/* Used throughout, eg as device file name */
#define DEVICE_NAME "extclkin"

// For C preprocessor stringification
#define xstr(s) str(s)
#define str(s) #s

// Using 64-bit unsigned values for nano second counts, giving a max value of 18446744073709551615
// TODO: more preprocessor to get 20 digits from u64_MAX
#define NS_VALUE_MAX_DIGITS 20
#define MAX_LINE_LENGTH (2 * NS_VALUE_MAX_DIGITS + 2)
#define READ_BUFFER_SIZE (MAX_LINE_LENGTH + 1)

#define SUCCESS 0

/* Timer Defines */
#define GPT_MEM_SIZE    0x00010000UL

/* Timer base addresses */
#define GPT1_BASE   0x302D0000UL
#define GPT2_BASE   0x302E0000UL
#define GPT3_BASE   0x302F0000UL
#define GPT6_BASE   0x306E0000UL
#define GPT5_BASE   0x306F0000UL
#define GPT4_BASE   0x30700000UL

/* Timer Register address offsets */
#define GPT_CR      0x0000U
#define GPT_PR      0x0004U
#define GPT_SR      0x0008U
#define GPT_IR      0x000CU
#define GPT_OCR1    0x0010U
#define GPT_OCR2    0x0014U
#define GPT_OCR3    0x0018U
#define GPT_ICR1    0x001CU
#define GPT_ICR2    0x0020U
#define GPT_CNT     0x0024U

/* Timer register bit definitions */
#define GPT_CR_EN               (1UL << 0)  // enable
#define GPT_CR_ENMOD            (1UL << 1)  // enable mode. 1: reset count when disabled
#define GPT_CR_DBGEN            (1UL << 2)  // debug mode enable, 1: GPT is enabled in debug mode
#define GPT_CR_WAITEN           (1UL << 3)  // Wait Mode: 1: GPT is enabled in wait mode
#define GPT_CR_DOZEEN           (1UL << 4)  // Doze Mode: 1: GPT is enabled in doze mode
#define GPT_CR_STOPEN           (1UL << 5)  // Stop Mode: 1: GPT is enabled in stop mode
#define GPT_CR_CLKSRC_MASK      (7UL << 6)  // mask for clocksource bits
#define GPT_CR_CLKSRC_NOCLK     (0UL << 6)  // no clock
#define GPT_CR_CLKSRC_PERIPH    (1UL << 6)  // peripheral clock (ipg_clk)
#define GPT_CR_CLKSRC_HIGHFREQ  (2UL << 6)  // high frequency reference clock (ipg_clk_highfreq)
#define GPT_CR_CLKSRC_EXT       (3UL << 6)  // external clock
#define GPT_CR_CLKSRC_32K       (4UL << 6)  // Low frequency reference clock (ipg_clk_32k)
#define GPT_CR_CLKSRC_XTAL      (5UL << 6)  // Crystal oscillator as reference clock (ipg_clk_24M)
#define GPT_CR_FFR              (1UL << 9)  // Free-Run or Restart mode: 1: Free-Run mode, rolls over to 0 after reaching 0xFFFF FFFF
#define GPT_CR_EN_24M           (1UL << 10) // 1: Enable 24 MHz clock input from crystal
#define GPT_CR_SWR              (1UL << 15) // Software reset 1: to/in reset

#define GPT_SR_OF1              (1UL << 0)  // Output compare 1 flag. 1: compare event has occurred
#define GPT_SR_OF2              (1UL << 1)  // Output compare 2 flag. 1: compare event has occurred
#define GPT_SR_OF3              (1UL << 2)  // Output compare 3 flag. 1: compare event has occurred
#define GPT_SR_IF1              (1UL << 3)  // Input capture 1 flag. 1: capture event has occurred.
#define GPT_SR_IF2              (1UL << 4)  // Input capture 2 flag. 1: capture event has occurred.
#define GPT_SR_ROLLOVER         (1UL << 5)  // Rollover flag. 1: rollover has occurred.

/* Select which timer to use */
#define GPT_BASE_USED GPT2_BASE

/* Prototypes */
static int device_open(struct inode *, struct file *);
static int device_release(struct inode *, struct file *);
static ssize_t device_read(struct file *, char *, size_t, loff_t *);
static inline volatile u32 reg_read(u16 offset);
static inline void reg_write(u16 offset, u32 value);

/* Wrapper for read/write operations to GPT IO memory space */
#define gptmem_readb(c)     readb(c)
#define gptmem_readw(c)     readw(c)
#define gptmem_readl(c)     readl(c)
#define gptmem_readq(c)     readq(c)
#define gptmem_writeb(v,c)  writeb(v,c)
#define gptmem_writew(v,c)  writew(v,c)
#define gptmem_writel(v,c)  writel(v,c)
#define gptmem_writeq(v,c)  writeq(v,c)


/* Global variables for file */
static int deviceMajor;
static dev_t devNo;
static struct class *devClass;
static struct device *thisDev;
static void __iomem *timerMem;
static char readBuf[READ_BUFFER_SIZE];

/* Protect state against multiple readers */
DEFINE_MUTEX(accessTimer);
/* This state must be protected against multiple users */
static u32 overflowCount = 0;

/* Default timer input frequency: 10 MHz */
static unsigned long frequency = 10000000UL;
module_param(frequency, ulong, 0644);
MODULE_PARM_DESC(frequency, "Frequency of clock input to timer in Hz as unsigned long");

struct file_operations fops = {
	.read = device_read,
	.open = device_open,
	.release = device_release
};

/*
 * Read a GPT register with the given offset from the base address,
 * returning the result.
 */
static inline volatile u32 reg_read(u16 offset)
{
	return (volatile u32) gptmem_readl(offset + timerMem);
}

/*
 * Write a GPT register with given offset the given value.
 */
static inline void reg_write(u16 offset, u32 value)
{
	gptmem_writel(value, offset + timerMem);
}

/*
 * Setups and enables the GPT to:
 *   - use external clock
 *   - count to max value (32bit) before resetting
 *   - run in wait mode
 * Performs an initial software reset on the timer.
 */
static inline void setup_gpt(void)
{
	u32 regVal;
	// Disable GPT
	regVal = reg_read(GPT_CR);
	regVal &= ~GPT_CR_EN;               // clear enable flag
	reg_write(GPT_CR, regVal);          // write back

	// Software reset
	reg_write(GPT_CR, GPT_CR_SWR);

	// Set control register
	regVal = 0UL;                       // start blank, reset value
	regVal |= GPT_CR_ENMOD;             // reset count when disabled
	regVal |= GPT_CR_WAITEN;            // enable in wait mode
	regVal |= GPT_CR_CLKSRC_EXT;        // use external clock
	regVal |= GPT_CR_FFR;               // free running mode
	reg_write(GPT_CR, regVal);          // write control reg

	// Clear roll over flag
	regVal = reg_read(GPT_SR);
	regVal &= ~GPT_SR_ROLLOVER;
	reg_write(GPT_SR, regVal);

	// Enable GPT
	regVal = reg_read(GPT_CR);
	regVal |= GPT_CR_EN;
	reg_write(GPT_CR, regVal);
}

/*
 * Stops the GPT
 */
static inline void stop_gpt(void)
{
	u32 regVal;
	regVal = reg_read(GPT_CR);
	regVal &= ~GPT_CR_EN;       // clear enable flag
	reg_write(GPT_CR, regVal);
}

/*
 * Get the timer count value.
 * Sets rollover to be 1 if rollover status bit set.
 * Else rollover is set to 0.
 * Rollover status bit cleared if set.
 */
static inline u32 get_count(u8 *rollover)
{
	u32 count;
	u32 status;
	// Get count from timer
	count = reg_read(GPT_CNT);

	// Check rollover
	status = reg_read(GPT_SR);
	if (status & GPT_SR_ROLLOVER) {
		*rollover = 1;
		// clear flag by writing 1 for rollover, and 0's for others as not clearing any other flags
		reg_write(GPT_SR, GPT_SR_ROLLOVER);
		// ensure it's clear
		status = reg_read(GPT_SR);
		if (status & GPT_SR_ROLLOVER) {
			dev_err(thisDev, "can't clear rollover flag.\n");
		}
	} else {
		*rollover = 0;
	}
	return count;
}

/*
 * Initialise the clock input driver, GPT variant.
 */
static int __init gpt_clkin_init(void)
{
	pr_debug("Driver init in %s\n", __FILE__);

	deviceMajor = register_chrdev(0, DEVICE_NAME, &fops);

	if (deviceMajor < 0) {
		pr_err("Registering char device %s failed with %d\n", DEVICE_NAME, deviceMajor);
		return deviceMajor;
	}

	devNo = MKDEV(deviceMajor, 0);

	devClass = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(devClass)) {
		pr_err("Can't create class %s err: %ld\n", DEVICE_NAME, PTR_ERR(devClass));
		return -EINVAL;
	}

	if (IS_ERR(thisDev = device_create(devClass, NULL, devNo, NULL, DEVICE_NAME))) {
		pr_err("Can't create device /dev/%s err: %ld\n", DEVICE_NAME, PTR_ERR(thisDev));
		class_destroy(devClass);
		return -EINVAL;
	}

	dev_dbg(thisDev, "Driver %s got major number %d. Create a dev file with 'mknod /dev/%s c %d 0'.\n", DEVICE_NAME, deviceMajor, DEVICE_NAME, deviceMajor);

	timerMem = ioremap_nocache(GPT_BASE_USED, GPT_MEM_SIZE);

	if (!timerMem) {
		dev_err(thisDev, "Can't ioremap memory for GPT\n");
		device_destroy(devClass, devNo);
		class_destroy(devClass);
		return -ENOMEM;
	}

	// setup timer/counter
	setup_gpt();

	dev_info(thisDev, "driver initialised\n");

	// Returning non-zero indicates module can't be loaded
	return SUCCESS;
}

/*
 * Exit out of the driver / module.
 * De-register, destroy, cleanup, stop, ... all the things
 */
static void __exit gpt_clkin_exit(void)
{
	dev_dbg(thisDev, "exiting...\n");

	stop_gpt();
	iounmap(timerMem);
	device_destroy(devClass, devNo);
	class_destroy(devClass);
	unregister_chrdev(deviceMajor, DEVICE_NAME);
}

/*
 * Called when a process tries to open the device file.
 * eg cat /dev/DEVICE_NAME
 */
static int device_open(struct inode *inode, struct file *file)
{

	dev_dbg(thisDev, "Device %s opened. Max line length: %u + 1\n", DEVICE_NAME, MAX_LINE_LENGTH);

	/* Increment usage count to protect against module removal. */
	try_module_get(THIS_MODULE);
	return SUCCESS;
}

/*
 * Called when a process closes the device file.
 */
static int device_release(struct inode *inode, struct file *file)
{
	/* Decrement usage count so module can be removed. */
	module_put(THIS_MODULE);
	dev_dbg(thisDev, "Device %s released.\n", DEVICE_NAME);
	return SUCCESS;
}

/*
 * Called when a process, which has the device file open, attempts to read from it.
 */
static ssize_t device_read(struct file *filp,   /* ref: include/linux/fs.h   */
			char *buffer,        /* buffer to fill with data  */
			size_t length,       /* length of the buffer      */
			loff_t *offset       /* offset into file for read */
	)
{
	ssize_t bytesRead;      // retval
	u8 rollover;            // rollover occurred?
	u32 count;              // GPT count
	u64 host_ns, audio_ns;  // times
	u64 nsPerCount;         // frequency multiplier
	unsigned long flags;    // flags for saving IRQ state


	dev_dbg(thisDev, "Device %s read length %lu pos %lld offset %lld\n", DEVICE_NAME, length, filp->f_pos, *offset);

	/* quick EOF return for reads that aren't from the start */
	if (*offset != 0 || filp->f_pos != 0) {
		return 0;
	}

	/* Prevent access to timer and rollover status changing overflow count from multiple accessors */
	mutex_lock(&accessTimer);
	/* Want to ensure the clock access are as close to each other as possible */
	preempt_disable();
	/* Let's also disable interrupts, just to be safe */
	local_irq_save(flags);

	/* Capture host monotonic raw clock to reference timer value against */
	/* Do this first as it's quick */
	host_ns = ktime_get_raw_ns();
	/* Get the timer/counter value and rollover status */
	count = get_count(&rollover);

	/* and re-enable them */
	local_irq_restore(flags);
	/* Can be pre-empted again now */
	preempt_enable();

	/* handle rollover condition */
	if (rollover) {
		overflowCount++;
		dev_dbg(thisDev, "Device %s: rollover detected, count %u.\n", DEVICE_NAME, overflowCount);
	}
	/* Finished with exclusivity */
	mutex_unlock(&accessTimer);

	/* Derived from frequency as 1s / 1ns / frequency = 10^9 / 10^7 = 100 for freq = 10 MHz */
	nsPerCount = 1000000000UL / frequency;

	/* Calculate the audio time using overflow, count and above value */
	audio_ns = (((u64)overflowCount * (1ULL << 32)) + (u64) count) * nsPerCount;

	// print string into internal buffer
	bytesRead = snprintf(readBuf, READ_BUFFER_SIZE, "%" xstr(NS_VALUE_MAX_DIGITS) "llu\t%" xstr(NS_VALUE_MAX_DIGITS) "llu\n", host_ns, audio_ns);

	// if user buffer length isn't enough, log this and return no bytes
	if (length < READ_BUFFER_SIZE) {
		dev_warn(thisDev, "Device %s: read request had insufficient buffer size of %ld. Minimum required is %ld.\n", DEVICE_NAME, length, READ_BUFFER_SIZE);
		return -EINVAL;
	}

	// write back to user buffer, check for error
	if (copy_to_user(buffer, readBuf, bytesRead)) {
		dev_err(thisDev, "Device %s: couldn't write to device read buffer.\n", DEVICE_NAME);
		return -EFAULT;
	}

	dev_dbg(thisDev, "Device %s: bytes read: %ld\n", DEVICE_NAME, bytesRead);
	return bytesRead;
}

module_init(gpt_clkin_init);
module_exit(gpt_clkin_exit);

MODULE_LICENSE(DRIVER_LICENSE);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_SUPPORTED_DEVICE(DEVICE_NAME);
MODULE_DESCRIPTION("NXP i.MX91 clock driver");
MODULE_LICENSE("GPL v2");

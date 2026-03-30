/*
 * AUDINATE EXAMPLE CODE LICENSE
 *
 * PERMISSION NOTICE
 *
 * Copyright © 2025 Audinate Pty Ltd.
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
 * extclkin-gpio.c
 * External LR clock input driver - i.MX 8M GPIO version
 *
 * This is a character device driver which returns time in nanoseconds
 * when read from - the linux monotonic raw time at the rising edge of the clock
 * connected to a GPIO pin.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/timekeeping.h>
#include <linux/mutex.h>
#include <linux/preempt.h>
#include <linux/uaccess.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>

/* Requires GPL compatible license for module */
#define DRIVER_LICENSE "GPL"

#define DRIVER_AUTHOR "Sam Morris <scmorris.dev@gmail.com>"
#define DRIVER_DESC "External clock input driver using i.MX general purpose input/output (GPIO)"

/* Used throughout, eg as device file name */
#define DEVICE_NAME "extclkin-gpio"

// For C preprocessor stringification
#define xstr(s) str(s)
#define str(s) #s

#define NS_VALUE_MAX_DIGITS 20
#define MAX_LINE_LENGTH (NS_VALUE_MAX_DIGITS + 1)
#define READ_BUFFER_SIZE (MAX_LINE_LENGTH + 1)

#define SUCCESS 0

/* Limit the number of LRCLK edge detection polls to prevent the driver from going into an infinite loop */
#define LRCLK_POLL_ATTEMPTS_MAX 10000

/* Prototypes */
static int device_open(struct inode *, struct file *);
static int device_release(struct inode *, struct file *);
static ssize_t device_read(struct file *, char *, size_t, loff_t *);

/* Global variables */
static int device_major;
static dev_t device_number;
static struct class *device_class;
static struct device *this_device;
static struct gpio_desc *lrclk_gpiod;
static char read_buffer[READ_BUFFER_SIZE];

/* Protect state against multiple readers */
DEFINE_MUTEX(access_gpio_clk);

struct file_operations fops = {
	.read = device_read,
	.open = device_open,
	.release = device_release
};

// Return the timestamp of the next falling LRCLK edge. Returns 0 in case of error.
static inline u64 get_lrclk_edge_ts(void)
{
	register u32 val;
	u32 attempt;
	register u64 host_ns;
	unsigned long flags;

	/* Prevent access to gpio from multiple accessors */
	mutex_lock(&access_gpio_clk);

	/* Disable preemption to ensure the clock access are as close to each other as possible */
	preempt_disable();

	/* Disable interrupts */
	local_irq_save(flags);

	// A toggling LRCLK can only have one 'wrong' edge here
	attempt = 2;
#pragma GCC unroll 1
	while (attempt) { // Loop over edges
		register unsigned limit = LRCLK_POLL_ATTEMPTS_MAX;
		val = gpiod_get_value(lrclk_gpiod);
		while ((val == gpiod_get_value(lrclk_gpiod)) && (--limit)); // Fast loop waiting for edge
		host_ns = ktime_get_raw_ns(); // Fetch timestamp
		if (limit == 0) { // Hit the limit
			host_ns = 0; // Flag error
			break;
		}
		if (val == 1) break; // Falling edge, we're done

		attempt--;
	}

	/* Re-enable interrupts */
	local_irq_restore(flags);

	/* Can be pre-empted again now */
	preempt_enable();

	/* Finished with exclusivity */
	mutex_unlock(&access_gpio_clk);

	return host_ns;
}

/*
 * Called when a process tries to open the device file.
 * eg cat /dev/DEVICE_NAME
 */
static int device_open(struct inode *inode, struct file *file)
{
	dev_dbg(this_device, "Device %s opened.\n", DEVICE_NAME);

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
	dev_dbg(this_device, "Device %s released.\n", DEVICE_NAME);
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
	ssize_t bytes_read;
	u64 host_ns;

	dev_dbg(this_device, "Device %s read length %lu pos %lld offset %lld\n",
		DEVICE_NAME, length, filp->f_pos, *offset);

	/* quick EOF return for reads that aren't from the start */
	if (*offset != 0 || filp->f_pos != 0)
		return 0;

	/* If user buffer length is not sufficient, log this and return no bytes */
	if (length < READ_BUFFER_SIZE) {
		dev_warn(this_device,
			 "Device %s: read request had insufficient buffer size of %ld. Minimum required is %ld.\n",
			 DEVICE_NAME, length, (ssize_t)READ_BUFFER_SIZE);
		return -EINVAL;
	}

	host_ns = get_lrclk_edge_ts();

	/* Print string into internal buffer */
	bytes_read = snprintf(read_buffer, READ_BUFFER_SIZE,
			      "%" xstr(NS_VALUE_MAX_DIGITS) "llu\n", host_ns);

	/* write back to user buffer, check for error */
	if (copy_to_user(buffer, read_buffer, bytes_read)) {
		dev_err(this_device, "Device %s: couldn't write to device read buffer.\n",
			DEVICE_NAME);
		return -EFAULT;
	}

	dev_dbg(this_device, "Device %s: bytes read: %ld\n", DEVICE_NAME, bytes_read);
	return bytes_read;
}

static const struct of_device_id extclkingpio_of_match[] = {
	{ .compatible = "audinate,extclkin-gpio", },
	{ /* Sentinel */ }
};

MODULE_DEVICE_TABLE(of, extclkingpio_of_match);

static int extclkingpio_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;

	dev_info(dev, "Driver probing for device tree node: %s\n",
		 of_node_full_name(pdev->dev.of_node));

	lrclk_gpiod = devm_gpiod_get(dev, NULL, GPIOD_IN);
	if (IS_ERR(lrclk_gpiod)) {
		dev_err(dev, "Failed to get LRCLK GPIO: %ld\n", PTR_ERR(lrclk_gpiod));
		return PTR_ERR(lrclk_gpiod);
	}

	device_major = register_chrdev(0, DEVICE_NAME, &fops);
	if (device_major < 0) {
		dev_err(dev, "Registering char device %s failed with %d\n",
			DEVICE_NAME, device_major);
		return device_major;
	}

	device_number = MKDEV(device_major, 0);

	device_class = class_create(DEVICE_NAME);
	if (IS_ERR(device_class)) {
		dev_err(dev, "Cannot create class %s err: %ld\n",
			DEVICE_NAME, PTR_ERR(device_class));
		ret = -EINVAL;
		goto err_unregister_chrdev;
	}

	this_device = device_create(device_class, dev, device_number, NULL, DEVICE_NAME);
	if (IS_ERR(this_device)) {
		dev_err(dev, "Cannot create device /dev/%s err: %ld\n",
			DEVICE_NAME, PTR_ERR(this_device));
		ret = -EINVAL;
		goto err_destroy_class;
	}

	dev_dbg(this_device,
		"Driver %s got major number %d. Create a dev file with 'mknod /dev/%s c %d 0'.\n",
		DEVICE_NAME, device_major, DEVICE_NAME, device_major);

	dev_info(dev, "driver initialised (GPIO %s pin %u)\n",
		 gpiod_get_direction(lrclk_gpiod) == 0 ? "out" : "in",
		 desc_to_gpio(lrclk_gpiod));

	return SUCCESS;

err_destroy_class:
	class_destroy(device_class);
err_unregister_chrdev:
	unregister_chrdev(device_major, DEVICE_NAME);

	return ret;
}

static int extclkingpio_remove(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "exiting...\n");

	device_destroy(device_class, device_number);
	class_destroy(device_class);
	unregister_chrdev(device_major, DEVICE_NAME);

	return 0;
}

static struct platform_driver extclkingpio_driver = {
	.probe = extclkingpio_probe,
	.remove = extclkingpio_remove,
	.driver = {
		.name = "extclkin-gpio",
		.of_match_table = of_match_ptr(extclkingpio_of_match),
	},
};

/* Replace module_init and module_exit with platform driver macros */
module_platform_driver(extclkingpio_driver);

MODULE_LICENSE(DRIVER_LICENSE);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);

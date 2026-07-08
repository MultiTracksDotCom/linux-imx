// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/extcon/extcon-stm32-ipc.c - STM32 SPI IPC and Virtual Extcon Driver
 *
 * Copyright (C) 2026 MultiTracks
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/extcon-provider.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/crc8.h>

#define DRIVER_NAME "stm-spi-ipc"

/* Protocol constants */
#define STM_IPC_MSG_MAGIC         0x5A
#define MSG_TYPE_USB_EVENT        0x01
#define STM_IPC_MSG_LEN_USB_EVENT 2
#define STM_IPC_CRC8_POLYNOMIAL   0x07

enum stm_ipc_usb_state {
	STM_IPC_USB_STATE_DISCONNECTED = 0,
	STM_IPC_USB_STATE_PERIPHERAL   = 1,
	STM_IPC_USB_STATE_HOST         = 2,
};

struct stm_ipc_packet {
	u8 magic;
	u8 type;
	u8 length;
	u8 port;
	u8 state; /* enum stm_ipc_usb_state */
	u8 crc;
} __packed;

/* Parent SPI controller private structure */
DECLARE_CRC8_TABLE(stm_crc8_table);
static struct dentry *stm_ipc_debugfs_dir;

struct stm_ipc_priv {
	struct spi_device *spi;
	struct mutex lock;
	struct dentry *debugfs_root;
};

/* Child connector platform device private structure */
struct stm_connector_priv {
	struct extcon_dev *edev;
	u8 port_id;
};

static const unsigned int stm_usb_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

/* Helper to update extcon state based on STM32 reporting */
static void stm_ipc_update_state(struct extcon_dev *edev, u8 state)
{
	switch (state) {
	case STM_IPC_USB_STATE_DISCONNECTED:
		extcon_set_state_sync(edev, EXTCON_USB, false);
		extcon_set_state_sync(edev, EXTCON_USB_HOST, false);
		break;
	case STM_IPC_USB_STATE_PERIPHERAL:
		extcon_set_state_sync(edev, EXTCON_USB_HOST, false);
		extcon_set_state_sync(edev, EXTCON_USB, true);
		break;
	case STM_IPC_USB_STATE_HOST:
		extcon_set_state_sync(edev, EXTCON_USB, false);
		extcon_set_state_sync(edev, EXTCON_USB_HOST, true);
		break;
	default:
		/* Fallback to safe disconnected state on protocol error */
		extcon_set_state_sync(edev, EXTCON_USB, false);
		extcon_set_state_sync(edev, EXTCON_USB_HOST, false);
		dev_warn_ratelimited(&edev->dev, "Invalid USB connector state: %u\n", state);
		break;
	}
}

/* Callback used to find and update state of correct child platform device */
static int match_and_update_state(struct device *dev, void *data)
{
	struct stm_connector_priv *priv = dev_get_drvdata(dev);
	u8 *port_info = data;
	u8 port_id = port_info[0];
	u8 state = port_info[1];

	if (priv && priv->port_id == port_id) {
		stm_ipc_update_state(priv->edev, state);
		return 1; /* Found and processed, stop iteration */
	}
	return 0;
}

/* Threaded IRQ Handler (safe to perform sleeping SPI transfers) */
static irqreturn_t stm_ipc_threaded_irq(int irq, void *dev_id)
{
	struct stm_ipc_priv *priv = dev_id;
	struct stm_ipc_packet tx_buf = {0};
	struct stm_ipc_packet rx_buf = {0};
	struct spi_transfer t = {
		.tx_buf = &tx_buf,
		.rx_buf = &rx_buf,
		.len = sizeof(struct stm_ipc_packet),
	};
	struct spi_message m;
	int ret;
	u8 calc_crc;

	mutex_lock(&priv->lock);

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret = spi_sync(priv->spi, &m);
	if (ret < 0) {
		dev_err(&priv->spi->dev, "SPI sync transfer failed: %d\n", ret);
		goto out;
	}

	/* Validate magic byte */
	if (rx_buf.magic != STM_IPC_MSG_MAGIC) {
		dev_warn_ratelimited(&priv->spi->dev, "Invalid magic byte: 0x%02x\n", rx_buf.magic);
		goto out;
	}

	/* Validate packet CRC */
	calc_crc = crc8(stm_crc8_table, (const u8 *)&rx_buf, offsetof(struct stm_ipc_packet, crc), 0);
	if (calc_crc != rx_buf.crc) {
		dev_warn_ratelimited(&priv->spi->dev, "CRC mismatch: read 0x%02x, calculated 0x%02x\n",
				     rx_buf.crc, calc_crc);
		goto out;
	}

	/* Validate packet payload length for USB events */
	if (rx_buf.type == MSG_TYPE_USB_EVENT && rx_buf.length != STM_IPC_MSG_LEN_USB_EVENT) {
		dev_warn_ratelimited(&priv->spi->dev, "Invalid packet length: %u (expected %d)\n",
				     rx_buf.length, STM_IPC_MSG_LEN_USB_EVENT);
		goto out;
	}

	/* Process USB event */
	if (rx_buf.type == MSG_TYPE_USB_EVENT) {
		u8 port_info[2] = { rx_buf.port, rx_buf.state };

		dev_dbg(&priv->spi->dev, "USB Event from STM32 on Port %d: State %d\n", rx_buf.port, rx_buf.state);
		device_for_each_child(&priv->spi->dev, port_info, match_and_update_state);
	}

out:
	mutex_unlock(&priv->lock);
	return IRQ_HANDLED;
}

/* Debugfs Simulation Entry (for mocking events) */
static int stm_ipc_sim_write(void *data, u64 val)
{
	struct stm_connector_priv *priv = data;

	if (val > STM_IPC_USB_STATE_HOST)
		return -EINVAL;

	stm_ipc_update_state(priv->edev, (u8)val);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(stm_ipc_sim_fops, NULL, stm_ipc_sim_write, "%llu\n");

/* Child Connector Platform Device Probe */
static int stm_usb_connector_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stm_connector_priv *priv;
	struct stm_ipc_priv *parent_priv;
	char name[32];
	u32 val;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = of_property_read_u32(dev->of_node, "port-id", &val);
	if (ret) {
		dev_err(dev, "Missing 'port-id' property\n");
		return ret;
	}
	if (val > 255) {
		dev_err(dev, "Invalid 'port-id' value: %u (must be <= 255)\n", val);
		return -EINVAL;
	}
	priv->port_id = (u8)val;

	priv->edev = devm_extcon_dev_allocate(dev, stm_usb_cable);
	if (IS_ERR(priv->edev))
		return PTR_ERR(priv->edev);

	ret = devm_extcon_dev_register(dev, priv->edev);
	if (ret) {
		dev_err(dev, "Failed to register extcon device: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, priv);

	/* Set up DebugFS for this virtual connector for runtime simulation */
	parent_priv = dev_get_drvdata(dev->parent);
	if (parent_priv && parent_priv->debugfs_root) {
		snprintf(name, sizeof(name), "usb%d_sim", priv->port_id + 1);
		debugfs_create_file(name, 0200, parent_priv->debugfs_root, priv, &stm_ipc_sim_fops);
	}

	dev_info(dev, "Registered virtual USB connector on Port %d\n", priv->port_id);
	return 0;
}

static const struct of_device_id stm_usb_connector_of_match[] = {
	{ .compatible = "multitracks,stm32-usb-connector" },
	{ }
};
MODULE_DEVICE_TABLE(of, stm_usb_connector_of_match);

static struct platform_driver stm_usb_connector_driver = {
	.probe = stm_usb_connector_probe,
	.driver = {
		.name = "stm32-usb-connector",
		.of_match_table = stm_usb_connector_of_match,
	},
};

/* Parent SPI Driver Probe */
static int stm_ipc_probe(struct spi_device *spi)
{
	struct stm_ipc_priv *priv;
	int ret;

	priv = devm_kzalloc(&spi->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->spi = spi;
	mutex_init(&priv->lock);
	spi_set_drvdata(spi, priv);



	/* Initialize DebugFS subdirectory under /sys/kernel/debug/stm_ipc/<device> */
	if (stm_ipc_debugfs_dir) {
		priv->debugfs_root = debugfs_create_dir(dev_name(&spi->dev), stm_ipc_debugfs_dir);
		if (IS_ERR_OR_NULL(priv->debugfs_root)) {
			dev_warn(&spi->dev, "Failed to create debugfs root directory\n");
			priv->debugfs_root = NULL;
		}
	} else {
		priv->debugfs_root = NULL;
	}

	/* Populate subnodes as platform devices (this probes the connector sub-drivers) */
	ret = devm_of_platform_populate(&spi->dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to populate child connectors: %d\n", ret);
		goto err_debugfs;
	}

	/* Request Attention Pin Interrupt */
	if (spi->irq > 0) {
		ret = devm_request_threaded_irq(&spi->dev, spi->irq, NULL,
						stm_ipc_threaded_irq,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						DRIVER_NAME, priv);
		if (ret) {
			dev_err(&spi->dev, "Failed to request threaded IRQ: %d\n", ret);
			goto err_debugfs;
		}
	}

	dev_info(&spi->dev, "STM32 SPI IPC Core initialized\n");
	return 0;

err_debugfs:
	debugfs_remove_recursive(priv->debugfs_root);
	return ret;
}

static void stm_ipc_remove(struct spi_device *spi)
{
	struct stm_ipc_priv *priv = spi_get_drvdata(spi);
	debugfs_remove_recursive(priv->debugfs_root);
}

static const struct of_device_id stm_ipc_of_match[] = {
	{ .compatible = "multitracks,stm32-spi-ipc" },
	{ }
};
MODULE_DEVICE_TABLE(of, stm_ipc_of_match);

static struct spi_driver stm_ipc_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = stm_ipc_of_match,
	},
	.probe = stm_ipc_probe,
	.remove = stm_ipc_remove,
};

static int __init stm_ipc_init(void)
{
	int ret;

	/* Setup CRC8 lookup table (polynomial 0x07) once */
	crc8_populate_msb(stm_crc8_table, STM_IPC_CRC8_POLYNOMIAL);

	/* Setup global debugfs parent directory */
	stm_ipc_debugfs_dir = debugfs_create_dir("stm_ipc", NULL);
	if (IS_ERR_OR_NULL(stm_ipc_debugfs_dir))
		stm_ipc_debugfs_dir = NULL;

	ret = spi_register_driver(&stm_ipc_driver);
	if (ret)
		goto err_debugfs;

	ret = platform_driver_register(&stm_usb_connector_driver);
	if (ret) {
		spi_unregister_driver(&stm_ipc_driver);
		goto err_debugfs;
	}

	return 0;

err_debugfs:
	debugfs_remove_recursive(stm_ipc_debugfs_dir);
	return ret;
}
static void __exit stm_ipc_exit(void)
{
	spi_unregister_driver(&stm_ipc_driver);
	platform_driver_unregister(&stm_usb_connector_driver);
	debugfs_remove_recursive(stm_ipc_debugfs_dir);
}

module_init(stm_ipc_init);
module_exit(stm_ipc_exit);

MODULE_AUTHOR("Samuel Morris <scmorris.dev@gmail.com>");
MODULE_DESCRIPTION("STM32 SPI IPC and virtual Extcon Driver");
MODULE_LICENSE("GPL v2");

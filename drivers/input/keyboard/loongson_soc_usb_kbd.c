// SPDX-License-Identifier: GPL-2.0-only
/*
 * Loongson SoC USB HID keyboard driver
 *
 * Drives the non-standard core_usb_host (USB 1.1 Full-Speed Host)
 * implemented in FPGA at 0x1feb0000. The controller exposes an AXI4-Lite
 * register interface; this driver performs minimal USB enumeration and then
 * polls the HID keyboard interrupt IN endpoint, reporting to Linux input.
 *
 * This is intentionally NOT using the Linux USB core (the controller is not
 * OHCI/EHCI compatible).
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/slab.h>

/* Registers (physical 0x1feb0000) */
#define USB_CTRL		0x00
#define USB_STATUS		0x04
#define USB_IRQ_ACK		0x08
#define USB_IRQ_STS		0x0c
#define USB_IRQ_MASK		0x10
#define USB_XFER_DATA		0x14
#define USB_XFER_TOKEN		0x18
#define USB_RX_STAT		0x1c
#define USB_WR_DATA		0x20
#define USB_RD_DATA		0x20

/* USB_CTRL */
#define USB_CTRL_TX_FLUSH	BIT(8)
#define USB_CTRL_DMPULLDOWN	BIT(7)
#define USB_CTRL_DPPULLDOWN	BIT(6)
#define USB_CTRL_TERMSELECT	BIT(5)
#define USB_CTRL_XCVRSELECT	(3 << 3)
#define USB_CTRL_OPMODE		(3 << 1)
#define USB_CTRL_ENABLE_SOF	BIT(0)

/* USB_IRQ_STS / ACK / MASK */
#define USB_IRQ_DEVICE_DETECT	BIT(3)
#define USB_IRQ_ERR		BIT(2)
#define USB_IRQ_DONE		BIT(1)
#define USB_IRQ_SOF		BIT(0)

/* USB_RX_STAT */
#define USB_RX_LEN_MASK		0xffff
#define USB_RX_ERROR		BIT(16)
#define USB_RX_IDLE		BIT(17)

/* USB_XFER_TOKEN */
#define USB_TOKEN_START		BIT(31)
#define USB_TOKEN_IN		BIT(30)
#define USB_TOKEN_ACK		BIT(29)
#define USB_TOKEN_DATA1		BIT(28)
#define USB_TOKEN_PID_SHIFT	16
#define USB_TOKEN_DEV_SHIFT	9
#define USB_TOKEN_EP_SHIFT	5

/* USB PIDs */
#define USB_PID_SETUP		0x2d
#define USB_PID_OUT		0xe1
#define USB_PID_IN		0x69

#define USB_POLL_MS		10
#define USB_XFER_TIMEOUT_MS	100
#define USB_MAX_PACKET		64
#define USB_HID_KEYBOARD_REPORT	8

struct loongson_soc_usb_kbd {
	void __iomem *base;
	struct input_dev *input;
	struct timer_list timer;
	unsigned long poll_jiffies;
	u8 last_keys[6];
	u8 last_mods;
	u8 dev_addr;
	u8 ep_in;
	bool ep_toggle;
	bool present;
};

/* USB HID usage -> Linux keycode (subset for keyboard) */
static const unsigned short usb_kbd_keymap[256] = {
	[0x04] = KEY_A, [0x05] = KEY_B, [0x06] = KEY_C, [0x07] = KEY_D,
	[0x08] = KEY_E, [0x09] = KEY_F, [0x0a] = KEY_G, [0x0b] = KEY_H,
	[0x0c] = KEY_I, [0x0d] = KEY_J, [0x0e] = KEY_K, [0x0f] = KEY_L,
	[0x10] = KEY_M, [0x11] = KEY_N, [0x12] = KEY_O, [0x13] = KEY_P,
	[0x14] = KEY_Q, [0x15] = KEY_R, [0x16] = KEY_S, [0x17] = KEY_T,
	[0x18] = KEY_U, [0x19] = KEY_V, [0x1a] = KEY_W, [0x1b] = KEY_X,
	[0x1c] = KEY_Y, [0x1d] = KEY_Z,
	[0x1e] = KEY_1, [0x1f] = KEY_2, [0x20] = KEY_3, [0x21] = KEY_4,
	[0x22] = KEY_5, [0x23] = KEY_6, [0x24] = KEY_7, [0x25] = KEY_8,
	[0x26] = KEY_9, [0x27] = KEY_0,
	[0x28] = KEY_ENTER, [0x29] = KEY_ESC, [0x2a] = KEY_BACKSPACE,
	[0x2b] = KEY_TAB, [0x2c] = KEY_SPACE, [0x2d] = KEY_MINUS,
	[0x2e] = KEY_EQUAL, [0x2f] = KEY_LEFTBRACE, [0x30] = KEY_RIGHTBRACE,
	[0x31] = KEY_BACKSLASH, [0x33] = KEY_SEMICOLON, [0x34] = KEY_APOSTROPHE,
	[0x35] = KEY_GRAVE, [0x36] = KEY_COMMA, [0x37] = KEY_DOT,
	[0x38] = KEY_SLASH, [0x39] = KEY_CAPSLOCK,
	[0x4f] = KEY_RIGHT, [0x50] = KEY_LEFT, [0x51] = KEY_DOWN, [0x52] = KEY_UP,
};

static inline u32 usb_read(struct loongson_soc_usb_kbd *kbd, u32 reg)
{
	return readl(kbd->base + reg);
}

static inline void usb_write(struct loongson_soc_usb_kbd *kbd, u32 reg, u32 val)
{
	writel(val, kbd->base + reg);
}

static void usb_irq_clear(struct loongson_soc_usb_kbd *kbd, u32 mask)
{
	usb_write(kbd, USB_IRQ_ACK, mask);
}

/* Wait for transfer DONE / error */
static int usb_wait_done(struct loongson_soc_usb_kbd *kbd)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(USB_XFER_TIMEOUT_MS);

	while (time_before(jiffies, timeout)) {
		u32 sts = usb_read(kbd, USB_IRQ_STS);

		if (sts & USB_IRQ_DONE) {
			usb_irq_clear(kbd, USB_IRQ_DONE | USB_IRQ_ERR);
			return 0;
		}
		if (sts & USB_IRQ_ERR) {
			usb_irq_clear(kbd, USB_IRQ_DONE | USB_IRQ_ERR);
			return -EIO;
		}
		usleep_range(200, 500);
	}
	return -ETIMEDOUT;
}

/* Start a single USB transfer */
static int usb_xfer(struct loongson_soc_usb_kbd *kbd, u8 pid, u8 dev, u8 ep,
		    bool in, bool data1, bool ack, u16 len)
{
	u32 token;

	usb_write(kbd, USB_XFER_DATA, len);
	token = USB_TOKEN_START | (pid << USB_TOKEN_PID_SHIFT) |
		(dev << USB_TOKEN_DEV_SHIFT) | (ep << USB_TOKEN_EP_SHIFT);
	if (in)
		token |= USB_TOKEN_IN;
	if (data1)
		token |= USB_TOKEN_DATA1;
	if (ack)
		token |= USB_TOKEN_ACK;
	usb_write(kbd, USB_XFER_TOKEN, token);

	return usb_wait_done(kbd);
}

/* Write TX FIFO (assume 32-bit words, little-endian) */
static void usb_write_fifo(struct loongson_soc_usb_kbd *kbd, const u8 *data, int len)
{
	int i;

	for (i = 0; i < len; i += 4) {
		u32 w = 0;
		int j;

		for (j = 0; j < 4 && (i + j) < len; j++)
			w |= (u32)data[i + j] << (8 * j);
		usb_write(kbd, USB_WR_DATA, w);
	}
}

/* Read RX FIFO */
static void usb_read_fifo(struct loongson_soc_usb_kbd *kbd, u8 *data, int len)
{
	int i;

	for (i = 0; i < len; i += 4) {
		u32 w = usb_read(kbd, USB_RD_DATA);
		int j;

		for (j = 0; j < 4 && (i + j) < len; j++)
			data[i + j] = (w >> (8 * j)) & 0xff;
	}
}

/* Control transfer: SETUP + optional DATA + STATUS */
static int usb_ctrl_transfer(struct loongson_soc_usb_kbd *kbd, u8 dev,
			     const u8 *setup, int setup_len,
			     bool in, u8 *data, int data_len, bool status_in)
{
	int ret;

	/* SETUP stage (DATA0) */
	usb_write_fifo(kbd, setup, setup_len);
	ret = usb_xfer(kbd, USB_PID_SETUP, dev, 0, false, false, false, setup_len);
	if (ret)
		return ret;

	/* DATA stage */
	if (data_len > 0) {
		if (in) {
			ret = usb_xfer(kbd, USB_PID_IN, dev, 0, true, true, true, data_len);
			if (ret)
				return ret;
			usb_read_fifo(kbd, data, data_len);
		} else {
			usb_write_fifo(kbd, data, data_len);
			ret = usb_xfer(kbd, USB_PID_OUT, dev, 0, false, true, false, data_len);
			if (ret)
				return ret;
		}
	}

	/* STATUS stage (opposite direction, zero length) */
	if (status_in)
		ret = usb_xfer(kbd, USB_PID_IN, dev, 0, true, true, true, 0);
	else
		ret = usb_xfer(kbd, USB_PID_OUT, dev, 0, false, true, false, 0);

	return ret;
}

static int usb_get_descriptor(struct loongson_soc_usb_kbd *kbd, u8 dev,
			      u8 type, u8 index, u8 *buf, int len)
{
	u8 setup[8] = { 0x80, 0x06, index, type, 0x00, 0x00, len >> 8, len & 0xff };

	return usb_ctrl_transfer(kbd, dev, setup, 8, true, buf, len, false);
}

static int usb_set_address(struct loongson_soc_usb_kbd *kbd, u8 addr)
{
	u8 setup[8] = { 0x00, 0x05, addr, 0x00, 0x00, 0x00, 0x00, 0x00 };

	return usb_ctrl_transfer(kbd, 0, setup, 8, false, NULL, 0, true);
}

static int usb_set_configuration(struct loongson_soc_usb_kbd *kbd, u8 dev, u8 cfg)
{
	u8 setup[8] = { 0x00, 0x09, cfg, 0x00, 0x00, 0x00, 0x00, 0x00 };

	return usb_ctrl_transfer(kbd, dev, setup, 8, false, NULL, 0, true);
}

/* Parse configuration descriptor to find keyboard IN endpoint */
static int usb_parse_config(struct loongson_soc_usb_kbd *kbd, u8 *buf, int len)
{
	int i = 0;

	while (i + 1 < len) {
		u8 dlen = buf[i];
		u8 dtype = buf[i + 1];

		if (dlen == 0 || i + dlen > len)
			break;

		if (dtype == 0x04 && i + 7 <= len) {
			/* Interface descriptor: class 3 = HID */
			if (buf[i + 5] == 3) {
				/* look for endpoint after interface */
				int j = i + dlen;

				while (j + 6 <= len && buf[j + 1] == 0x05) {
					u8 ep = buf[j + 2];
					u8 attr = buf[j + 3];

					if ((attr & 0x03) == 0x02 && (ep & 0x80)) {
						kbd->ep_in = ep & 0x0f;
						return 0;
					}
					j += buf[j];
				}
			}
		}
		i += dlen;
	}
	return -ENODEV;
}

static int usb_enumerate(struct loongson_soc_usb_kbd *kbd)
{
	u8 buf[64];
	u8 cfg[256];
	int ret, len;

	/* Give the device time after bus reset / power up */
	msleep(100);

	/* Get first 8 bytes of device descriptor */
	ret = usb_get_descriptor(kbd, 0, 1, 0, buf, 8);
	if (ret)
		return ret;

	/* Assign address 1 */
	ret = usb_set_address(kbd, 1);
	if (ret)
		return ret;
	kbd->dev_addr = 1;
	usleep_range(10000, 20000);

	/* Full device descriptor */
	ret = usb_get_descriptor(kbd, 1, 1, 0, buf, 18);
	if (ret)
		return ret;

	/* Get config descriptor header (9 bytes) */
	ret = usb_get_descriptor(kbd, 1, 2, 0, buf, 9);
	if (ret)
		return ret;
	len = buf[2] | (buf[3] << 8);
	if (len <= 0 || len > sizeof(cfg))
		return -EINVAL;

	ret = usb_get_descriptor(kbd, 1, 2, 0, cfg, len);
	if (ret)
		return ret;

	ret = usb_parse_config(kbd, cfg, len);
	if (ret)
		return ret;

	ret = usb_set_configuration(kbd, 1, cfg[5]);
	if (ret)
		return ret;

	kbd->present = true;
	dev_info(&kbd->input->dev, "USB keyboard enumerated, EP IN 0x%02x\n",
		 kbd->ep_in);
	return 0;
}

static void usb_kbd_report(struct loongson_soc_usb_kbd *kbd, u8 *report)
{
	u8 mods = report[0];
	u8 keys[6];
	int i;

	memcpy(keys, &report[2], 6);

	/* Modifiers */
	for (i = 0; i < 8; i++) {
		unsigned int key = 0;
		u8 mask = BIT(i);
		int pressed = !!(mods & mask);

		switch (mask) {
		case 0x01:
			key = KEY_LEFTCTRL;
			break;
		case 0x02:
			key = KEY_LEFTSHIFT;
			break;
		case 0x04:
			key = KEY_LEFTALT;
			break;
		case 0x08:
			key = KEY_LEFTMETA;
			break;
		case 0x10:
			key = KEY_RIGHTCTRL;
			break;
		case 0x20:
			key = KEY_RIGHTSHIFT;
			break;
		case 0x40:
			key = KEY_RIGHTALT;
			break;
		case 0x80:
			key = KEY_RIGHTMETA;
			break;
		}
		if (key)
			input_event(kbd->input, EV_KEY, key, pressed);
	}

	/* Released keys */
	for (i = 0; i < 6; i++) {
		u8 old = kbd->last_keys[i];
		int j;
		bool still = false;

		if (!old)
			continue;
		for (j = 0; j < 6; j++)
			if (keys[j] == old)
				still = true;
		if (!still && old < ARRAY_SIZE(usb_kbd_keymap))
			input_event(kbd->input, EV_KEY, usb_kbd_keymap[old], 0);
	}

	/* Pressed keys */
	for (i = 0; i < 6; i++) {
		u8 code = keys[i];
		int j;
		bool was = false;

		if (!code || code >= ARRAY_SIZE(usb_kbd_keymap))
			continue;
		for (j = 0; j < 6; j++)
			if (kbd->last_keys[j] == code)
				was = true;
		if (!was)
			input_event(kbd->input, EV_KEY, usb_kbd_keymap[code], 1);
	}

	memcpy(kbd->last_keys, keys, 6);
	kbd->last_mods = mods;
	input_sync(kbd->input);
}

static void usb_kbd_poll(struct timer_list *t)
{
	struct loongson_soc_usb_kbd *kbd = timer_container_of(kbd, t, timer);
	u8 report[USB_HID_KEYBOARD_REPORT];
	int ret;

	if (!kbd->present)
		goto out;

	ret = usb_xfer(kbd, USB_PID_IN, kbd->dev_addr, kbd->ep_in,
		       true, kbd->ep_toggle, true, USB_HID_KEYBOARD_REPORT);
	if (ret == 0) {
		usb_read_fifo(kbd, report, USB_HID_KEYBOARD_REPORT);
		usb_kbd_report(kbd, report);
		kbd->ep_toggle = !kbd->ep_toggle;
	}

out:
	mod_timer(&kbd->timer, jiffies + kbd->poll_jiffies);
}

static int loongson_soc_usb_kbd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct loongson_soc_usb_kbd *kbd;
	struct resource *res;
	int error, i;

	kbd = devm_kzalloc(dev, sizeof(*kbd), GFP_KERNEL);
	if (!kbd)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no register resource\n");
		return -ENODEV;
	}

	kbd->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(kbd->base))
		return PTR_ERR(kbd->base);

	kbd->input = devm_input_allocate_device(dev);
	if (!kbd->input)
		return -ENOMEM;

	kbd->input->name = "Loongson SoC USB Keyboard";
	kbd->input->phys = "loongson-soc-usb-kbd/input0";
	kbd->input->id.bustype = BUS_HOST;
	kbd->input->dev.parent = dev;

	__set_bit(EV_KEY, kbd->input->evbit);
	__set_bit(EV_REP, kbd->input->evbit);
	for (i = 0; i < ARRAY_SIZE(usb_kbd_keymap); i++)
		if (usb_kbd_keymap[i])
			__set_bit(usb_kbd_keymap[i], kbd->input->keybit);

	error = input_register_device(kbd->input);
	if (error) {
		dev_err(dev, "failed to register input device: %d\n", error);
		return error;
	}

	/* Init PHY: FS transceiver, FS termination, no pulldowns, enable SOF */
	usb_write(kbd, USB_CTRL,
		  (1 << 1) |		/* OPMODE = 01 (non-driving) */
		  (1 << 3) |		/* XCVRSELECT = 01 (FS) */
		  USB_CTRL_TERMSELECT |
		  USB_CTRL_ENABLE_SOF);
	usb_write(kbd, USB_IRQ_MASK, USB_IRQ_DONE | USB_IRQ_ERR);
	usb_irq_clear(kbd, 0x0f);

	/* Try to enumerate a keyboard */
	error = usb_enumerate(kbd);
	if (error)
		dev_warn(dev, "USB keyboard enumeration failed: %d\n", error);

	kbd->poll_jiffies = msecs_to_jiffies(USB_POLL_MS);
	timer_setup(&kbd->timer, usb_kbd_poll, 0);
	mod_timer(&kbd->timer, jiffies + kbd->poll_jiffies);

	platform_set_drvdata(pdev, kbd);
	dev_info(dev, "Loongson SoC USB keyboard registered\n");
	return 0;
}

static void loongson_soc_usb_kbd_remove(struct platform_device *pdev)
{
	struct loongson_soc_usb_kbd *kbd = platform_get_drvdata(pdev);

	timer_delete_sync(&kbd->timer);
}

static const struct of_device_id loongson_soc_usb_kbd_of_match[] = {
	{ .compatible = "loongson-edu,usb-kbd" },
	{ }
};
MODULE_DEVICE_TABLE(of, loongson_soc_usb_kbd_of_match);

static struct platform_driver loongson_soc_usb_kbd_driver = {
	.probe	= loongson_soc_usb_kbd_probe,
	.remove	= loongson_soc_usb_kbd_remove,
	.driver	= {
		.name		= "loongson-soc-usb-kbd",
		.of_match_table	= loongson_soc_usb_kbd_of_match,
	},
};
module_platform_driver(loongson_soc_usb_kbd_driver);

MODULE_AUTHOR("Loongson Education FPGA Lab");
MODULE_DESCRIPTION("Loongson SoC USB HID keyboard driver");
MODULE_LICENSE("GPL v2");

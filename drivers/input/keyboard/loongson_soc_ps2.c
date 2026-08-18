// SPDX-License-Identifier: GPL-2.0-only
/*
 * Loongson SoC PS/2 keyboard driver
 *
 * The FPGA CONFREG block receives PS/2 scancodes and exposes them through
 * two registers:
 *   0x1fd0f040  DATA   read returns one scancode byte and pops the FIFO
 *   0x1fd0f044  STATUS bit0 = FIFO not empty, bit1 = FIFO full,
 *                       bits[7:4] = FIFO count
 */
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/timer.h>

#define PS2_DATA		0x00
#define PS2_STATUS		0x04
#define PS2_STATUS_VALID	BIT(0)
#define PS2_STATUS_FULL		BIT(1)
#define PS2_POLL_MS		5

#define PS2_CODE_EXTENDED	0xe0
#define PS2_CODE_BREAK		0xf0

/* PS/2 Set 2 scancode -> Linux keycode (normal keys) */
static const unsigned short loongson_soc_ps2_keymap[256] = {
	[0x0d] = KEY_TAB,
	[0x11] = KEY_LEFTALT,
	[0x12] = KEY_LEFTSHIFT,
	[0x14] = KEY_LEFTCTRL,
	[0x15] = KEY_Q,
	[0x16] = KEY_1,
	[0x1a] = KEY_Z,
	[0x1b] = KEY_S,
	[0x1c] = KEY_A,
	[0x1d] = KEY_W,
	[0x1e] = KEY_2,
	[0x21] = KEY_C,
	[0x22] = KEY_X,
	[0x23] = KEY_D,
	[0x24] = KEY_E,
	[0x25] = KEY_4,
	[0x26] = KEY_3,
	[0x29] = KEY_SPACE,
	[0x2a] = KEY_V,
	[0x2b] = KEY_F,
	[0x2c] = KEY_T,
	[0x2d] = KEY_R,
	[0x2e] = KEY_5,
	[0x31] = KEY_N,
	[0x32] = KEY_B,
	[0x33] = KEY_H,
	[0x34] = KEY_G,
	[0x35] = KEY_Y,
	[0x36] = KEY_6,
	[0x3a] = KEY_M,
	[0x3b] = KEY_J,
	[0x3c] = KEY_U,
	[0x3d] = KEY_7,
	[0x3e] = KEY_8,
	[0x41] = KEY_COMMA,
	[0x42] = KEY_K,
	[0x43] = KEY_I,
	[0x44] = KEY_O,
	[0x45] = KEY_0,
	[0x46] = KEY_9,
	[0x49] = KEY_DOT,
	[0x4a] = KEY_SLASH,
	[0x4b] = KEY_L,
	[0x4c] = KEY_SEMICOLON,
	[0x4d] = KEY_P,
	[0x4e] = KEY_MINUS,
	[0x52] = KEY_APOSTROPHE,
	[0x54] = KEY_LEFTBRACE,
	[0x55] = KEY_EQUAL,
	[0x58] = KEY_CAPSLOCK,
	[0x59] = KEY_RIGHTSHIFT,
	[0x5a] = KEY_ENTER,
	[0x5b] = KEY_RIGHTBRACE,
	[0x5d] = KEY_BACKSLASH,
	[0x66] = KEY_BACKSPACE,
	[0x76] = KEY_ESC,
};

/* PS/2 Set 2 scancode -> Linux keycode (extended E0 keys) */
static const unsigned short loongson_soc_ps2_keymap_ext[256] = {
	[0x11] = KEY_RIGHTALT,
	[0x14] = KEY_RIGHTCTRL,
	[0x69] = KEY_END,
	[0x6b] = KEY_LEFT,
	[0x6c] = KEY_HOME,
	[0x70] = KEY_INSERT,
	[0x71] = KEY_DELETE,
	[0x72] = KEY_DOWN,
	[0x74] = KEY_RIGHT,
	[0x75] = KEY_UP,
};

struct loongson_soc_ps2 {
	void __iomem *base;
	struct input_dev *input;
	struct timer_list timer;
	unsigned long poll_jiffies;
	bool extended;
	bool release;
};

static void loongson_soc_ps2_handle_byte(struct loongson_soc_ps2 *ps2,
					 u8 code, bool *changed)
{
	unsigned short keycode;

	if (code == PS2_CODE_EXTENDED) {
		ps2->extended = true;
		return;
	}

	if (code == PS2_CODE_BREAK) {
		ps2->release = true;
		return;
	}

	/* Ignore keyboard self-test/acknowledge bytes. */
	if (code == 0xaa || code == 0xfa || code == 0xee) {
		ps2->extended = false;
		ps2->release = false;
		return;
	}

	if (ps2->extended)
		keycode = loongson_soc_ps2_keymap_ext[code];
	else
		keycode = loongson_soc_ps2_keymap[code];

	if (keycode) {
		input_event(ps2->input, EV_KEY, keycode,
			    ps2->release ? 0 : 1);
		*changed = true;
	}

	ps2->extended = false;
	ps2->release = false;
}

static void loongson_soc_ps2_poll(struct timer_list *t)
{
	struct loongson_soc_ps2 *ps2 = timer_container_of(ps2, t, timer);
	u32 status;
	bool changed = false;

	status = readl(ps2->base + PS2_STATUS);
	while (status & PS2_STATUS_VALID) {
		u8 code = readl(ps2->base + PS2_DATA) & 0xff;

		loongson_soc_ps2_handle_byte(ps2, code, &changed);
		status = readl(ps2->base + PS2_STATUS);
	}

	if (changed)
		input_sync(ps2->input);

	mod_timer(&ps2->timer, jiffies + ps2->poll_jiffies);
}

static int loongson_soc_ps2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct loongson_soc_ps2 *ps2;
	struct resource *res;
	int i;
	int error;

	ps2 = devm_kzalloc(dev, sizeof(*ps2), GFP_KERNEL);
	if (!ps2)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no register resource\n");
		return -ENODEV;
	}

	ps2->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ps2->base))
		return PTR_ERR(ps2->base);

	ps2->input = input_allocate_device();
	if (!ps2->input)
		return -ENOMEM;

	ps2->input->name = "Loongson SoC PS/2 Keyboard";
	ps2->input->phys = "loongson-soc-ps2/input0";
	ps2->input->id.bustype = BUS_HOST;
	ps2->input->dev.parent = dev;

	__set_bit(EV_KEY, ps2->input->evbit);
	__set_bit(EV_REP, ps2->input->evbit);

	for (i = 0; i < ARRAY_SIZE(loongson_soc_ps2_keymap); i++) {
		if (loongson_soc_ps2_keymap[i])
			__set_bit(loongson_soc_ps2_keymap[i],
				  ps2->input->keybit);
		if (loongson_soc_ps2_keymap_ext[i])
			__set_bit(loongson_soc_ps2_keymap_ext[i],
				  ps2->input->keybit);
	}

	error = input_register_device(ps2->input);
	if (error) {
		dev_err(dev, "failed to register input device: %d\n", error);
		input_free_device(ps2->input);
		return error;
	}

	ps2->poll_jiffies = msecs_to_jiffies(PS2_POLL_MS);
	timer_setup(&ps2->timer, loongson_soc_ps2_poll, 0);
	mod_timer(&ps2->timer, jiffies + ps2->poll_jiffies);

	platform_set_drvdata(pdev, ps2);

	dev_info(dev, "Loongson SoC PS/2 keyboard registered\n");
	return 0;
}

static void loongson_soc_ps2_remove(struct platform_device *pdev)
{
	struct loongson_soc_ps2 *ps2 = platform_get_drvdata(pdev);

	timer_delete_sync(&ps2->timer);
	input_unregister_device(ps2->input);
}

static const struct of_device_id loongson_soc_ps2_of_match[] = {
	{ .compatible = "loongson-edu,ps2-keyboard" },
	{ }
};
MODULE_DEVICE_TABLE(of, loongson_soc_ps2_of_match);

static struct platform_driver loongson_soc_ps2_driver = {
	.probe		= loongson_soc_ps2_probe,
	.remove		= loongson_soc_ps2_remove,
	.driver		= {
		.name		= "loongson-soc-ps2",
		.of_match_table	= loongson_soc_ps2_of_match,
	},
};
module_platform_driver(loongson_soc_ps2_driver);

MODULE_AUTHOR("Loongson Education FPGA Lab");
MODULE_DESCRIPTION("Loongson SoC PS/2 keyboard driver");
MODULE_LICENSE("GPL v2");

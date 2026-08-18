// SPDX-License-Identifier: GPL-2.0-only
/*
 * Loongson SoC 4x4 matrix keypad driver
 *
 * The FPGA CONFREG block scans the 4x4 matrix and exposes a 16-bit
 * pressed-key bitmask at 0x1fd0f024. Bit index = row * 4 + col.
 */
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/timer.h>

#define KEYPAD_ROWS		4
#define KEYPAD_COLS		4
#define KEYPAD_KEYS		16
#define KEYPAD_POLL_MS		20

struct loongson_soc_matrix_keypad {
	void __iomem *base;
	struct input_dev *input;
	struct timer_list timer;
	unsigned long poll_jiffies;
	u16 last_state;
	unsigned short *keymap;
};

static void loongson_soc_matrix_keypad_poll(struct timer_list *t)
{
	struct loongson_soc_matrix_keypad *keypad = timer_container_of(keypad, t, timer);
	u32 val = readl(keypad->base) & 0xffff;
	u16 state = val;
	u16 changed = state ^ keypad->last_state;
	int i;

	for (i = 0; i < KEYPAD_KEYS; i++) {
		if (!(changed & BIT(i)))
			continue;

		input_event(keypad->input, EV_KEY, keypad->keymap[i],
			    !!(state & BIT(i)));
	}

	if (changed)
		input_sync(keypad->input);

	keypad->last_state = state;
	mod_timer(&keypad->timer, jiffies + keypad->poll_jiffies);
}

static int loongson_soc_matrix_keypad_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct loongson_soc_matrix_keypad *keypad;
	struct resource *res;
	int error;

	keypad = devm_kzalloc(dev, sizeof(*keypad), GFP_KERNEL);
	if (!keypad)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no register resource\n");
		return -ENODEV;
	}

	keypad->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(keypad->base))
		return PTR_ERR(keypad->base);

	keypad->input = input_allocate_device();
	if (!keypad->input)
		return -ENOMEM;

	keypad->input->name = "Loongson SoC Matrix Keypad";
	keypad->input->phys = "loongson-soc-matrix-keypad/input0";
	keypad->input->id.bustype = BUS_HOST;
	keypad->input->dev.parent = dev;

	error = matrix_keypad_build_keymap(NULL, NULL, KEYPAD_ROWS, KEYPAD_COLS,
					   NULL, keypad->input);
	if (error) {
		dev_err(dev, "failed to build keymap: %d\n", error);
		input_free_device(keypad->input);
		return error;
	}
	keypad->keymap = keypad->input->keycode;

	__set_bit(EV_REP, keypad->input->evbit);

	error = input_register_device(keypad->input);
	if (error) {
		dev_err(dev, "failed to register input device: %d\n", error);
		input_free_device(keypad->input);
		return error;
	}

	keypad->last_state = readl(keypad->base) & 0xffff;
	keypad->poll_jiffies = msecs_to_jiffies(KEYPAD_POLL_MS);
	timer_setup(&keypad->timer, loongson_soc_matrix_keypad_poll, 0);
	mod_timer(&keypad->timer, jiffies + keypad->poll_jiffies);

	platform_set_drvdata(pdev, keypad);

	dev_info(dev, "Loongson SoC matrix keypad registered\n");
	return 0;
}

static void loongson_soc_matrix_keypad_remove(struct platform_device *pdev)
{
	struct loongson_soc_matrix_keypad *keypad = platform_get_drvdata(pdev);

	timer_delete_sync(&keypad->timer);
	input_unregister_device(keypad->input);
}

static const struct of_device_id loongson_soc_matrix_keypad_of_match[] = {
	{ .compatible = "loongson-edu,matrix-keypad" },
	{ }
};
MODULE_DEVICE_TABLE(of, loongson_soc_matrix_keypad_of_match);

static struct platform_driver loongson_soc_matrix_keypad_driver = {
	.probe		= loongson_soc_matrix_keypad_probe,
	.remove		= loongson_soc_matrix_keypad_remove,
	.driver		= {
		.name		= "loongson-soc-matrix-keypad",
		.of_match_table	= loongson_soc_matrix_keypad_of_match,
	},
};
module_platform_driver(loongson_soc_matrix_keypad_driver);

MODULE_AUTHOR("Loongson Education FPGA Lab");
MODULE_DESCRIPTION("Loongson SoC 4x4 matrix keypad driver");
MODULE_LICENSE("GPL v2");

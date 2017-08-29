/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>

#include <device.h>
#include <watchdog.h>
#include "board.h"
#include <misc/printk.h>

static const char *get_reason(struct device *dev)
{
	switch (wdt_get_reason(dev)) {
	case WDT_REASON_UNKNOWN:
		return "unknown";
	case WDT_REASON_CPU_RESET:
		return "CPU reset";
	case WDT_REASON_SYS_RESET:
		return "system reset";
	case WDT_REASON_BROWN_OUT:
		return "brown out";
	}

	return "?";
}

/* WDT Requires a callback, there is no interrupt enable / disable. */
void wdt_example_cb(struct device *dev)
{
	printk("Watchdog timer interrupt called. Reason: %s\n",
	       get_reason(dev));

	wdt_reload(dev);
}

void main(void)
{
	struct wdt_config wr_cfg;
	struct wdt_config cfg;
	struct device *wdt_dev;

	printk("Start watchdog test\n");
	wr_cfg.timeout = WDT_2_27_CYCLES;
	wr_cfg.mode = WDT_MODE_INTERRUPT_RESET;
	wr_cfg.interrupt_fn = wdt_example_cb;

	wdt_dev = device_get_binding("WATCHDOG_0");

	wdt_enable(wdt_dev);
	wdt_set_config(wdt_dev, &wr_cfg);

	wdt_get_config(wdt_dev, &cfg);
	printk("timeout: %d\n", cfg.timeout);
	printk("mode: %d\n", cfg.mode);

	printk("System boot reason: %s\n", get_reason(wdt_dev));
}

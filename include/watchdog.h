/*
 * Copyright (c) 2015 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _WDT_H_
#define _WDT_H_

#include <zephyr/types.h>
#include <device.h>
#include <misc/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WDT_MODE		(BIT(1))
#define WDT_MODE_OFFSET         (1)
#define WDT_TIMEOUT_MASK        (0xF)

enum wdt_mode {
	WDT_MODE_RESET = 0,
	WDT_MODE_INTERRUPT_RESET
};

/**
 * WDT clock cycles for timeout type.
 */
enum wdt_clock_timeout_cycles {
	WDT_2_16_CYCLES,
	WDT_2_17_CYCLES,
	WDT_2_18_CYCLES,
	WDT_2_19_CYCLES,
	WDT_2_20_CYCLES,
	WDT_2_21_CYCLES,
	WDT_2_22_CYCLES,
	WDT_2_23_CYCLES,
	WDT_2_24_CYCLES,
	WDT_2_25_CYCLES,
	WDT_2_26_CYCLES,
	WDT_2_27_CYCLES,
	WDT_2_28_CYCLES,
	WDT_2_29_CYCLES,
	WDT_2_30_CYCLES,
	WDT_2_31_CYCLES
};

/**
 * WDT reboot reason.
 */
enum wdt_reboot_reason {
	WDT_REASON_UNKNOWN,
	WDT_REASON_CPU_RESET,
	WDT_REASON_SYS_RESET,
	WDT_REASON_BROWN_OUT,
};

/**
 * WDT configuration struct.
 */
struct wdt_config {
	u32_t timeout;
	enum wdt_mode mode;
	void (*interrupt_fn)(struct device *dev);
};

typedef void (*wdt_api_enable)(struct device *dev);
typedef void (*wdt_api_disable)(struct device *dev);
typedef int (*wdt_api_set_config)(struct device *dev,
				  struct wdt_config *config);
typedef void (*wdt_api_get_config)(struct device *dev,
				   struct wdt_config *config);
typedef void (*wdt_api_reload)(struct device *dev);
typedef enum wdt_reboot_reason (*wdt_api_get_reason)(struct device *dev);
typedef int (*wdt_api_access_retained)(struct device *dev,
					u32_t *value,
					bool read);

struct wdt_driver_api {
	wdt_api_enable enable;
	wdt_api_disable disable;
	wdt_api_get_config get_config;
	wdt_api_set_config set_config;
	wdt_api_reload reload;
	wdt_api_get_reason get_reason;
	wdt_api_access_retained access_retained;
};

static inline void wdt_enable(struct device *dev)
{
	const struct wdt_driver_api *api = dev->driver_api;

	api->enable(dev);
}

static inline void wdt_disable(struct device *dev)
{
	const struct wdt_driver_api *api = dev->driver_api;

	api->disable(dev);
}

static inline void wdt_get_config(struct device *dev,
				  struct wdt_config *config)
{
	const struct wdt_driver_api *api = dev->driver_api;

	api->get_config(dev, config);
}

static inline int wdt_set_config(struct device *dev,
				 struct wdt_config *config)
{
	const struct wdt_driver_api *api = dev->driver_api;

	return api->set_config(dev, config);
}

static inline void wdt_reload(struct device *dev)
{
	const struct wdt_driver_api *api = dev->driver_api;

	api->reload(dev);
}

static inline enum wdt_reboot_reason wdt_get_reason(struct device *dev)
{
	const struct wdt_driver_api *api = dev->driver_api;

	return api->get_reason(dev);
}

static int wdt_get_retained(struct device *dev, u32_t *output)
{
	const struct wdt_driver_api *api = dev->driver_api;

	return api->access_retained(dev, output, true);
}

static int wdt_set_retained(struct device *dev, u32_t value)
{
	const struct wdt_driver_api *api = dev->driver_api;

	return api->access_retained(dev, &value, false);
}

#ifdef __cplusplus
}
#endif

#endif

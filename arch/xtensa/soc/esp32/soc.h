/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SOC_H__
#define __SOC_H__

#include <rom/ets_sys.h>

#include <zephyr/types.h>
#include <stdbool.h>

/* Instructions must be in IRAM while flash cache isn't enabled */
#define IRAM_SECTION __attribute__((section(".iram1")))

extern int esp32_rom_intr_matrix_set(int cpu_no,
				     int interrupt_src,
				     int interrupt_line);

extern int esp32_rom_gpio_matrix_in(u32_t gpio, u32_t signal_index,
				    bool inverted);
extern int esp32_rom_gpio_matrix_out(u32_t gpio, u32_t signal_index,
				     bool out_inverted,
				     bool out_enabled_inverted);

extern void esp32_rom_uart_attach();
extern STATUS esp32_rom_uart_tx_one_char(u8_t chr);
extern STATUS esp32_rom_uart_rx_one_char(u8_t *chr);

extern void esp32_rom_mmu_init(int cpu_no);
extern void esp32_rom_cache_flush(int cpu_no);
extern void esp32_rom_cache_read_enable(int cpu_no);
extern void esp32_rom_cache_read_disable(int cpu_no);
extern int esp32_rom_cache_flash_mmu_set(int cpu_no,
					 int pid,
					 unsigned int virt_addr,
					 unsigned int phys_addr,
					 int page_size,
					 int num_pages);

#endif /* __SOC_H__ */

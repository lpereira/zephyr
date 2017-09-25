/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Include esp-idf headers first to avoid redefining BIT() macro */
#include <soc/dport_reg.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/timer_group_reg.h>
#include <xtensa/config/core-isa.h>
#include <xtensa/corebits.h>

#include <kernel.h>
#include <misc/util.h>
#include <soc.h>
#include <string.h>
#include <toolchain/gcc.h>
#include <zephyr/types.h>

extern char _init_start[];
extern char _bss_start[];
extern char _bss_end[];

extern void _Cstart(void);

#define BOOT_SECTION __attribute__((section(".bootloader")))

static void BOOT_SECTION reset_mmu(void)
{
	volatile u32_t *reg;

	/* At this stage, the flash cache should be disabled.  But, just
	 * to be sure, disable it before changing the MMU bits.
	 */
	esp32_rom_cache_read_disable(0);
	esp32_rom_cache_read_disable(1);

	esp32_rom_cache_flush(0);
	esp32_rom_cache_flush(1);

	/* According to esp-idf, toggling the DPORT_APP_CACHE_MMU_IA_CLR bit
	 * is necessary as a workaround to a hardware bug.  No additional
	 * details are given as to why this is necessary: the flash cache
	 * isn't pubicly documented as of September 2017.
	 */
	esp32_rom_mmu_init(0);
	reg = (u32_t *)DPORT_APP_CACHE_CTRL1_REG;
	*reg |= DPORT_APP_CACHE_MMU_IA_CLR;
	esp32_rom_mmu_init(1);
	*reg &= ~DPORT_APP_CACHE_MMU_IA_CLR;
}

static inline int BOOT_SECTION calc_pages(void *start, void *end)
{
	uintptr_t size = (uintptr_t)start - (uintptr_t)end;

	return (int)ROUND_UP(size + 0xFFFF, 0x10000);
}

static inline u32_t BOOT_SECTION mask_addr(void *addr)
{
	return (u32_t)((uintptr_t)addr & 0xffff0000);
}

static inline u32_t BOOT_SECTION set_up_cache_region(void *start_addr,
						     void *end_addr)
{
	const int pages = calc_pages(start_addr, end_addr);
	const u32_t addr = mask_addr(start_addr);

	return esp32_rom_cache_flash_mmu_set(0, 0, addr, addr, 64, pages);
}

static void BOOT_SECTION setup_mmu(void)
{
	extern char _rodata_start[], _rodata_end[];
	extern char _irom_text_start[], _irom_text_end[];
	volatile u32_t *flash_mmu_tbl = DPORT_PRO_FLASH_MMU_TABLE;
	int i;

	for (i = 0; i < DPORT_FLASH_MMU_TABLE_SIZE; i++) {
		flash_mmu_tbl[i] = DPORT_FLASH_MMU_TABLE_INVALID_VAL;
	}

	/* FIXME: should the return value of these be ignored?  esp-idf does. */
	set_up_cache_region(_rodata_start, _rodata_end);
	set_up_cache_region(_irom_text_start, _irom_text_end);
}

static void BOOT_SECTION enable_flash_cache_cpu0(void)
{
	volatile u32_t *reg;

	reset_mmu();

	/* Unmask the DROM0 on PRO CPU, and ensure the one for APP CPU is
	 * kept masked while SMP support isn't in place.
	 */
	reg = (u32_t *)DPORT_PRO_CACHE_CTRL1_REG;
	*reg &= ~DPORT_PRO_CACHE_MASK_DROM0;

	reg = (u32_t *)DPORT_APP_CACHE_CTRL1_REG;
	*reg |= DPORT_APP_CACHE_MASK_DROM0;

	setup_mmu();

	/* Unmask flash cache bits only for PRO CPU at the moment. */
	reg = (u32_t *)DPORT_PRO_CACHE_CTRL1_REG;
	*reg &= ~(DPORT_PRO_CACHE_MASK_IRAM0 |
		DPORT_PRO_CACHE_MASK_IRAM1 |
		DPORT_PRO_CACHE_MASK_IROM0 |
		DPORT_PRO_CACHE_MASK_DROM0 |
		DPORT_PRO_CACHE_MASK_DRAM1);

	reg = (u32_t *)DPORT_APP_CACHE_CTRL1_REG;
	*reg |= DPORT_APP_CACHE_MASK_IRAM0 |
		DPORT_APP_CACHE_MASK_IRAM1 |
		DPORT_APP_CACHE_MASK_IROM0 |
		DPORT_APP_CACHE_MASK_DROM0 |
		DPORT_APP_CACHE_MASK_DRAM1;

	esp32_rom_cache_read_enable(0);
}

static void BOOT_SECTION copy_kernel_to_iram(void)
{
	
}

void BOOT_SECTION __some_boot_func(void)
{
	volatile u32_t *wdt_rtc_reg = (u32_t *)RTC_CNTL_WDTCONFIG0_REG;
	volatile u32_t *wdt_timg_reg = (u32_t *)TIMG_WDTCONFIG0_REG(0);
	volatile u32_t *app_cpu_config_reg = (u32_t *)DPORT_APPCPU_CTRL_B_REG;
	void *(*esp32_rom_memset)(void *s, int c, size_t n) = (void *)0x4000c44c;

	/* Use memset() from ROM as we don't have ours this early. */
	esp32_rom_memset(_bss_start, 0, _bss_end - _bss_start);

	/* Disable CPU1 while we figure out how to have SMP in Zephyr. */
	*app_cpu_config_reg &= ~DPORT_APPCPU_CLKGATE_EN;

	/* The watchdog timer is enabled in the bootloader.  We're done booting,
	 * so disable it.
	 */
	*wdt_rtc_reg &= ~RTC_CNTL_WDT_FLASHBOOT_MOD_EN;
	*wdt_timg_reg &= ~TIMG_WDT_FLASHBOOT_MOD_EN;

	/* Set up flash cache for PRO CPU. */
	enable_flash_cache_cpu0();

	copy_kernel_to_iram();
}

/* We have some things set up from the first stage boot loader inside the ESP32
 * mask ROM.  So write this in C rather than assembly.
 */
void IRAM_SECTION __start(void)
{
	__some_boot_func();
	/* Move the exception vector table to IRAM. */
	__asm__ __volatile__ (
		"wsr %0, vecbase"
		:
		: "r"(_init_start));

	/* Zero out BSS.  Clobber _bss_start to avoid memset() elision. */
	memset(_bss_start, 0, _bss_end - _bss_start);
	__asm__ __volatile__ (
		""
		:
		: "g"(_bss_start)
		: "memory");

	/* Disable normal interrupts. */
	__asm__ __volatile__ (
		"wsr %0, PS"
		:
		: "r"(PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM | PS_WOE));

	/* Start Zephyr */
	_Cstart();

	CODE_UNREACHABLE;
}

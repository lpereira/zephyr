/*
 * Copyright (c) 2017 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <misc/util.h>

#define SPI_ESP32_SPIN_THRESHOLD 10
#define SPI_ESP32_DATA_BUFFER_LEN 64

struct spi_esp32_config {
	int index;
	void (*configure_irq)(struct device *device);
};

struct spi_esp32_peripheral {
	int clk;
	int rst;
};

struct spi_esp32_pins {
	int mosi;
	int miso;
	int clk;
	int cs;

	int clk_idx;
	int miso_idx;
	int mosi_idx;

	int cs_disable_bit;

	/* No need to store the pin mux functions on this table: CS and CLK
	 * lines have the same function for all SPI controllers, and
	 * MOSI/MISO use the GPIO function.
	 */
};

static int spi_esp32_operation_to_mode(u16_t operation)
{
	int cpol = SPI_MODE_GET(operation) & SPI_MODE_CPOL;
	int cpha = SPI_MODE_GET(operation) & SPI_MODE_CPHA;

	if (cpol == 0) {
		if (cpha == 0) {
			return 0;
		}

		return 1;
	}

	if (cpha == 0) {
		return 2;
	}

	return 3;
}

static void spi_esp32_configure_pins(int index)
{
	static const struct spi_esp32_pins pin_tbl[] = {
		[0] = {
			.miso = 12,
			.mosi = 13,
			.clk = 14,
			.cs = 11,
			.clk_idx = SPICLK_OUT_IDX,
			.miso_idx = SPIQ_OUT_IDX,
			.mosi_idx = SPID_IN_IDX,
			.cs_disable_bit = SPI_CS0_DIS_S,
		},
		[1] = {
			.miso = 19,
			.mosi = 23,
			.clk = 18,
			.cs = 15,
			.clk_idx = HSPICLK_OUT_IDX,
			.miso_idx = HSPIQ_OUT_IDX,
			.mosi_idx = HSPID_IN_IDX,
			.cs_disable_bit = SPI_CS1_DIS_S,
		},
		[2] = {
			.miso = 7,
			.mosi = 8,
			.clk = 6,
			.cs = 5,
			.clk_idx = VSPICLK_OUT_IDX,
			.miso_idx = VSPIQ_OUT_IDX,
			.mosi_idx = VSPID_IN_IDX,
			.cs_disable_bit = SPI_CS2_DIS_S,
		},
	};
	const struct spi_esp32_pins *pins = pin_tbl[index];
	struct device *pinmux = device_get_binding(CONFIG_PINMUX_NAME);

	pinmux_pin_input_enable(pins->clk, PINMUX_OUTPUT_ENABLED);
	pinmux_pin_set(pins->clk, PINMUX_FUNC_B);
	esp32_rom_gpio_matrix_out(pins->clk, pins->clk_idx, false, false);

	pinmux_pin_input_enable(pins->mosi, PINMUX_OUTPUT_ENABLED);
	pinmux_pin_set(pins->mosi, PIN_FUNC_GPIO);
	esp32_rom_gpio_matrix_out(pins->mosi, pins->mosi_idx, false, false);

	pinmux_pin_input_enable(pins->miso, PINMUX_INPUT_ENABLED);
	pinmux_pin_set(pins->miso, PIN_FUNC_GPIO);
	esp32_rom_gpio_matrix_in(pins->miso, pins->miso_idx, false);

	pinmux_pin_input_enable(pins->cs, PINMUX_OUTPUT_ENABLED);
	pinmux_pin_set(pins->cs, PINMUX_FUNC_B);
	esp32_rom_gpio_matrix_out(pins->cs, false, false);
	sys_clear_bit(SPI_PIN_REG(index), pins->cs_disable_bit);
}

static void spi_esp32_enable_peripheral(int index)
{
	static const struct spi_esp32_peripheral periph_tbl[] = {
		[0] = { DPORT_SPI_CLK_EN, DPORT_SPI_RST },
		[1] = { DPORT_SPI_CLK_EN_1, DPORT_SPI_RST_1 },
		[2] = { DPORT_SPI_CLK_EN_2, DPORT_SPI_RST_2 },
	};

	sys_set_bit(DPORT_PERIP_CLK_EN_REG, periph_tbl[index].clk);
	sys_clear_bit(DPORT_PERIP_RST_EN_REG, periph_tbl[index].rst);
}

static int spi_esp32_calculate_div_prediv(int frequency, int *div, int *prediv)
{
	/* According to the ESP32 TRM, the SPI frequency is calculated as
	 * follows:
	 *
	 *   freq = f_apb / ((spi_clkcnt_n + 1) * (spi_clkdiv_pre + 1))
	 *
	 * Where:
	 *   - f_apb is a constant value (80MHz)
	 *   - spi_clkcnt_n is the divisor
	 *   - spi_clkcnt_pre is the pre-divisor
	 *
	 * This function tries to approximate the values of the divisor and
	 * pre-divisor so that the frequency is as close as possible to the
	 * desired value.  It should be exact if frequency is a multiple of
	 * f_apb; otherwise, it'll try to approximate as much as possible.
	 *
	 * The search isn't exhaustive.  For instance, a better
	 * approximation could be made by iterating over possible all
	 * possible pre-divisor values instead of only the divisor values,
	 * as it's a larger range, but this would be more expensive.
	 *
	 * It will return -EINVAL if it couldn't find a pair of values that
	 * fits.
	 */
	const int apb_over_freq = APB_CLK_FREQ / frequency;
	int best_div = -1;
	int best_prediv;
	int best_diff = MAX_INT;
	int cur_div;

	for (cur_div = 2; cur_div < SPI_CLKCNT_N; cur_div++) {
		int cur_prediv, cur_freq, diff;

		cur_prediv = (apb_over_freq / (cur_div - 1)) - 1;
		if (cur_prediv < 0) {
			/* If cur_prediv reaches -1, don't bother trying
			 * other values for the pre-divisor: all remaining
			 * values will be negative.
			 */
			break;
		}
		if (cur_prediv > SPI_CLKDIV_PRE) {
			continue;
		}

		cur_freq = APB_CLK_FREQ / ((cur_div + 1) * (cur_prediv + 1));

		if (curr_freq == frequency) {
			*div = cur_div;
			*prediv = cur_prediv;

			return 0;
		}

		diff = cur_freq - frequency;
		diff = (diff < 0) ? -diff : diff;
		if (diff < best_diff) {
			best_div = cur_div;
			best_prediv = cur_prediv;
			best_diff = diff;
		}
	}

	if (best_div < 0) {
		/* Couldn't find fitting N and PRE values. */
		return -EINVAL;
	}

	*div = best_div;
	*prediv = best_prediv;

	return 0;
}

static int spi_esp32_configure_frequency(const struct spi_config *config)
{
	u32_t clock_reg_val;

	if (config->frequency >= CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC) {
		/* SPI can operate at the system clock frequency; in this
		 * case, avoid the expensive calculation to find the best
		 * pre-divider and divider values.
		 */
		clock_reg_val = 1 << SPI_CLK_EQU_SYSCLK_S;
	} else {
		int div, prediv;
		int ret;

		assert(config->frequency > 0);

		ret = spi_esp32_calculate_div_prediv(config->frequency,
						     &div, &prediv);
		if (ret < 0) {
			return ret;
		}

		clock_reg_val = (prediv << SPI_CLKDIV_PRE_S) |
				(div << SPI_CLKCNT_N_S);

		/* H is set for a 50% duty cicle using the formula suggested
		 * in the TRM.  FIXME: make this a device-specific setting?
		 */
		clock_reg_val |= (((div + 1) / 2 - 1) << SPI_CLKCNT_H_S);

		if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
			/* Per the ESP32 TRM, SPI_CLKCNT_L must have the
			 * same value as the divisor while in master mode,
			 * and 0 otherwise.
			 */
			clock_reg_val |= (div << SPI_CLKCNT_L_S);
		}
	}

	sys_write32(clock_reg_val, SPI_CLOCK_REG(config->index));

	return 0;
}

static int spi_esp32_configure(struct spi_config *spi_config)
{
	struct spi_esp32_data *data = spi_config->dev->driver_data;
	const struct spi_esp32_config *config =
		spi_config->dev->config->config_info;
	/* Compact representation for tables found in the ESP32 TRM, chapter
	 * 7.4.1, "GP-SPI Clock Polarity and Clock Phase".
	 */
	static const u8_t clock_idle_edge_tbl = BIT(0) | BIT(1);
	static const u8_t clock_in_out_tbl = BIT(1) | BIT(2);
	static const u8_t mosi_miso_delay_mode_tbl[] = { 2, 1, 1, 2 };
	static const int int_enable_mask = SPI_INT_EN | SPI_TRANS_DONE |
		SPI_SLV_WR_BUF_INT | SPI_SLV_RD_BUF_DONE;
	int miso_delay_mode;
	int mosi_delay_mode;
	int mode;
	int ret;

	if (spi_context_configured(data->ctx, spi_config)) {
		return 0;
	}

	if (spi_config->cs) {
		/* ESP32 SPI peripheral controls the chip select itself. */
		return -EINVAL;
	}

	spi_esp32_enable_peripheral(config->index);

	ret = spi_esp32_configure_frequency(config);
	if (ret < 0) {
		return ret;
	}

	mode = spi_esp32_operation_to_mode(config->operation);

	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
		sys_clear_bit(SPI_SLAVE_REG(config->index), SPI_SLAVE_MODE_S);

		if (clock_in_out_tbl & BIT(mode)) {
			sys_set_bit(SPI_USER_REG(config->index),
				    SPI_CK_OUT_EDGE_S);
		} else {
			sys_clear_bit(SPI_USER_REG(config->index),
				      SPI_CK_OUT_EDGE_S);
		}

		miso_delay_mode = mosi_miso_delay_mode_tbl[mode];
		mosi_delay_mode = 0;

		clear_mask(int_enable_mask, SPI_SLAVE_REG(config->index));
	} else {
		sys_set_bit(SPI_SLAVE_REG(config->index), SPI_SLAVE_MODE_S);

		if (clock_in_out_tbl & BIT(MODE)) {
			sys_set_bit(SPI_USER_REG(config->index),
				    SPI_CK_I_EDGE_S);
		} else {
			sys_clear_bit(SPI_USER_REG(config->index),
				      SPI_CK_I_EDGE_S);
		}

		miso_delay_mode = 0;
		mosi_delay_mode = mosi_miso_delay_mode_tbl[mode];

		set_mask(int_enable_mask, SPI_SLAVE_REG(config->index));
	}

	set_mask(SPI_CTRL2_REG(config->index),
		 miso_delay_mode << SPI_MISO_DELAY_MODE_S);
	set_mask(SPI_CTRL2_REG(config->index), 0 << SPI_MISO_DELAY_NUM_S);

	set_mask(SPI_CTRL2_REG(config->index),
		 mosi_delay_mode << SPI_MOSI_DELAY_MODE_S);
	set_mask(SPI_CTRL2_REG(config->index), 0 << SPI_MOSI_DELAY_NUM_S);

	if (clock_idle_edge_tbl & BIT(mode)) {
		sys_set_bit(SPI_PIN_REG(config->index), SPI_CK_IDLE_EDGE_S);
	} else {
		sys_clear_bit(SPI_PIN_REG(config->index), SPI_CK_IDLE_EDGE_S);
	}

	if (config->operation & SPI_TRANSFER_LSB) {
		sys_set_bit(SPI_CTRL_REG(config->index),
			    SPI_WR_BIT_ORDER_S);
		sys_set_bit(SPI_CTRL_REG(config->index),
			    SPI_RD_BIT_ORDER_S);
	} else {
		sys_clear_bit(SPI_CTRL_REG(config->index),
			      SPI_WR_BIT_ORDER_S);
		sys_clear_bit(SPI_CTRL_REG(config->index),
			      SPI_RD_BIT_ORDER_S);
	}

	spi_esp32_configure_pins(config->index);

	set_mask(SPI_USER_REG(config->index),
		 1 << SPI_DOUTDIN_S | /* Enable full duplex communication */
		 1 << SPI_MISO | /* Enable read-data phase */
		 1 << SPI_MOSI); /* Enable write-data phase */

	return 0;
}

static size_t spi_esp32_calc_len(const struct spi_buf *bufs, size_t n_bufs)
{
	size_t size;
	size_t i;

	if (!bufs || !n_bufs) {
		return 0;
	}

	for (i = size = 0; i < n_bufs; i++) {
		size += bufs[i]->len;
	}

	return size;
}

static int spi_esp32_transceive(struct spi_config *config,
				const struct spi_buf *tx_bufs,
				size_t tx_count,
				struct spi_buf *rx_bufs,
				size_t rx_count)
{
	const struct spi_esp32_config *info = config->dev->config->config_info;
	struct spi_esp32_data *spi = config->dev->driver_data;
	struct spi_esp32_buffer_split *buffer_split;
	size_t tx_len = spi_esp32_calc_len(tx_bufs, tx_count);
	size_t rx_len = spi_esp32_calc_len(rx_bufs, rx_count);
	size_t len;
	int ret;

	/* FIXME: is device busy? can we test that? */
	spi_context_lock(&spi->ctx, false, NULL);

	ret = spi_esp32_configure(info, spi, config);
	if (ret < 0) {
		goto out;
	}

	/* spi_context_buffers_setup()? */

	if ((rx_len + tx_len) > 64) {
		/* GP-SPI transfers are limited to a 64-byte buffer. */
		ret = -ENOBUFS;
		goto out;
	}

	if ((rx_len > 32 && tx_len) || (tx_len > 32 && rx_len)) {
		/* If receiving more than 32 bytes, can't transmit, and
		 * vice-versa.
		 */
		ret = -ENOBUFS;
		goto out;
	}

	sys_write32(SPI_MOSI_DLEN_REG(config->index),
		    ((rx_len * CHAR_BIT) - 1) << SPI_USR_MOSI_DBITLEN_S);
	sys_write32(SPI_MISO_DLEN_REG(config->index),
		    ((tx_len * CHAR_BIT) - 1) << SPI_USR_MISO_DBITLEN_S);

	/* How MOSI_HIGHPART and MISO_HIGHPART fields in SPI_USER_REG are
	 * set seems inverted, but it's correct: if MOSI_HIGHPART is set,
	 * data transmitted to the slave device is read from
	 * SPI_W8_REG()...SPI_W15_REG(), leaving the space from
	 * SPI_W0_REG()...SPI_W7_REG() for reception.  If unset, the
	 * whole range from SPI_W0_REG()...SPI_W15_REG() is usable
	 * for either reception or transmission.
	 */
	if (rx_len) {
		sys_set_bit(SPI_USER_REG(config->index),
			    SPI_MOSI_HIGHPART_S);
	} else {
		sys_clear_bit(SPI_USER_REG(config->index),
			      SPI_MOSI_HIGHPART_S);
	}

	if (tx_len) {
		u8_t *buf;
		size_t i;

		sys_set_bit(SPI_USER_REG(config->index),
			    SPI_MISO_HIGHPART_S);

		if (rx_len) {
			buf = (u8_t *)SPI_W8_REG(config->index);
		} else {
			buf = (u8_t *)SPI_W0_REG(config->index);
		}

		for (i = 0; i < tx_count; i++) {
			memcpy(buf, tx_bufs[i].buf, tx_bufs[i].len);
			buf += tx_bufs[i].len;
		}
	} else {
		sys_clear_bit(SPI_USER_REG(config->index),
			      SPI_MISO_HIGHPART_S);
	}

	sys_set_bit(SPI_USER_REG(config->index), SPI_USR_COMMAND_M);

	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
		int spins = 0;

		/* Interrupts for master mode are only available if
		 * transfers are performed using DMA.
		 */
		while (true) {
			if (!sys_test_bit(SPI_USER_REG(config->index),
					  SPI_USR_COMMAND_S)) {
				break;
			}

			if (spins > SPI_ESP32_SPIN_THRESHOLD) {
				spins = 0;
				k_yield();
			}

			spins++;
		}

		ret = 0;
	} else {
		ret = spi_context_wait_for_completion(&spi->ctx);
	}

	if (rx_len) {
		u8_t *buf;
		size_t i;

		if (tx_len) {
			buf = (u8_t *)SPI_W8_REG(config->index);
		} else {
			buf = (u8_t *)SPI_W0_REG(config->index);
		}

		for (i = 0; i < rx_count; i++) {
			memcpy(rx_bufs[i].buf, buf, rx_bufs[i].len);
			buf += rx_bufs[i].len;
		}
	}

out:
	spi_context_release(&spi->ctx, ret);

	return ret;
}

static int spi_esp32_release(struct spi_config *config)
{
	const struct spi_esp32_config *info = config->dev->config->config_info;
	struct spi_esp32_data *spi = config->dev->driver_data;

	if (!spi_context_configured(&spi->ctx, config)) {
		return -EBUSY;
	}

	spi_context_unlock_unconditionally(&spi->ctx);

	return 0;
}

static const struct spi_driver_api {
	.transceive = spi_esp32_transceive,
	.release = spi_esp32_release,
};

static int spi_esp32_init(struct device *device)
{
	const struct spi_esp32_config *info = dev->config->config_info;
	struct spi_esp32_data *spi = dev->driver_data;

	info->configure_irq();

	spi_context_unlock_unconditionally(&spi->ctx);

	return 0;
}

/* Although ESP32 has 4 SPI devices, only 3 are exposed.  The reason is that
 * the internal flash device is connected directly to the SPI0 device.
 */
#ifdef CONFIG_SPI_0
static void spi_esp32_config_irq_0(const struct spi_esp32_config *config)
{
	IRQ_CONNECT(CONFIG_SPI_0_IRQ, CONFIG_SPI_0_IRQ_PRI,
		    spi_esp32_isr, 0, 0);
	irq_disable(CONFIG_SPI_0_IRQ);
}

static struct struct spi_esp32_data spi_esp32_0_data {
	SPI_CONTEXT_INIT_LOCK(spi_esp32_0_data, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_esp32_0_data, ctx),
};

static const struct spi_esp32_config spi_esp32_0_config {
	.index = 0,
	.configure_irq = spi_esp32_config_irq_0
};

DEVICE_DEFINE(spi_dev_0, CONFIG_SPI_0_NAME, spi_esp32_init,
	      &spi_esp32_0_data, NULL, &spi_esp32_0_config,
	      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, NULL);
#endif

#ifdef CONFIG_SPI_1
static void spi_esp32_config_irq_0(const struct spi_esp32_config *config)
{
	IRQ_CONNECT(CONFIG_SPI_1_IRQ, CONFIG_SPI_0_IRQ_PRI,
		    spi_esp32_isr, 0, 0);
	irq_disable(CONFIG_SPI_1_IRQ);
}

static struct struct spi_esp32_data spi_esp32_1_data {
	SPI_CONTEXT_INIT_LOCK(spi_esp32_1_data, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_esp32_1_data, ctx),
};

static const struct spi_esp32_config spi_esp32_1_config {
	.index = 1,
	.configure_irq = spi_esp32_config_irq_1
};

DEVICE_DEFINE(spi_dev_1, CONFIG_SPI_1_NAME, spi_esp32_init,
	      &spi_esp32_1_data, NULL, &spi_esp32_1_config,
	      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, NULL);
#endif

#ifdef CONFIG_SPI_2
static void spi_esp32__2(const struct spi_esp32_config *config)
{
	IRQ_CONNECT(CONFIG_SPI_2_IRQ, CONFIG_SPI_0_IRQ_PRI,
		    spi_esp32_isr, 0, 0);
	irq_disable(CONFIG_SPI_2_IRQ);
}

static struct struct spi_esp32_data spi_esp32_2_data {
	SPI_CONTEXT_INIT_LOCK(spi_esp32_2_data, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_esp32_2_data, ctx),
};

static const struct spi_esp32_config spi_esp32_2_config {
	.index = 2,
	.configure_irq = spi_esp32_config_irq_2
};

DEVICE_DEFINE(spi_dev_2, CONFIG_SPI_2_NAME, spi_esp32_init,
	      &spi_esp32_2_data, NULL, &spi_esp32_2_config,
	      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, NULL);
#endif

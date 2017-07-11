#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/module.h>

#include "gf_common.h"

extern u8 g_debug_level;

/*GPIO pins reference.*/
int gf_get_gpio_dts_info(struct gf_device *gf_dev)
{
	int rc;

    /*get reset resource*/
	gf_dev->reset_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_reset", 0);
	if (!gpio_is_valid(gf_dev->reset_gpio)) {
		gf_debug(ERR_LOG, "%s, RESET GPIO is invalid.\n", __func__);
		return -1;
	}
	rc = gpio_request(gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		gf_debug(ERR_LOG, "%s, Failed to request RESET GPIO. rc = %d\n", __func__, rc);
		return -1;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);

    /*get irq resourece*/
	gf_dev->irq_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio_irq", 0);
	gf_debug(DEBUG_LOG, "%s, gf:irq_gpio:%d\n", __func__, gf_dev->irq_gpio);
	if (!gpio_is_valid(gf_dev->irq_gpio)) {
		gf_debug(ERR_LOG, "%s, IRQ GPIO is invalid.\n", __func__);
		return -1;
	}

	rc = gpio_request(gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		gf_debug(ERR_LOG, "%s, Failed to request IRQ GPIO. rc = %d\n", __func__, rc);
		return -1;
	}
	gpio_direction_input(gf_dev->irq_gpio);

	return 0;
}

void gf_cleanup_info(struct gf_device *gf_dev)
{
	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		gf_debug(DEBUG_LOG, "%s, remove irq_gpio success\n", __func__);
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		gf_debug(DEBUG_LOG, "%s, remove reset_gpio success\n", __func__);
	}
	if (gpio_is_valid(gf_dev->cs_gpio)) {
		gpio_free(gf_dev->cs_gpio);
		gf_debug(DEBUG_LOG, "%s, remove reset_gpio success\n", __func__);
	}
}
/*power management*/


/**
 * gf_power_init - Initialize device power
 * @gf_dev: driver private data
 *
 * Returns zero on success, else an error.
 */
int gf_power_init(struct gf_device *gf_dev)
{
	int ret;

	gf_dev->avdd = devm_regulator_get(&gf_dev->spi->dev, "avdd");
	if (IS_ERR(gf_dev->avdd)) {
		ret = PTR_ERR(gf_dev->avdd);
		dev_info(&gf_dev->spi->dev,
			"Regulator get failed avdd ret=%d\n", ret);
	}

	return 0;
}

int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

int gf_power(struct gf_device *gf_dev, bool on)
{
	int rc;

	if (on) {
		if (!IS_ERR(gf_dev->avdd)) {
			rc = regulator_set_voltage(gf_dev->avdd, GOODIX_VTG_MIN_UV,
					GOODIX_VTG_MAX_UV);
			if (rc) {
				dev_err(&gf_dev->spi->dev,
						"Regulator set_vtg failed avdd rc=%d\n", rc);
				goto err_set_vtg_avdd;
			}
			rc = reg_set_optimum_mode_check(gf_dev->avdd,
					GOODIX_AVDD_LOAD_MAX_UA);
			if (rc < 0) {
				dev_err(&gf_dev->spi->dev,
						"Regulator avdd set_opt failed rc=%d\n", rc);
				goto err_set_opt_avdd;
			}
			rc = regulator_enable(gf_dev->avdd);
			if (rc) {
				dev_err(&gf_dev->spi->dev,
						"Regulator avdd enable failed rc=%d\n", rc);
				goto fail_enable_reg;
			}

		}
	} else {
		if (!IS_ERR(gf_dev->avdd)) {
			rc = regulator_disable(gf_dev->avdd);
			if (rc) {
				dev_err(&gf_dev->spi->dev,
						"Regulator avdd disable failed rc=%d\n", rc);
				return rc;
			}
		}
	}
	return 0;

fail_enable_reg:
err_set_opt_avdd:
err_set_vtg_avdd:
	return rc;
}

void gf_hw_power_enable(struct gf_device *gf_dev, u8 onoff)
{
	/* TODO: LDO configure */

	if (onoff) {
		gf_power(gf_dev, true);
		msleep(10);
		pr_info("---- power on ok ----\n");
	}
	else if (!onoff) {
		gf_power(gf_dev, false);
		pr_info("---- power off ----\n");
	}
}

void gf_hw_reset(struct gf_device *gf_dev, u8 delay)
{
	if (gf_dev == NULL) {
		gf_debug(ERR_LOG, "%s, Input buff is NULL.\n", __func__);
		return;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay);
}

void gf_irq_gpio_cfg(struct gf_device *gf_dev)
{
	if (gf_dev == NULL) {
		gf_debug(ERR_LOG, "%s, Input buff is NULL.\n", __func__);
		return;
	}
	gf_dev->irq = gpio_to_irq(gf_dev->irq_gpio);
}

void gf_spi_clk_enable(struct gf_device *gf_dev, u8 bonoff)
{
	static int count;

	if (bonoff && (count == 0))
		count = 1;
	else if ((count > 0) && (bonoff == 0))
		count = 0;
}


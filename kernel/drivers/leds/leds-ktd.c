#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <soc/qcom/socinfo.h>
#include <linux/leds.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

struct ktd_chip {
    struct led_classdev led_red_cdev;
    struct led_classdev led_green_cdev;
    struct led_classdev led_blue_cdev;
    struct regulator    *ktd_vcc;
    struct i2c_client   *i2c_client;
    unsigned char       rgb_enable;
    unsigned char       red_led_brightness;
    unsigned char       green_led_brightness;
    unsigned char       blue_led_brightness;
    int                 red_led_blink;
    int                 green_led_blink;
    int                 blue_led_blink;
    int                 usb_led_disable_gpio;
};

static void ktd_led_red_brightness_set(struct led_classdev *cdev,
                enum led_brightness value)
{
    struct ktd_chip *chip = container_of(cdev, struct ktd_chip, led_red_cdev);

    if ( value > LED_OFF )
	chip->rgb_enable = (chip->rgb_enable & 0xFC) | 0x1;
    else
	chip->rgb_enable = (chip->rgb_enable & 0xFC);

    chip->red_led_brightness = value;

    i2c_smbus_write_byte_data(chip->i2c_client, 0x06, value);
    i2c_smbus_write_byte_data(chip->i2c_client, 0x04, chip->rgb_enable);

    dev_dbg(&chip->i2c_client->dev, "%s: set the led brightness to value=%d\n", __func__, value);
}

static enum led_brightness ktd_led_red_brightness_get(struct led_classdev *cdev)
{
    struct ktd_chip *chip = container_of(cdev, struct ktd_chip, led_red_cdev);
    dev_dbg(&chip->i2c_client->dev, "%s: called to read the red led brightnes\n", __func__);
    return (chip->red_led_brightness > LED_OFF ? LED_FULL : LED_OFF);
}

static ssize_t ktd_led_red_blink_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    int     blinking;
    ssize_t rc = -EINVAL;
    struct  led_classdev    *cdev = dev_get_drvdata(dev);
    struct  ktd_chip        *chip = container_of(cdev, struct ktd_chip, led_red_cdev);

    rc = kstrtoint(buf, 10, &blinking);
    if ( rc ) {
        dev_err(&chip->i2c_client->dev, "%s: failed to read the input buffer", __func__);
        return rc;
    }

    dev_dbg(&chip->i2c_client->dev, "%s: called with blink=%d len=%d\n", __func__, blinking, (int)len);
    chip->red_led_blink = blinking;

    if ( blinking == 1 ) {
        chip->rgb_enable = (chip->rgb_enable & 0xFC) | 0x2;
        chip->red_led_blink = 0;
    }
    else if ( blinking == 2 ) {
        chip->rgb_enable = (chip->rgb_enable & 0xFC) | 0x3;
        chip->red_led_blink = 0;
    }

    if(chip->red_led_brightness == 0) {
        chip->red_led_brightness = 40;
	i2c_smbus_write_byte_data(chip->i2c_client, 0x06, chip->red_led_brightness);
    }

    i2c_smbus_write_byte_data(chip->i2c_client, 0x04, chip->rgb_enable);

    return len;
}

struct device_attribute dev_attr_led_red_blink = __ATTR(blink, 0644, NULL, ktd_led_red_blink_store);

static struct attribute *led_red_blink_attributes[] = {
    &dev_attr_led_red_blink.attr,
    NULL,
};

static struct attribute_group ktd_led_red_attr_group = {
    .attrs = led_red_blink_attributes
};

static enum led_brightness ktd_led_green_brightness_get(struct led_classdev *cdev)
{
    struct ktd_chip *chip = container_of(cdev, struct ktd_chip, led_green_cdev);
    dev_dbg(&chip->i2c_client->dev, "%s: called to read the green led brightnes\n", __func__);
    return (chip->green_led_brightness > LED_OFF ? LED_FULL : LED_OFF);
}

static void ktd_led_green_brightness_set(struct led_classdev *cdev,
                enum led_brightness value)
{
    struct ktd_chip *chip = container_of(cdev, struct ktd_chip, led_green_cdev);

    if ( value > LED_OFF )
	chip->rgb_enable = (chip->rgb_enable & 0xF3) | 0x4;
    else
	chip->rgb_enable = (chip->rgb_enable & 0xF3);

    chip->green_led_brightness = value;

    i2c_smbus_write_byte_data(chip->i2c_client, 0x07, value);
    i2c_smbus_write_byte_data(chip->i2c_client, 0x04, chip->rgb_enable);

    dev_dbg(&chip->i2c_client->dev, "%s: set the led brightness to value=%d\n", __func__, value);
}

static ssize_t ktd_led_green_blink_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    int     blinking;
    ssize_t rc = -EINVAL;
    struct  led_classdev    *cdev = dev_get_drvdata(dev);
    struct  ktd_chip        *chip = container_of(cdev, struct ktd_chip, led_green_cdev);

    rc = kstrtoint(buf, 10, &blinking);
    if ( rc ) {
        dev_err(&chip->i2c_client->dev, "%s: failed to read the input buffer", __func__);
        return rc;
    }

    dev_dbg(&chip->i2c_client->dev, "%s: called with blink=%d len=%d\n", __func__, blinking, (int)len);
    chip->green_led_blink = blinking;

    if ( blinking == 1 ) {
        chip->rgb_enable = (chip->rgb_enable & 0xF3) | 0x8;
        chip->green_led_blink = 0;
    }
    else if ( blinking == 2 ) {
        chip->rgb_enable = (chip->rgb_enable & 0xF3) | 0xC;
        chip->green_led_blink = 0;
    }

    if(chip->green_led_brightness == 0) {
        chip->green_led_brightness = 40;
	i2c_smbus_write_byte_data(chip->i2c_client, 0x07, chip->green_led_brightness);
    }

    i2c_smbus_write_byte_data(chip->i2c_client, 0x04, chip->rgb_enable);

    return len;
}

struct device_attribute dev_attr_led_green_blink = __ATTR(blink, 0644, NULL, ktd_led_green_blink_store);

static struct attribute *led_green_blink_attributes[] = {
    &dev_attr_led_green_blink.attr,
    NULL,
};

static struct attribute_group ktd_led_green_attr_group = {
    .attrs = led_green_blink_attributes
};

static enum led_brightness ktd_led_blue_brightness_get(struct led_classdev *cdev)
{
    struct ktd_chip *chip = container_of(cdev, struct ktd_chip, led_blue_cdev);
    dev_dbg(&chip->i2c_client->dev, "%s: called to read the blue led brightnes\n", __func__);
    return (chip->blue_led_brightness > LED_OFF ? LED_FULL : LED_OFF);
}


static void ktd_led_blue_brightness_set(struct led_classdev *cdev,
                enum led_brightness value)
{
    struct ktd_chip *chip = container_of(cdev, struct ktd_chip, led_blue_cdev);

    if ( value > LED_OFF )
	chip->rgb_enable = (chip->rgb_enable & 0xCF) | 0x10;
    else
	chip->rgb_enable = (chip->rgb_enable & 0xCF);

    chip->blue_led_brightness = value;

    i2c_smbus_write_byte_data(chip->i2c_client, 0x08, value);
    i2c_smbus_write_byte_data(chip->i2c_client, 0x04, chip->rgb_enable);

    dev_dbg(&chip->i2c_client->dev, "%s: set the led brightness to value=%d\n", __func__, value);
}

static ssize_t ktd_led_blue_blink_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    int     blinking;
    ssize_t rc = -EINVAL;
    struct  led_classdev    *cdev = dev_get_drvdata(dev);
    struct  ktd_chip        *chip = container_of(cdev, struct ktd_chip, led_blue_cdev);

    rc = kstrtoint(buf, 10, &blinking);
    if ( rc ) {
        dev_err(&chip->i2c_client->dev, "%s: failed to read the input buffer", __func__);
        return rc;
    }

    dev_dbg(&chip->i2c_client->dev, "%s: called with blink=%d len=%d\n", __func__, blinking, (int)len);
    chip->blue_led_blink = blinking;

    if ( blinking == 1 ) {
        chip->rgb_enable = (chip->rgb_enable & 0xCF) | 0x20;
        chip->blue_led_blink = 0;
    }
    else if ( blinking == 2 ) {
        chip->rgb_enable = (chip->rgb_enable & 0xCF) | 0x30;
        chip->blue_led_blink = 0;
    }

    if(chip->blue_led_brightness == 0) {
        chip->blue_led_brightness = 40;
	i2c_smbus_write_byte_data(chip->i2c_client, 0x08, chip->blue_led_brightness);
    }

    i2c_smbus_write_byte_data(chip->i2c_client, 0x04, chip->rgb_enable);

    return len;
}

struct device_attribute dev_attr_led_blue_blink = __ATTR(blink, 0644, NULL, ktd_led_blue_blink_store);

static struct attribute *led_blue_blink_attributes[] = {
    &dev_attr_led_blue_blink.attr,
    NULL,
};

static struct attribute_group ktd_led_blue_attr_group = {
    .attrs = led_blue_blink_attributes
};

static int ktd_register_leds(struct ktd_chip *chip)
{
    int rc;

    chip->led_red_cdev.name = "rgb_r";
    chip->led_red_cdev.brightness_set = ktd_led_red_brightness_set;
    chip->led_red_cdev.brightness_get = ktd_led_red_brightness_get;

    rc = led_classdev_register(&chip->i2c_client->dev, &chip->led_red_cdev);
    if (rc) {
        dev_err(&chip->i2c_client->dev, "unable to register red led, rc=%d\n", rc);
        return rc;
    }

    rc = sysfs_create_group(&chip->led_red_cdev.dev->kobj,
                        &ktd_led_red_attr_group);
    if (rc) {
        dev_err(&chip->i2c_client->dev, "led sysfs rc: %d\n", rc);
        return rc;
    }

    chip->led_green_cdev.name = "rgb_g";
    chip->led_green_cdev.brightness_set = ktd_led_green_brightness_set;
    chip->led_green_cdev.brightness_get = ktd_led_green_brightness_get;

    rc = led_classdev_register(&chip->i2c_client->dev, &chip->led_green_cdev);
    if (rc) {
        dev_err(&chip->i2c_client->dev, "unable to register green led, rc=%d\n", rc);
        return rc;
    }

    rc = sysfs_create_group(&chip->led_green_cdev.dev->kobj,
                        &ktd_led_green_attr_group);
    if (rc) {
        dev_err(&chip->i2c_client->dev, "led sysfs rc: %d\n", rc);
        return rc;
    }

    chip->led_blue_cdev.name = "rgb_b";
    chip->led_blue_cdev.brightness_set = ktd_led_blue_brightness_set;
    chip->led_blue_cdev.brightness_get = ktd_led_blue_brightness_get;

    rc = led_classdev_register(&chip->i2c_client->dev, &chip->led_blue_cdev);
    if (rc) {
        dev_err(&chip->i2c_client->dev, "unable to register blue led, rc=%d\n", rc);
        return rc;
    }

    rc = sysfs_create_group(&chip->led_blue_cdev.dev->kobj,
                        &ktd_led_blue_attr_group);
    if (rc) {
        dev_err(&chip->i2c_client->dev, "led sysfs rc: %d\n", rc);
        return rc;
    }

    return rc;
}

static int ktd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

    struct ktd_chip *chip;
    int rc = 0;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "%s: check_functionality failed.\n", __func__);
        return -ENODEV;
    }

    chip = kzalloc( sizeof(struct ktd_chip), GFP_KERNEL);
    if (!chip) {
        dev_err(&client->dev,"%s: memory allocation failed.\n", __func__);
        return -ENOMEM;
    }

    /* Intialize the regulator */
    chip->ktd_vcc = regulator_get(&client->dev, "vcc_i2c");
    if (IS_ERR(chip->ktd_vcc)) {
        rc = PTR_ERR(chip->ktd_vcc);
        dev_err(&client->dev,"%s: regulator_get failed to get the vcc_i2c rc=%d\n", __func__, rc);
        return rc;
    }

    rc = regulator_enable(chip->ktd_vcc);
    if (rc) {
        dev_err(&client->dev,"%s: Failed to enabled vcc_i2c regulator, rc=%d\n", __func__, rc);
        regulator_put(chip->ktd_vcc);
        return rc;
    }

    chip->usb_led_disable_gpio = of_get_named_gpio(client->dev.of_node, "ktd,gpio", 0);
    if ( chip->usb_led_disable_gpio ) {
        gpio_request(chip->usb_led_disable_gpio, "usb_led_disable_gpio");
        gpio_direction_output(chip->usb_led_disable_gpio, 1);
    }

    chip->i2c_client = client;
    i2c_set_clientdata(client, chip);
    dev_set_drvdata(&client->dev, chip);

    ktd_register_leds(chip);

    i2c_smbus_write_byte_data(chip->i2c_client, 0x00, 0);
    mdelay(1);
    i2c_smbus_write_byte_data(chip->i2c_client, 0x04, 0);
#if 0
    i2c_smbus_write_byte_data(chip->i2c_client, 0x01, 14);
    i2c_smbus_write_byte_data(chip->i2c_client, 0x02, 64);
    i2c_smbus_write_byte_data(chip->i2c_client, 0x03, 16);
#else
//    i2c_smbus_write_byte_data(chip->i2c_client, 0x00, 32);
    i2c_smbus_write_byte_data(chip->i2c_client, 0x01, 28);
    i2c_smbus_write_byte_data(chip->i2c_client, 0x02, 41);
    i2c_smbus_write_byte_data(chip->i2c_client, 0x03, 16);
    i2c_smbus_write_byte_data(chip->i2c_client, 0x05, 0x66);
#endif
    dev_dbg(&client->dev,"%s() called\n", __func__);

    return 0;
};

static int ktd_remove(struct i2c_client *client)
{
    struct ktd_chip *chip = i2c_get_clientdata(client);
    led_classdev_unregister(&chip->led_red_cdev);
    led_classdev_unregister(&chip->led_green_cdev);
    led_classdev_unregister(&chip->led_blue_cdev);

    regulator_disable(chip->ktd_vcc);
    regulator_put(chip->ktd_vcc);

    i2c_smbus_write_byte_data(chip->i2c_client, 0x04, 0);
    i2c_smbus_write_byte_data(chip->i2c_client, 0x06, 0);
    i2c_smbus_write_byte_data(chip->i2c_client, 0x07, 0);
    i2c_smbus_write_byte_data(chip->i2c_client, 0x08, 0);

    kfree(chip);
    return 0;
};

static const struct i2c_device_id ktd_i2c_dev_id[] = {
    { "ktd202x", 0},
    { }
};

#ifdef CONFIG_OF
static struct of_device_id ktd_match_table[] = {
    { .compatible = "ktd,ktd20x"},
    { },
};
#else
#define ktd20xx_match_table NULL
#endif

static struct i2c_driver ktd_driver = {
    .driver = {
        .name         = "ktd202x",
        .owner         = THIS_MODULE,
        .of_match_table = ktd_match_table,
    },
    .probe         = ktd_probe,
    .remove     = ktd_remove,
    .id_table     = ktd_i2c_dev_id,
};

module_i2c_driver(ktd_driver);

MODULE_AUTHOR("Rajiv Shankar <rajiv.shankar@smartron.com>");
MODULE_DESCRIPTION("Driver for KTD LED Controller");
MODULE_LICENSE("GPL");

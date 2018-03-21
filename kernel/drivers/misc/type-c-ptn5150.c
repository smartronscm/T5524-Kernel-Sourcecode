/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define DEBUG 1
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/debugfs.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/list.h>


#define PERICOM_I2C_NAME	"usb-type-c-ptn5150"
#define PERICOM_I2C_DELAY_MS	30

#define CCD_DEFAULT		0x1
#define CCD_MEDIUM		0x2
#define CCD_HIGH		0x3

#define MAX_CURRENT_BC1P2	500
#define MAX_CURRENT_MEDIUM     1500
#define MAX_CURRENT_HIGH       3000

#define	UFP_MODE 0
#define 	DRP_MODE 1
#define 	DFP_MODE 2




/*----------------- REGISTER DEFINATION -----------------*/

#define PTN5150_7BIT_I2C_ADDR_LOW	0x1d
#define PTN5150_7BIT_I2C_ADDR_HIGH	0x3d

#define	PTN5150_VERSION_REG			0x01

#define	VERSION_ID_MASK				0xf8
#define	VENDOR_ID_MASK				0x07

#define	PTN5150_CONTROL_REG			0x02

#define	RP_SELECT_DEFULAT			0x00
#define	RP_SELECT_MEDIUM			0x08
#define	RP_SELECT_HIGH				0x18

#define	PORT_SET_UFP				(0x0 << 1)
#define	PORT_SET_DFP				(0x1 << 1)
#define	PORT_SET_DRP				(0x2 << 1)

#define	ATTACH_INT_MASK_EN			0x00
#define	ATTACH_INT_MASK_DIS			0x01

#define	PTN5150_INT_STATUS_REG		0x03

#define	CABLE_DETACH_INT			0x02
#define	CABLE_ATTACH_INT			0x01

#define	PTN5150_CC_STATUS_REG		0x04

#define	VBUS_DETECTION				0x80

#define	RP_DET_AS_NONE				0x00
#define	RP_DET_AS_DEFAULT			0x01
#define	RP_DET_AS_MEDIUM			0x02
#define	RP_DET_AS_HIGH				0x03

#define	ATTACHED_IS_NONE			0x00
#define	ATTACHED_IS_DFP				0x01
#define	ATTACHED_IS_UFP				0x02
#define	ATTACHED_IS_AUDIO			0x03
#define	ATTACHED_IS_DEBUG			0x04

#define	CC_POLARITY_NONE			0x00
#define	CC_POLARITY_CC1				0x01
#define	CC_POLARITY_CC2				0x02

#define	PTN5150_CONDET_REG			0x09

#define	CON_DET_EN					0x00
#define	CON_DET_DIS					0x01

#define	PTN5150_VCONN_REG			0x0a

#define	VCONN_STANDBY				0x00
#define	VCONN_CC1					0x01
#define	VCONN_CC2					0x02

#define	PTN5150_VCONN_ACCESS_REG	0x43

#define	VCONN_ACCESS_CODE			0xe0

/*----------------- REGISTER DEFINATION -----------------*/


/*----------------- TRY SINK STATUS -----------------*/
#define  Try_Sink_Idle_DRP      				0
#define  Try_Sink_Attached_Wait_Src    			1
#define  Try_Sink_Source_To_Sink    			2
#define  Try_Sink_Attached_Wait_Src_Detached   	3
#define  Try_Sink_Attached_As_UFP    			4
#define  Try_Sink_tDRPTry_Expire    			5
#define  Try_Sink_Try_Wait_Src     				6
#define  Try_Sink_Attached_As_DFP    			7
#define  Try_Sink_Attached_As_Audio    			8
#define  Try_Sink_Attached_As_Debug    			9
/*----------------- TRY SINK STATUS -----------------*/




static bool disable_on_suspend;
module_param(disable_on_suspend , bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(disable_on_suspend,
	"Whether to disable chip on suspend if state is not attached");

static int Try_Sink_State = Try_Sink_Idle_DRP;

struct ptn5150_usb_type_c {

	struct delayed_work trysink_check_work;
	struct work_struct 	check_work;

	struct i2c_client	*client;
	struct device *dev;
	struct power_supply	*usb_psy;

	int			max_current;
	bool			attach_state;

	unsigned			irq_gpio;
	unsigned 			irq_no;

//	unsigned 			dir_sel_gpio;

	bool				irq_enabled;
	bool				id_irq_enabled;
	spinlock_t			irq_enabled_lock;

	struct dentry			*debug_root;
	 int				peek_poke_address;
};
static struct ptn5150_usb_type_c *ptn_usb;


static int ptn5150_usb_i2c_read_byte(int reg,u8 *val)
{
        int retval;

        retval = i2c_smbus_read_byte_data(ptn_usb->client, reg);
        if (retval < 0) {
                dev_err(&ptn_usb->client->dev, "%s failed, reg: %d, error: %d\n",
                        __func__, reg, retval);
                return retval;
        }
        *val = (u16)retval;
        return retval;
}


static int ptn5150_usb_i2c_write_byte(int reg, u8 val)
{
        int retval = 0;

        retval = i2c_smbus_write_byte_data(ptn_usb->client, reg, val);
        if (retval < 0) {
                dev_err(ptn_usb->dev,
                        "%s failed, reg: %d, val: %d, error: %d\n",
                        __func__, reg, val, retval);
                return retval;
        }
        return 0;
}

static irqreturn_t ptn5150_usb_irq(int irq, void *data)
{
	//int ret;
	//struct pi_usb_type_c *pi_usb = (struct pi_usb_type_c *)data;
	struct ptn5150_usb_type_c *chip  = data;
	unsigned long flags;

	//pr_debug("handle irq(%d), client->irq(%d). +\n", irq, ptn5150_dev->client->irq);

	if(NULL == chip->client) {
		pr_err("NULL == ptn5150_dev->client, return IRQ_NONE.\n");
		return IRQ_NONE;
	}

	if (chip->irq_enabled && irq) {
		pr_debug("handle irq(%d) +\n", irq);
		spin_lock_irqsave(&chip->irq_enabled_lock, flags);
		disable_irq_nosync(irq);
		spin_unlock_irqrestore(&chip->irq_enabled_lock, flags);
		chip->irq_enabled = false;
		schedule_work(&chip->check_work);
	}

	return IRQ_HANDLED;
}



static int get_reg(void *data, u64 *val)
{
	struct ptn5150_usb_type_c *chip = data;
	int rc;
	u8 temp;

	rc = ptn5150_usb_i2c_read_byte(chip->peek_poke_address, &temp);
	if (rc < 0) {
		dev_err(&(chip->client->dev),
			"Couldn't read reg %x rc = %d\n",
			chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	struct ptn5150_usb_type_c *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = ptn5150_usb_i2c_write_byte(chip->peek_poke_address, temp);
	if (rc < 0) {
		dev_err(&chip->client->dev,
			"Couldn't write 0x%02x to 0x%02x rc= %d\n",
			chip->peek_poke_address, temp, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(poke_poke_debug_ops, get_reg, set_reg, "0x%02llx\n");


//dumpall bq registers.
static int show_ptn_dumpall(struct seq_file *m, void *data)
{
	int i,rc;
	u8 temp;
	struct ptn5150_usb_type_c *chip = m->private;

	for (i = 0x1; i < 0x5; i++){
		rc = ptn5150_usb_i2c_read_byte(i, &temp);
		if (rc < 0) {
			dev_err(chip->dev,"Couldn't read reg %x rc = %d\n", i, rc);
			return -EAGAIN;
		}
		seq_printf(m, "read reg: 0x%02x = 0x%02x\n", i, temp);
	}

	for (i = 0x9; i < 0x10; i++){
		rc = ptn5150_usb_i2c_read_byte(i, &temp);
		if (rc < 0) {
			dev_err(chip->dev,"Couldn't read reg %x rc = %d\n", i, rc);
			return -EAGAIN;
		}
		seq_printf(m, "read reg: 0x%02x = 0x%02x\n", i, temp);
	}

	for (i = 0x18; i < 0x1a; i++){
		rc = ptn5150_usb_i2c_read_byte(i, &temp);
		if (rc < 0) {
			dev_err(chip->dev,"Couldn't read reg %x rc = %d\n", i, rc);
			return -EAGAIN;
		}
		seq_printf(m, "read reg: 0x%02x = 0x%02x\n", i, temp);
	}

	return 0;
}

static int ptn_dumpall_open(struct inode *inode, struct file *file)
{
	struct ptn5150_usb_type_c *chip = inode->i_private;

	return single_open(file, show_ptn_dumpall, chip);
}

static const struct file_operations ptn_dumpall_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= ptn_dumpall_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int create_ptn5150_debugfs_entries(struct ptn5150_usb_type_c *chip)
{

      chip->debug_root = debugfs_create_dir("ptn5150", NULL);
      if (!chip->debug_root)
		pr_err( "Couldn't create ptn5150 debug dir\n");
      if (chip->debug_root) {

		struct dentry *ent;
		ent = debugfs_create_x32("address", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->peek_poke_address));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create address debug file\n");

		ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &poke_poke_debug_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file\n");

		ent = debugfs_create_file("ptn_dumpall", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &ptn_dumpall_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file\n");

      }
     return 0;
}

static int reset_chip(struct ptn5150_usb_type_c *chip){

	 ptn5150_usb_i2c_write_byte(0x10,0x00);
	 msleep(20);
	 return 0;
}

int ptn5150_set_mode(struct ptn5150_usb_type_c *chip, unsigned char mode)
{
	int ret = 0;
	unsigned char data = 0xff;

	if((mode != PORT_SET_UFP) && (mode != PORT_SET_DFP) && (mode != PORT_SET_DRP))
	{
		pr_err("Set mode error, mode = 0x%02x.\n", mode);
		return -1;
	}
	ptn5150_usb_i2c_read_byte(0x02, &data);
	data &= ~(3 << 1);
	data |= (mode & 0xfe);	//Enable connect/disconnect interrupt
	ret = ptn5150_usb_i2c_write_byte(0x02, data);
	pr_debug("mode = 0x%02x, 0x%02x => reg[0x02].\n", mode, data);

	return ret;
}


static int ptn5150_hw_init(struct ptn5150_usb_type_c *chip){
	ptn5150_usb_i2c_write_byte(0x43,0x40);
	ptn5150_usb_i2c_write_byte(0x4c, 0x34);
	ptn5150_set_mode(chip, PORT_SET_DRP);	//Enable DRP mode, Enable Interrupt
	return 0;
}


static void ptn5150_trysink_status_check(struct work_struct *work)
{
	struct ptn5150_usb_type_c *chip = ptn_usb;
	if (Try_Sink_State == Try_Sink_Attached_Wait_Src_Detached)	// If timer expires
	{
		Try_Sink_State = Try_Sink_tDRPTry_Expire;
		ptn5150_set_mode(chip, PORT_SET_DFP);	// Force in DFP mode
		Try_Sink_State = Try_Sink_Try_Wait_Src;
	}
	pr_debug("Try_Sink_State = %d.\n", Try_Sink_State);
}

static void ptn5150_update_max_current(struct ptn5150_usb_type_c *chip,const unsigned char  ccstatus)
{
	char chg_mode=  (ccstatus >> 5) & 0x03;
	union power_supply_propval chg_current;

	dev_dbg(&chip->client->dev,"cc status 0x%02x  and chg_mode=%d\n",ccstatus,chg_mode);
	switch (chg_mode) {
	case CCD_DEFAULT:
		chip->max_current = MAX_CURRENT_BC1P2;
		break;
	case CCD_MEDIUM:
		chip->max_current = MAX_CURRENT_MEDIUM;
		break;
	case CCD_HIGH:
		chip->max_current = MAX_CURRENT_HIGH;
		break;
	default:
		dev_dbg(&chip->client->dev, "wrong chg mode %x\n", chg_mode);
		chip->max_current = MAX_CURRENT_BC1P2;
	}

	chg_current.intval = chip->max_current ;
	if (chip->usb_psy)
		 chip->usb_psy->set_property(chip->usb_psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, &chg_current);

	dev_dbg(&chip->client->dev, "chg mode: %x, mA:%u\n", chg_mode,
							chip->max_current);
}
/*
static unsigned char  ptn5150_select_usbPort(struct ptn5150_usb_type_c *chip, unsigned char polarity)
{
      //	int ret = 0;
	unsigned char level = (polarity & 0x02)>>1;//polarity=Reg[4] & 0x03 CC 的值CC1=1 CC2=2level=0为CC2,穃u017d之为CC1，
	pr_debug("Set select_gpio(%d) = %d, polarity(%d).\n", chip->dir_sel_gpio, level, polarity);
	if (chip->dir_sel_gpio) {
		 gpio_direction_output(chip->dir_sel_gpio, level);
		pr_debug("REG[0x04] = 0x%02x select_gpio= 0x%02x.\n", polarity,level);
	}
	return level;
}
*/
static void ptn5150_status_check(struct work_struct *work)
{
	struct ptn5150_usb_type_c *chip =
			container_of(work, struct ptn5150_usb_type_c,
			check_work);
	unsigned char Reg_Data;
	unsigned char Attach_Status;
	unsigned char Polarity;
	//unsigned long flags;

	if(NULL == chip->client) {
		pr_err("NULL == ptn5150_dev->client, return.\n");
		return;
	}
	/*read interrupt status, clear the interrupt flag*/
	ptn5150_usb_i2c_read_byte(PTN5150_INT_STATUS_REG, &Reg_Data);
	pr_debug("reg[0x03] = 0x%02x.\n", Reg_Data);

	switch (Reg_Data)
	{
		case CABLE_DETACH_INT:
			if ((Try_Sink_State == Try_Sink_Attached_As_DFP) || (Try_Sink_State == Try_Sink_Attached_As_UFP))
			{
				ptn5150_set_mode(chip, PORT_SET_DRP);	// Force in DRP mode
				Try_Sink_State = Try_Sink_Idle_DRP;
			}
			else if (Try_Sink_State == Try_Sink_Source_To_Sink)
			{
				// Start 150 ms timer
				Try_Sink_State = Try_Sink_Attached_Wait_Src_Detached;
				schedule_delayed_work(&chip->trysink_check_work, msecs_to_jiffies(200));
			}
			break;
		case CABLE_ATTACH_INT:
		default:	// if both attach and detach are set
			ptn5150_usb_i2c_read_byte(PTN5150_CC_STATUS_REG, &Reg_Data);
			pr_debug("reg[0x04] = 0x%02x.\n", Reg_Data);

			ptn5150_update_max_current(chip,Reg_Data);

			/*Check CC Polarity: 01: CC1, 10: CC2*/
			Polarity = Reg_Data & 0x03;
			//ptn5150_select_usbPort(chip, Polarity);

			/*let's figure out what's connected or what's not connected*/
			Attach_Status = (Reg_Data >> 2) & 0x07;
			switch (Attach_Status)
			{
				case ATTACHED_IS_NONE:
					break;
				case ATTACHED_IS_DFP:
					if (Try_Sink_State == Try_Sink_Attached_Wait_Src_Detached)
					{
						Try_Sink_State = Try_Sink_Attached_As_UFP;
					}
					else
					{
						Try_Sink_State = Try_Sink_Attached_As_UFP;
					}
					//TODO
					break;
				case ATTACHED_IS_UFP:
					if (Try_Sink_State == Try_Sink_Idle_DRP)
					{
						Try_Sink_State = Try_Sink_Attached_Wait_Src;
						ptn5150_set_mode(chip, PORT_SET_UFP);  // Force in UFP mode
						Try_Sink_State = Try_Sink_Source_To_Sink;
					}
					else if (Try_Sink_State == Try_Sink_Try_Wait_Src)
					{
						Try_Sink_State = Try_Sink_Attached_As_DFP;
					}
					break;
				case ATTACHED_IS_AUDIO:
					Try_Sink_State = Try_Sink_Attached_As_Audio;
					break;
				case ATTACHED_IS_DEBUG:
					Try_Sink_State = Try_Sink_Attached_As_Debug;
					break;
				default:
					break;
			}
			break;
	}
	pr_debug("Try_Sink_State = %d.\n", Try_Sink_State);

	//spin_lock_irqsave(&ptn5150_dev->irq_enabled_lock, flags);
	if(!chip->irq_enabled && chip->irq_no) {
		enable_irq(chip->irq_no);
		chip->irq_enabled = true;
		pr_debug("ptn5150 Enable IRQ(%d).\n", chip->irq_no);
	}
	//spin_unlock_irqrestore(&ptn5150_dev->irq_enabled_lock, flags);
}


static int ptn5150_usb_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int ret;
	u8 version;
	struct power_supply *usb_psy;
	struct device_node *np = i2c->dev.of_node;


	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C))
                return -ENODEV;


	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_err(&i2c->dev, "USB power_supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}

	ptn_usb = devm_kzalloc(&i2c->dev, sizeof(struct ptn5150_usb_type_c),
				GFP_KERNEL);
	if (!ptn_usb)
		return -ENOMEM;

	i2c_set_clientdata(i2c, ptn_usb);
	ptn_usb->client = i2c;
	ptn_usb->dev =&i2c->dev;
	ptn_usb->usb_psy = usb_psy;

	ptn_usb->irq_gpio = of_get_named_gpio(np, "ptn5150,irq-gpio", 0);
	if (!gpio_is_valid(ptn_usb->irq_gpio )) {
		dev_err(&i2c->dev, "irq gpio error:%d\n",ptn_usb->irq_gpio);
		goto mem_failed;
	}
/*
	ptn_usb->dir_sel_gpio = of_get_named_gpio(np, "ptn36043,dir-sel-gpio", 0);
	if (!gpio_is_valid(ptn_usb->dir_sel_gpio )) {
		dev_err(&i2c->dev, "dir-sel-gpio error:%d\n",ptn_usb->dir_sel_gpio);
		goto mem_failed;
	}

	ret = gpio_request( ptn_usb->dir_sel_gpio,"ptn36043_dir_sel");
	if (ret) {
		pr_err("unable to request gpio [%d]\n", ptn_usb->dir_sel_gpio);
		goto mem_failed;
	}
*/

	ptn_usb->irq_no = gpio_to_irq(ptn_usb->irq_gpio);
       if (ptn_usb->irq_no) {
		ret = request_irq(ptn_usb->irq_no,
					  ptn5150_usb_irq,
					  IRQF_TRIGGER_RISING |
					  IRQF_TRIGGER_FALLING,
					  "ptn5150_usb_type_c_irq", ptn_usb);
		if (ret) {
				pr_err("request irq failed for  ptn5150 IRQ\n");
				//goto gpio_failed;
				goto mem_failed;
		}
	}

	INIT_WORK(&ptn_usb->check_work, ptn5150_status_check);
	INIT_DELAYED_WORK(&ptn_usb->trysink_check_work, ptn5150_trysink_status_check);

	ptn_usb->irq_enabled = true;
	ptn_usb->id_irq_enabled = true;

	spin_lock_init(&ptn_usb->irq_enabled_lock);

	reset_chip(ptn_usb);
	create_ptn5150_debugfs_entries(ptn_usb);

	ptn5150_hw_init(ptn_usb);
	ptn5150_usb_i2c_read_byte(0x01, &version);
	dev_err(&i2c->dev, "%s finished, version:%d,subversion:%d, addr:%d\n", __func__, i2c->addr,version>>3, version||0x07);
pr_err("*** ABCDEF *** ptn5150_usb_probe probe succeed\n");
	return 0;

/*
gpio_failed:
	gpio_free(ptn_usb->dir_sel_gpio);
*/
mem_failed:
	kfree(ptn_usb);
	dev_err(&i2c->dev, "%s finished error occured \n", __func__);
	return ret;
}

static int ptn5150_usb_remove(struct i2c_client *i2c)
{

	//struct ptn5150_usb_type_c *pi_usb = i2c_get_clientdata(i2c);
	kfree(ptn_usb);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ptn5150_usb_i2c_suspend(struct device *dev)
{
	return 0;
}

static int ptn5150_usb_i2c_resume(struct device *dev)
{
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ptn5150_usb_i2c_pm_ops, ptn5150_usb_i2c_suspend,
			  ptn5150_usb_i2c_resume);

static const struct i2c_device_id piusb_id[] = {
	{ PERICOM_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, piusb_id);

#ifdef CONFIG_OF
static const struct of_device_id piusb_of_match[] = {
	{ .compatible = "nxp,ptn5150-usb-type-c", },
	{},
};
MODULE_DEVICE_TABLE(of, piusb_of_match);
#endif

static struct i2c_driver piusb_driver = {
	.driver = {
		.name = PERICOM_I2C_NAME,
		.of_match_table = of_match_ptr(piusb_of_match),
		.pm	= &ptn5150_usb_i2c_pm_ops,
	},
	.probe		= ptn5150_usb_probe,
	.remove		= ptn5150_usb_remove,
	.id_table	= piusb_id,
};

module_i2c_driver(piusb_driver);

MODULE_DESCRIPTION("NXP PTN5150 TypeC Detection driver");
MODULE_LICENSE("GPL v2");

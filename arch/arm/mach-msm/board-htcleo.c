/* linux/arch/arm/mach-msm/board-htcleo.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
 * Author: Dima Zavin <dima@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-msm.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/bootmem.h>
#include <linux/platform_device.h>
#include <linux/android_pmem.h>
#include <linux/regulator/machine.h>
#include <linux/usb/composite.h>
#include <linux/usb/android_composite.h>
#include <linux/leds.h>
#include <linux/spi/spi.h>
#include <linux/bma150.h>
#include <linux/akm8973.h>
#include <../../../drivers/staging/android/timed_gpio.h>
#include <linux/ds2746_battery.h>
#include <linux/msm_kgsl.h>
#include <linux/regulator/machine.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/msm_iomap.h>
#include <mach/perflock.h>
#include <mach/htc_usb.h>
#include <mach/msm_flashlight.h>
#include <mach/msm_serial_hs.h>
#ifdef CONFIG_USB_MSM_OTG_72K
#include <mach/msm_hsusb.h>
#else
#include <linux/usb/msm_hsusb.h>
#endif
#include <mach/msm_hsusb_hw.h>
#ifdef CONFIG_SERIAL_BCM_BT_LPM
#include <mach/bcm_bt_lpm.h>
#endif
#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>

#include <mach/board-htcleo-mac.h>
#include <mach/board-htcleo-microp.h>
#include <mach/board-htcleo-ts.h>
#include <mach/socinfo.h>
#include <mach/msm_memtypes.h>

#include "acpuclock.h"
#include "board-htcleo.h"
#include "devices.h"
#include "proc_comm.h"
#include "dex_comm.h"
#include "footswitch.h"
#include "pm.h"
#include "pm-boot.h"

#define ATAG_MAGLDR_BOOT    0x4C47414D
struct tag_magldr_entry
{
     _Bool fNoNandBoot;
};

extern int __init htcleo_init_mmc(unsigned debug_uart);
extern void __init htcleo_audio_init(void);
extern unsigned char *get_bt_bd_ram(void);
static unsigned int nand_boot = 0;

///////////////////////////////////////////////////////////////////////
// Nand boot Option
///////////////////////////////////////////////////////////////////////
int htcleo_is_nand_boot(void)
{
	return nand_boot;
}

static int __init parse_tag_nand_boot(const struct tag *tag)
{
	struct tag_magldr_entry *mentry = (struct tag_magldr_entry *)(&tag->u);
	nand_boot = !(unsigned int)mentry->fNoNandBoot;
	if(*((unsigned*)&tag->u)==0x004b4c63) nand_boot = 2; // cLK signature
	pr_info("Nand Boot: %d\n", nand_boot);
	return 0;
}
__tagtable(ATAG_MAGLDR_BOOT, parse_tag_nand_boot);



///////////////////////////////////////////////////////////////////////
// Regulator
///////////////////////////////////////////////////////////////////////

static struct regulator_consumer_supply tps65023_dcdc1_supplies[] =
{
    {
        .supply = "acpu_vcore",
    },
};

static struct regulator_init_data tps65023_data[5] =
{
    {
        .constraints = {
            .name = "dcdc1", /* VREG_MSMC2_1V29 */
            .min_uV = HTCLEO_TPS65023_MIN_UV_MV * 1000,
            .max_uV = HTCLEO_TPS65023_MAX_UV_MV * 1000,
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
        },
        .consumer_supplies = tps65023_dcdc1_supplies,
        .num_consumer_supplies = ARRAY_SIZE(tps65023_dcdc1_supplies),
    },
    /* dummy values for unused regulators to not crash driver: */
    {
        .constraints = {
            .name = "dcdc2", /* VREG_MSMC1_1V26 */
            .min_uV = 1260000,
            .max_uV = 1260000,
        },
    },
    {
        .constraints = {
            .name = "dcdc3", /* unused */
            .min_uV = 800000,
            .max_uV = 3300000,
        },
    },
    {
        .constraints = {
            .name = "ldo1", /* unused */
            .min_uV = 1000000,
            .max_uV = 3150000,
        },
    },
    {
        .constraints = {
            .name = "ldo2", /* V_USBPHY_3V3 */
            .min_uV = 3300000,
            .max_uV = 3300000,
        },
    },
};
///////////////////////////////////////////////////////////////////////
// Headset
///////////////////////////////////////////////////////////////////////

static struct htc_headset_mgr_platform_data htc_headset_mgr_data = {
};

static struct platform_device htc_headset_mgr = {
	.name	= "HTC_HEADSET_MGR",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_mgr_data,
	},
};

static struct htc_headset_gpio_platform_data htc_headset_gpio_data = {
	.hpin_gpio		= HTCLEO_GPIO_HDS_DET,
	.mic_detect_gpio	= HTCLEO_GPIO_HDS_MIC,
	.microp_channel		= 1,
	.key_enable_gpio	= 0,
	.mic_select_gpio	= 0,
};

static struct platform_device htc_headset_gpio = {
	.name	= "HTC_HEADSET_GPIO",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_gpio_data,
	},
};



///////////////////////////////////////////////////////////////////////
// Compass
///////////////////////////////////////////////////////////////////////
static struct akm8973_platform_data compass_platform_data =
{
	.layouts = HTCLEO_LAYOUTS,
	.project_name = HTCLEO_PROJECT_NAME,
	.reset = HTCLEO_GPIO_COMPASS_RST_N,
	.intr = HTCLEO_GPIO_COMPASS_INT_N,
};


///////////////////////////////////////////////////////////////////////
// LED Driver (drivers/leds/leds-microp.c - Atmega microp driver
///////////////////////////////////////////////////////////////////////

static struct microp_led_config led_config[] = {
        {
                .name = "amber",
                .type = LED_RGB,
        },
        {
                .name = "green",
                .type = LED_RGB,
        },
};

static struct microp_led_platform_data microp_leds_data = {
        .num_leds       = ARRAY_SIZE(led_config),
        .led_config     = led_config,
};

///////////////////////////////////////////////////////////////////////
// Microp
///////////////////////////////////////////////////////////////////////
static struct bma150_platform_data htcleo_g_sensor_pdata = {
	.microp_new_cmd = 0,
	.chip_layout = 1,
};

static struct platform_device microp_devices[] = {
	{
		.name = BMA150_G_SENSOR_NAME,
		.dev = {
			.platform_data = &htcleo_g_sensor_pdata,
		},
	},
	{
		.name = "htcleo-backlight",
		.id = -1,
	},
	{
		.name = "htcleo-proximity",
		.id = -1,
	},
	{
		.name = "leds-microp",
		.id = -1,
		.dev = {
			.platform_data = &microp_leds_data,
		},

	},
	{
		.name = "htcleo-lsensor",
		.id = -1,
	},
};

static struct microp_i2c_platform_data microp_data = {
	.num_devices = ARRAY_SIZE(microp_devices),
	.microp_devices = microp_devices,
	.gpio_reset = HTCLEO_GPIO_UP_RESET_N,
};

static struct i2c_board_info base_i2c_devices[] =
{
	{
		// Only a dummy
		I2C_BOARD_INFO(LEO_TOUCH_DRV_NAME, 0x22),
	},
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.platform_data = tps65023_data,
	},
	{
		I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
		.platform_data = &microp_data,
		.irq = MSM_GPIO_TO_INT(HTCLEO_GPIO_UP_INT_N)
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(HTCLEO_GPIO_COMPASS_INT_N),
	},
	{
	        I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1),
	},
};

///////////////////////////////////////////////////////////////////////
// USB
///////////////////////////////////////////////////////////////////////

static unsigned ulpi_on_gpio_table[] = {
	GPIO_CFG(0x68, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),
	GPIO_CFG(0x6f, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x70, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x71, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x72, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x73, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x74, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x75, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x76, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x77, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x78, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x79, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
};

static unsigned ulpi_off_gpio_table[] = {
	GPIO_CFG(0x68, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_4MA),
	GPIO_CFG(0x6f, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x70, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x71, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x72, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x73, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x74, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x75, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x76, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x77, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x78, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x79, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),
};

static void usb_gpio_init(void)
{
	if (gpio_request(0x68, "htcleo_3v3_enable"))
		pr_err("failed to request gpio htcleo_3v3_enable\n");
	if (gpio_request(0x6f, "ulpi_data_0"))
		pr_err("failed to request gpio ulpi_data_0\n");
	if (gpio_request(0x70, "ulpi_data_1"))
		pr_err("failed to request gpio ulpi_data_1\n");
	if (gpio_request(0x71, "ulpi_data_2"))
		pr_err("failed to request gpio ulpi_data_2\n");
	if (gpio_request(0x72, "ulpi_data_3"))
		pr_err("failed to request gpio ulpi_data_3\n");
	if (gpio_request(0x73, "ulpi_data_4"))
		pr_err("failed to request gpio ulpi_data_4\n");
	if (gpio_request(0x74, "ulpi_data_5"))
		pr_err("failed to request gpio ulpi_data_5\n");
	if (gpio_request(0x75, "ulpi_data_6"))
		pr_err("failed to request gpio ulpi_data_6\n");
	if (gpio_request(0x76, "ulpi_data_7"))
		pr_err("failed to request gpio ulpi_data_7\n");
	if (gpio_request(0x77, "ulpi_dir"))
		pr_err("failed to request gpio ulpi_dir\n");
	if (gpio_request(0x78, "ulpi_next"))
		pr_err("failed to request gpio ulpi_next\n");
	if (gpio_request(0x79, "ulpi_stop"))
		pr_err("failed to request gpio ulpi_stop\n");
}

static int usb_config_gpio(int config)
{
	int pin, rc;

	if (config) {
		for (pin = 0; pin < ARRAY_SIZE(ulpi_on_gpio_table); pin++) {			
			rc = gpio_tlmm_config(ulpi_on_gpio_table[pin],
					      GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, ulpi_off_gpio_table[pin], rc);
				return -EIO;
			}
		}
	} else {
		for (pin = 0; pin < ARRAY_SIZE(ulpi_off_gpio_table); pin++) {
			rc = gpio_tlmm_config(ulpi_off_gpio_table[pin],
					      GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, ulpi_on_gpio_table[pin], rc);
				return -EIO;
			}
		}
	}

	return 0;
}

static void usb_phy_shutdown(void)
{
	printk("%s: %s\n", __FILE__, __func__);
	gpio_set_value(HTCLEO_GPIO_USBPHY_3V3_ENABLE, 1); 
	mdelay(3);
	gpio_set_value(HTCLEO_GPIO_USBPHY_3V3_ENABLE, 0);
	mdelay(3);
}

int usb_phy_reset(void  __iomem *regs)
{
	printk("%s: %s\n", __FILE__, __func__);
	usb_phy_shutdown();
	gpio_set_value(HTCLEO_GPIO_USBPHY_3V3_ENABLE, 0); 
	mdelay(3);
	gpio_set_value(HTCLEO_GPIO_USBPHY_3V3_ENABLE, 1);
	mdelay(3);
	usb_config_gpio(1);

	return 0;
}

#define USB_LINK_RESET_TIMEOUT      (msecs_to_jiffies(10))
#define CLKRGM_APPS_RESET_USBH      37
#define CLKRGM_APPS_RESET_USB_PHY   34

#define ULPI_VERIFY_MAX_LOOP_COUNT  3
static void *usb_base;
#ifndef MSM_USB_BASE
#define MSM_USB_BASE              ((unsigned)usb_base)
#endif
static unsigned htcleo_ulpi_read(void __iomem *usb_base, unsigned reg)
{
	unsigned timeout = 100000;

	/* initiate read operation */
	writel(ULPI_RUN | ULPI_READ | ULPI_ADDR(reg),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
		cpu_relax();

	if (timeout == 0) {
		printk(KERN_ERR "ulpi_read: timeout %08x\n",
			readl(USB_ULPI_VIEWPORT));
		return 0xffffffff;
	}
	return ULPI_DATA_READ(readl(USB_ULPI_VIEWPORT));
}

static int htcleo_ulpi_write(void __iomem *usb_base, unsigned val, unsigned reg)
{
	unsigned timeout = 10000;

	/* initiate write operation */
	writel(ULPI_RUN | ULPI_WRITE |
	       ULPI_ADDR(reg) | ULPI_DATA(val),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
		cpu_relax();

	if (timeout == 0) {
		printk(KERN_ERR "ulpi_write: timeout\n");
		return -1;
	}

	return 0;
}

void msm_hsusb_apps_reset_link(int reset)
{
	int ret;
	unsigned usb_id = CLKRGM_APPS_RESET_USBH;

	if (reset)
		ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_ASSERT,
				&usb_id, NULL);
	else
		ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_DEASSERT,
				&usb_id, NULL);
	if (ret)
		printk(KERN_INFO "%s: Cannot set reset to %d (%d)\n",
			__func__, reset, ret);
}
EXPORT_SYMBOL(msm_hsusb_apps_reset_link);

void msm_hsusb_apps_reset_phy(void)
{
	int ret;
	unsigned usb_phy_id = CLKRGM_APPS_RESET_USB_PHY;

	ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_ASSERT,
			&usb_phy_id, NULL);
	if (ret) {
		printk(KERN_INFO "%s: Cannot assert (%d)\n", __func__, ret);
		return;
	}
	msleep(1);
	ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_DEASSERT,
			&usb_phy_id, NULL);
	if (ret) {
		printk(KERN_INFO "%s: Cannot assert (%d)\n", __func__, ret);
		return;
	}
}
EXPORT_SYMBOL(msm_hsusb_apps_reset_phy);

static int msm_hsusb_phy_verify_access(void __iomem *usb_base)
{
	int temp;

	for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
		if (htcleo_ulpi_read(usb_base, ULPI_DEBUG) != (unsigned)-1)
			break;
		msm_hsusb_apps_reset_phy();
	}

	if (temp == ULPI_VERIFY_MAX_LOOP_COUNT) {
		pr_err("%s: ulpi read failed for %d times\n",
				__func__, ULPI_VERIFY_MAX_LOOP_COUNT);
		return -1;
	}

	return 0;
}

static unsigned msm_hsusb_ulpi_read_with_reset(void __iomem *usb_base, unsigned reg)
{
	int temp;
	unsigned res;

	for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
		res = htcleo_ulpi_read(usb_base, reg);
		if (res != -1)
			return res;
		msm_hsusb_apps_reset_phy();
	}

	pr_err("%s: ulpi read failed for %d times\n",
			__func__, ULPI_VERIFY_MAX_LOOP_COUNT);

	return -1;
}

static int msm_hsusb_ulpi_write_with_reset(void __iomem *usb_base,
		unsigned val, unsigned reg)
{
	int temp;
	int res;

	for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
		res = htcleo_ulpi_write(usb_base, val, reg);
		if (!res)
			return 0;
		msm_hsusb_apps_reset_phy();
	}

	pr_err("%s: ulpi write failed for %d times\n",
			__func__, ULPI_VERIFY_MAX_LOOP_COUNT);
	return -1;
}

static int msm_hsusb_phy_caliberate(void __iomem *usb_base)
{
	int ret;
	unsigned res;

	ret = msm_hsusb_phy_verify_access(usb_base);
	if (ret)
		return -ETIMEDOUT;

	res = msm_hsusb_ulpi_read_with_reset(usb_base, ULPI_FUNC_CTRL_CLR);
	if (res == -1)
		return -ETIMEDOUT;

	res = msm_hsusb_ulpi_write_with_reset(usb_base,
			res | ULPI_SUSPENDM,
			ULPI_FUNC_CTRL_CLR);
	if (res)
		return -ETIMEDOUT;

	msm_hsusb_apps_reset_phy();

	return msm_hsusb_phy_verify_access(usb_base);
}

void msm_hsusb_8x50_phy_reset(void)
{
	u32 temp;
	unsigned long timeout;
	int ret, usb_phy_error;
	printk(KERN_INFO "msm_hsusb_phy_reset\n");
	usb_base = ioremap(MSM_HSUSB_PHYS, 4096);

	msm_hsusb_apps_reset_link(1);
	msm_hsusb_apps_reset_phy();
	msm_hsusb_apps_reset_link(0);

	/* select ULPI phy */
	temp = (readl(USB_PORTSC) & ~PORTSC_PTS);
	writel(temp | PORTSC_PTS_ULPI, USB_PORTSC);

	if ((ret = msm_hsusb_phy_caliberate(usb_base))) {
		usb_phy_error = 1;
		pr_err("msm_hsusb_phy_caliberate returned with %i\n", ret);
		return;
	}

	/* soft reset phy */
	writel(USBCMD_RESET, USB_USBCMD);
	timeout = jiffies + USB_LINK_RESET_TIMEOUT;
	while (readl(USB_USBCMD) & USBCMD_RESET) {
		if (time_after(jiffies, timeout)) {
			pr_err("usb link reset timeout\n");
			break;
		}
		msleep(1);
	}
	usb_phy_error = 0;

	return;
}

#if 0
static struct msm_otg_platform_data msm_otg_pdata = {
	.phy_reset		= msm_hsusb_8x50_phy_reset,
	.pemp_level		= PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		= CDR_AUTO_RESET_DISABLE,
	.drv_ampl		= HS_DRV_AMPLITUDE_DEFAULT,
	.se1_gating		= SE1_GATING_DISABLE,
};
#else
static int htcleo_phy_init_seq[] ={0x0C, 0x31, 0x30, 0x32, 0x1D, 0x0D, 0x1D, 0x10, -1};

static struct msm_otg_platform_data msm_otg_pdata = {
	.phy_init_seq		= htcleo_phy_init_seq,
	.mode			= USB_PERIPHERAL,
	.otg_control		= OTG_PHY_CONTROL,
};
#endif

#if 0
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
};
#endif

static uint32_t usb_phy_3v3_table[] =
{
    PCOM_GPIO_CFG(HTCLEO_GPIO_USBPHY_3V3_ENABLE, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA)
};
#if 0

// modified to further reflect bravo kernel code
static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq		= htcleo_phy_init_seq,
	.phy_reset		= msm_hsusb_8x50_phy_reset,
	.accessory_detect = 0, /* detect by ID pin gpio */
};
#endif
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "HTC",
	.product	= "HD2",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	.vendorID	= 0x0bb4,
	.vendorDescr	= "HTC",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};
#endif

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0bb4,
	.product_id	= 0x0c02,
	.version	= 0x0100,
	.product_name		= "HD2",
	.manufacturer_name	= "HTC",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
static void htcleo_add_usb_devices(void)
{
#if 0
	android_usb_pdata.products[0].product_id =
		android_usb_pdata.product_id;
	android_usb_pdata.serial_number = board_serialno();
	msm_hsusb_pdata.serial_number = board_serialno();
	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	config_gpio_table(usb_phy_3v3_table, ARRAY_SIZE(usb_phy_3v3_table));
	gpio_set_value(HTCLEO_GPIO_USBPHY_3V3_ENABLE, 1);
	platform_device_register(&msm_device_hsusb);
	platform_device_register(&usb_mass_storage_device);
#ifdef CONFIG_USB_ANDROID_RNDIS
	platform_device_register(&rndis_device);
#endif
#endif
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	//msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
	msm_device_gadget_peripheral.dev.parent = &msm_device_otg.dev;
	usb_gpio_init();
	platform_device_register(&msm_device_gadget_peripheral);
	platform_device_register(&android_usb_device);
}

unsigned htcleo_get_vbus_state(void)
{
	if(readl(MSM_SHARED_RAM_BASE+0xef20c))
		return 1;
	else
		return 0;
}


///////////////////////////////////////////////////////////////////////
// Flashlight
///////////////////////////////////////////////////////////////////////

static uint32_t flashlight_gpio_table[] =
{
	PCOM_GPIO_CFG(HTCLEO_GPIO_FLASHLIGHT_TORCH, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_FLASHLIGHT_FLASH, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
};

static void config_htcleo_flashlight_gpios(void)
{
	config_gpio_table(flashlight_gpio_table, ARRAY_SIZE(flashlight_gpio_table));
}

static struct flashlight_platform_data htcleo_flashlight_data =
{
	.gpio_init  = config_htcleo_flashlight_gpios,
	.torch = HTCLEO_GPIO_FLASHLIGHT_TORCH,
	.flash = HTCLEO_GPIO_FLASHLIGHT_FLASH,
	.flash_duration_ms = 600
};

static struct platform_device htcleo_flashlight_device =
{
	.name = "flashlight",
	.dev =
	{
		.platform_data  = &htcleo_flashlight_data,
	},
};

///////////////////////////////////////////////////////////////////////
// Camera
///////////////////////////////////////////////////////////////////////

static uint32_t camera_off_gpio_table[] =
{
	PCOM_GPIO_CFG(0, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* HSYNC */
	PCOM_GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* VSYNC */
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] =
{
	PCOM_GPIO_CFG(0, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_16MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* HSYNC */
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* VSYNC */
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* MCLK */
};

int config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table, ARRAY_SIZE(camera_on_gpio_table));

	return 0;
}

void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table, ARRAY_SIZE(camera_off_gpio_table));
}

static struct resource msm_camera_resources[] =
{
	{
		.start	= MSM_VFE_PHYS,
		.end	= MSM_VFE_PHYS + MSM_VFE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		 INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_camera_device_platform_data msm_camera_device_data =
{
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static int flashlight_control(int mode)
{
        return aat1271_flashlight_control(mode);
}

static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
	.camera_flash		= flashlight_control,
	.num_flash_levels	= FLASHLIGHT_NUM,
	.low_temp_limit		= 5,
	.low_cap_limit		= 15,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data =
{
	.sensor_name = "s5k3e2fx",
	.sensor_reset = 144,
	/* CAM1_PWDN, enabled in a9 */
	//.sensor_pwd = 143,
	/* CAM1_VCM_EN, enabled in a9 */
	//.vcm_pwd = 31,
	.pdata = &msm_camera_device_data,
	.flash_type = MSM_CAMERA_FLASH_LED,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
	.flash_cfg = &msm_camera_sensor_flash_cfg,
};

static struct platform_device msm_camera_sensor_s5k3e2fx =
{
	.name     = "msm_camera_s5k3e2fx",
	.dev      = {
		.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};

///////////////////////////////////////////////////////////////////////
// bluetooth
///////////////////////////////////////////////////////////////////////

/* AOSP style interface */

/*
 * bluetooth mac address will be parsed in msm_nand_probe
 * see drivers/mtd/devices/htcleo_nand.c
 */
char bdaddr[BDADDR_STR_SIZE];

module_param_string(bdaddr, bdaddr, sizeof(bdaddr), 0400);
MODULE_PARM_DESC(bdaddr, "bluetooth address");
/* end AOSP style interface */

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.rx_wakeup_irq = -1,
	.inject_rx_on_wakeup = 0,
#ifdef CONFIG_SERIAL_BCM_BT_LPM
	.exit_lpm_cb = bcm_bt_lpm_exit_lpm_locked,
#endif
};

#ifdef CONFIG_SERIAL_BCM_BT_LPM
static struct bcm_bt_lpm_platform_data bcm_bt_lpm_pdata = {
	.gpio_wake = HTCLEO_GPIO_BT_CHIP_WAKE,
	.gpio_host_wake = HTCLEO_GPIO_BT_HOST_WAKE,
	.request_clock_off_locked = msm_hs_request_clock_off,
	.request_clock_on_locked = msm_hs_request_clock_on_locked,
};

struct platform_device bcm_bt_lpm_device = {
	.name = "bcm_bt_lpm",
	.id = 0,
	.dev = {
		.platform_data = &bcm_bt_lpm_pdata,
	},
};
#endif
#endif

static uint32_t bt_gpio_table[] = {
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_UART1_RTS, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_UART1_CTS, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_UART1_RX, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_UART1_TX, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_RESET_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_SHUTDOWN_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_CHIP_WAKE, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_HOST_WAKE, 0, GPIO_INPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
};

static struct platform_device htcleo_rfkill =
{
	.name = "htcleo_rfkill",
	.id = -1,
};

///////////////////////////////////////////////////////////////////////
// PM Platform data
///////////////////////////////////////////////////////////////////////

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 8594,
		.residency = 23740,
	},

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 4594,
		.residency = 23740,
	},

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 0,
		.suspend_enabled = 1,
		.latency = 443,
		.residency = 1098,
	},

	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 2,
		.residency = 0,
	},
};

///////////////////////////////////////////////////////////////////////
// SPI
///////////////////////////////////////////////////////////////////////

static struct resource qsd_spi_resources[] = {
	{
		.name   = "spi_irq_in",
		.start  = INT_SPI_INPUT,
		.end    = INT_SPI_INPUT,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start  = INT_SPI_OUTPUT,
		.end    = INT_SPI_OUTPUT,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start  = INT_SPI_ERROR,
		.end    = INT_SPI_ERROR,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_base",
		.start  = 0xA1200000,
		.end    = 0xA1200000 + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "spi_clk",
		.start  = 17,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_mosi",
		.start  = 18,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_miso",
		.start  = 19,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_cs0",
		.start  = 20,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_pwr",
		.start  = 21,
		.end    = 0,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_cs0",
		.start  = 22,
		.end    = 0,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct spi_platform_data htcleo_spi_pdata = {
	.clk_rate	= 4800000,
};

static struct platform_device qsd_device_spi = {
	.name           = "spi_qsd",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(qsd_spi_resources),
	.resource       = qsd_spi_resources,
	.dev		= {
		.platform_data = &htcleo_spi_pdata
	},
};

///////////////////////////////////////////////////////////////////////
// KGSL (HW3D support)#include <linux/android_pmem.h>
///////////////////////////////////////////////////////////////////////

/* start kgsl */
static struct resource kgsl_3d0_resources[] = {
	{
		.name  = KGSL_3D0_REG_MEMORY,
		.start = 0xA0000000,
		.end = 0xA001ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = KGSL_3D0_IRQ,
		.start = INT_GRAPHICS,
		.end = INT_GRAPHICS,
		.flags = IORESOURCE_IRQ,
	},
};

static struct kgsl_device_platform_data kgsl_3d0_pdata = {
	.pwr_data = {
		.pwrlevel = {
			{
				.gpu_freq = 0,
				.bus_freq = 128000000,
			},
		},
		.init_level = 0,
		.num_levels = 1,
		.set_grp_async = NULL,
		.idle_timeout = HZ/5,
	},
	.clk = {
		.name = {
			.clk = "core_clk",
		},
	},
	.imem_clk_name = {
		.clk = "iface_clk",
	},
};

struct platform_device msm_kgsl_3d0 = {
	.name = "kgsl-3d0",
	.id = 0,
	.num_resources = ARRAY_SIZE(kgsl_3d0_resources),
	.resource = kgsl_3d0_resources,
	.dev = {
		.platform_data = &kgsl_3d0_pdata,
	},
};
/* end kgsl */

/* start footswitch regulator */
struct platform_device *msm_footswitch_devices[] = {
	FS_PCOM(FS_GFX3D,  "fs_gfx3d"),
};
unsigned msm_num_footswitch_devices = ARRAY_SIZE(msm_footswitch_devices);
/* end footswitch regulator */

///////////////////////////////////////////////////////////////////////
// Memory
///////////////////////////////////////////////////////////////////////
#define MSM_AUDIO_SIZE		0x80000

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION

static struct android_pmem_platform_data android_pmem_kernel_smi_pdata = {
	.name = PMEM_KERNEL_SMI_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
	.cached = 0,
};

#endif

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct android_pmem_platform_data android_pmem_venc_pdata = {
	.name = "pmem_venc",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
static struct platform_device android_pmem_kernel_smi_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_kernel_smi_pdata },
};
#endif

static struct platform_device android_pmem_venc_device = {
	.name = "android_pmem",
	.id = 5,
	.dev = { .platform_data = &android_pmem_venc_pdata },
};

///////////////////////////////////////////////////////////////////////
// RAM-Console
///////////////////////////////////////////////////////////////////////

static struct resource ram_console_resources[] = {
	{
		.start	= (resource_size_t) MSM_RAM_CONSOLE_BASE,
		.end	= (resource_size_t) (MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1),
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name		= "ram_console",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
};
///////////////////////////////////////////////////////////////////////
// Power/Battery
///////////////////////////////////////////////////////////////////////

int htcleo_support_super_charger(void)
{
	return 1;
}

static struct htc_battery_platform_data htc_battery_pdev_data = {
	.func_show_batt_attr = htc_battery_show_attr,
	.gpio_mbat_in = -1,
	.gpio_mchg_en_n = HTCLEO_GPIO_BATTERY_CHARGER_ENABLE,
	.gpio_iset = HTCLEO_GPIO_BATTERY_CHARGER_CURRENT,
	.gpio_adp_9v = HTCLEO_GPIO_POWER_USB,
	.guage_driver = GUAGE_DS2746,
	.charger = LINEAR_CHARGER,
	.m2a_cable_detect = 0,
//	.force_no_rpc = 1,
	.func_is_support_super_charger = htcleo_support_super_charger,
/*	.int_data = {
		.chg_int = HTCLEO_GPIO_BATTERY_OVER_CHG,
	},*/
};

static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id = -1,
	.dev	= {
		.platform_data = &htc_battery_pdev_data,
	},
};

#ifdef CONFIG_BATTERY_DS2746
static int get_thermal_id(void)
{
	return THERMAL_300_47_3440;
}

static int get_battery_id(void)
{
	return BATTERY_ID_SANYO_1300MAH_TWS;
}

/* battery parameters */
/*! star_lee 20100426 - update KADC discharge parameter */
uint32_t m_parameter_sony_1300mah_formosa[] =
{
  /* capacity (in 0.01%) -> voltage (in mV)*/
  10000, 4100, 5500, 3839, 2400, 3759, 400, 3667, 0, 3397,
};

uint32_t m_parameter_default[] =
{
  /* capacity (in 0.01%) -> voltage (in mV)*/
  10000, 4135, 7500, 3960, 4700, 3800, 1700, 3727, 900, 3674, 300, 3640, 0, 3420,
};

uint32_t m_parameter_samsung_1230mah_formosa[] =
{
  /* capacity (in 0.01%) -> voltage (in mV)*/
  10000, 4135, 7500, 3960, 4700, 3800, 1700, 3727, 900, 3674, 300, 3640, 0, 3420,
};

uint32_t m_parameter_htc_2300mah_formosa[] =
{
  /* capacity (in 0.01%) -> voltage (in mV)*/
  10000, 4135, 7500, 3960, 4700, 3800, 1700, 3727, 900, 3674, 300, 3640, 0, 3420,
};

static uint32_t* m_param_tbl[] = {
	m_parameter_sony_1300mah_formosa,
	m_parameter_sony_1300mah_formosa,
	m_parameter_sony_1300mah_formosa,
	m_parameter_sony_1300mah_formosa,
	m_parameter_samsung_1230mah_formosa,
	m_parameter_htc_2300mah_formosa
};

static uint32_t fl_25[] = {
	2300, /* Unknown battery */
	1280, /* Sony 1300mAh (Formosa) */
	1280, /* Sony 1300mAh (HTE) */
	1250, /* Sanyo 1300mAh (HTE) */
	1230, /* Samsung 1230mAh */
	2300, /* HTC Extended 2300mAh */
};

static uint32_t pd_m_coef[] = {
	24, /* Unknown battery */
	24, /* Sony 1300mAh (Formosa) */
	24, /* Sony 1300mAh (HTE) */
	27, /* Sanyo 1300mAh (HTE) */
	30, /* Samsung 1230mAh */
	30, /* HTC Extended 2300mAh */ 
};

static uint32_t pd_m_resl[] = {
	100, /* Unknown battery */
	100, /* Sony 1300mAh (Formosa) */
	100, /* Sony 1300mAh (HTE) */
	100, /* Sanyo 1300mAh (HTE) */
	100, /* Samsung 1230mAh */
	100, /* HTC Extended 2300mAh */ 
};

static uint32_t pd_t_coef[] = {
	/* Ex: 140 -> 0.014, 156 -> 0.0156*/
	140, /* Unknown battery */
	140, /* Sony 1300mAh (Formosa) */
	140, /* Sony 1300mAh (HTE) */
	156, /* Sanyo 1300mAh (HTE) */
	250, /* Samsung 1230mAh */
	250, /* HTC Extended 2300mAh */ 
};

static int32_t padc[] = {
	/* mapping temp to temp_index*/
	200, /* ~20C */
	100, /* 20~10C  */
	50, /* 10~5C */
	0, /* 5~0C */
	-5, /* 0~-5C */
	-10, /* -5~-10C */
	-3000, /* -10C~ */
};

// PW is always 5 for DS2746
static int32_t pw[] = {
	5,
	5,
	5,
	5,
	5,
	5,
};

static uint32_t* pd_m_coef_tbl[] = {pd_m_coef,};
static uint32_t* pd_m_resl_tbl[] = {pd_m_resl,};
static uint32_t capacity_deduction_tbl_01p[] = {
	0, /* ~20C,    upper capacity 100 is usable */
	0, /* 20~10C,  upper capacity 95 is usable */
	0, /* 10~5C,   upper capacity 92 is usable */
	0, /* 5~0C,	upper capacity 90 is usable */
	0, /* 0~-5C,   upper capacity 87 is usable */
	0, /* -5~-10C, upper capacity 85 is usable */
	0, /* -10C~,   upper capacity 85 is usable */
};

static struct battery_parameter htcleo_battery_parameter = {
	.fl_25 = fl_25,
	.pd_m_coef_tbl = pd_m_coef_tbl,
	.pd_m_coef_tbl_boot = pd_m_coef_tbl,
	.pd_m_resl_tbl = pd_m_resl_tbl,
	.pd_m_resl_tbl_boot = pd_m_resl_tbl,
	.pd_t_coef = pd_t_coef,
	.padc = padc,
	.pw = pw,
	.capacity_deduction_tbl_01p = capacity_deduction_tbl_01p,
	.id_tbl = NULL,
	.temp_index_tbl = NULL,
	.m_param_tbl = m_param_tbl,
	.m_param_tbl_size = sizeof(m_param_tbl)/sizeof(uint32_t*),
};

static ds2746_platform_data ds2746_pdev_data = {
	.func_get_thermal_id = get_thermal_id,
	.func_get_battery_id = get_battery_id,
	.func_poweralg_config_init = NULL,	/* by default */
	.func_update_charging_protect_flag = NULL,	/* by default */
	.r2_kohm = 0,	/* use get_battery_id, doesn't need this */
	.batt_param = &htcleo_battery_parameter,
};

static struct platform_device ds2746_battery_pdev = {
	.name = "ds2746-battery",
	.id = -1,
	.dev = {
		.platform_data = &ds2746_pdev_data,
	},
};
#endif

///////////////////////////////////////////////////////////////////////
// Real Time Clock
///////////////////////////////////////////////////////////////////////

struct platform_device msm_device_rtc = {
	.name = "msm_rtc",
	.id = -1,
};

///////////////////////////////////////////////////////////////////////
// Button backlight manager
///////////////////////////////////////////////////////////////////////
#ifdef CONFIG_HTCLEO_BTN_BACKLIGHT_MANAGER
struct platform_device btn_backlight_manager = {
    .name   = "btn_backlight_manager",
    .id     = -1,
};
#endif

///////////////////////////////////////////////////////////////////////
// Platform Devices
///////////////////////////////////////////////////////////////////////


static struct platform_device *devices[] __initdata =
{
	&ram_console_device,
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart1,
#endif
#ifdef CONFIG_SERIAL_BCM_BT_LPM
	&bcm_bt_lpm_device,
#endif
#ifdef CONFIG_720P_CAMERA
    //&android_pmem_venc_device,
#endif
	&msm_device_uart_dm1,
	&htcleo_rfkill,
	&msm_device_otg,
//	&msm_device_gadget_peripheral,
	&qsd_device_spi,
	&msm_device_dmov,
	&msm_device_nand,
	&msm_device_smd,
	&msm_device_rtc,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_venc_device,
	&msm_device_i2c,
	&htc_battery_pdev,
	&ds2746_battery_pdev,
	&msm_kgsl_3d0,
	&msm_camera_sensor_s5k3e2fx,
	&htcleo_flashlight_device,
	&htc_headset_mgr,
	&htc_headset_gpio,
#ifdef CONFIG_HTCLEO_BTN_BACKLIGHT_MANAGER
	&btn_backlight_manager,
#endif
};
///////////////////////////////////////////////////////////////////////
// Vibrator
///////////////////////////////////////////////////////////////////////

static struct timed_gpio timed_gpios[] = {
	{
		.name = "vibrator",
		.gpio = HTCLEO_GPIO_VIBRATOR_ON,
		.max_timeout = 15000,
	},
};

static struct timed_gpio_platform_data timed_gpio_data = {
	.num_gpios	= ARRAY_SIZE(timed_gpios),
	.gpios		= timed_gpios,
};

static struct platform_device htcleo_timed_gpios = {
	.name		= "timed-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &timed_gpio_data,
	},
};
///////////////////////////////////////////////////////////////////////
// I2C
///////////////////////////////////////////////////////////////////////

// Cotulla: old definition can not be used with new msm i2c driver
#if 0
static struct msm_i2c_device_platform_data msm_i2c_pdata = {
	.i2c_clock = 400000,
	.clock_strength = GPIO_8MA,
	.data_strength = GPIO_8MA,
};
#else

#define GPIO_I2C_CLK 95
#define GPIO_I2C_DAT 96

static void msm_i2c_gpio_config(int adap_id, int config_type)
{
	unsigned id;


	if (adap_id > 0) return;

	if (config_type == 0) 
	{
		id = GPIO_CFG(GPIO_I2C_CLK, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		id = GPIO_CFG(GPIO_I2C_DAT, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	} 
	else 
	{
		id = GPIO_CFG(GPIO_I2C_CLK, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_8MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		id = GPIO_CFG(GPIO_I2C_DAT , 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_8MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}


static struct msm_i2c_platform_data msm_i2c_pdata = 
{
	.clk_freq = 400000,
	.pri_clk = GPIO_I2C_CLK,
	.pri_dat = GPIO_I2C_DAT,
	.rmutex  = 0,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

#endif

static void __init msm_device_i2c_init(void)
{
	msm_i2c_gpio_init();
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

///////////////////////////////////////////////////////////////////////
// Clocks
///////////////////////////////////////////////////////////////////////
/*
static struct msm_acpu_clock_platform_data htcleo_clock_data = {
	.acpu_switch_time_us	= 20,
	.max_speed_delta_khz	= 256000,
	.vdd_switch_time_us	= 62,
	.power_collapse_khz	= 128000,
	.wait_for_irq_khz	= 128000,
//	.wait_for_irq_khz	= 19200,   // TCXO
};
*/
static unsigned htcleo_perf_acpu_table[] = {
	245000000,
	576000000,
	998400000,
};

static struct perflock_platform_data htcleo_perflock_data = {
	.perf_acpu_table = htcleo_perf_acpu_table,
	.table_size = ARRAY_SIZE(htcleo_perf_acpu_table),
};
///////////////////////////////////////////////////////////////////////
// Reset
///////////////////////////////////////////////////////////////////////

static void htcleo_reset(void)
{
	// 25 - 16 = 9
	while (1)
	{
	        writel(readl(MSM_GPIOCFG2_BASE + 0x504) | (1 << 9), MSM_GPIOCFG2_BASE + 0x504);// owner
		gpio_set_value(HTCLEO_GPIO_PS_HOLD, 0);
	}
}

static void do_grp_reset(void)
{
   	writel(0x20000, MSM_CLK_CTL_BASE + 0x214);
}

static void do_sdc1_reset(void)
{
	volatile uint32_t* sdc1_clk = MSM_CLK_CTL_BASE + 0x218;

	*sdc1_clk |= (1 << 9);
   	mdelay(1);
	*sdc1_clk &= ~(1 << 9);
}

#ifdef CONFIG_HTCLEO_BLINK_ON_BOOT
static void __init htcleo_blink_camera_led(void){
	volatile unsigned *bank6_in, *bank6_out;
	bank6_in = (unsigned int*)(MSM_GPIO1_BASE + 0x0864);
	bank6_out = (unsigned int*)(MSM_GPIO1_BASE + 0x0814);
	*bank6_out = *bank6_in ^ 0x200000;
	mdelay(50);
	*bank6_out = *bank6_in | 0x200000;
	mdelay(200);
}
#endif // CONFIG_HTCLEO_BLINK_ON_BOOT

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static int __init pmem_mdp_size_setup(char *p)
{
	pmem_mdp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_mdp_size", pmem_mdp_size_setup);

static unsigned pmem_venc_size = MSM_PMEM_VENC_SIZE;
static int __init pmem_venc_size_setup(char *p)
{
	pmem_venc_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_venc_size", pmem_venc_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static struct memtype_reserve qsd8x50_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init size_pmem_device(struct android_pmem_platform_data *pdata, unsigned long start, unsigned long size)
{
	pdata->size = size;
	pr_info("%s: pmem %s requests %lu bytes dynamically.\n",
			__func__, pdata->name, size);
}

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
	size_pmem_device(&android_pmem_adsp_pdata, 0, pmem_adsp_size);
	size_pmem_device(&android_pmem_pdata, 0, pmem_mdp_size);
	size_pmem_device(&android_pmem_venc_pdata, 0, pmem_venc_size);
	qsd8x50_reserve_table[MEMTYPE_EBI1].size += PMEM_KERNEL_EBI1_SIZE;
#endif
}



static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	//qsd8x50_reserve_table[p->memory_type].size += p->size;
	pr_info("%s: reserve %lu bytes from memory %d for %s.\n", __func__, p->size, p->memory_type, p->name);
	qsd8x50_reserve_table[p->memory_type].size += p->size;
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_pdata);
#endif
}


static void __init qsd8x50_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
}

static int qsd8x50_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

static struct reserve_info qsd8x50_reserve_info __initdata = {
	.memtype_reserve_table = qsd8x50_reserve_table,
	.calculate_reserve_sizes = qsd8x50_calculate_reserve_sizes,
	.paddr_to_memtype = qsd8x50_paddr_to_memtype,
};

static void __init qsd8x50_reserve(void)
{
	reserve_info = &qsd8x50_reserve_info;
	msm_reserve();
}

///////////////////////////////////////////////////////////////////////
// Init
///////////////////////////////////////////////////////////////////////

static void __init htcleo_init(void)
{
// Cotulla vibro test
//	*(uint32_t*)0xF800380C |= 0x20;

	printk("htcleo_init()\n");
	msm_hw_reset_hook = htcleo_reset;

	do_grp_reset();
	do_sdc1_reset();
	msm_clock_init(&qds8x50_clock_init_data);
	acpuclk_init(&acpuclk_8x50_soc_data);
	
	init_dex_comm();

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
	msm_device_uart_dm1.name = "msm_serial_hs_brcm"; /* for bcm */
	msm_device_uart_dm1.resource[3].end = 6;
#endif

	config_gpio_table(bt_gpio_table, ARRAY_SIZE(bt_gpio_table));

	htcleo_audio_init();

	msm_device_i2c_init();

	platform_add_devices(devices, ARRAY_SIZE(devices));

	platform_add_devices(msm_footswitch_devices,
			msm_num_footswitch_devices);

	htcleo_init_panel();

#ifdef CONFIG_USB_G_ANDROID
	htcleo_add_usb_devices();
#endif
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));

	i2c_register_board_info(0, base_i2c_devices, ARRAY_SIZE(base_i2c_devices));
	
	htcleo_init_mmc(0);
	platform_device_register(&htcleo_timed_gpios);
	
#ifdef CONFIG_HTCLEO_BLINK_ON_BOOT
	/* Blink the camera LED shortly to show that we're alive! */
	htcleo_blink_camera_led();
#endif // CONFIG_HTCLEO_BLINK_ON_BOOT

}

///////////////////////////////////////////////////////////////////////
// Bootfunctions
///////////////////////////////////////////////////////////////////////

static void __init htcleo_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	/* Blink the camera LED shortly to show that we're alive! */
	mi->nr_banks = 1;
	mi->bank[0].start = MSM_EBI1_BANK0_BASE;
	//mi->bank[0].node = PHYS_TO_NID(MSM_EBI1_BANK0_BASE);
	mi->bank[0].size = MSM_EBI1_BANK0_SIZE;
}

#if defined(CONFIG_VERY_EARLY_CONSOLE)
#if defined(CONFIG_HTC_FB_CONSOLE)
int __init htc_fb_console_init(void);
#endif
#if defined(CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT)
int __init ram_console_early_init(void);
#endif
#endif

static void __init htcleo_map_io(void)
{
	msm_map_qsd8x50_io();
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",__func__);
	
#if defined(CONFIG_VERY_EARLY_CONSOLE)
// Init our consoles _really_ early
#if defined(CONFIG_HTC_FB_CONSOLE)
	htc_fb_console_init();
#endif
#if defined(CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT)
	ram_console_early_init();
#endif
#endif
	printk(KERN_ERR "%s: ramconsole init done!\n",__func__);
//	*(uint32_t*)0xF800380C |= 0x20;
}

extern struct sys_timer msm_timer;

MACHINE_START(HTCLEO, "htcleo")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= (CONFIG_PHYS_OFFSET + 0x00000100),
	.fixup		= htcleo_fixup,
	.map_io		= htcleo_map_io,
    	.reserve	= qsd8x50_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= htcleo_init,
	.timer		= &msm_timer,
MACHINE_END

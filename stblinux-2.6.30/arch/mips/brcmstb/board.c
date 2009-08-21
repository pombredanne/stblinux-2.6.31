/*
 * Copyright (C) 2009 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/autoconf.h>
#include <linux/smp.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mtd/partitions.h>

#include <asm/addrspace.h>
#include <asm/page.h>
#include <asm/r4kcache.h>
#include <asm/brcmstb/brcmstb.h>

/* board features */
int brcm_docsis_platform = 0;
int brcm_enet_no_mdio = 0;
char brcm_cfe_boardname[COMMAND_LINE_SIZE];

/* MTD partition layout */
unsigned long brcm_mtd_rootfs_start = 0;
unsigned long brcm_mtd_rootfs_len = 0;
unsigned long brcm_mtd_kernel_start = 0;
unsigned long brcm_mtd_kernel_len = 0;
unsigned long brcm_mtd_ocap_start = 0;
unsigned long brcm_mtd_ocap_len = 0;
unsigned long brcm_mtd_flash_size_mb = 0;
char brcm_mtd_flash_type[COMMAND_LINE_SIZE];

/* MoCA 3450 I2C port */
unsigned long brcm_moca_i2c_base = 0;

#ifdef CONFIG_BRCM_HAS_PCI23

/***********************************************************************
 * PCI 2.3 slot->interrupt mappings
 ***********************************************************************/

#define PCI_A0		BRCM_IRQ_PCI_A0
#define PCI_A1		BRCM_IRQ_PCI_A1
#define PCI_A2		BRCM_IRQ_PCI_A2

#define NUM_SLOTS	16

/* Note for supporting customized boards: slot_number = idsel_line - 16 */

#if   defined(CONFIG_BCM7038)
char irq_tab_brcmstb[NUM_SLOTS][4] __devinitdata = {
	[1]  =	{  PCI_A1,	0,	0,	0	}, /* BCM325x */
	[4]  =	{  PCI_A2,	0,	0,	0	}, /* 7042 */
	[13] =	{  PCI_A1,	0,	0,	0	}, /* EXT PCI */
	[14] =	{  PCI_A0,	0,	0,	0	}, /* 1394 */
};
#elif defined(CONFIG_BCM7401)
char irq_tab_brcmstb[NUM_SLOTS][4] __devinitdata = {
	[4]  =	{  PCI_A2, PCI_A0,	0,	0	}, /* mPCI */
	[13] =	{  PCI_A0, PCI_A2,	0,	0	}, /* EXT PCI */
	[14] =	{  PCI_A1,	0,	0,	0	}, /* 1394 */
};
#else
char irq_tab_brcmstb[NUM_SLOTS][4] __devinitdata = {
	[4]  =	{  PCI_A2, PCI_A0,	0,	0	}, /* mPCI */
	[13] =	{  PCI_A0, PCI_A1,	0,	0	}, /* EXT PCI */
	[14] =	{  PCI_A1,	0,	0,	0	}, /* 1394 */
};
#endif

char irq_tab_brcmstb_docsis[NUM_SLOTS][4] __devinitdata = {
	[4]  =	{  PCI_A2, PCI_A0,	0,	0	}, /* mPCI */
	[7]  =	{  PCI_A1,	0,	0,	0	}, /* BCM325x */
	[13] =	{  PCI_A0, PCI_A1,	0,	0	}, /* EXT PCI */
	[14] =	{  PCI_A2,	0,	0,	0	}, /* 1394 */
};

#endif

/***********************************************************************
 * PIN_MUX setup
 *
 * NOTE: This code assumes that the bootloader set up the pinmux for
 * the primary UART (almost always UARTA) and PCI/EBI.
 ***********************************************************************/

#define PINMUX(reg, field, val) do { \
	BDEV_WR(BCHP_SUN_TOP_CTRL_PIN_MUX_CTRL_##reg, \
		(BDEV_RD(BCHP_SUN_TOP_CTRL_PIN_MUX_CTRL_##reg) & \
		 ~BCHP_SUN_TOP_CTRL_PIN_MUX_CTRL_##reg##_##field##_MASK) | \
		((val) << \
		 BCHP_SUN_TOP_CTRL_PIN_MUX_CTRL_##reg##_##field##_SHIFT)); \
	} while(0)

void __init board_pinmux_setup(void)
{
#if ! defined(CONFIG_BRCM_IKOS)
#if   defined(CONFIG_BCM3548B0)

	PINMUX(6, gpio_30, 2);		// UARTB TX
	PINMUX(6, gpio_31, 2);		// UARTB RX
	PINMUX(7, gpio_43, 2);		// UARTC TX
	PINMUX(7, gpio_42, 2);		// UARTC RX

	PINMUX(6, gpio_32, 2);		// SPI
	PINMUX(6, gpio_33, 2);
	PINMUX(6, gpio_34, 2);
	PINMUX(6, gpio_35, 2);
	PINMUX(6, gpio_36, 2);

#elif defined(CONFIG_BCM3563C0)

	/* UARTB RX requires board mod; UARTC is on RS232 daughtercard */
	PINMUX(10, gpio_47, 1);		// UARTB TX
	PINMUX(10, gpio_46, 2);		// UARTB RX
	PINMUX(7, gpio_17, 1);		// UARTC TX
	PINMUX(7, gpio_16, 1);		// UARTC RX

#elif defined(CONFIG_BCM7038C0)

	PINMUX(9, gpio_47, 2);		// UARTB TX
	PINMUX(9, gpio_46, 2);		// UARTB RX

#elif defined(CONFIG_BCM7118C0)

	PINMUX(1, uart_txdb, 0);	// UARTB TX
	PINMUX(1, uart_rxdb, 0);	// UARTB RX
	PINMUX(11, gpio_54, 2);		// UARTC TX
	PINMUX(11, gpio_55, 2);		// UARTC RX

#elif defined(CONFIG_BCM7325B0)

	PINMUX(11, uart_txdb, 0);	// UARTB TX
	PINMUX(11, uart_rxdb, 0);	// UARTB RX

#if defined(CONFIG_BCMEMAC_EXTMII)
	PINMUX(5, gpio_32, 1);		// MII
	PINMUX(5, gpio_33, 1);
	PINMUX(5, gpio_34, 1);
	PINMUX(5, gpio_35, 1);
	PINMUX(5, gpio_36, 1);
	PINMUX(5, gpio_37, 1);
	PINMUX(6, gpio_38, 1);
	PINMUX(6, gpio_39, 1);
	PINMUX(6, gpio_40, 1);
	PINMUX(6, gpio_41, 1);
	PINMUX(6, gpio_42, 1);
	PINMUX(6, gpio_43, 1);
	PINMUX(6, gpio_44, 1);
	PINMUX(6, gpio_45, 1);
	PINMUX(6, gpio_46, 1);
	PINMUX(6, gpio_47, 1);
	PINMUX(7, gpio_48, 1);
	PINMUX(7, gpio_49, 1);
#endif

#elif defined(CONFIG_BCM7335B0)

	PINMUX(7, gpio_034, 1);		// UARTB TX
	PINMUX(7, gpio_035, 1);		// UARTB RX
	PINMUX(7, gpio_038, 1);		// UARTC TX
	PINMUX(7, gpio_039, 1);		// UARTC RX

	PINMUX(9, gpio_054, 3);		// MII
	PINMUX(9, gpio_055, 3);
	PINMUX(9, gpio_056, 3);
	PINMUX(9, gpio_057, 3);
	PINMUX(9, gpio_058, 3);
	PINMUX(9, gpio_059, 3);
	PINMUX(9, gpio_060, 3);
	PINMUX(9, gpio_061, 3);
	PINMUX(9, gpio_062, 3);
	PINMUX(10, gpio_063, 3);
	PINMUX(10, gpio_065, 3);
	PINMUX(10, gpio_066, 3);
	PINMUX(10, gpio_067, 3);
	PINMUX(10, gpio_068, 3);
	PINMUX(10, gpio_069, 3);
	PINMUX(10, gpio_070, 3);
	PINMUX(10, gpio_071, 3);
	PINMUX(10, gpio_072, 3);

#elif defined(CONFIG_BCM7400D0)

	PINMUX(3, gpio_008, 2);		// UARTB TX
	PINMUX(3, gpio_007, 2);		// UARTB RX
	PINMUX(3, gpio_012, 2);		// UARTC TX
	PINMUX(3, gpio_011, 2);		// UARTC RX

	/* CFE forgot to set these */
	PINMUX(2, gpio_000, 1);		// enet_activity
	PINMUX(2, gpio_001, 1);		// enet_link

#elif defined(CONFIG_BCM7401C0)

	PINMUX(11, gpio_49, 1);		// UARTA TX
	PINMUX(11, gpio_50, 1);		// UARTA RX
	/* default console is on UARTB */
	PINMUX(10, gpio_42, 1);		// UARTC TX
	PINMUX(10, gpio_39, 1);		// UARTC RX

	/* CFE forgot to set these */
	PINMUX(10, gpio_43, 1);		// enet_link
	PINMUX(10, gpio_45, 1);		// enet_activity

#elif defined(CONFIG_BCM7403A0)

	PINMUX(11, gpio_49, 1);		// UARTA TX
	PINMUX(11, gpio_50, 1);		// UARTA RX
	/* default console is on UARTB */
	PINMUX(10, gpio_42, 1);		// UARTC TX
	PINMUX(10, gpio_39, 1);		// UARTC RX

	/* CFE forgot to set these */
	PINMUX(10, gpio_43, 1);		// enet_link
	PINMUX(10, gpio_45, 1);		// enet_activity

#elif defined(CONFIG_BCM7405B0)

#if !defined(CONFIG_KGDB)
	/*
	 * Default: use MII, no UARTB/UARTC.
	 * BCM97405 SW2801-7 should be OFF
	 */
	PINMUX(2, gpio_002, 1);		// MII
	PINMUX(2, gpio_003, 1);
	PINMUX(3, gpio_004, 1);
	PINMUX(3, gpio_005, 1);
	PINMUX(3, gpio_006, 1);
	PINMUX(3, gpio_007, 1);
	PINMUX(3, gpio_008, 1);
	PINMUX(3, gpio_009, 1);
	PINMUX(3, gpio_010, 1);
	PINMUX(3, gpio_011, 1);
	PINMUX(3, gpio_012, 1);
	PINMUX(3, gpio_013, 1);
	PINMUX(4, gpio_014, 1);
	PINMUX(4, gpio_015, 1);
	PINMUX(4, gpio_016, 1);
	PINMUX(4, gpio_017, 1);
	PINMUX(4, gpio_018, 1);
	PINMUX(4, gpio_019, 1);
#else
	/*
	 * Alternate: use UARTB, UARTC.  Required for kgdb.
	 * BCM97405 SW2801-7 should be ON
	 */
	PINMUX(3, gpio_008, 2);		// UARTB TX
	PINMUX(3, gpio_007, 2);		// UARTB RX
	PINMUX(3, gpio_012, 2);		// UARTC TX
	PINMUX(3, gpio_011, 2);		// UARTC RX

	printk(KERN_WARNING "%s: disabling MII to enable extra UARTs\n",
		__FUNCTION__);
#endif

#elif defined(CONFIG_BCM7420A0)

	PINMUX(7, gpio_000, 1);		// ENET LEDs
	PINMUX(7, gpio_001, 1);

	PINMUX(21, sgpio_02, 1);	// MoCA I2C on BSCB
	PINMUX(21, sgpio_03, 1);
	brcm_moca_i2c_base = BPHYSADDR(BCHP_BSCB_REG_START);

	PINMUX(9, gpio_017, 1);		// MoCA LEDs
	PINMUX(9, gpio_019, 1);

	PINMUX(7, gpio_002, 1);		// RGMII
	PINMUX(7, gpio_003, 1);
	PINMUX(7, gpio_004, 1);
	PINMUX(7, gpio_005, 1);
	PINMUX(7, gpio_006, 1);
	PINMUX(7, gpio_007, 1);
	PINMUX(8, gpio_009, 1);
	PINMUX(8, gpio_010, 1);
	PINMUX(8, gpio_011, 1);
	PINMUX(8, gpio_012, 1);
	PINMUX(8, gpio_013, 1);
	PINMUX(8, gpio_014, 1);

	PINMUX(20, gpio_108, 4);	// RGMII SDC/SDL
	PINMUX(20, gpio_109, 5);

	/* set RGMII lines to 2.5V */
	BDEV_WR_F(SUN_TOP_CTRL_GENERAL_CTRL_NO_SCAN_1, pad_mode_gpio_002, 1);
	BDEV_WR_F(SUN_TOP_CTRL_GENERAL_CTRL_NO_SCAN_1, pad_mode_gpio_003, 1);
	BDEV_WR_F(SUN_TOP_CTRL_GENERAL_CTRL_NO_SCAN_1, pad_mode_gpio_004, 1);
	BDEV_WR_F(SUN_TOP_CTRL_GENERAL_CTRL_NO_SCAN_1, pad_mode_gpio_005, 1);
	BDEV_WR_F(SUN_TOP_CTRL_GENERAL_CTRL_NO_SCAN_1, pad_mode_gpio_006, 1);
	BDEV_WR_F(SUN_TOP_CTRL_GENERAL_CTRL_NO_SCAN_1, pad_mode_gpio_007, 1);
	BDEV_WR_F(SUN_TOP_CTRL_GENERAL_CTRL_NO_SCAN_1, pad_mode_gpio_009, 1);
	BDEV_WR_F(SUN_TOP_CTRL_GENERAL_CTRL_NO_SCAN_1, pad_mode_gpio_010, 1);
	BDEV_WR_F(SUN_TOP_CTRL_GENERAL_CTRL_NO_SCAN_1, pad_mode_gpio_011, 1);
	BDEV_WR_F(SUN_TOP_CTRL_GENERAL_CTRL_NO_SCAN_1, pad_mode_gpio_012, 1);
	BDEV_WR_F(SUN_TOP_CTRL_GENERAL_CTRL_NO_SCAN_1, pad_mode_gpio_013, 1);
	BDEV_WR_F(SUN_TOP_CTRL_GENERAL_CTRL_NO_SCAN_1, pad_mode_gpio_014, 1);

#elif defined(CONFIG_BCM7420B0)

	/* set RGMII lines to 2.5V */
	BDEV_WR_F(SUN_TOP_CTRL_GENERAL_CTRL_NO_SCAN_1, rgmii_pad_mode, 1);

#endif /* chip type */
#endif /* ! defined(CONFIG_BRCM_IKOS) */
}

/***********************************************************************
 * RAM and FLASH configuration
 ***********************************************************************/

#define MAGIC0		0xdeadbeef
#define MAGIC1		0xfeedcafe

static inline unsigned int __init probe_ram_size(void)
{
	volatile uint32_t *addr = (volatile void *)KSEG1, *taddr;
	uint32_t olddata;
	unsigned long flags;
	unsigned int i, memsize = 256;

	printk("Probing system memory size... ");

	local_irq_save(flags);
	cache_op(Hit_Writeback_Inv_D, KSEG0);
	olddata = *addr;

	/*
	 * Try to figure out where memory wraps around.  If it does not
	 * wrap, assume 256MB
 	*/
	for(i = 4; i <= 128; i <<= 1) {
		taddr = (volatile void *)(KSEG1 + (i * 1048576));
		*addr = MAGIC0;
		if(*taddr == MAGIC0) {
			*addr = MAGIC1;
			if(*taddr == MAGIC1) {
				memsize = i;
				break;
			}
		}
	}

	*addr = olddata;
	cache_op(Hit_Writeback_Inv_D, KSEG0);
	local_irq_restore(flags);

	printk("found %u MB\n", memsize);

	return(memsize);
}

void __init board_get_ram_size(unsigned long *size)
{
#if defined(CONFIG_BRCM_OVERRIDE_RAM_SIZE)
	*size = CONFIG_BRCM_FORCED_DRAM0_SIZE;
	printk("Using %lu MB RAM (from kernel configuration)\n", *size);
#else
	/* DRAM0_SIZE variable from CFE */
	if(*size) {
		printk("Using %lu MB RAM (from CFE)\n", *size);
		return;
	}

	*size = bchip_strap_ram_size();
	if(*size)
		printk("Using %lu MB RAM (from straps)\n", *size);
	else
		*size = probe_ram_size();
#endif
}

/*
 * Construct the partition map for the primary flash device, based on
 * CFE environment variables that were read from prom.c
 *
 * Note that this does NOT have all of the partitions that CFE recognizes
 * (e.g. macadr, nvram).  It only has the rootfs, entire_device, and
 * optionally the kernel image partition.
 */
int __init board_get_partition_map(struct mtd_partition **p)
{
	struct mtd_partition *ret;
	int nr_parts;

	if (brcm_mtd_rootfs_len == 0)
		return -ENODEV;

	nr_parts = 2;
	if (brcm_mtd_kernel_len != 0)
		nr_parts++;

	ret = kzalloc(sizeof(struct mtd_partition) * nr_parts, GFP_KERNEL);
	if (! ret)
		return -ENOMEM;

	ret[0].offset = brcm_mtd_rootfs_start;
	ret[0].size = brcm_mtd_rootfs_len;
	ret[0].name = "rootfs";

	ret[1].offset = 0;
	ret[1].size = MTDPART_SIZ_FULL;
	ret[1].name = "entire_device";

	if (brcm_mtd_kernel_len != 0) {
		ret[2].offset = brcm_mtd_kernel_start;
		ret[2].size = brcm_mtd_kernel_len;
		ret[2].name = "kernel";
	}

	*p = ret;
	return nr_parts;
}

void brcm_get_ocap_info(struct brcm_ocap_info *info)
{
	info->ocap_part_start = brcm_mtd_ocap_start;
	info->ocap_part_len = brcm_mtd_ocap_len;
	info->docsis_platform = brcm_docsis_platform;
}
EXPORT_SYMBOL(brcm_get_ocap_info);

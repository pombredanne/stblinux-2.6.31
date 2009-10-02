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

#include <linux/module.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/serial_8250.h>
#include <linux/platform_device.h>
#include <linux/bootmem.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/bmoca.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#include <asm/addrspace.h>
#include <asm/irq.h>
#include <asm/reboot.h>
#include <asm/time.h>
#include <asm/delay.h>
#include <asm/serial.h>
#include <asm/cpu-features.h>
#include <asm/war.h>
#include <asm/io.h>
#include <asm/brcmstb/brcmstb.h>

#ifndef CONFIG_MTD
/* squash MTD warning on IKOS builds */
#define CONFIG_MTD_MAP_BANK_WIDTH_1 1
#endif

#include <linux/mtd/mtd.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/map.h>

#define LINUX_MIN_MEM		64

/***********************************************************************
 * Platform device setup
 ***********************************************************************/

#ifdef CONFIG_BRCM_HAS_16550
#define BRCM_16550_PLAT_DEVICE(uart_addr, uart_irq) \
	{ \
		.mapbase = BPHYSADDR(uart_addr), \
		.irq = (uart_irq), \
		.uartclk = BASE_BAUD * 16, \
		.regshift = 2, \
		.iotype = UPIO_MEM32, \
		.flags = UPF_BOOT_AUTOCONF | UPF_IOREMAP, \
	},

static struct plat_serial8250_port brcm_16550_ports[] = {
#ifdef CONFIG_BRCM_UARTA_IS_16550
BRCM_16550_PLAT_DEVICE(BCHP_UARTA_REG_START, BRCM_IRQ_UARTA)
#endif
#ifdef CONFIG_BRCM_UARTB_IS_16550
BRCM_16550_PLAT_DEVICE(BCHP_UARTB_REG_START, BRCM_IRQ_UARTB)
#endif
#ifdef CONFIG_BRCM_UARTC_IS_16550
BRCM_16550_PLAT_DEVICE(BCHP_UARTC_REG_START, BRCM_IRQ_UARTC)
#endif
	{
		.flags = 0,
	}
};

static struct platform_device brcm_16550_uarts = {
	.name = "serial8250",
	.dev = {
		.platform_data = &brcm_16550_ports,
	},
};
#endif

#ifdef CONFIG_BRCM_HAS_3250

#define BRCM_3250_PLAT_DEVICE(uart, i) \
	static struct resource bcm3250_##uart##_resources[] = { \
		[0] = { \
			.start = BPHYSADDR(BCHP_##uart##_REG_START), \
			.end = BPHYSADDR(BCHP_##uart##_REG_END) + 3, \
			.flags = IORESOURCE_MEM, \
		}, \
		[1] = { \
			.start = BRCM_IRQ_##uart, \
			.end = BRCM_IRQ_##uart, \
			.flags = IORESOURCE_IRQ, \
		}, \
	}; \
	static struct platform_device bcm3250_##uart##_device = { \
		.name = "bcm3250_serial", \
		.num_resources = ARRAY_SIZE(bcm3250_##uart##_resources), \
		.resource = bcm3250_##uart##_resources, \
		.id = i, \
	};

#ifdef CONFIG_BRCM_UARTA_IS_3250
BRCM_3250_PLAT_DEVICE(UARTA, 0)
#endif
#ifdef CONFIG_BRCM_UARTB_IS_3250
BRCM_3250_PLAT_DEVICE(UARTB, 1)
#endif
#ifdef CONFIG_BRCM_UARTC_IS_3250
BRCM_3250_PLAT_DEVICE(UARTC, 2)
#endif

#endif

static u64 brcm_usb_dmamask = ~(u32)0;

#define BRCM_USB_PLAT_DEVICE(driver, hci, num, reg, irq) \
	static struct resource brcm_##hci##_resources[] = { \
		{ \
			.start = BPHYSADDR(BCHP_USB_##reg##_REG_START), \
			.end = BPHYSADDR(BCHP_USB_##reg##_REG_END) + 3, \
			.flags = IORESOURCE_MEM, \
		}, { \
			.start = BRCM_IRQ_##irq, \
			.end = BRCM_IRQ_##irq, \
			.flags = IORESOURCE_IRQ, \
		} \
	}; \
	static struct platform_device brcm_##hci##_device = { \
		.name = driver, \
		.id = num, \
		.dev = { \
			.dma_mask = &brcm_usb_dmamask, \
			.coherent_dma_mask = 0xffffffff, \
		}, \
		.num_resources = ARRAY_SIZE(brcm_##hci##_resources), \
		.resource = brcm_##hci##_resources, \
	};

#if defined(BCHP_USB_EHCI_REG_START) && !defined(CONFIG_BRCM_DISABLE_USB0)
BRCM_USB_PLAT_DEVICE("ehci-brcm", ehci0, 0, EHCI, EHCI_0)
#endif
#if defined(BCHP_USB_EHCI1_REG_START) && !defined(CONFIG_BRCM_DISABLE_USB1)
BRCM_USB_PLAT_DEVICE("ehci-brcm", ehci1, 1, EHCI1, EHCI_1)
#endif
#if defined(BCHP_USB_EHCI2_REG_START) && !defined(CONFIG_BRCM_DISABLE_USB2)
BRCM_USB_PLAT_DEVICE("ehci-brcm", ehci2, 2, EHCI2, EHCI_2)
#endif
#if defined(BCHP_USB_OHCI_REG_START) && !defined(CONFIG_BRCM_DISABLE_USB0)
BRCM_USB_PLAT_DEVICE("ohci-brcm", ohci0, 0, OHCI, OHCI_0)
#endif
#if defined(BCHP_USB_OHCI1_REG_START) && !defined(CONFIG_BRCM_DISABLE_USB1)
BRCM_USB_PLAT_DEVICE("ohci-brcm", ohci1, 1, OHCI1, OHCI_1)
#endif
#if defined(BCHP_USB_OHCI2_REG_START) && !defined(CONFIG_BRCM_DISABLE_USB2)
BRCM_USB_PLAT_DEVICE("ohci-brcm", ohci2, 2, OHCI2, OHCI_2)
#endif

static inline void brcm_bogus_release(struct device *dev)
{
}

#if defined(CONFIG_BRCM_HAS_EMAC_0)

static struct resource bcmemac_0_resources[] = {
	[0] = {
		.start		= BPHYSADDR(BCHP_EMAC_0_REG_START),
		.end		= BPHYSADDR(BCHP_EMAC_0_REG_END) + 3,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= BRCM_IRQ_EMAC_0,
		.end		= BRCM_IRQ_EMAC_0,
		.flags		= IORESOURCE_IRQ,
	},
};

struct bcmemac_platform_data bcmemac_0_plat_data = {
#if ! defined(CONFIG_BCMEMAC_EXTMII)
	.phy_type		= BRCM_PHY_TYPE_INT,
#else
	.phy_type		= BRCM_PHY_TYPE_EXT_MII,
#endif
	.phy_id			= BRCM_PHY_ID_AUTO,
};

static struct platform_device bcmemac_0_plat_dev = {
	.name			= "bcmemac",
	.id			= 0,
	.num_resources		= ARRAY_SIZE(bcmemac_0_resources),
	.resource		= bcmemac_0_resources,
	.dev = {
		.platform_data	= &bcmemac_0_plat_data,
		.release	= &brcm_bogus_release,
	},
};

#endif /* defined(CONFIG_BRCM_HAS_EMAC_0) */

#if defined(CONFIG_BRCM_HAS_EMAC_1)

static struct resource bcmemac_1_resources[] = {
	[0] = {
		.start		= BPHYSADDR(BCHP_EMAC_1_REG_START),
		.end		= BPHYSADDR(BCHP_EMAC_1_REG_END) + 3,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= BRCM_IRQ_EMAC_1,
		.end		= BRCM_IRQ_EMAC_1,
		.flags		= IORESOURCE_IRQ,
	},
};

struct bcmemac_platform_data bcmemac_1_plat_data = {
	.phy_type		= BRCM_PHY_TYPE_EXT_MII,
	.phy_id			= BRCM_PHY_ID_AUTO,
};

static struct platform_device bcmemac_1_plat_dev = {
	.name			= "bcmemac",
	.id			= 1,
	.num_resources		= ARRAY_SIZE(bcmemac_1_resources),
	.resource		= bcmemac_1_resources,
	.dev = {
		.platform_data	= &bcmemac_1_plat_data,
		.release	= &brcm_bogus_release,
	},
};
#endif /* defined(CONFIG_BRCM_HAS_EMAC_1) */

#if defined(CONFIG_BRCM_HAS_GENET)
static struct bcmemac_platform_data genet_plat_data = {
#if defined(CONFIG_BCMGENET_0_GPHY)
	.phy_type 		= BRCM_PHY_TYPE_EXT_GMII,
	.phy_id			= BRCM_PHY_ID_AUTO,
#else
	.phy_type		= BRCM_PHY_TYPE_INT,
	.phy_id			= 1,
#endif
};

static struct resource genet_resources[] = {
	[0] = {
		.start		= BPHYSADDR(BCHP_GENET_SYS_REG_START),
		.end		= BPHYSADDR(BCHP_GENET_TDMA_REG_END) + 3,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= BRCM_IRQ_GENET_0_A,
		.end		= BRCM_IRQ_GENET_0_A,
		.flags		= IORESOURCE_IRQ,
	},
	[2] = {
		.start		= BRCM_IRQ_GENET_0_B,
		.end		= BRCM_IRQ_GENET_0_B,
		.flags		= IORESOURCE_IRQ,
	}
};

static struct platform_device genet_plat_dev = {
	.name			= "bcmgenet",
	.id			= 0,
	.num_resources		= ARRAY_SIZE(genet_resources),
	.resource		= genet_resources,
	.dev			= {
		.platform_data	= &genet_plat_data,
		.release	= brcm_bogus_release,
	},
};

#endif /* defined(CONFIG_BRCM_HAS_GENET) */

#if defined(CONFIG_BRCM_HAS_MOCA)

#if defined(CONFIG_BRCM_HAS_GENET)
/* "main" GENET is first device, MoCA is second */
#define MOCA_GENET_ID		1
#else
/* MoCA is the only GENET device */
#define MOCA_GENET_ID		0
#endif

static struct bcmemac_platform_data moca_genet_plat_data = {
	.phy_type 		= BRCM_PHY_TYPE_MOCA,
	.phy_id			= BRCM_PHY_ID_NONE,
};

static struct resource moca_genet_resources[] = {
	[0] = {
		.start		= BPHYSADDR(BCHP_MOCA_GENET_SYS_REG_START),
		.end		= BPHYSADDR(BCHP_MOCA_GENET_TDMA_REG_END) + 3,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= BRCM_IRQ_MOCA_GENET_0_A,
		.end		= BRCM_IRQ_MOCA_GENET_0_A,
		.flags		= IORESOURCE_IRQ,
	},
	[2] = {
		.start		= BRCM_IRQ_MOCA_GENET_0_B,
		.end		= BRCM_IRQ_MOCA_GENET_0_B,
		.flags		= IORESOURCE_IRQ,
	}
};

static struct platform_device moca_genet_plat_dev = {
	.name			= "bcmgenet",
	.id			= MOCA_GENET_ID,
	.num_resources		= ARRAY_SIZE(moca_genet_resources),
	.resource		= moca_genet_resources,
	.dev			= {
		.platform_data	= &moca_genet_plat_data,
		.release	= brcm_bogus_release,
	},
};

static struct moca_platform_data moca_data = {
	.enet_name =		"bcmgenet",
	.enet_id =		MOCA_GENET_ID,
	.bcm3450_i2c_addr =	0x70,
};

static struct resource moca_resources[] = {
	[0] = {
		.start		= BPHYSADDR(BCHP_DATA_MEM_REG_START),
		.end		= BPHYSADDR(BCHP_MOCA_MOCAM2M_REG_END) + 3,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= BRCM_IRQ_MOCA,
		.end		= BRCM_IRQ_MOCA,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device moca_plat_dev = {
	.name			= "bmoca",
	.id			= 0,
	.num_resources		= ARRAY_SIZE(moca_resources),
	.resource		= moca_resources,
	.dev = {
		.platform_data	= &moca_data,
		.release	= brcm_bogus_release,
	},
};

#endif /* defined(CONFIG_BRCM_HAS_MOCA) */

#if defined(CONFIG_BRCM_HAS_SDIO)

#ifdef BCHP_SDIO_CORE_REG_START
#define BCHP_SDIO_REG_START	BCHP_SDIO_CORE_REG_START
#define BCHP_SDIO_REG_END	BCHP_SDIO_CORE_REG_END
#endif

static struct resource sdio_resources[] = {
	[0] = {
		.start		= BPHYSADDR(BCHP_SDIO_REG_START),
		.end		= BPHYSADDR(BCHP_SDIO_REG_END) + 3,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= BRCM_IRQ_HIF,
		.end		= BRCM_IRQ_HIF,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device sdio_plat_dev = {
	.name			= "sdhci-brcm",
	.id			= 0,
	.num_resources		= ARRAY_SIZE(sdio_resources),
	.resource		= sdio_resources,
};

#endif /* defined(CONFIG_BRCM_HAS_SDIO) */

static int __init platform_devices_setup(void)
{
	/* UARTs */

#ifdef CONFIG_BRCM_HAS_16550
	platform_device_register(&brcm_16550_uarts);
#endif
#ifdef CONFIG_BRCM_UARTA_IS_3250
	platform_device_register(&bcm3250_UARTA_device);
#endif
#ifdef CONFIG_BRCM_UARTB_IS_3250
	platform_device_register(&bcm3250_UARTB_device);
#endif
#ifdef CONFIG_BRCM_UARTC_IS_3250
	platform_device_register(&bcm3250_UARTC_device);
#endif

#if defined(CONFIG_BRCM_IKOS)
	/* the remaining devices do not exist in emulation */
	return 0;
#endif

	/* USB controllers */

	bchip_usb_init();
#if defined(BCHP_USB_EHCI_REG_START) && !defined(CONFIG_BRCM_DISABLE_USB0)
	platform_device_register(&brcm_ehci0_device);
#endif
#if defined(BCHP_USB_EHCI1_REG_START) && !defined(CONFIG_BRCM_DISABLE_USB1)
	platform_device_register(&brcm_ehci1_device);
#endif
#if defined(BCHP_USB_EHCI2_REG_START) && !defined(CONFIG_BRCM_DISABLE_USB2)
	platform_device_register(&brcm_ehci2_device);
#endif
#if defined(BCHP_USB_OHCI_REG_START) && !defined(CONFIG_BRCM_DISABLE_USB0)
	platform_device_register(&brcm_ohci0_device);
#endif
#if defined(BCHP_USB_OHCI1_REG_START) && !defined(CONFIG_BRCM_DISABLE_USB1)
	platform_device_register(&brcm_ohci1_device);
#endif
#if defined(BCHP_USB_OHCI2_REG_START) && !defined(CONFIG_BRCM_DISABLE_USB2)
	platform_device_register(&brcm_ohci2_device);
#endif

	/* Network interfaces */

#if defined(CONFIG_BRCM_HAS_EMAC_0)
	brcm_alloc_macaddr(bcmemac_0_plat_data.macaddr);
	platform_device_register(&bcmemac_0_plat_dev);
#endif

#if defined(CONFIG_BRCM_HAS_EMAC_1)
	if (brcm_emac_1_enabled) {
		brcm_alloc_macaddr(bcmemac_1_plat_data.macaddr);
		if (brcm_enet_no_mdio)
			bcmemac_1_plat_data.phy_id = BRCM_PHY_ID_NONE;
		platform_device_register(&bcmemac_1_plat_dev);
	}
#endif

#if defined(CONFIG_BRCM_HAS_GENET)
	brcm_alloc_macaddr(genet_plat_data.macaddr);
	platform_device_register(&genet_plat_dev);
#endif

#if defined(CONFIG_BRCM_HAS_MOCA)
	if (brcm_moca_enabled) {
		u8 macaddr[ETH_ALEN];

		bchip_moca_init();

		/* MOCA_GENET datapath */
		brcm_alloc_macaddr(macaddr);
		memcpy(moca_genet_plat_data.macaddr, macaddr, ETH_ALEN);
		platform_device_register(&moca_genet_plat_dev);

		/* MOCA control path */
		brcm_alloc_macaddr(macaddr);
		mac_to_u32(&moca_data.macaddr_hi, &moca_data.macaddr_lo,
			macaddr);
		moca_data.bcm3450_i2c_base = brcm_moca_i2c_base;
		moca_data.hw_rev =
			BDEV_RD(BCHP_SUN_TOP_CTRL_PROD_REVISION) + 0xa0;
		moca_data.rf_band = brcm_moca_rf_band;
		platform_device_register(&moca_plat_dev);
	}
#endif

#if defined(CONFIG_BRCM_HAS_SDIO)
	platform_device_register(&sdio_plat_dev);
#endif

	return(0);
}

arch_initcall(platform_devices_setup);

#if defined(CONFIG_BRCM_FLASH)

/***********************************************************************
 * Flash device setup
 ***********************************************************************/

#ifdef BCHP_EBI_CS_BASE_5
#define NUM_CS			6
#else
#define NUM_CS			2
#endif

#define TYPE_NONE		0
#define TYPE_NOR		1
#define TYPE_NAND		2
#define TYPE_SPI		3
#define TYPE_MAX		TYPE_SPI

static const char type_names[][8] = { "NONE", "NOR", "NAND", "SPI" };

struct ebi_cs_info {
	int			type;
	unsigned long		start;
	unsigned long		len;
};

static struct ebi_cs_info cs_info[NUM_CS];

#ifdef CONFIG_BRCM_HAS_SPI
static int __init brcm_setup_spi_flash(int cs, int bus_num, int nr_parts,
	struct mtd_partition *parts)
{
	struct spi_board_info board_info;
	struct flash_platform_data *pdata;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

#if 0
		pdata->nr_parts = nr_parts;
		pdata->parts = parts;
#else
		/* XXX HACK: need to handle address inversion on SPI flash */
		if (nr_parts) {
			pdata->nr_parts = 4;
			pdata->parts = kzalloc(sizeof(struct mtd_partition) *
				pdata->nr_parts, GFP_KERNEL);
			if (!pdata->parts) {
				kfree(pdata);
				return -ENOMEM;
			}

			pdata->parts[0].name = "rootfs";
			pdata->parts[0].offset = 0x00400000;
			pdata->parts[0].size = 0x00c00000;

			pdata->parts[1].name = "entire_flash";
			pdata->parts[1].offset = 0;
			pdata->parts[1].size = MTDPART_SIZ_FULL;

			pdata->parts[2].name = "kernel";
			pdata->parts[2].offset = 0x80000;
			pdata->parts[2].size = 13 * 256 * 1024;

			pdata->parts[3].name = "cfe";
			pdata->parts[3].offset = 0x00000000;
			pdata->parts[3].size = 0x80000;
		}
#endif

	memset(&board_info, 0, sizeof(board_info));

	strcpy(board_info.modalias, "m25p80");
	board_info.bus_num = bus_num;
	board_info.chip_select = 0;
	board_info.mode = SPI_MODE_3;
	board_info.platform_data = pdata;

	if (spi_register_board_info(&board_info, 1) != 0) {
		kfree(pdata);
		return -ENODEV;
	}

	return 0;
}

static int __init brcm_setup_spi_master(int cs, int bus_id)
{
	struct resource res[2];
	struct brcmspi_platform_data pdata;
	struct platform_device *pdev;

	memset(&pdata, 0, sizeof(pdata));

	pdata.flash_cs = cs;

	memset(&res, 0, sizeof(res));

	res[0].start = BPHYSADDR(BCHP_HIF_MSPI_REG_START);
	res[0].end = BPHYSADDR(BCHP_HIF_MSPI_REG_START) + 3;
	res[0].flags = IORESOURCE_MEM;

	res[1].start = BRCM_IRQ_HIF_SPI;
	res[1].end = BRCM_IRQ_HIF_SPI;
	res[1].flags = IORESOURCE_IRQ;

	pdev = platform_device_alloc("spi_brcmstb", bus_id);
	if (!pdev ||
	    platform_device_add_resources(pdev, res, 2) ||
	    platform_device_add(pdev)) {
		platform_device_put(pdev);
		return -ENODEV;
	}
	return 0;
}
#endif

static void __init brcm_setup_cs(int cs, int nr_parts,
	struct mtd_partition *parts)
{
	struct platform_device *pdev;

	switch (cs_info[cs].type) {
	case TYPE_NOR: {
		struct physmap_flash_data pdata;
		struct resource res;

		memset(&res, 0, sizeof(res));
		memset(&pdata, 0, sizeof(pdata));

		pdata.width = 2;
		pdata.nr_parts = nr_parts;
		pdata.parts = parts;

		res.start = cs_info[cs].start;
		res.end = cs_info[cs].start + cs_info[cs].len - 1;
		res.flags = IORESOURCE_MEM;

		pdev = platform_device_alloc("physmap-flash", 0);
		if (!pdev ||
		    platform_device_add_resources(pdev, &res, 1) ||
		    platform_device_add_data(pdev, &pdata, sizeof(pdata)) ||
		    platform_device_add(pdev))
			platform_device_put(pdev);
		break;
	}
	case TYPE_NAND: {
		struct brcmnand_platform_data pdata;

		pdata.chip_select = cs;
		pdata.nr_parts = nr_parts;
		pdata.parts = parts;

		pdev = platform_device_alloc("brcmnand", 0);
		if (!pdev ||
		    platform_device_add_data(pdev, &pdata, sizeof(pdata)) ||
		    platform_device_add(pdev))
			platform_device_put(pdev);
		break;
	}
	case TYPE_SPI: {
#ifdef CONFIG_BRCM_HAS_SPI
		const int bus_num = 0;
		int ret;

		ret = brcm_setup_spi_flash(cs, bus_num, nr_parts, parts);
		if (ret < 0) {
			printk(KERN_WARNING
				"%s: can't register SPI flash (error %d)\n",
				__FUNCTION__, ret);
			break;
		}
		ret = brcm_setup_spi_master(cs, bus_num);
		if (ret) {
			printk(KERN_WARNING
				"%s: can't register SPI master (error %d)\n",
				__FUNCTION__, ret);
			break;
		}
#endif
		break;
	}
	default:
		BUG();
	}
}

static int __initdata noflash = 0;
static int __initdata nandcs[NUM_CS];

static struct map_info brcm_dummy_map = {
	.name			= "DUMMY",
};

static int __init brcmstb_mtd_setup(void)
{
	struct mtd_partition *parts;
	int nr_parts;
	int i, first = -1, primary = -1, primary_type = TYPE_NAND;

	if (noflash)
		return 0;

	nr_parts = board_get_partition_map(&parts);
	if (nr_parts <= 0) {
		struct mtd_info *mtd;

		/*
		 * mtd0 is normally the rootfs partition.  If it is missing,
		 * create a dummy partition so that any person or script
		 * trying to mount or erase mtd0 does not corrupt the
		 * whole flash device.
		 */
		mtd = do_map_probe("map_absent", &brcm_dummy_map);
		if (mtd)
			add_mtd_device(mtd);

		printk(KERN_WARNING
			"warning: unable to build a flash partition map, "
			"using entire device\n");

		nr_parts = 0;
		parts = NULL;
	}

	/* parse CFE FLASH_TYPE variable */
	for (i = TYPE_NOR; i <= TYPE_MAX; i++)
		if (strcmp(brcm_mtd_flash_type, type_names[i]) == 0)
			primary_type = i;

	/* scan each chip select to see what (if anything) lives there */
	for (i = 0; i < NUM_CS; i++) {
		u32 base;

		cs_info[i].type = TYPE_NONE;

		base = BDEV_RD(BCHP_EBI_CS_BASE_0 + (i * 8));

		cs_info[i].start = (base >> 13) << 13;
		cs_info[i].len = 8192UL << (base & 0xf);

#ifdef BCHP_EBI_CS_CONFIG_0
		if (BDEV_RD(BCHP_EBI_CS_CONFIG_0 + (i * 8)) &
				BCHP_EBI_CS_CONFIG_0_enable_MASK)
			cs_info[i].type = TYPE_NOR;
#endif
#ifdef BCHP_EBI_CS_SPI_SELECT
		if (BDEV_RD(BCHP_EBI_CS_SPI_SELECT) & (1 << i))
			cs_info[i].type = TYPE_SPI;
#endif
#ifdef BCHP_NAND_CS_NAND_SELECT
		if (BDEV_RD(BCHP_NAND_CS_NAND_SELECT) & (0x100 << i))
			cs_info[i].type = TYPE_NAND;
#endif

		if (cs_info[i].type != TYPE_NAND && nandcs[i] != 0) {
			cs_info[i].type = TYPE_NAND;
		} else {
			/*
			 * nandcs = ineligible to be primary.  The partition
			 * information reported by CFE is not for the NAND
			 * flash if CFE doesn't know the system has a
			 * NAND flash.
			 */
			if (primary == -1 && primary_type == cs_info[i].type)
				primary = i;
		}
		if (first == -1)
			first = i;
	}

	if (primary == -1) {
		if (first == -1) {
			printk(KERN_INFO "EBI: No flash devices detected\n");
			return -ENODEV;
		}
		primary = first;
		primary_type = cs_info[primary].type;
	}

	/* set up primary first, so that it owns mtd0/mtd1/(mtd2) */
	printk(KERN_INFO "EBI CS%d: setting up %s flash (primary)\n", primary,
		type_names[primary_type]);
	brcm_setup_cs(primary, nr_parts, parts);

	for (i = 0; i < NUM_CS; i++) {
		if (i != primary && cs_info[i].type != TYPE_NONE) {
			printk(KERN_INFO "EBI CS%d: setting up %s flash\n", i,
				type_names[cs_info[i].type]);
			brcm_setup_cs(i, 0, NULL);
		}
	}

	return 0;
}

/*
 * late_initcall means the flash drivers are already loaded, so we control
 * the order in which the /dev/mtd* devices get created.
 */
late_initcall(brcmstb_mtd_setup);

static int noflash_setup(char *str)
{
	noflash = 1;
	return 0;
}

__setup("noflash", noflash_setup);

static int nandcs_setup(char *str)
{
	int opts[NUM_CS + 1], i;

	if (*str == 0)
		return 0;
	get_options(str + 1, NUM_CS, opts);

	for (i = 0; i < opts[0]; i++) {
		int cs = opts[i + 1];
		if ((cs >= 0) && (cs < NUM_CS))
			nandcs[cs] = 1;
	}
	return 0;
}

__setup("nandcs", nandcs_setup);

#endif /* defined(CONFIG_BRCM_FLASH) */

/***********************************************************************
 * BMEM (reserved A/V buffer memory) support
 ***********************************************************************/

#define MAX_BMEM_REGIONS	4

struct bmem_region {
	unsigned long		addr;
	unsigned long		size;
};

static struct bmem_region bmem_regions[MAX_BMEM_REGIONS];
static unsigned int n_bmem_regions = 0;
static unsigned int bmem_disabled = 0;

/*
 * Parses command line for bmem= options
 */
static int __init bmem_setup(char *str)
{
	unsigned long addr = 0, size;
	unsigned long lower_mem_bytes;
	char *orig_str = str;

	lower_mem_bytes = (brcm_dram0_size_mb > BRCM_MAX_LOWER_MB) ?
		(BRCM_MAX_LOWER_MB << 20) : (brcm_dram0_size_mb << 20);

	size = (unsigned long)memparse(str, &str);
	if (*str == '@')
		addr = (unsigned long)memparse(str + 1, &str);
	
	if ((size > lower_mem_bytes) || (addr > 0x80000000)) {
		printk(KERN_WARNING "bmem: argument '%s' "
			"is out of range, ignoring\n", orig_str);
		return 0;
	}
	
	if (size == 0) {
		printk(KERN_INFO "bmem: disabling reserved memory\n");
		bmem_disabled = 1;
		return 0;
	}

	if ((addr & ~PAGE_MASK) || (size & ~PAGE_MASK)) {
		printk(KERN_WARNING "bmem: ignoring invalid range '%s' "
			"(is it missing an 'M' suffix?)\n", orig_str);
		return 0;
	}

	if (addr == 0) {
		/*
		 * default: bmem=xxM allocates xx megabytes at the end of
		 * lower memory
		 */
		if (size >= lower_mem_bytes) {
			printk(KERN_WARNING "bmem: '%s' is larger than "
				"lower memory (%lu MB), ignoring\n",
				orig_str, brcm_dram0_size_mb);
			return 0;
		}
		addr = lower_mem_bytes - size;
	}

	if (n_bmem_regions == MAX_BMEM_REGIONS) {
		printk(KERN_WARNING "bmem: too many regions, "
			"ignoring extras\n");
		return 0;
	}

	bmem_regions[n_bmem_regions].addr = addr;
	bmem_regions[n_bmem_regions].size = size;
	n_bmem_regions++;

	return 0;
}

early_param("bmem", bmem_setup);

/*
 * Returns index if the supplied range falls entirely within a bmem region
 */
int bmem_find_region(unsigned long addr, unsigned long size)
{
	int i;

	for (i = 0; i < n_bmem_regions; i++) {
		if ((addr >= bmem_regions[i].addr) &&
		    ((addr + size) <=
			(bmem_regions[i].addr + bmem_regions[i].size))) {
			return(i);
		}
	}
	return -ENOENT;
}
EXPORT_SYMBOL(bmem_find_region);

/*
 * Looks up the bmem region by index, and fills in addr/size if it exists.
 * Returns 0 on success, <0 on failure.
 */
int bmem_region_info(int idx, unsigned long *addr, unsigned long *size)
{
	if ((unsigned int)idx >= n_bmem_regions)
		return -ENOENT;

	*addr = bmem_regions[idx].addr;
	*size = bmem_regions[idx].size;
	return 0;
}
EXPORT_SYMBOL(bmem_region_info);

/*
 * Invokes free_bootmem(), but truncates ranges where necessary to
 * avoid releasing the bmem region(s) back to the VM
 */
void __init brcm_free_bootmem(unsigned long addr, unsigned long size)
{
	/*
	 * Default: (no valid bmem= options specified)
	 *
	 * Lower memory is the first 256MB of system RAM.
	 * bmem gets all but (LINUX_MIN_MEM) megabytes of lower memory.
	 * Linux gets all upper and high memory (if present).
	 *
	 * Options:
	 *
	 * Define one or more custom bmem regions, e.g.:
	 *   bmem=128M (128MB region at the end of lower memory)
	 *   bmem=128M@64M (128MB hole at 64MB mark)
	 *   bmem=16M@64M bmem=4M@128M bmem=16M@200M (multiple holes)
	 *
	 * Disable bmem; give all system RAM to the Linux VM:
	 *   bmem=0
	 *
	 * Overlapping or invalid regions will usually be silently dropped.
	 * If you are doing something tricky, watch the boot messages to
	 * make sure it turns out the way you intended.
	 */
	if (bmem_disabled) {
		n_bmem_regions = 0;
	} else {
		if (n_bmem_regions == 0 &&
		    (brcm_dram0_size_mb > LINUX_MIN_MEM)) {
			n_bmem_regions = 1;
			bmem_regions[0].addr = LINUX_MIN_MEM << 20;
			bmem_regions[0].size =
				(((brcm_dram0_size_mb <= BRCM_MAX_LOWER_MB) ?
				  brcm_dram0_size_mb : BRCM_MAX_LOWER_MB) -
				 LINUX_MIN_MEM) << 20;
		}
	}

	while (size) {
		unsigned long chunksize = size;
		int i;
		struct bmem_region *r = NULL;

		/*
		 * Find the first bmem region (if any) that fits entirely
		 * within the current bootmem address range.
		 */
		for (i = 0; i < n_bmem_regions; i++) {
			if ((bmem_regions[i].addr >= addr) &&
			    ((bmem_regions[i].addr + bmem_regions[i].size) <=
			     (addr + size))) {
				if (!r || (r->addr > bmem_regions[i].addr))
					r = &bmem_regions[i];
			}
		}

		/*
		 * Skip over every bmem region; call free_bootmem() for
		 * every Linux region.  A Linux region is created for
		 * each chunk of the memory map that is not reserved
		 * for bmem.
		 */
		if (r) {
			if (addr == r->addr) {
				printk(KERN_INFO "bmem: adding %lu MB "
					"RESERVED region at %lu MB "
					"(0x%08lx@0x%08lx)\n",
					r->size >> 20, r->addr >> 20,
					r->size, r->addr);
				chunksize = r->size;
				goto skip;
			} else {
				BUG_ON(addr > r->addr);
				chunksize = r->addr - addr;
			}
		}
		BUG_ON(chunksize > size);

		printk(KERN_DEBUG "bmem: adding %lu MB LINUX region at %lu MB "
			"(0x%08lx@0x%08lx)\n", chunksize >> 20, addr >> 20,
			chunksize, addr);
		free_bootmem(addr, chunksize);

skip:
		addr += chunksize;
		size -= chunksize;
	}
}

/*
 * Create /proc/iomem entries for bmem
 */
static int __init bmem_region_setup(void)
{
	int i;

	for (i = 0; i < n_bmem_regions; i++) {
		struct resource *r;
		char *name;

		r = kzalloc(sizeof(*r), GFP_KERNEL);
		name = kzalloc(16, GFP_KERNEL);
		if (! r || ! name)
			break;

		sprintf(name, "bmem.%d", i);
		r->start = bmem_regions[i].addr;
		r->end = bmem_regions[i].addr + bmem_regions[i].size - 1;
		r->flags = IORESOURCE_MEM;
		r->name = name;

		insert_resource(&iomem_resource, r);
	}
	return 0;
}

arch_initcall(bmem_region_setup);

/***********************************************************************
 * Miscellaneous platform-specific functions
 ***********************************************************************/

void __init bus_error_init(void)
{
}

static void brcm_machine_restart(char *command)
{
/* PR21527 - Fix SMP reboot problem */
#ifdef CONFIG_SMP
	smp_send_stop();
	udelay(10);
#endif

	BDEV_WR_F_RB(SUN_TOP_CTRL_RESET_CTRL, master_reset_en, 1);
	BDEV_WR_F_RB(SUN_TOP_CTRL_SW_RESET, chip_master_reset, 1);

	while(1) { }
}

static void brcm_machine_halt(void)
{
	while (1) { }
}

char *__devinit brcmstb_pcibios_setup(char *str)
{
	/* implement "pci=off" command line option */
	if (!strcmp(str, "off")) {
		brcm_pci_enabled = 0;
		brcm_sata_enabled = 0;
		brcm_pcie_enabled = 0;
		return NULL;
	}
	return str;
}

void __init plat_mem_setup(void)
{
	_machine_restart = brcm_machine_restart;
	_machine_halt = brcm_machine_halt;
	pm_power_off = brcm_machine_halt;

 	panic_timeout = 180;

#ifdef CONFIG_PCI
	pcibios_plat_setup = brcmstb_pcibios_setup;
#endif

#if defined(CONFIG_BRCM_HAS_16550) || defined(CONFIG_SERIAL_BCM3250_TTYS)
	add_preferred_console("ttyS", CONFIG_BRCM_CONSOLE_DEVICE, "115200");
#else
	add_preferred_console("ttyBCM", CONFIG_BRCM_CONSOLE_DEVICE, "115200");
#endif
}

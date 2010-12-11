/*
 * Copyright (C) 2010 Broadcom Corporation
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

#include <linux/usb.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/clk.h>
#include <linux/version.h>
#include <linux/module.h>
#include <asm/brcmstb/brcmstb.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
#include <linux/usb/hcd.h>
#else
#include "../core/hcd.h"
#endif

#define MAX_HCD			8

static DEFINE_MUTEX(brcm_usb_mutex);
static struct clk *usb_clk;

struct brcm_hcd_status {
	struct usb_hcd		*hcd;
	int			irq;
};

static struct brcm_hcd_status brcm_hcd[MAX_HCD];
static int brcm_hcd_count;
static int brcm_usb_active = 1;

static int brcm_usb_pwr(int event, void *arg)
{
	int i;

	mutex_lock(&brcm_usb_mutex);

	switch (event) {
	case PM_EVENT_SUSPEND:
		brcm_usb_active = 0;
		for (i = brcm_hcd_count - 1; i >= 0; i--) {
			usb_remove_hcd(brcm_hcd[i].hcd);
			clk_disable(usb_clk);
		}
		break;
	case PM_EVENT_RESUME:
		for (i = 0; i < brcm_hcd_count; i++) {
			clk_enable(usb_clk);
			if (usb_add_hcd(brcm_hcd[i].hcd,
					brcm_hcd[i].irq,
					IRQF_DISABLED) != 0) {
				printk(KERN_WARNING "%s: can't resume hcd %d\n",
					__func__, i);
				clk_disable(usb_clk);
			}
		}
		brcm_usb_active = 1;
		break;
	}

	mutex_unlock(&brcm_usb_mutex);
	return 0;
}

int brcm_usb_probe(struct platform_device *pdev, char *hcd_name,
	const struct hc_driver *hc_driver)
{
	struct resource *res = NULL;
	struct usb_hcd *hcd = NULL;
	int irq, ret, len;

	if (usb_disabled())
		return -ENODEV;

	if (!usb_clk)
		usb_clk = clk_get(NULL, "usb");
	clk_enable(usb_clk);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		err("platform_get_resource error.");
		return -ENODEV;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		err("platform_get_irq error.");
		return -ENODEV;
	}

	/* initialize hcd */
	hcd = usb_create_hcd(hc_driver, &pdev->dev, (char *)hcd_name);
	if (!hcd) {
		err("Failed to create hcd");
		return -ENOMEM;
	}

	len = res->end - res->start + 1;
	hcd->regs = ioremap(res->start, len);
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = len;
	ret = usb_add_hcd(hcd, irq, IRQF_DISABLED);
	if (ret != 0) {
		err("Failed to add hcd");
		iounmap(hcd->regs);
		usb_put_hcd(hcd);
		clk_disable(usb_clk);
		return ret;
	}

	mutex_lock(&brcm_usb_mutex);

	BUG_ON(brcm_hcd_count >= MAX_HCD);
	brcm_hcd[brcm_hcd_count].hcd = hcd;
	brcm_hcd[brcm_hcd_count].irq = irq;
	brcm_hcd_count++;

	mutex_unlock(&brcm_usb_mutex);

	return ret;
}
EXPORT_SYMBOL(brcm_usb_probe);

int brcm_usb_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	int i;

	mutex_lock(&brcm_usb_mutex);
	for (i = 0; i < brcm_hcd_count; i++) {
		if (brcm_hcd[i].hcd == hcd) {
			brcm_hcd[i].hcd = NULL;
			clk_disable(usb_clk);
		}
	}
	mutex_unlock(&brcm_usb_mutex);

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	usb_put_hcd(hcd);

	return 0;
}
EXPORT_SYMBOL(brcm_usb_remove);

void brcm_usb_suspend(struct usb_hcd *hcd)
{
	clk_disable(usb_clk);
}
EXPORT_SYMBOL(brcm_usb_suspend);

void brcm_usb_resume(struct usb_hcd *hcd)
{
	clk_enable(usb_clk);
}
EXPORT_SYMBOL(brcm_usb_resume);

/*
 * If the USB interface has already been disabled through runtime PM, the
 * suspend/resume functions should turn into no-ops.  USB will not be
 * resumed until the application makes an explicit request through pmlib.
 */
int brcm_usb_is_inactive(void)
{
	int ret;
	mutex_lock(&brcm_usb_mutex);
	ret = !brcm_usb_active;
	mutex_unlock(&brcm_usb_mutex);
	return ret;
}
EXPORT_SYMBOL(brcm_usb_is_inactive);

static int __init brcm_usb_init(void)
{
	if (brcm_pm_register_cb("usb", brcm_usb_pwr, NULL))
		printk(KERN_WARNING "%s: can't register PM callback\n",
			__func__);
	return 0;
}

static void __exit brcm_usb_exit(void)
{
	brcm_pm_unregister_cb("usb");
}

module_init(brcm_usb_init)
module_exit(brcm_usb_exit)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("Broadcom USB common functions");

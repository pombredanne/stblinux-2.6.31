.PHONY: all clean distclean romfs
all:

clean distclean:
	rm -rf moca wlan

romfs:
	if [ ! -z "$$CONFIG_USER_NONFREE_MOCA" ]; then \
		$(ROMFSINST) moca/bin /bin ; \
		mkdir -p $(ROMFSDIR)/etc/moca ; \
		$(ROMFSINST) moca/fw/mocacore*.bin /etc/moca/ ; \
	fi
	if [ ! -z "$$CONFIG_USER_NONFREE_WLAN" ]; then \
		if [ "$(CONFIG_USER_NONFREE_WLAN_PCI)" = "y" -o \
			"$(CONFIG_USER_NONFREE_WLAN_USB)" = "y" ]; then \
			$(ROMFSINST) wlan/bin /bin ; \
		fi ; \
		$(ROMFSINST) -e CONFIG_USER_NONFREE_WLAN_PCI \
			wlan/mod/wl_pci.ko /lib/modules ; \
		$(ROMFSINST) -e CONFIG_USER_NONFREE_WLAN_USB \
			wlan/mod/usbabs.ko /lib/modules ; \
		$(ROMFSINST) -e CONFIG_USER_NONFREE_WLAN_USB \
			wlan/mod/wl_usb.ko /lib/modules ; \
	fi

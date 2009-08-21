include $(ROOTDIR)/config.arch

DIST		:= $(ROOTDIR)/../nonfree

ifeq ($(MACHINE),mipsel)
ENDIAN		:=
else
ENDIAN		:= _be
endif

BINDIR		:= bin$(ENDIAN)

# need to strip suffixes (except for _be)
TARGET		:= $(firstword $(subst -, ,$(cat $(ROOTDIR)/.target)))

.PHONY: all clean distclean romfs
all clean distclean:

romfs:
	$(ROMFSINST) -e CONFIG_USER_NONFREE_MOCA \
		$(DIST)/moca/$(BINDIR)/mocactl /bin/mocactl
	$(ROMFSINST) -e CONFIG_USER_NONFREE_MOCA \
		$(DIST)/moca/$(BINDIR)/mocad /bin/mocad
	$(ROMFSINST) -e CONFIG_USER_NONFREE_WLAN \
		$(DIST)/wlan/$(BINDIR)/wl /bin/wl
	$(ROMFSINST) -e CONFIG_USER_NONFREE_WLAN \
		$(DIST)/wlan/$(BINDIR)/bcmdl /bin/bcmdl
	$(ROMFSINST) -e CONFIG_USER_NONFREE_WLAN \
		$(DIST)/wlan/$(BINDIR)/libusb-0.1.so.4 /lib/libusb-0.1.so.4
	$(ROMFSINST) -e CONFIG_USER_NONFREE_WLAN \
		$(DIST)/wlan/$(BINDIR)/wpsenr /bin/wpsenr
	$(ROMFSINST) -e CONFIG_USER_NONFREE_WLAN \
		$(DIST)/wlan/$(TARGET)/wlan_usb.ko /lib/modules/wlan_usb.ko
	-$(ROMFSINST) -e CONFIG_USER_NONFREE_WLAN \
		$(DIST)/wlan/$(TARGET)/wlan_pci.ko /lib/modules/wlan_pci.ko

#
# General settings
#

include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(MACHINE)-linux-gcc
STRIP			:= $(MACHINE)-linux-strip
CFLAGS			:= -Os

export CC CFLAGS

#
# Settings that are specific to this package
#

TARGETS		:= flash_eraseall flash_erase nanddump flash_unlock \
	flash_lock flash_info mtd_debug nandwrite nandtest sumtool
BUILDDIR	:= .

export BUILDDIR

DESTDIR		:= $(shell pwd)/out

#
# Build targets
#

.PHONY: all
all::
	$(MAKE) -f Makefile TARGETS="$(TARGETS)" SUBDIRS=""
	$(MAKE) -C ubi-utils SUBDIRS=""

.PHONY: romfs
romfs::
	rm -rf $(DESTDIR)
	mkdir -p $(DESTDIR)
	$(MAKE) -C ubi-utils install SUBDIRS="" DESTDIR=$(DESTDIR) SBINDIR=bin
	$(STRIP) $(TARGETS) $(DESTDIR)/bin/*
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_FLASH_ERASE /bin/flash_erase
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_FLASH_ERASEALL /bin/flash_eraseall
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_FLASH_UNLOCK /bin/flash_unlock
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_FLASH_LOCK /bin/flash_lock
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_FLASH_INFO /bin/flash_info
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_NANDDUMP /bin/nanddump
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_NANDTEST /bin/nandtest
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_NANDWRITE /bin/nandwrite
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_MTD_DEBUG /bin/mtd_debug
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_SUMTOOL /bin/sumtool
	$(ROMFSINST) -e CONFIG_USER_MTDUTILS_UBI_UTILS $(DESTDIR)/bin /bin/

.PHONY: clean distclean
clean distclean::
	$(MAKE) -f Makefile clean SUBDIRS=""
	$(MAKE) -C ubi-utils clean SUBDIRS=""
	rm -rf $(DESTDIR)

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@

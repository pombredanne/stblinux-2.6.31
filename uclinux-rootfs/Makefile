#
# STB Linux build system v2.0
# Copyright (C) 2009 Broadcom Corporation
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#

SAVEVARS	:= PATH SHELL HOME USER MFLAGS MAKEFLAGS MAKELEVEL \
		MAKEOVERRIDES TERM
unexport $(shell perl -w bin/cleanenv.pl $(SAVEVARS))

# don't pass command line options like SUBDIRS
MAKEOVERRIDES	:=

ROOTDIR		:= $(shell pwd)
LINUXDIR	:= linux-2.6.x
LINUX_CONFIG	:= $(ROOTDIR)/$(LINUXDIR)/.config
CONFIG_CONFIG	:= $(ROOTDIR)/config/.config
ARCH_CONFIG	:= $(ROOTDIR)/config.arch
ROMFSDIR	:= $(ROOTDIR)/romfs
ROMFSINST	:= $(ROOTDIR)/bin/romfs-inst.sh
MAKEARCH	 = $(MAKE)
LIBCDIR		:= uClibc

CFG		:= perl -w bin/config.pl
CHKPATH		:= perl -w bin/checkpath.pl

SHELL		:= /bin/bash
PATH		:= $(shell $(CHKPATH) -a)

export ROOTDIR LINUXDIR LINUX_CONFIG CONFIG_CONFIG ARCH_CONFIG ROMFSDIR \
	ROMFSINST MAKEARCH LIBCDIR SHELL PATH

TFTPHOST	:= $(shell hostname)
TFTPDIR		:= $$USER
MAKEOPTS	:= TFTPHOST=$(TFTPHOST) TFTPDIR=$(TFTPDIR)
TFTPBOOT	:= /tftpboot/$(TFTPDIR)

PLAT		:= $(shell cat .target 2> /dev/null)
VERSION		:= $(shell cat version)
DOTVERSION	:= $(patsubst 26%,2.6.%,$(VERSION))

REL_TFTPHOST	:= stb-irva-01
REL_TFTPDIR	:= $(VERSION)
RELOPTS		:= TFTPHOST=$(REL_TFTPHOST) TFTPDIR=$(REL_TFTPDIR)

#############################################################################
# BASIC KERNEL/ROOTFS BUILDS
#############################################################################

.PHONY: images
images: .target
	$(MAKE) $(MAKEOPTS) initrd_kernel kernel
	bin/build_rootfs_images.sh
	@echo ""
	@echo "Build complete.  Images are in images/"
	@echo ""
	@echo "To install under $(TFTPDIR), use \"make install\""
	@echo ""

.PHONY: initrd_kernel
initrd_kernel: .target chkpath
	# prepare rootfs skeleton
	if [ -z "$(SUBDIRS)" ]; then \
		rm -rf $(ROMFSDIR) ; \
		cp -a skel romfs || exit 1 ; \
		chmod -R a+x romfs/{bin,sbin} ; \
		mkdir -p romfs/{bin,sbin,etc,dev,proc,sys,lib,root,tmp} ; \
		mkdir -p romfs/lib/modules ; \
		mkdir -p romfs/mnt/{hd,usb,flash,nfs} ; \
		mkdir -p romfs/var/{log,run,tmp} romfs/{opt,data} ; \
		ln -s bin/busybox romfs/init ; \
		ln -s /tmp/resolv.conf romfs/etc/resolv.conf ; \
		ln -s /proc/mounts romfs/etc/mtab ; \
		rm -f romfs/etc/brcmstb.conf ; \
		echo "TFTPHOST=$(TFTPHOST)" > romfs/etc/brcmstb.conf ; \
		echo "TFTPDIR=$(TFTPDIR)" >> romfs/etc/brcmstb.conf ; \
		echo "PLAT=$(PLAT)" >> romfs/etc/brcmstb.conf ; \
		echo "VERSION=$(VERSION)" >> romfs/etc/brcmstb.conf ; \
	fi
	# build/install rootfs contents
	if [[ "$(SUBDIRS)" != "linux*" ]]; then \
		$(MAKE) -C lib all romfs \
			INITRD_SUBDIR="$(SUBDIRS)" || exit 1 ; \
		$(MAKE) -C user all romfs \
			INITRD_SUBDIR="$(SUBDIRS)" || exit 1 ; \
	fi
	# configure kernel for initramfs, then build it
	$(CFG) initramfs
	$(MAKE) -C $(LINUXDIR) vmlinux
	# copy completed gzipped kernel to images/
	mkdir -p images
	gzip -9 < $(LINUXDIR)/vmlinux > \
		images/vmlinuz-initrd-$(PLAT)

.PHONY: kernel
kernel: .target chkpath
	$(CFG) noinitramfs
	$(MAKE) -C $(LINUXDIR) vmlinux
	mkdir -p images
	gzip -9 < $(LINUXDIR)/vmlinux > \
		images/vmlinuz-$(PLAT)

#############################################################################
# PRIMARY BUILD TARGETS
#############################################################################

.PHONY: vmlinuz-initrd-%
vmlinuz-initrd-%:
	$(MAKE) $(MAKEOPTS) defaults-$*
	$(MAKE) $(MAKEOPTS) initrd_kernel

.PHONY: vmlinuz-%
vmlinuz-%:
	$(MAKE) $(MAKEOPTS) defaults-$*
	$(MAKE) $(MAKEOPTS) kernel

.PHONY: kernels-%
kernels-%:
	$(MAKE) $(MAKEOPTS) vmlinuz-initrd-$* vmlinuz-$*

.PHONY: images-%
images-%:
	$(MAKE) $(MAKEOPTS) defaults-$* images

.PHONY: release
release:
	@if [ -e .dirty -o -e images -o -e release ]; then \
		echo "ERROR: \"make release\" must be run on a pristine view" ; \
		exit 1 ; \
	fi
	cd .. ; perl -w tools/unpack.pl -r || exit 1
	chmod -R u+w . linux-2.6.x/
	chmod -R 0755 bin/
	export LINK=`readlink linux-2.6.x`; \
		export STBLINUX=`basename $$LINK`; \
		tar -C .. -jcf ../stblinux-$(DOTVERSION).tar.bz2 $$STBLINUX
	tar -C .. -jcf ../uclinux-rootfs-$(DOTVERSION).tar.bz2 uclinux-rootfs
	# $(MAKE) -C ../../LinuxSupport/nonfree install ROOTDIR=$(ROOTDIR)
	# tar -C .. -jcf ../nonfree-$(DOTVERSION).tar.bz2 nonfree
	for x in $(shell cat misc/release_builds); do \
		$(MAKE) $(RELOPTS) images-$$x || exit 1; \
	done
	mkdir -p release/src
	mv ../stblinux-$(DOTVERSION).tar.bz2 \
		../uclinux-rootfs-$(DOTVERSION).tar.bz2 \
		release/src/
	mkdir release/$(VERSION)
	mv images/* release/$(VERSION)/

#############################################################################
# MISC CHECKS
#############################################################################

.target:
	@echo "Configuration defaults not set.  Try: \"make help\""
	@exit 1

.PHONY: chkpath
chkpath:
	@$(CHKPATH)

#############################################################################
# BUILD CONFIGURATION
#############################################################################

.PHONY: silentoldconfig
silentoldconfig: .target chkpath
	$(MAKE) -C $(LINUXDIR) silentoldconfig prepare scripts
	$(MAKE) -C lib/uClibc oldconfig
	$(MAKE) -C user/busybox silentoldconfig
	$(MAKE) -C config oldconfig

.PHONY: oldconfig
oldconfig: .target chkpath
	$(MAKE) -C $(LINUXDIR) oldconfig prepare scripts
	$(MAKE) -C lib/uClibc oldconfig
	$(MAKE) -C user/busybox oldconfig
	$(MAKE) -C config oldconfig

.PHONY: defaults-%
defaults-%: chkpath
	@if [ -x ../tools/unpack.pl ]; then \
		cd .. ; perl -w tools/unpack.pl || exit 1 ; \
	fi
	@touch .dirty
	@chmod -R 0755 config/scripts/{Configure,Menuconfig} bin
	$(CFG) defaults $*
	$(MAKE) silentoldconfig

.PHONY: save_defaults
save_defaults: .target
	$(CFG) save_defaults $(PLAT)

.PHONY: menuconfig-linux menuconfig-vendor menuconfig-uclibc menuconfig-busybox
menuconfig-linux: .target
	$(MAKE) -C $(LINUXDIR) menuconfig

menuconfig-vendor: .target
	$(MAKE) -C config menuconfig

menuconfig-uclibc: .target
	$(MAKE) -C lib/uClibc menuconfig

menuconfig-busybox: .target
	$(MAKE) -C user/busybox menuconfig

#############################################################################
# INSTALL/CLEAN
#############################################################################

.PHONY: install
install:
	mkdir -p $(TFTPBOOT)
	cp -f images/* $(TFTPBOOT)/
	@echo ""
	@echo "Images have been copied to $(TFTPBOOT)"
	@echo ""

.PHONY: clean
clean:
	if [ -e .target ]; then \
		if [ -z "$(SUBDIRS)" ]; then \
			$(MAKE) -C lib clean ; \
			$(MAKE) -C user clean ; \
			$(MAKE) -C $(LINUXDIR) clean ; \
		elif [[ "$(SUBDIRS)" != "linux*" ]]; then \
			$(MAKE) -C lib clean \
				INITRD_SUBDIR="$(SUBDIRS)" || exit 1 ; \
			$(MAKE) -C user clean \
				INITRD_SUBDIR="$(SUBDIRS)" || exit 1 ; \
		elif [[ "$(SUBDIRS)" = "linux*" ]]; then \
			$(MAKE) -C $(LINUXDIR) clean ; \
		fi ; \
	fi

.PHONY: distclean
distclean:
	if [ -e .target ]; then \
		$(MAKE) -C lib distclean clean ; \
		$(MAKE) -C user distclean clean ; \
		$(MAKE) -C $(LINUXDIR) mrproper ; \
		$(MAKE) -C config clean ; \
	fi
	rm -rf romfs tmp
	rm -f $(CONFIG_CONFIG) $(ARCH_CONFIG) .target

#############################################################################
# INFORMATIONAL TARGETS
#############################################################################

.PHONY: help
help:
	@echo ""
	@echo "STB Linux build targets:"
	@echo ""
	@echo "vmlinuz-<target>           - Build non-initramfs kernel for TARGET"
	@echo "vmlinuz-initrd-<target>    - Build initramfs kernel for TARGET"
	@echo "kernels-<target>           - Build both kernels for TARGET"
	@echo "images-<target>            - Build kernels + flash images for TARGET"
	@echo ""
	@echo "clean                      - Clean objects but not configurations"
	@echo "distclean                  - Clean everything"
	@echo ""
	@echo "defaults-<target>          - Install default config files for TARGET"
	@echo "menuconfig-linux           - Customize Linux config"
	@echo "menuconfig-vendor          - Customize user/lib package config"
	@echo "menuconfig-uclibc          - Customize uClibc config"
	@echo "menuconfig-busybox         - Customize busybox config"
	@echo "save_defaults              - Overwrite default config with current settings"
	@echo ""
	@echo "images                     - Build initramfs kernel for current target,"
	@echo "                             then build flash images / tarballs"
	@echo "initrd_kernel              - Build initramfs kernel for current target"
	@echo "kernel                     - Build non-initramfs kernel for current target"
	@echo ""
	@echo "install                    - Copy all kernels/images/tarballs to TFTPDIR"
	@echo ""
	@echo "release                    - Create source tarballs, then build all targets"
	@echo ""
	@echo "Examples:"
	@echo ""
	@echo "  make images-7335b0       - Build all images for 7335b0"
	@echo "  make vmlinuz-7335b0      - Build non-initrd kernel for 7335b0"
	@echo ""
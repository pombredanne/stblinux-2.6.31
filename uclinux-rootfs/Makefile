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
		MAKEOVERRIDES TERM DISPLAY
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
TFTPBOOT	:= /tftpboot/$(TFTPDIR)

PLAT		:= $(shell cat .target 2> /dev/null)
VERSION		:= $(shell cat version)
DOTVERSION	:= $(patsubst 26%,2.6.%,$(VERSION))

REL_TFTPHOST	:= stb-irva-01
REL_TFTPDIR	:= $(VERSION)
RELOPTS		:= TFTPHOST=$(REL_TFTPHOST) TFTPDIR=$(REL_TFTPDIR)

# note that these are also hard-coded into config.pl
NONFREE_BIN	:= $(ROOTDIR)/../nonfree
NONFREE_SRC	:= $(ROOTDIR)/../nonfree_src
NONFREE_PLAT	:= $(firstword $(subst -, ,$(PLAT)))
NONFREE_DESTDIR	:= $(NONFREE_BIN)/$(NONFREE_PLAT)

MAKEOPTS	:= TFTPHOST=$(TFTPHOST) TFTPDIR=$(TFTPDIR)

#############################################################################
# BASIC KERNEL/ROOTFS BUILDS
#############################################################################

.PHONY: images
images: .target
	# initrd_kernel implicitly builds kernel in this case
	@if $(CFG) test_vendor CONFIG_USER_NONFREE_USES_KBUILD; then \
		$(MAKE) $(MAKEOPTS) initrd_kernel || exit 1 ; \
	else \
		$(MAKE) $(MAKEOPTS) initrd_kernel kernel || exit 1 ; \
	fi
	bin/build_rootfs_images.sh
	@echo ""
	@echo "Build complete.  Images are in images/"
	@echo ""
	@echo "To install under $(TFTPDIR), use \"make install\""
	@echo ""

.PHONY: initrd_kernel
initrd_kernel: .target chkpath
	mkdir -p images
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
	# build non-initrd kernel for drivers that need Module.symvers
	if $(CFG) test_vendor CONFIG_USER_NONFREE_USES_KBUILD; then \
		$(MAKE) kernel || exit 1 ; \
	fi
	# build nonfree components from source, then create binary tarball
	if $(CFG) test_vendor CONFIG_USER_NONFREE_SRC; then \
		if $(CFG) test_vendor CONFIG_USER_NONFREE_MOCA_SRC; then \
			rm -rf $(NONFREE_DESTDIR)/moca ; \
			$(MAKE) -C $(NONFREE_SRC)/moca ROOTDIR=$(ROOTDIR) \
				DESTDIR=$(NONFREE_DESTDIR)/moca \
				nonfree-$(NONFREE_PLAT) || exit 1 ; \
		fi ; \
		if $(CFG) test_vendor CONFIG_USER_NONFREE_WLAN_SRC; then \
			rm -rf $(NONFREE_DESTDIR)/wlan ; \
			$(MAKE) -C $(NONFREE_SRC)/wlan ROOTDIR=$(ROOTDIR) \
				DESTDIR=$(NONFREE_DESTDIR)/wlan \
				nonfree-$(NONFREE_PLAT) || exit 1 ; \
		fi ; \
		tar -C $(NONFREE_BIN)/.. -jcf images/nonfree-$(PLAT).tar.bz2 \
			nonfree/$(NONFREE_PLAT) || exit 1 ; \
	fi
	# copy nonfree binaries (if present) into place
	rm -rf user/nonfree/{moca,wlan}
	if $(CFG) test_vendor CONFIG_USER_NONFREE_MOCA; then \
		cp -a $(NONFREE_BIN)/$(NONFREE_PLAT)/moca user/nonfree/ \
			|| exit 1 ; \
	fi
	if $(CFG) test_vendor CONFIG_USER_NONFREE_WLAN; then \
		cp -a $(NONFREE_BIN)/$(NONFREE_PLAT)/wlan user/nonfree/ \
			|| exit 1 ; \
	fi
	# build/install rootfs contents
	if [[ "$(SUBDIRS)" != linux* ]]; then \
		$(MAKE) -C lib all romfs \
			INITRD_SUBDIR="$(SUBDIRS)" || exit 1 ; \
		$(MAKE) -C user all romfs \
			INITRD_SUBDIR="$(SUBDIRS)" || exit 1 ; \
	fi
	# configure kernel for initramfs, then build it
	$(CFG) initramfs
	$(MAKE) -C $(LINUXDIR) vmlinux SUBDIRS=
	# copy completed gzipped kernel to images/
	gzip -9 < $(LINUXDIR)/vmlinux > \
		images/vmlinuz-initrd-$(PLAT)

.PHONY: kernel
kernel: .target chkpath
	$(CFG) noinitramfs
	$(MAKE) -C $(LINUXDIR) vmlinux
	mkdir -p images
	gzip -9 < $(LINUXDIR)/vmlinux > \
		images/vmlinuz-$(PLAT)

.PHONY: kernel_sign
kernel_sign: .target chkpath
	@if [ -z "$(SIGLIST)" ]; then \
		echo "!!!No signature list file is specified!!!"; exit 1;\
	fi; \
	#Generate initram fs skeleton
	rm -rf $(ROOTDIR)/initramfs; \
	mkdir -p initramfs/{bin,dev,etc,proc,root,sbin,tmp};\
	chmod -R a+x initramfs/{bin,sbin}
	#build rootfs signing app.
	$(MAKE) -C user all romfs BUILD_ROOT_SIGN=y SIGLIST=$(SIGLIST) ROOTFSDIR=$(ROOTFSDIR)
	#build kernel.
	$(CFG) initramfs-sign 
	$(MAKE) -C $(LINUXDIR) vmlinux SUBDIRS= 
	gzip -9 < $(LINUXDIR)/vmlinux > \
	images/vmlinuz-sign-$(PLAT)
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
	# make sure this is a clean view
	@if [ -e .dirty -o -e images -o -e release ]; then \
		echo "ERROR: \"make release\" must be run on a pristine view" ; \
		exit 1 ; \
	fi
	# check for nonfree sources
	if [ ! -d ../../LinuxSupport/nonfree_src ]; then \
		echo "ERROR: missing nonfree sources" ; \
		exit 1 ; \
	fi
	ln -sf ../LinuxSupport/nonfree_src ../nonfree_src
	# unpack tarballs/* then build pristine source tarballs for release
	cd .. ; perl -w tools/unpack.pl -r || exit 1
	chmod -R u+w . linux-2.6.x/
	chmod -R 0755 bin/
	export LINK=`readlink linux-2.6.x`; \
		export STBLINUX=`basename $$LINK`; \
		tar -C .. -jcf ../stblinux-$(DOTVERSION).tar.bz2 $$STBLINUX
	tar -C .. -jcf ../uclinux-rootfs-$(DOTVERSION).tar.bz2 uclinux-rootfs
	# build pristine nonfree source tarballs
	chmod -R u+w $(NONFREE_SRC)/
	mkdir -p RESTRICTED
	tar -C .. -jcf RESTRICTED/moca_src-$(DOTVERSION).tar.bz2 \
		nonfree_src/moca/
	tar -C .. -jcf RESTRICTED/wlan_src-$(DOTVERSION).tar.bz2 \
		nonfree_src/wlan/
	# build all platforms
	for x in $(shell cat misc/release_builds); do \
		$(MAKE) $(RELOPTS) images-$$x || exit 1; \
	done
	# move files into the release directory
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
	@echo "***************************************************************"
	@echo "* Build configuration"
	@echo "*"
	@echo "* Target:                  $*"
	@echo "* Version:                 $(DOTVERSION)"
	@echo "* TFTP host:               $(TFTPHOST)"
	@echo "* Remote TFTP path:        $(TFTPDIR)"
	@echo "* Local TFTP path:         $(TFTPBOOT)"
	@if $(CFG) test_vendor CONFIG_USER_NONFREE_MOCA; then \
		if $(CFG) test_vendor CONFIG_USER_NONFREE_MOCA_SRC; then \
			echo "* MoCA drivers:            Building from source" ; \
		else \
			echo "* MoCA drivers:            Using precompiled binaries" ; \
		fi ; \
	else \
		echo "* MoCA drivers:            NOT ENABLED" ; \
	fi
	@if $(CFG) test_vendor CONFIG_USER_NONFREE_WLAN; then \
		if $(CFG) test_vendor CONFIG_USER_NONFREE_WLAN_SRC; then \
			echo "* WLAN drivers:            Building from source" ; \
		else \
			echo "* WLAN drivers:            Using precompiled binaries" ; \
		fi ; \
	else \
		echo "* WLAN drivers:            NOT ENABLED" ; \
	fi
	@echo "***************************************************************"
	@echo ""

.PHONY: save_defaults
save_defaults: .target
	$(CFG) save_defaults $(PLAT)

.PHONY: menuconfig-linux menuconfig-vendor menuconfig-uclibc \
	menuconfig-busybox xconfig-linux
menuconfig-linux: .target
	$(MAKE) -C $(LINUXDIR) menuconfig

menuconfig-vendor: .target
	$(MAKE) -C config menuconfig

menuconfig-uclibc: .target
	$(MAKE) -C lib/uClibc menuconfig

menuconfig-busybox: .target
	$(MAKE) -C user/busybox menuconfig

xconfig-linux: .target
	$(MAKE) -C $(LINUXDIR) xconfig

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

.PHONY: nonfree_clean
nonfree_clean:
	if $(CFG) test_vendor CONFIG_USER_NONFREE_MOCA_SRC; then \
		make -C $(NONFREE_SRC)/moca ROOTDIR=$(ROOTDIR) \
			DESTDIR=$(NONFREE_DESTDIR)/moca clean || exit 1 ; \
	fi
	if $(CFG) test_vendor CONFIG_USER_NONFREE_WLAN_SRC; then \
		make -C $(NONFREE_SRC)/wlan ROOTDIR=$(ROOTDIR) \
			DESTDIR=$(NONFREE_DESTDIR)/wlan clean || exit 1 ; \
	fi

.PHONY: clean
clean:
	if [ -e .target ]; then \
		if [ -z "$(SUBDIRS)" ]; then \
			$(MAKE) -C lib clean ; \
			$(MAKE) -C user clean ; \
			$(MAKE) -C $(LINUXDIR) clean ; \
			$(MAKE) $(MAKEOPTS) nonfree_clean ; \
		elif [[ "$(SUBDIRS)" != "linux*" ]]; then \
			$(MAKE) -C lib clean \
				INITRD_SUBDIR="$(SUBDIRS)" || exit 1 ; \
			$(MAKE) -C user clean \
				INITRD_SUBDIR="$(SUBDIRS)" || exit 1 ; \
		elif [[ "$(SUBDIRS)" = "linux*" ]]; then \
			$(MAKE) -C $(LINUXDIR) clean ; \
		fi ; \
		if [ -d user/root_sign ]; then \
			$(MAKE) -C user/root_sign clean; \
		fi ;\
	fi

.PHONY: distclean
distclean:
	if [ -e .target ]; then \
		$(MAKE) -C lib distclean clean ; \
		$(MAKE) -C user distclean clean ; \
		$(MAKE) -C $(LINUXDIR) mrproper ; \
		$(MAKE) $(MAKEOPTS) nonfree_clean ; \
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

include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(MACHINE)-linux-gcc
STRIP			:= $(MACHINE)-linux-strip
CFLAGS			:= -Os

export CC CFLAGS

.PHONY: all
all::
	if [ ! -e Makefile ]; then \
		./configure --host=$(MACHINE)-linux \
			bash_cv_job_control_missing=present \
			bash_cv_func_sigsetjmp=present \
			bash_cv_sys_named_pipes=present || exit 1 ; \
	fi
	$(MAKE) -f Makefile

.PHONY: romfs
romfs::
	$(STRIP) bash
	$(ROMFSINST) -e CONFIG_USER_BASH_BASH /bin/bash
	$(ROMFSINST) -e CONFIG_USER_BASH_BASH -s bash /bin/sh

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@

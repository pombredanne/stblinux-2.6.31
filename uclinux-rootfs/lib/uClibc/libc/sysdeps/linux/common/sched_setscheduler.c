/* vi: set sw=4 ts=4: */
/*
 * sched_setscheduler() for uClibc
 *
 * Copyright (C) 2000-2006 Erik Andersen <andersen@uclibc.org>
 *
 * Licensed under the LGPL v2.1, see the file COPYING.LIB in this tarball.
 */

#include "syscalls.h"
#include <sched.h>

#define __NR___syscall_sched_setscheduler __NR_sched_setscheduler
static inline _syscall3(int, __syscall_sched_setscheduler,
		__kernel_pid_t, pid, int, policy, const struct sched_param *, p);

int sched_setscheduler(pid_t pid, int policy, const struct sched_param *p)
{
	return (__syscall_sched_setscheduler(pid, policy, p));
}

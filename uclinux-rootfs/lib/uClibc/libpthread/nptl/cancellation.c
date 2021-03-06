/* Copyright (C) 2002, 2003 Free Software Foundation, Inc.
   This file is part of the GNU C Library.
   Contributed by Ulrich Drepper <drepper@redhat.com>, 2002.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, write to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.  */

#include <setjmp.h>
#include <stdlib.h>
#include "pthreadP.h"


/* The next two functions are similar to pthread_setcanceltype() but
   more specialized for the use in the cancelable functions like write().
   They do not need to check parameters etc.  */
int
attribute_hidden
__pthread_enable_asynccancel (void)
{
  struct pthread *self = THREAD_SELF;
  int oldval = THREAD_GETMEM (self, cancelhandling);

//printf("%s\n", __FUNCTION__);
  while (1)
    {
      int newval = oldval | CANCELTYPE_BITMASK;

      if (newval == oldval)
	break;

      int curval = THREAD_ATOMIC_CMPXCHG_VAL (self, cancelhandling, newval,
					      oldval);
      if (__builtin_expect (curval == oldval, 1))
	{
	  if (CANCEL_ENABLED_AND_CANCELED_AND_ASYNCHRONOUS (newval))
	    {
	      THREAD_SETMEM (self, result, PTHREAD_CANCELED);
	      __do_cancel ();
	    }

	  break;
	}

      /* Prepare the next round.  */
      oldval = curval;
    }

  return oldval;
}


void
internal_function attribute_hidden
__pthread_disable_asynccancel (int oldtype)
{
//printf("%s\n", __FUNCTION__);
  /* If asynchronous cancellation was enabled before we do not have
     anything to do.  */
  if (oldtype & CANCELTYPE_BITMASK)
    return;

  struct pthread *self = THREAD_SELF;
  int oldval = THREAD_GETMEM (self, cancelhandling);

  while (1)
    {
      int newval = oldval & ~CANCELTYPE_BITMASK;

      if (newval == oldval)
	break;

      int curval = THREAD_ATOMIC_CMPXCHG_VAL (self, cancelhandling, newval,
					      oldval);
      if (__builtin_expect (curval == oldval, 1))
	break;

      /* Prepare the next round.  */
      oldval = curval;
    }
}

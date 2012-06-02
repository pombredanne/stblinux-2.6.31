/*
 * drivers/mtd/brcmnand/brcmnand_isr.c
 *
 *  Copyright (c) 2005-2009 Broadcom Corp.
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
 *
 * Implement Interrupt Service Routine
 * 
 * when		who		what
 * 20090318	tht		Original coding
 */

//#define ISR_DEBUG_SMP
#undef ISR_DEBUG_SMP

#ifdef ISR_DEBUG_SMP
#include <asm/atomic.h>
#endif


//#define DEBUG_ISR
#include "brcmnand_priv.h"

#ifdef CONFIG_MTD_BRCMNAND_EDU
#include "edu.h"
#include "eduproto.h"

#elif defined(CONFIG_MTD_BRCMNAND_ISR_NOEDU)
#include "isrnonedu.h"
#endif

#include <linux/dma-mapping.h>

#define PRINTK(...)
//#define PRINTK printk

#ifdef ISR_DEBUG_SMP
static atomic_t v = ATOMIC_INIT(1);
#define PRINTK1(...) if (!atomic_dec_and_test(&v)) printk("<")
#define PRINTK2(...) atomic_inc(&v)  //, printk(">"))
#define PRINTK5(...) if (!atomic_dec_and_test(&v))  printk("+");
#define PRINTK6(...) atomic_inc(&v)  // printk("-");
#define PRINTK3(...) if (!atomic_dec_and_test(&v)) printk("[");
#define PRINTK4(...) atomic_inc(&v) // printk("]");

#else
#define PRINTK1(...)
#define PRINTK2(...)
#define PRINTK3(...)
#define PRINTK4(...)
#define PRINTK5(...)
#define PRINTK6(...)
#endif
 

 // Wakes up the sleeping calling thread.
static DECLARE_WAIT_QUEUE_HEAD(gIsrWaitQ);

//eduIsrNode_t gEduIsrData; 
eduIsrNode_t gEduIsrPool[MAX_JOB_QUEUE_SIZE+2]; /* ReadOp Pool, add 2 for Pushed WAR jobs */

isrJobQ_t gJobQ; /* Job Queue */

extern int gdebug;


/*
 * Queue next sector for read/write, assuming caller holds queue lock
 */
eduIsrNode_t* 
ISR_queue_read_request(struct mtd_info *mtd,
        void* buffer, u_char* oobarea, loff_t offset)
{
	eduIsrNode_t* entry; 
	struct list_head* node;

	// Grab one request from avail list
	if (list_empty(&gJobQ.availList)) {
		printk("%s: Empty avail list\n", __FUNCTION__);
		BUG();
	}
	node = gJobQ.availList.next;
	if (!node) {
		printk("%s: Empty avail list\n", __FUNCTION__);
		BUG();
	}
	entry = list_entry(node, eduIsrNode_t, list);
	list_del(node);

	// Queue entry
	list_add_tail(node, &gJobQ.jobQ);
	spin_lock_init(&entry->lock);
	entry->mtd = mtd;
	entry->buffer = buffer;
	entry->oobarea = oobarea;
	entry->offset = offset;
	entry->ret = -1;
	entry->refCount = 1;
	entry->opComplete = ISR_OP_QUEUED;
	
	return entry;
}

eduIsrNode_t* 
ISR_queue_write_request(struct mtd_info *mtd,
        const void* buffer, const u_char* oobarea, loff_t offset)
{
	eduIsrNode_t* entry; 
	struct list_head* node;

	// Grab one request from avail list
	if (list_empty(&gJobQ.availList)) {
		printk("%s: Empty avail list\n", __FUNCTION__);
		BUG();
	}
	node = gJobQ.availList.next;
	if (!node) {
		printk("%s: Empty avail list\n", __FUNCTION__);
		BUG();
	}
	entry = list_entry(node, eduIsrNode_t, list);
	list_del(node);

	// Queue entry
	list_add_tail(node, &gJobQ.jobQ);
	spin_lock_init(&entry->lock);
	entry->mtd = mtd;
	entry->buffer = (void *)buffer;
	entry->oobarea = (u_char *)oobarea;
	entry->offset = offset;
	entry->ret = -1;
	entry->refCount = 1;
	entry->opComplete = ISR_OP_QUEUED;

	return entry;
}


/*
 * Push next sector for dummy read to head of queue, assuming caller holds queue lock
 * Job will be next to be executed
 */
eduIsrNode_t*  
ISR_push_request(struct mtd_info *mtd,
        void* buffer, u_char* oobarea, loff_t offset) 
{
	eduIsrNode_t* entry; 
	struct list_head* node;

	// Grab one request from avail list
	if (list_empty(&gJobQ.availList)) {
		printk("%s: Empty avail list\n", __FUNCTION__);
		BUG();
	}
	node = gJobQ.availList.next;
	if (!node) {
		printk("%s: Empty avail list\n", __FUNCTION__);
		BUG();
	}
	entry = list_entry(node, eduIsrNode_t, list);
	list_del(node);

	// Push to head of queue
	list_add(node, &gJobQ.jobQ);
	spin_lock_init(&entry->lock);
	entry->mtd = mtd;
	entry->buffer = buffer;
	entry->oobarea = oobarea;
	entry->offset = offset;
	entry->ret = -1;
	entry->refCount = 1;
	entry->opComplete = ISR_OP_QUEUED;

	return entry;	
}


/*
 * Called with ReqdQ Read lock held
 * Returns pointer to node that satisfies opStatus, 
 * with spin lock held (spin_lock()'ed assuming queue lock has been held))
 */
eduIsrNode_t*
ISR_find_request( isrOpStatus_t opStatus)
{
	eduIsrNode_t* req;

	list_for_each_entry(req, &gJobQ.jobQ, list) {
		
		// We called this with spin_lock_irqsave on queue lock, so no need for the irq variant
		spin_lock(&req->lock);
		if (req->opComplete == opStatus) {
			return req;
		}
		spin_unlock(&req->lock);
	}
	return (eduIsrNode_t*) 0;;
}

#if 0
static void
ISR_print_queue(void)
{
	eduIsrNode_t* req;
	int i=0;

	list_for_each_entry(req, &gJobQ.jobQ, list) {
		
		// We called this with spin_lock_irqsave on queue lock, so no need for the irq variant
		printk("I=%d req=%p, offset=%0llx, opComp=%d, list=%p, next=%p, prev=%p\n",
			i, req, req->offset, req->opComplete, &req->list, req->list.next, req->list.prev);
		i++;
	}
	return (eduIsrNode_t*) 0;;
}
#endif


/*
 * We've got interrupted, and verified that job is complete. 
 * Job lock has been held by caller.
 * Do Read completion routines
 * runs in interrupt context.
 * Return returned value of read-op.
 */



#if 0 //def EDU_DOUBLE_BUFFER_READ

/* Save this to be revived when we are sure that EDU's double buffering works */
static int
ISR_read_completion(eduIsrNode_t* req)
{
	/* Make sure that the current request does not cause an UNC ERR, as
	 * that would require a read from the LKGS to reset EDU
	 */
	if (req->status & HIF_INTR2_EDU_ERR) {
		uint32_t edu_err_status;

		edu_err_status = EDU_volatileRead(EDU_BASE_ADDRESS + EDU_ERR_STATUS);
		if (edu_err_status && edu_err_status != EDU_ERR_STATUS_NandECCcor) {

			/* If error, we must stop the on-going EDU op, because it will be dropped by EDU.  
			 * This is VLSI PR2389
			 */
			edu_status = EDU_volatileRead(EDU_BASE_ADDRESS + EDU_STATUS);
			if (edu_status & BCHP_EDU_STATUS_Active_MASK) {
				uint32_t edu_done = EDU_volatileRead(EDU_BASE_ADDRESS + EDU_DONE);


				// Abort current command
				EDU_volatileWrite(EDU_BASE_ADDRESS + EDU_STOP, BCHP_EDU_STOP_Stop_MASK);

				// Wait for Done to increment
				while (edu_done == EDU_volatileRead(EDU_BASE_ADDRESS + EDU_DONE))
					udelay(10);
				// Wait for Pending and Active to Clear
				while (0 != (edu_status = EDU_volatileRead(EDU_BASE_ADDRESS + EDU_STATUS)))
					udelay(10);
				// Reset Stop
				EDU_volatileWrite(EDU_BASE_ADDRESS + EDU_STOP, 0);
				// Let the process context thread handle the WAR,
				// But we need to requeue the current op (req2)
				req2 = req->list.next;
				down(&req2->lock);
				if (req2 && req2->opComplete == ISR_OP_SUBMITTED) {
					req2->opComplete = ISR_OP_QUEUED;
				}
				up(&req2->lock);
			}
		}
			
	}
	 // ReadOp completes with no errors, queue next requests until Pending is set
			

}

#endif	

/*
 * The requests are queued, some with ISR_OP_SUBMITTED status, some with ISR_OP_QUEUED
 * When an interrupt comes in, we just look for the one that are in submitted status, and mark them
 * as ISR_OP_COMPLETE, and wake up the wait queue.
 * However, if (1) there is an error that requires a workaround, or (2) that the operation is not yet completed,
 * we need to take appropriate action depending on the case.
 * In (1), we have a false uncorrectable error, that need a read from the last known good sector, 
 * so if double buffering is in effect, we need to abort the current EDU job, in order to do the workaround.
 * In (2) we just update the current job, and let the HW interrupt us again.
 * 
 * Runs in interrupt context.
 */ 
static irqreturn_t 
ISR_isr(int irq, void *devid)
{
	uint32_t status, rd_data;
	uint32_t intrMask;  
	eduIsrNode_t* req;
	//struct list_head* node;
	uint32_t flashAddr;
	unsigned long flags;
	struct brcmnand_chip* chip;

	/*
	 * Not mine
	 */
	if (devid != (void*) &gJobQ) {
		return IRQ_NONE;
	}

#ifdef CONFIG_MTD_BRCMNAND_EDU

	if (!HIF_TEST_IRQ(EDU_DONE) && !HIF_TEST_IRQ(EDU_ERR))
		return IRQ_NONE;

#elif defined(CONFIG_MTD_BRCMNAND_ISR_NOEDU)

	if (!HIF_TEST_IRQ(NAND_CTLRDY) && !HIF_TEST_IRQ(NAND_UNC) && !HIF_TEST_IRQ(NAND_CORR) 
		&& !HIF_TEST_IRQ(EBI_TIMEOUT) 
	)
		return IRQ_NONE;
#endif

	spin_lock_irqsave(&gJobQ.lock, flags);
	
	if (list_empty(&gJobQ.jobQ)) { 
		printk("%s: Impossible no job to process\n", __FUNCTION__);
		//BUG();
		// CLear interrupt and return
		intrMask = ISR_volatileRead(BCHP_HIF_INTR2_CPU_MASK_STATUS);
		ISR_disable_irq(intrMask);
		spin_unlock_irqrestore(&gJobQ.lock, flags);
		return IRQ_HANDLED;
	} 

#ifdef CONFIG_MTD_BRCMNAND_EDU	
	flashAddr = EDU_volatileRead(EDU_EXT_ADDR) - (EDU_LENGTH_VALUE-1);
	flashAddr &= ~(EDU_LENGTH_VALUE-1);
	
#elif defined(CONFIG_MTD_BRCMNAND_ISR_NOEDU)
	flashAddr = ISR_volatileRead(BCHP_NAND_FLASH_READ_ADDR);
	
#endif
	
	req = ISR_find_request(ISR_OP_SUBMITTED);

	// Paranoia
	if (!req) {
		printk("%s: Impossible failed to find queued job\n", __FUNCTION__);
		BUG();
	}

	// req->lock held here.
			
	/*
	 * Remember the status, as there can be several L1 interrupts before completion.
	 * Grab the lock first, we don't want any race condition.
	 */
	// spin_lock(&req->lock);  Already locked by ISR_find_request
	intrMask = ISR_volatileRead(BCHP_HIF_INTR2_CPU_MASK_STATUS);
	rd_data = ISR_volatileRead(BCHP_HIF_INTR2_CPU_STATUS);
	
PRINTK("==> %s: Awaken rd_data=%08x, intrMask=%08x, cmd=%d, flashAddr=%08x\n", __FUNCTION__, 
	rd_data, intrMask, gJobQ.cmd, req->edu_ldw);

	req->status |= rd_data;
	status = req->status & req->mask;
	
	/*
	 * Evaluate exit/completion condition. 
	 */
	switch (gJobQ.cmd) {
	case EDU_READ:
	case NAND_CTRL_READY:
		if  ((req->expect == (req->status & req->expect)) || 
								(req->status & req->error))
		{
			req->opComplete = ISR_OP_COMPLETED;
		}
		break;
		
	case EDU_WRITE:
		/* 
		 * We wait for both DONE|ERR +CTRL_READY
		 */
		if ((req->expect == (req->status & req->expect) ||
									(req->status & req->error))
								&&
								(req->status & HIF_INTR2_CTRL_READY))
		{
			req->opComplete = ISR_OP_COMPLETED;
#ifdef CONFIG_MTD_BRCMNAND_EDU
			(void) dma_unmap_single(NULL, req->physAddr, EDU_LENGTH_VALUE, DMA_TO_DEVICE);
#endif
		}
		break;	
		
	default:
		printk("%s: Invalid command %08x\n", __FUNCTION__, gJobQ.cmd);
		BUG();
	}
	if (ISR_OP_COMPLETED == req->opComplete) {
		int submitted;

		/* ACK interrupt */
		ISR_disable_irq(req->intr);

		// Do we need to do WAR for EDU, since EDU stop dead in its track regardless of the kind of errors.  Bummer!
		if (req->status & HIF_INTR2_EDU_ERR) {

#ifdef CONFIG_MTD_BRCMNAND_EDU
  #if (CONFIG_MTD_BRCMNAND_VERSION < CONFIG_MTD_BRCMNAND_VERS_3_3)
  
			/*
			 * We need to do WAR for EDU, which just stops dead on its tracks if there is any error, correctable or not.
			 * Problem is, the WAR needs to be done in process context,
			 * so we wake up the process context thread, and handle the WAR there.
			 */
PRINTK("%s: Awaken process context thread for EDU WAR, flashAddr=%08x, status=%08x, hif_intr2=%08x\n", 
__FUNCTION__, req->edu_ldw, req->status, HIF_INTR2_EDU_ERR);
			gJobQ.needWakeUp= 1;
			req->opComplete = ISR_OP_NEED_WAR;
			wake_up(&gIsrWaitQ);
			spin_unlock(&req->lock);
			spin_unlock_irqrestore(&gJobQ.lock, flags);
			return IRQ_HANDLED;

  #else  //EDU v3.3 or later or ISR_NOEDU, no war is needed
	/* Do nothing on platforms that do not need WAR: v3.3 or later: Just clear the error and bail */
	// gdebug=4;
			req->ret = brcmnand_edu_read_completion(req->mtd, req->buffer, req->oobarea, req->offset,
						req->status);
			if (req->ret == BRCMNAND_CORRECTABLE_ECC_ERROR) {
				gJobQ.corrected++;  // We only increment mtd->stats.corrected once per page, however.
				req->ret = 0;
			}
	// gdebug=0;
  #endif
 #elif defined(CONFIG_MTD_BRCMNAND_ISR_NOEDU)
 			/* Same as v3.3+ case, no WAR needed */
 			req->ret = brcmnand_nedu_read_completion(req->mtd, req->buffer, req->oobarea, req->offset,
						req->status);
			if (req->ret == BRCMNAND_CORRECTABLE_ECC_ERROR) {
				gJobQ.corrected++;  // We only increment mtd->stats.corrected once per page, however.
				req->ret = 0;
			}
 #endif
		}

		else {
			/*
			 * Get here only if there are no errors, call job completion routine.
			 */
			switch (gJobQ.cmd) {
			case EDU_READ:
#ifdef CONFIG_MTD_BRCMNAND_EDU
				/* All is left to do is to handle the OOB read */
				req->ret = brcmnand_edu_read_comp_intr(req->mtd, req->buffer, req->oobarea, req->offset,
							req->status);
#elif defined(CONFIG_MTD_BRCMNAND_ISR_NOEDU)
				/* We need to also copy the data from the NAND controller cache over */
				req->ret = brcmnand_nedu_read_comp_intr(req->mtd, req->buffer, req->oobarea, req->offset,
							req->status);
#endif
				break;

			case EDU_WRITE:
				{
					/*
					 * Even if there are no HIF_INTR2_ERR, we still need to check
					 * the flash status.  If it is set, we need to update the BBT
					 * which requires process context WAR
					 */
					struct brcmnand_chip *chip = req->mtd->priv;
					uint32_t flashStatus = chip->ctrl_read(BCHP_NAND_INTFC_STATUS);
					int retries=1000;

					req->needBBT=0;
					/* Just to be dead sure */

					/* 
					 * Here we already have the CTRL_READY interrupt, but the controller side is slow,
					 * so we know that the write op has completed with or without errors.
					 * The error indicator is in the flash status bit, however.
					 */
					while (!(flashStatus & BCHP_NAND_INTFC_STATUS_CTLR_READY_MASK) && retries > 0) {
						retries--;
						udelay(10); /* For a total of 10 ms */
					}
/*
					if (!(flashStatus & BCHP_NAND_INTFC_STATUS_CTLR_READY_MASK)) {
						printk("%s: Impossible, CTRL-READY already asserted\n", __FUNCTION__);
						BUG();
					}
*/
					/* Check for flash write error, in which case tell process context thread to handle it */
					if (flashStatus & 0x1) {
						req->needBBT = 1;
						gJobQ.needWakeUp= 1;
						req->opComplete = ISR_OP_NEED_WAR;
						wake_up(&gIsrWaitQ);
						spin_unlock(&req->lock);
						spin_unlock_irqrestore(&gJobQ.lock, flags);
						return IRQ_HANDLED;
					}
					/* Nothing to be done when everything is OK 
					*else
					*	req->ret = brcmnand_edu_write_completion(req->mtd, req->buffer, req->oobarea, req->offset,
					*		req->status, req->physAddr, rq->needBBT);
					*/
				}
				break;
			}
		}

		// V3.3: Jop completes with or withno errors, queue next requests until Pending is set
		list_del(&req->list);

		list_add_tail(&req->list, &gJobQ.availList);
		spin_unlock(&req->lock);
		
		submitted = brcmnand_isr_submit_job();

		if (!submitted) { /* No more job to submit, we are done, wake up process context thread */
			wake_up(&gIsrWaitQ);
		}

	}
		
	else {
		/* Ack only the ones that show */
		uint32_t ack = req->status & req->intr;
		
PRINTK("%s: opComp=0, intr=%08x, mask=%08x, expect=%08x, err=%08x, status=%08x, rd_data=%08x, intrMask=%08x, offset=%0llx\n", __FUNCTION__, 
req->intr, req->mask, req->expect, req->error, req->status, rd_data, intrMask, req->offset);

		// Just disable the ones that are triggered
		ISR_disable_irq(ack);
		req->intr &= ~ack;

		if (req->intr) {
			// Re-arm
			ISR_enable_irq(req);
		}
		else {
			printk(KERN_ERR "%s: Lost interrupt\n", __FUNCTION__);
			BUG();
		}
		spin_unlock(&req->lock);
	}
	
	spin_unlock_irqrestore(&gJobQ.lock, flags);
	
PRINTK2("<== %s: \n", __FUNCTION__);
	return IRQ_HANDLED;
}



/*
 * Called with no lock
 * Wait until the Read Queue is empty
 * Run in process context. 
 * Return 0 if all jobs complete successfully
 * Return error codes and abort if any job returned un-correctable errors.
 */
int
ISR_wait_for_queue_completion(void)
{
	//uint32_t rd_data;
//volatile unsigned int c = 0xfedeadad;
	int ret = -ERESTARTSYS;
	int waitret;
	unsigned long to_jiffies = 3*HZ; /* 3 secs */
	//unsigned long cur_jiffies = jiffies;
	unsigned long expired = jiffies + to_jiffies;
	eduIsrNode_t* req;
	eduIsrNode_t saveReq;
	int submitted;
	unsigned long flags;
	
	/* Loop is for wait_event_interruptible_timeout */
	do {
		waitret = wait_event_timeout(gIsrWaitQ, list_empty(&gJobQ.jobQ) || gJobQ.needWakeUp, to_jiffies);
		if (waitret == 0) { /* TimeOut */
			ret = BRCMNAND_TIMED_OUT;
			break;
		}
		spin_lock_irqsave(&gJobQ.lock, flags);
		if (gJobQ.needWakeUp) { /* Need to do process context WAR */			
			req = ISR_find_request(ISR_OP_NEED_WAR);

			if (!req) {
				printk("%s: Cannot find job that need WAR\n", __FUNCTION__);
				BUG();
			}

			// Make a copy 
			saveReq = *req;

			/* Mark the job as complete and free it */
			req->opComplete = ISR_OP_COMPLETED;
			gJobQ.needWakeUp = 0;
			
			// Job, with error, is now complete, remove it from queue, and submit next request
			list_del(&req->list);

			list_add_tail(&req->list, &gJobQ.availList);
			
			spin_unlock(&req->lock);

			// req lock held inside ISR_find_request
			switch (gJobQ.cmd) {
			case EDU_READ:
#ifdef CONFIG_MTD_BRCMNAND_EDU
				ret = brcmnand_edu_read_completion(
								saveReq.mtd, saveReq.buffer, saveReq.oobarea, saveReq.offset,
								saveReq.status);
#elif defined (CONFIG_MTD_BRCMNAND_ISR_NOEDU)
				ret = brcmnand_nedu_read_completion(
								saveReq.mtd, saveReq.buffer, saveReq.oobarea, saveReq.offset,
								saveReq.status);
#endif
				if (ret == BRCMNAND_CORRECTABLE_ECC_ERROR) {
					gJobQ.corrected++;
					ret = 0;
				}
				break;
			case EDU_WRITE:
				ret = brcmnand_edu_write_war(
							saveReq.mtd, saveReq.buffer, saveReq.oobarea, saveReq.offset,
							saveReq.status, saveReq.needBBT);
				break;
			default:
				printk("%s: Unknown command %d\n", __FUNCTION__, gJobQ.cmd);
				BUG();
			}
			if (ret == 0) { /* WAR worked */
				// Submit next job (which is our dummy job in WAR)
				submitted = brcmnand_isr_submit_job();
			}
			else {
				eduIsrNode_t* tmp;

				// Abort queue, TBD
				list_for_each_entry_safe(req, tmp, &gJobQ.jobQ, list) {
					list_del(&req->list);

					list_add_tail(&req->list, &gJobQ.availList);
				}
			}
		}
		else { // List is empty
			ret = 0; // Loop exit condition
		}
		spin_unlock_irqrestore(&gJobQ.lock, flags);	
	} while ((ret == -ERESTARTSYS) && time_before(jiffies, expired));
	return ret;
}


#if 0  //ndef CONFIG_MTD_BRCMNAND_ISR_QUEUE

/*
 * Wait for completion when not using queue
 */
uint32_t ISR_wait_for_completion(void)
{
	//uint32_t rd_data;
//volatile unsigned int c = 0xfedeadad;
	int ret = -ERESTARTSYS;
	unsigned long to_jiffies = 3*HZ; /* 3 secs */
	//unsigned long cur_jiffies = jiffies;
	unsigned long expired = jiffies + to_jiffies;
	int cmd;
	int retries = 2;
	//unsigned long flags;
//volatile unsigned int counter = 0xAABBCCDD;
//static int erestartsys = 0;

	
	while (ret == -ERESTARTSYS ) {
//printk("%s: jiffies=%08lx, expired=%08lx\n", __FUNCTION__, jiffies, expired);
		if (((retries--) < 0) || time_after(jiffies, expired)) {
			ret = 0; // Timed out
			return ERESTARTSYS;
		}
		else  {
			// Recalculate TO, for retries
			to_jiffies = expired - jiffies;
			//ret = wait_event_interruptible_timeout(gIsrWaitQ, gEduIsrData.opComplete, to_jiffies);
			ret = wait_event_timeout(gIsrWaitQ, gEduIsrData.opComplete, to_jiffies);
		}

PRINTK3("==>%s\n", __FUNCTION__);
		down(&gEduIsrData.lock);

		cmd = gEduIsrData.cmd;
		gEduIsrData.cmd = -1;

		if (!gEduIsrData.opComplete && ret <= 0) {
			ISR_disable_irq(gEduIsrData.intr);

			if (ret == -ERESTARTSYS) {
				up(&gEduIsrData.lock);

//if (5 >= erestartsys++)
//printk("Pending signals: %08lx-%08lx-%08lx-%08lx\n", 
//current->pending.signal.sig[0], current->pending.signal.sig[1],current->pending.signal.sig[2], current->pending.signal.sig[3]);
				continue;
			}	
			else if (ret == 0) { 
				//gEduIsrData.opComplete = 1;
				PRINTK("%s: DMA timedout\n", __FUNCTION__);

				up(&gEduIsrData.lock);
//printk("<==%s, ret=0 TimeOut\n", __FUNCTION__);
PRINTK4("<==%s, ret=0 TimeOut\n", __FUNCTION__);

				return 0; // Timed Out
			}

			
			
			// DMA completes on Done or Error.
			//rd_data = ISR_volatileRead(BCM_BASE_ADDRESS  + BCHP_HIF_INTR2_CPU_STATUS);
		
PRINTK("%s: EDU completes but Status is %08x\n", __FUNCTION__, gEduIsrData.status);
			//rd_data = 0; // Treat as a timeout
		}

		up(&gEduIsrData.lock);
	}

	return gEduIsrData.status;
}
#endif

/*
 * Since we cannot use the interrupt, or call schedule, we will have to busy-wait for controller ready.
 * Executes in interrupt context
 */
int 
ISR_cache_is_valid(void)
{
	uint32_t rd_data; 
	unsigned long expired = jiffies + HZ/10000; /* 100 usec, enough for any flash op to complete */

	do {
		rd_data = ISR_volatileRead(BCHP_HIF_INTR2_CPU_STATUS);

	} while (!(rd_data & HIF_INTR2_CTRL_READY) && time_before(jiffies, expired));
	return (0 != (rd_data & HIF_INTR2_CTRL_READY)) ;
}

void ISR_init(void)
{
	int i, ret;
	unsigned long flags;

PRINTK("-->%s\n", __FUNCTION__);

	//init_MUTEX(&gEduIsrData.lock); // Write lock
	spin_lock_init(&gJobQ.lock);		// Read queue lock
	
	INIT_LIST_HEAD(&gJobQ.jobQ);
	INIT_LIST_HEAD(&gJobQ.availList);
	/* Add all nodes from pool to avail list */

	spin_lock_irqsave(&gJobQ.lock, flags);
PRINTK("%s: B4\n", __FUNCTION__);
ISR_print_avail_list();
	for (i=0; i<MAX_JOB_QUEUE_SIZE;i++) {
		eduIsrNode_t* e = &gEduIsrPool[i];

		//init_MUTEX(&e->lock);
		list_add_tail(&e->list, &gJobQ.availList);
	}
	spin_unlock_irqrestore(&gJobQ.lock, flags);
PRINTK("%s: After\n", __FUNCTION__);
ISR_print_avail_list();
//BUG();

#ifdef CONFIG_MTD_BRCMNAND_EDU

	// Mask all L2 interrupts
	HIF_DISABLE_IRQ(EDU_DONE);
	HIF_DISABLE_IRQ(EDU_ERR);

#elif defined(CONFIG_MTD_BRCMNAND_ISR_NOEDU)


        // Clear the interrupt for next time
        ISR_volatileWrite(BCHP_HIF_INTR2_CPU_CLEAR, HIF_INTR2_EDU_CLEAR_MASK); 
PRINTK("<--%s:\n", __FUNCTION__);

	// Mask all L2 interrupts
	HIF_DISABLE_IRQ(NAND_CTLRDY);
	HIF_DISABLE_IRQ(NAND_UNC);
	HIF_DISABLE_IRQ(NAND_CORR);
	HIF_DISABLE_IRQ(EBI_TIMEOUT);


#endif

	BARRIER;

#ifdef CONFIG_MTD_BRCMNAND_EDU
	ret = request_irq(BRCM_IRQ_HIF, ISR_isr, IRQF_SHARED, "brcmnand EDU", &gJobQ);

#elif defined(CONFIG_MTD_BRCMNAND_ISR_NOEDU)
PRINTK("Calling request IRQ for no-EDU\n");
	ret = request_irq(BRCM_IRQ_HIF, ISR_isr, IRQF_SHARED, "brcmnand ISR", &gJobQ);
#endif
	if (ret) {
		printk(KERN_INFO "%s: request_irq(BRCM_IRQ_HIF) failed ret=%d.  Someone not sharing?\n", 
			__FUNCTION__, ret);
	}
}


 

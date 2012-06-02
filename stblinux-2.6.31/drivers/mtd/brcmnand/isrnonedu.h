/*
 *  drivers/mtd/brcmnand/nonedu.h
 *
    Copyright (c) 2005-2011 Broadcom Corporation                 
    
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

    File: brcmnand_base.c

    Description: 
    NAND driver with Broadcom NAND controller.
    

when	who what
-----	---	----
111123	tht	dummy out EDU routines
 */


#ifndef NONEDU_H__
#define NONEDU_H__

#include <asm/brcmstb/brcmstb.h>

#define NUMBER_OF_PASS                  256 * 8 * 10 

//#define EDU_BASE_ADDRESS                0

//#define BCHP_SUN_TOP_CTRL_STRAP_VALUE   0x0040401c
/*
 * We just use the same EDU prefix in order to share common codes.
 */

#define HIF_INTR2_EDU_DONE              BCHP_HIF_INTR2_CPU_STATUS_NAND_CTLRDY_INTR_MASK

#define HIF_INTR2_EDU_ERR              	(BCHP_HIF_INTR2_CPU_STATUS_NAND_UNC_INTR_MASK\
									| BCHP_HIF_INTR2_CPU_STATUS_NAND_CORR_INTR_MASK \
									| BCHP_HIF_INTR2_CPU_STATUS_EBI_TIMEOUT_INTR_MASK \
									)

#define HIF_INTR2_EDU_DONE_MASK    BCHP_HIF_INTR2_CPU_STATUS_NAND_CTLRDY_INTR_MASK

#define HIF_INTR2_EDU_INTR_MASK   	(BCHP_HIF_INTR2_CPU_STATUS_NAND_CTLRDY_INTR_MASK \
									| BCHP_HIF_INTR2_CPU_STATUS_NAND_UNC_INTR_MASK \
									| BCHP_HIF_INTR2_CPU_STATUS_NAND_CORR_INTR_MASK \
									| BCHP_HIF_INTR2_CPU_STATUS_EBI_TIMEOUT_INTR_MASK \
									)
									
#define HIF_INTR2_EDU_CLEAR_MASK   (BCHP_HIF_INTR2_CPU_CLEAR_NAND_CORR_INTR_MASK \
									| BCHP_HIF_INTR2_CPU_CLEAR_NAND_UNC_INTR_MASK \
									| BCHP_HIF_INTR2_CPU_CLEAR_NAND_CTLRDY_INTR_MASK \
									| BCHP_HIF_INTR2_CPU_CLEAR_EBI_TIMEOUT_INTR_MASK \
									)

#define HIF_INTR2_CTRL_READY    		BCHP_HIF_INTR2_CPU_STATUS_NAND_CTLRDY_INTR_MASK

#define HIF_INTR2_EBI_TIMEOUT		BCHP_HIF_INTR2_CPU_STATUS_EBI_TIMEOUT_INTR_MASK


/* EDU/NEDU interface */
#define EDU_WRITE			0
#define NEDU_WRITE			0
#define EDU_READ			1
#define NEDU_READ			1
#define NAND_CTRL_READY	2

// Macro for IRQ tests
//#define EDU_DONE		NAND_CTLRDY
//#define EDU_ERR

#define EDU_buffer_OK(...) (1)
#define EDU_submit_read(req) NEDU_submit_read(req)
#define EDU_submit_write(req) NEDU_submit_write(req)
static inline uint32_t EDU_volatileRead(uint32_t reg) { return BDEV_RD(reg); }
static inline void EDU_volatileWrite(uint32_t reg, uint32_t val) { BDEV_WR(reg, val); }

/* Error path completion routine */
extern int
brcmnand_nedu_read_completion(struct mtd_info* mtd, 
        void* buffer, u_char* oobarea, loff_t offset, uint32_t intr_status);


/* Success path completion routine */
extern int
brcmnand_nedu_read_comp_intr(struct mtd_info* mtd, 
        void* buffer, u_char* oobarea, loff_t offset, uint32_t intr_status);

static inline void ISR_noEDU_Init(void)
{
	ISR_init();
}


#endif



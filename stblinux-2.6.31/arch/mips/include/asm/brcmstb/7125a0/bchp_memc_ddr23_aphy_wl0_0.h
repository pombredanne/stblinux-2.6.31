/***************************************************************************
 *     Copyright (c) 1999-2009, Broadcom Corporation
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
 * Module Description:
 *                     DO NOT EDIT THIS FILE DIRECTLY
 *
 * This module was generated magically with RDB from a source description
 * file. You must edit the source file for changes to be made to this file.
 *
 *
 * Date:           Generated on         Mon Apr  6 16:49:15 2009
 *                 MD5 Checksum         23f012e8625b409b261f44ab79228a7b
 *
 * Compiled with:  RDB Utility          combo_header.pl
 *                 RDB Parser           3.0
 *                 unknown              unknown
 *                 Perl Interpreter     5.008008
 *                 Operating System     linux
 *
 * Revision History:
 *
 * $brcm_Log: /magnum/basemodules/chp/7125/rdb/a0/bchp_memc_ddr23_aphy_wl0_0.h $
 * 
 * Hydra_Software_Devel/1   4/6/09 8:37p albertl
 * PR53730: Initial revision.
 *
 ***************************************************************************/

#ifndef BCHP_MEMC_DDR23_APHY_WL0_0_H__
#define BCHP_MEMC_DDR23_APHY_WL0_0_H__

/***************************************************************************
 *MEMC_DDR23_APHY_WL0_0 - DDR23 APHY Wordlane 0 Control Registers 0
 ***************************************************************************/
#define BCHP_MEMC_DDR23_APHY_WL0_0_WORD_SLICE_DLL_RESET 0x003b7800 /* WORDSLICE DLL reset register */
#define BCHP_MEMC_DDR23_APHY_WL0_0_WORDSLICE_CNTRL_0 0x003b7804 /* Analog WORDSLICE Control register */
#define BCHP_MEMC_DDR23_APHY_WL0_0_WORDSLICE_CNTRL_1 0x003b7808 /* Analog WORDSLICE Control register */
#define BCHP_MEMC_DDR23_APHY_WL0_0_BYTE0_VCDL_PHASE_CNTL 0x003b7810 /* VCDL Phase Control Register for bytelane 0 */
#define BCHP_MEMC_DDR23_APHY_WL0_0_BYTE1_VCDL_PHASE_CNTL 0x003b7814 /* VCDL Phase Control Register for bytelane 1 */
#define BCHP_MEMC_DDR23_APHY_WL0_0_READ_DQS_GATE_CNTRL 0x003b7818 /* Read DQS gating control register */
#define BCHP_MEMC_DDR23_APHY_WL0_0_RX_ODT_CNTRL  0x003b781c /* "Receive ODT control register" */
#define BCHP_MEMC_DDR23_APHY_WL0_0_ANALOG_BYPASS_CNTRL 0x003b7820 /* Analog macro register bypass control */
#define BCHP_MEMC_DDR23_APHY_WL0_0_PFIFO_RD_WR_PNTR 0x003b7824 /* DQS read fifo (PFIFO) read & write pointers for debug purpose */
#define BCHP_MEMC_DDR23_APHY_WL0_0_PAD_SSTL_DDR2_MODE 0x003b7828 /* Pad Mode Control Register */
#define BCHP_MEMC_DDR23_APHY_WL0_0_DDR_PAD_CNTRL 0x003b782c /* DDR Pad control register */
#define BCHP_MEMC_DDR23_APHY_WL0_0_DDR_PAD_SLEW_CNTRL 0x003b7830 /* DDR Pad slew control register */
#define BCHP_MEMC_DDR23_APHY_WL0_0_DDR_PAD_RX_DRV_CNTRL 0x003b7834 /* DDR Pad Rx power control register */
#define BCHP_MEMC_DDR23_APHY_WL0_0_DDR_PAD_TX_DRV_CNTRL 0x003b7838 /* DDR Pad Tx power control register */
#define BCHP_MEMC_DDR23_APHY_WL0_0_MISC          0x003b783c /* MiscellaneousDDR register */
#define BCHP_MEMC_DDR23_APHY_WL0_0_SPARE0_RW     0x003b7840 /* Spare register */
#define BCHP_MEMC_DDR23_APHY_WL0_0_SPARE0_RO     0x003b7844 /* Spare register */

#endif /* #ifndef BCHP_MEMC_DDR23_APHY_WL0_0_H__ */

/* End of File */

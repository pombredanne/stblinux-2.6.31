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
 * Date:           Generated on         Mon Apr  6 16:44:20 2009
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
 * $brcm_Log: /magnum/basemodules/chp/7125/rdb/a0/bchp_bspi.h $
 * 
 * Hydra_Software_Devel/1   4/6/09 7:42p albertl
 * PR53730: Initial revision.
 *
 ***************************************************************************/

#ifndef BCHP_BSPI_H__
#define BCHP_BSPI_H__

/***************************************************************************
 *BSPI - Public BSPI Control Registers
 ***************************************************************************/
#define BCHP_BSPI_REVISION_ID                    0x00443000 /* Revision ID */
#define BCHP_BSPI_SCRATCH                        0x00443004 /* Revision ID */
#define BCHP_BSPI_MAST_N_BOOT_CTRL               0x00443008 /* Master/Boot SPI Control Register */
#define BCHP_BSPI_BUSY_STATUS                    0x0044300c /* BSPI Busy Status Register */
#define BCHP_BSPI_INTR_STATUS                    0x00443010 /* Interrupt Status Register */
#define BCHP_BSPI_B0_STATUS                      0x00443014 /* Prefetch Buffer 0 Status Register */
#define BCHP_BSPI_B0_CTRL                        0x00443018 /* Prefetch Buffer 0 Control Register */
#define BCHP_BSPI_B1_STATUS                      0x0044301c /* Prefetch Buffer 1 Status Register */
#define BCHP_BSPI_B1_CTRL                        0x00443020 /* Prefetch Buffer 1 Control Register */
#define BCHP_BSPI_STRAP_OVERRIDE_CTRL            0x00443024 /* Dual/Single Receive Mode Control Register */

/***************************************************************************
 *REVISION_ID - Revision ID
 ***************************************************************************/
/* BSPI :: REVISION_ID :: reserved0 [31:16] */
#define BCHP_BSPI_REVISION_ID_reserved0_MASK                       0xffff0000
#define BCHP_BSPI_REVISION_ID_reserved0_SHIFT                      16

/* BSPI :: REVISION_ID :: MAJOR [15:08] */
#define BCHP_BSPI_REVISION_ID_MAJOR_MASK                           0x0000ff00
#define BCHP_BSPI_REVISION_ID_MAJOR_SHIFT                          8

/* BSPI :: REVISION_ID :: MINOR [07:00] */
#define BCHP_BSPI_REVISION_ID_MINOR_MASK                           0x000000ff
#define BCHP_BSPI_REVISION_ID_MINOR_SHIFT                          0

/***************************************************************************
 *SCRATCH - Revision ID
 ***************************************************************************/
/* BSPI :: SCRATCH :: SCRATCH [31:00] */
#define BCHP_BSPI_SCRATCH_SCRATCH_MASK                             0xffffffff
#define BCHP_BSPI_SCRATCH_SCRATCH_SHIFT                            0

/***************************************************************************
 *MAST_N_BOOT_CTRL - Master/Boot SPI Control Register
 ***************************************************************************/
/* BSPI :: MAST_N_BOOT_CTRL :: reserved0 [31:01] */
#define BCHP_BSPI_MAST_N_BOOT_CTRL_reserved0_MASK                  0xfffffffe
#define BCHP_BSPI_MAST_N_BOOT_CTRL_reserved0_SHIFT                 1

/* BSPI :: MAST_N_BOOT_CTRL :: mast_n_boot [00:00] */
#define BCHP_BSPI_MAST_N_BOOT_CTRL_mast_n_boot_MASK                0x00000001
#define BCHP_BSPI_MAST_N_BOOT_CTRL_mast_n_boot_SHIFT               0

/***************************************************************************
 *BUSY_STATUS - BSPI Busy Status Register
 ***************************************************************************/
/* BSPI :: BUSY_STATUS :: reserved0 [31:01] */
#define BCHP_BSPI_BUSY_STATUS_reserved0_MASK                       0xfffffffe
#define BCHP_BSPI_BUSY_STATUS_reserved0_SHIFT                      1

/* BSPI :: BUSY_STATUS :: busy [00:00] */
#define BCHP_BSPI_BUSY_STATUS_busy_MASK                            0x00000001
#define BCHP_BSPI_BUSY_STATUS_busy_SHIFT                           0

/***************************************************************************
 *INTR_STATUS - Interrupt Status Register
 ***************************************************************************/
/* BSPI :: INTR_STATUS :: reserved0 [31:02] */
#define BCHP_BSPI_INTR_STATUS_reserved0_MASK                       0xfffffffc
#define BCHP_BSPI_INTR_STATUS_reserved0_SHIFT                      2

/* BSPI :: INTR_STATUS :: intr_1 [01:01] */
#define BCHP_BSPI_INTR_STATUS_intr_1_MASK                          0x00000002
#define BCHP_BSPI_INTR_STATUS_intr_1_SHIFT                         1

/* BSPI :: INTR_STATUS :: intr_0 [00:00] */
#define BCHP_BSPI_INTR_STATUS_intr_0_MASK                          0x00000001
#define BCHP_BSPI_INTR_STATUS_intr_0_SHIFT                         0

/***************************************************************************
 *B0_STATUS - Prefetch Buffer 0 Status Register
 ***************************************************************************/
/* BSPI :: B0_STATUS :: reserved0 [31:31] */
#define BCHP_BSPI_B0_STATUS_reserved0_MASK                         0x80000000
#define BCHP_BSPI_B0_STATUS_reserved0_SHIFT                        31

/* BSPI :: B0_STATUS :: b0_prefetch_active [30:30] */
#define BCHP_BSPI_B0_STATUS_b0_prefetch_active_MASK                0x40000000
#define BCHP_BSPI_B0_STATUS_b0_prefetch_active_SHIFT               30

/* BSPI :: B0_STATUS :: b0_full [29:29] */
#define BCHP_BSPI_B0_STATUS_b0_full_MASK                           0x20000000
#define BCHP_BSPI_B0_STATUS_b0_full_SHIFT                          29

/* BSPI :: B0_STATUS :: b0_empty [28:28] */
#define BCHP_BSPI_B0_STATUS_b0_empty_MASK                          0x10000000
#define BCHP_BSPI_B0_STATUS_b0_empty_SHIFT                         28

/* BSPI :: B0_STATUS :: b0_miss [27:27] */
#define BCHP_BSPI_B0_STATUS_b0_miss_MASK                           0x08000000
#define BCHP_BSPI_B0_STATUS_b0_miss_SHIFT                          27

/* BSPI :: B0_STATUS :: b0_hit [26:26] */
#define BCHP_BSPI_B0_STATUS_b0_hit_MASK                            0x04000000
#define BCHP_BSPI_B0_STATUS_b0_hit_SHIFT                           26

/* BSPI :: B0_STATUS :: b0_address [25:00] */
#define BCHP_BSPI_B0_STATUS_b0_address_MASK                        0x03ffffff
#define BCHP_BSPI_B0_STATUS_b0_address_SHIFT                       0

/***************************************************************************
 *B0_CTRL - Prefetch Buffer 0 Control Register
 ***************************************************************************/
/* BSPI :: B0_CTRL :: reserved0 [31:01] */
#define BCHP_BSPI_B0_CTRL_reserved0_MASK                           0xfffffffe
#define BCHP_BSPI_B0_CTRL_reserved0_SHIFT                          1

/* BSPI :: B0_CTRL :: b0_flush [00:00] */
#define BCHP_BSPI_B0_CTRL_b0_flush_MASK                            0x00000001
#define BCHP_BSPI_B0_CTRL_b0_flush_SHIFT                           0

/***************************************************************************
 *B1_STATUS - Prefetch Buffer 1 Status Register
 ***************************************************************************/
/* BSPI :: B1_STATUS :: reserved0 [31:31] */
#define BCHP_BSPI_B1_STATUS_reserved0_MASK                         0x80000000
#define BCHP_BSPI_B1_STATUS_reserved0_SHIFT                        31

/* BSPI :: B1_STATUS :: b1_prefetch_active [30:30] */
#define BCHP_BSPI_B1_STATUS_b1_prefetch_active_MASK                0x40000000
#define BCHP_BSPI_B1_STATUS_b1_prefetch_active_SHIFT               30

/* BSPI :: B1_STATUS :: b1_full [29:29] */
#define BCHP_BSPI_B1_STATUS_b1_full_MASK                           0x20000000
#define BCHP_BSPI_B1_STATUS_b1_full_SHIFT                          29

/* BSPI :: B1_STATUS :: b1_empty [28:28] */
#define BCHP_BSPI_B1_STATUS_b1_empty_MASK                          0x10000000
#define BCHP_BSPI_B1_STATUS_b1_empty_SHIFT                         28

/* BSPI :: B1_STATUS :: b1_miss [27:27] */
#define BCHP_BSPI_B1_STATUS_b1_miss_MASK                           0x08000000
#define BCHP_BSPI_B1_STATUS_b1_miss_SHIFT                          27

/* BSPI :: B1_STATUS :: b1_hit [26:26] */
#define BCHP_BSPI_B1_STATUS_b1_hit_MASK                            0x04000000
#define BCHP_BSPI_B1_STATUS_b1_hit_SHIFT                           26

/* BSPI :: B1_STATUS :: b1_address [25:00] */
#define BCHP_BSPI_B1_STATUS_b1_address_MASK                        0x03ffffff
#define BCHP_BSPI_B1_STATUS_b1_address_SHIFT                       0

/***************************************************************************
 *B1_CTRL - Prefetch Buffer 1 Control Register
 ***************************************************************************/
/* BSPI :: B1_CTRL :: reserved0 [31:01] */
#define BCHP_BSPI_B1_CTRL_reserved0_MASK                           0xfffffffe
#define BCHP_BSPI_B1_CTRL_reserved0_SHIFT                          1

/* BSPI :: B1_CTRL :: b1_flush [00:00] */
#define BCHP_BSPI_B1_CTRL_b1_flush_MASK                            0x00000001
#define BCHP_BSPI_B1_CTRL_b1_flush_SHIFT                           0

/***************************************************************************
 *STRAP_OVERRIDE_CTRL - Dual/Single Receive Mode Control Register
 ***************************************************************************/
/* BSPI :: STRAP_OVERRIDE_CTRL :: reserved0 [31:02] */
#define BCHP_BSPI_STRAP_OVERRIDE_CTRL_reserved0_MASK               0xfffffffc
#define BCHP_BSPI_STRAP_OVERRIDE_CTRL_reserved0_SHIFT              2

/* BSPI :: STRAP_OVERRIDE_CTRL :: data_dual_n_sgl [01:01] */
#define BCHP_BSPI_STRAP_OVERRIDE_CTRL_data_dual_n_sgl_MASK         0x00000002
#define BCHP_BSPI_STRAP_OVERRIDE_CTRL_data_dual_n_sgl_SHIFT        1

/* BSPI :: STRAP_OVERRIDE_CTRL :: override [00:00] */
#define BCHP_BSPI_STRAP_OVERRIDE_CTRL_override_MASK                0x00000001
#define BCHP_BSPI_STRAP_OVERRIDE_CTRL_override_SHIFT               0

#endif /* #ifndef BCHP_BSPI_H__ */

/* End of File */
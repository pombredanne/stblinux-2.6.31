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
 * Date:           Generated on         Fri Sep 11 19:45:40 2009
 *                 MD5 Checksum         957f01e03a68c1766fd2e8ad6484f5f9
 *
 * Compiled with:  RDB Utility          combo_header.pl
 *                 RDB Parser           3.0
 *                 unknown              unknown
 *                 Perl Interpreter     5.008008
 *                 Operating System     linux
 *
 * Revision History:
 *
 * $brcm_Log: /magnum/basemodules/chp/7468/rdb/a0/bchp_hif_intr2.h $
 * 
 * Hydra_Software_Devel/1   9/14/09 4:31p albertl
 * SW7468-3: Initial revision.
 *
 ***************************************************************************/

#ifndef BCHP_HIF_INTR2_H__
#define BCHP_HIF_INTR2_H__

/***************************************************************************
 *HIF_INTR2 - HIF Level 2 Interrupt Controller Registers
 ***************************************************************************/
#define BCHP_HIF_INTR2_CPU_STATUS                0x00311000 /* CPU interrupt Status Register */
#define BCHP_HIF_INTR2_CPU_SET                   0x00311004 /* CPU interrupt Set Register */
#define BCHP_HIF_INTR2_CPU_CLEAR                 0x00311008 /* CPU interrupt Clear Register */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS           0x0031100c /* CPU interrupt Mask Status Register */
#define BCHP_HIF_INTR2_CPU_MASK_SET              0x00311010 /* CPU interrupt Mask Set Register */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR            0x00311014 /* CPU interrupt Mask Clear Register */

/***************************************************************************
 *CPU_STATUS - CPU interrupt Status Register
 ***************************************************************************/
/* HIF_INTR2 :: CPU_STATUS :: reserved0 [31:31] */
#define BCHP_HIF_INTR2_CPU_STATUS_reserved0_MASK                   0x80000000
#define BCHP_HIF_INTR2_CPU_STATUS_reserved0_SHIFT                  31

/* HIF_INTR2 :: CPU_STATUS :: SDIO_INTR [30:30] */
#define BCHP_HIF_INTR2_CPU_STATUS_SDIO_INTR_MASK                   0x40000000
#define BCHP_HIF_INTR2_CPU_STATUS_SDIO_INTR_SHIFT                  30

/* HIF_INTR2 :: CPU_STATUS :: EDU_DONE_INTR [29:29] */
#define BCHP_HIF_INTR2_CPU_STATUS_EDU_DONE_INTR_MASK               0x20000000
#define BCHP_HIF_INTR2_CPU_STATUS_EDU_DONE_INTR_SHIFT              29

/* HIF_INTR2 :: CPU_STATUS :: EDU_ERR_INTR [28:28] */
#define BCHP_HIF_INTR2_CPU_STATUS_EDU_ERR_INTR_MASK                0x10000000
#define BCHP_HIF_INTR2_CPU_STATUS_EDU_ERR_INTR_SHIFT               28

/* HIF_INTR2 :: CPU_STATUS :: NAND_CORR_INTR [27:27] */
#define BCHP_HIF_INTR2_CPU_STATUS_NAND_CORR_INTR_MASK              0x08000000
#define BCHP_HIF_INTR2_CPU_STATUS_NAND_CORR_INTR_SHIFT             27

/* HIF_INTR2 :: CPU_STATUS :: NAND_UNC_INTR [26:26] */
#define BCHP_HIF_INTR2_CPU_STATUS_NAND_UNC_INTR_MASK               0x04000000
#define BCHP_HIF_INTR2_CPU_STATUS_NAND_UNC_INTR_SHIFT              26

/* HIF_INTR2 :: CPU_STATUS :: NAND_RBPIN_INTR [25:25] */
#define BCHP_HIF_INTR2_CPU_STATUS_NAND_RBPIN_INTR_MASK             0x02000000
#define BCHP_HIF_INTR2_CPU_STATUS_NAND_RBPIN_INTR_SHIFT            25

/* HIF_INTR2 :: CPU_STATUS :: NAND_CTLRDY_INTR [24:24] */
#define BCHP_HIF_INTR2_CPU_STATUS_NAND_CTLRDY_INTR_MASK            0x01000000
#define BCHP_HIF_INTR2_CPU_STATUS_NAND_CTLRDY_INTR_SHIFT           24

/* HIF_INTR2 :: CPU_STATUS :: NAND_PGMPG_INTR [23:23] */
#define BCHP_HIF_INTR2_CPU_STATUS_NAND_PGMPG_INTR_MASK             0x00800000
#define BCHP_HIF_INTR2_CPU_STATUS_NAND_PGMPG_INTR_SHIFT            23

/* HIF_INTR2 :: CPU_STATUS :: NAND_CPYBK_INTR [22:22] */
#define BCHP_HIF_INTR2_CPU_STATUS_NAND_CPYBK_INTR_MASK             0x00400000
#define BCHP_HIF_INTR2_CPU_STATUS_NAND_CPYBK_INTR_SHIFT            22

/* HIF_INTR2 :: CPU_STATUS :: NAND_BLKERA_INTR [21:21] */
#define BCHP_HIF_INTR2_CPU_STATUS_NAND_BLKERA_INTR_MASK            0x00200000
#define BCHP_HIF_INTR2_CPU_STATUS_NAND_BLKERA_INTR_SHIFT           21

/* HIF_INTR2 :: CPU_STATUS :: NAND_NP_READ_INTR [20:20] */
#define BCHP_HIF_INTR2_CPU_STATUS_NAND_NP_READ_INTR_MASK           0x00100000
#define BCHP_HIF_INTR2_CPU_STATUS_NAND_NP_READ_INTR_SHIFT          20

/* HIF_INTR2 :: CPU_STATUS :: MICH_INST0_RD_INTR [19:19] */
#define BCHP_HIF_INTR2_CPU_STATUS_MICH_INST0_RD_INTR_MASK          0x00080000
#define BCHP_HIF_INTR2_CPU_STATUS_MICH_INST0_RD_INTR_SHIFT         19

/* HIF_INTR2 :: CPU_STATUS :: MICH_WR_INTR [18:18] */
#define BCHP_HIF_INTR2_CPU_STATUS_MICH_WR_INTR_MASK                0x00040000
#define BCHP_HIF_INTR2_CPU_STATUS_MICH_WR_INTR_SHIFT               18

/* HIF_INTR2 :: CPU_STATUS :: reserved1 [17:01] */
#define BCHP_HIF_INTR2_CPU_STATUS_reserved1_MASK                   0x0003fffe
#define BCHP_HIF_INTR2_CPU_STATUS_reserved1_SHIFT                  1

/* HIF_INTR2 :: CPU_STATUS :: HIF_RGR2_BRIDGE_INTR [00:00] */
#define BCHP_HIF_INTR2_CPU_STATUS_HIF_RGR2_BRIDGE_INTR_MASK        0x00000001
#define BCHP_HIF_INTR2_CPU_STATUS_HIF_RGR2_BRIDGE_INTR_SHIFT       0

/***************************************************************************
 *CPU_SET - CPU interrupt Set Register
 ***************************************************************************/
/* HIF_INTR2 :: CPU_SET :: reserved0 [31:31] */
#define BCHP_HIF_INTR2_CPU_SET_reserved0_MASK                      0x80000000
#define BCHP_HIF_INTR2_CPU_SET_reserved0_SHIFT                     31

/* HIF_INTR2 :: CPU_SET :: SDIO_INTR [30:30] */
#define BCHP_HIF_INTR2_CPU_SET_SDIO_INTR_MASK                      0x40000000
#define BCHP_HIF_INTR2_CPU_SET_SDIO_INTR_SHIFT                     30

/* HIF_INTR2 :: CPU_SET :: EDU_DONE_INTR [29:29] */
#define BCHP_HIF_INTR2_CPU_SET_EDU_DONE_INTR_MASK                  0x20000000
#define BCHP_HIF_INTR2_CPU_SET_EDU_DONE_INTR_SHIFT                 29

/* HIF_INTR2 :: CPU_SET :: EDU_ERR_INTR [28:28] */
#define BCHP_HIF_INTR2_CPU_SET_EDU_ERR_INTR_MASK                   0x10000000
#define BCHP_HIF_INTR2_CPU_SET_EDU_ERR_INTR_SHIFT                  28

/* HIF_INTR2 :: CPU_SET :: NAND_CORR_INTR [27:27] */
#define BCHP_HIF_INTR2_CPU_SET_NAND_CORR_INTR_MASK                 0x08000000
#define BCHP_HIF_INTR2_CPU_SET_NAND_CORR_INTR_SHIFT                27

/* HIF_INTR2 :: CPU_SET :: NAND_UNC_INTR [26:26] */
#define BCHP_HIF_INTR2_CPU_SET_NAND_UNC_INTR_MASK                  0x04000000
#define BCHP_HIF_INTR2_CPU_SET_NAND_UNC_INTR_SHIFT                 26

/* HIF_INTR2 :: CPU_SET :: NAND_RBPIN_INTR [25:25] */
#define BCHP_HIF_INTR2_CPU_SET_NAND_RBPIN_INTR_MASK                0x02000000
#define BCHP_HIF_INTR2_CPU_SET_NAND_RBPIN_INTR_SHIFT               25

/* HIF_INTR2 :: CPU_SET :: NAND_CTLRDY_INTR [24:24] */
#define BCHP_HIF_INTR2_CPU_SET_NAND_CTLRDY_INTR_MASK               0x01000000
#define BCHP_HIF_INTR2_CPU_SET_NAND_CTLRDY_INTR_SHIFT              24

/* HIF_INTR2 :: CPU_SET :: NAND_PGMPG_INTR [23:23] */
#define BCHP_HIF_INTR2_CPU_SET_NAND_PGMPG_INTR_MASK                0x00800000
#define BCHP_HIF_INTR2_CPU_SET_NAND_PGMPG_INTR_SHIFT               23

/* HIF_INTR2 :: CPU_SET :: NAND_CPYBK_INTR [22:22] */
#define BCHP_HIF_INTR2_CPU_SET_NAND_CPYBK_INTR_MASK                0x00400000
#define BCHP_HIF_INTR2_CPU_SET_NAND_CPYBK_INTR_SHIFT               22

/* HIF_INTR2 :: CPU_SET :: NAND_BLKERA_INTR [21:21] */
#define BCHP_HIF_INTR2_CPU_SET_NAND_BLKERA_INTR_MASK               0x00200000
#define BCHP_HIF_INTR2_CPU_SET_NAND_BLKERA_INTR_SHIFT              21

/* HIF_INTR2 :: CPU_SET :: NAND_NP_READ_INTR [20:20] */
#define BCHP_HIF_INTR2_CPU_SET_NAND_NP_READ_INTR_MASK              0x00100000
#define BCHP_HIF_INTR2_CPU_SET_NAND_NP_READ_INTR_SHIFT             20

/* HIF_INTR2 :: CPU_SET :: MICH_INST0_RD_INTR [19:19] */
#define BCHP_HIF_INTR2_CPU_SET_MICH_INST0_RD_INTR_MASK             0x00080000
#define BCHP_HIF_INTR2_CPU_SET_MICH_INST0_RD_INTR_SHIFT            19

/* HIF_INTR2 :: CPU_SET :: MICH_WR_INTR [18:18] */
#define BCHP_HIF_INTR2_CPU_SET_MICH_WR_INTR_MASK                   0x00040000
#define BCHP_HIF_INTR2_CPU_SET_MICH_WR_INTR_SHIFT                  18

/* HIF_INTR2 :: CPU_SET :: reserved1 [17:01] */
#define BCHP_HIF_INTR2_CPU_SET_reserved1_MASK                      0x0003fffe
#define BCHP_HIF_INTR2_CPU_SET_reserved1_SHIFT                     1

/* HIF_INTR2 :: CPU_SET :: HIF_RGR2_BRIDGE_INTR [00:00] */
#define BCHP_HIF_INTR2_CPU_SET_HIF_RGR2_BRIDGE_INTR_MASK           0x00000001
#define BCHP_HIF_INTR2_CPU_SET_HIF_RGR2_BRIDGE_INTR_SHIFT          0

/***************************************************************************
 *CPU_CLEAR - CPU interrupt Clear Register
 ***************************************************************************/
/* HIF_INTR2 :: CPU_CLEAR :: reserved0 [31:31] */
#define BCHP_HIF_INTR2_CPU_CLEAR_reserved0_MASK                    0x80000000
#define BCHP_HIF_INTR2_CPU_CLEAR_reserved0_SHIFT                   31

/* HIF_INTR2 :: CPU_CLEAR :: SDIO_INTR [30:30] */
#define BCHP_HIF_INTR2_CPU_CLEAR_SDIO_INTR_MASK                    0x40000000
#define BCHP_HIF_INTR2_CPU_CLEAR_SDIO_INTR_SHIFT                   30

/* HIF_INTR2 :: CPU_CLEAR :: EDU_DONE_INTR [29:29] */
#define BCHP_HIF_INTR2_CPU_CLEAR_EDU_DONE_INTR_MASK                0x20000000
#define BCHP_HIF_INTR2_CPU_CLEAR_EDU_DONE_INTR_SHIFT               29

/* HIF_INTR2 :: CPU_CLEAR :: EDU_ERR_INTR [28:28] */
#define BCHP_HIF_INTR2_CPU_CLEAR_EDU_ERR_INTR_MASK                 0x10000000
#define BCHP_HIF_INTR2_CPU_CLEAR_EDU_ERR_INTR_SHIFT                28

/* HIF_INTR2 :: CPU_CLEAR :: NAND_CORR_INTR [27:27] */
#define BCHP_HIF_INTR2_CPU_CLEAR_NAND_CORR_INTR_MASK               0x08000000
#define BCHP_HIF_INTR2_CPU_CLEAR_NAND_CORR_INTR_SHIFT              27

/* HIF_INTR2 :: CPU_CLEAR :: NAND_UNC_INTR [26:26] */
#define BCHP_HIF_INTR2_CPU_CLEAR_NAND_UNC_INTR_MASK                0x04000000
#define BCHP_HIF_INTR2_CPU_CLEAR_NAND_UNC_INTR_SHIFT               26

/* HIF_INTR2 :: CPU_CLEAR :: NAND_RBPIN_INTR [25:25] */
#define BCHP_HIF_INTR2_CPU_CLEAR_NAND_RBPIN_INTR_MASK              0x02000000
#define BCHP_HIF_INTR2_CPU_CLEAR_NAND_RBPIN_INTR_SHIFT             25

/* HIF_INTR2 :: CPU_CLEAR :: NAND_CTLRDY_INTR [24:24] */
#define BCHP_HIF_INTR2_CPU_CLEAR_NAND_CTLRDY_INTR_MASK             0x01000000
#define BCHP_HIF_INTR2_CPU_CLEAR_NAND_CTLRDY_INTR_SHIFT            24

/* HIF_INTR2 :: CPU_CLEAR :: NAND_PGMPG_INTR [23:23] */
#define BCHP_HIF_INTR2_CPU_CLEAR_NAND_PGMPG_INTR_MASK              0x00800000
#define BCHP_HIF_INTR2_CPU_CLEAR_NAND_PGMPG_INTR_SHIFT             23

/* HIF_INTR2 :: CPU_CLEAR :: NAND_CPYBK_INTR [22:22] */
#define BCHP_HIF_INTR2_CPU_CLEAR_NAND_CPYBK_INTR_MASK              0x00400000
#define BCHP_HIF_INTR2_CPU_CLEAR_NAND_CPYBK_INTR_SHIFT             22

/* HIF_INTR2 :: CPU_CLEAR :: NAND_BLKERA_INTR [21:21] */
#define BCHP_HIF_INTR2_CPU_CLEAR_NAND_BLKERA_INTR_MASK             0x00200000
#define BCHP_HIF_INTR2_CPU_CLEAR_NAND_BLKERA_INTR_SHIFT            21

/* HIF_INTR2 :: CPU_CLEAR :: NAND_NP_READ_INTR [20:20] */
#define BCHP_HIF_INTR2_CPU_CLEAR_NAND_NP_READ_INTR_MASK            0x00100000
#define BCHP_HIF_INTR2_CPU_CLEAR_NAND_NP_READ_INTR_SHIFT           20

/* HIF_INTR2 :: CPU_CLEAR :: MICH_INST0_RD_INTR [19:19] */
#define BCHP_HIF_INTR2_CPU_CLEAR_MICH_INST0_RD_INTR_MASK           0x00080000
#define BCHP_HIF_INTR2_CPU_CLEAR_MICH_INST0_RD_INTR_SHIFT          19

/* HIF_INTR2 :: CPU_CLEAR :: MICH_WR_INTR [18:18] */
#define BCHP_HIF_INTR2_CPU_CLEAR_MICH_WR_INTR_MASK                 0x00040000
#define BCHP_HIF_INTR2_CPU_CLEAR_MICH_WR_INTR_SHIFT                18

/* HIF_INTR2 :: CPU_CLEAR :: reserved1 [17:01] */
#define BCHP_HIF_INTR2_CPU_CLEAR_reserved1_MASK                    0x0003fffe
#define BCHP_HIF_INTR2_CPU_CLEAR_reserved1_SHIFT                   1

/* HIF_INTR2 :: CPU_CLEAR :: HIF_RGR2_BRIDGE_INTR [00:00] */
#define BCHP_HIF_INTR2_CPU_CLEAR_HIF_RGR2_BRIDGE_INTR_MASK         0x00000001
#define BCHP_HIF_INTR2_CPU_CLEAR_HIF_RGR2_BRIDGE_INTR_SHIFT        0

/***************************************************************************
 *CPU_MASK_STATUS - CPU interrupt Mask Status Register
 ***************************************************************************/
/* HIF_INTR2 :: CPU_MASK_STATUS :: reserved0 [31:31] */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_reserved0_MASK              0x80000000
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_reserved0_SHIFT             31

/* HIF_INTR2 :: CPU_MASK_STATUS :: SDIO_INTR [30:30] */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_SDIO_INTR_MASK              0x40000000
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_SDIO_INTR_SHIFT             30

/* HIF_INTR2 :: CPU_MASK_STATUS :: EDU_DONE_INTR [29:29] */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_EDU_DONE_INTR_MASK          0x20000000
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_EDU_DONE_INTR_SHIFT         29

/* HIF_INTR2 :: CPU_MASK_STATUS :: EDU_ERR_INTR [28:28] */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_EDU_ERR_INTR_MASK           0x10000000
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_EDU_ERR_INTR_SHIFT          28

/* HIF_INTR2 :: CPU_MASK_STATUS :: NAND_CORR_INTR [27:27] */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_NAND_CORR_INTR_MASK         0x08000000
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_NAND_CORR_INTR_SHIFT        27

/* HIF_INTR2 :: CPU_MASK_STATUS :: NAND_UNC_INTR [26:26] */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_NAND_UNC_INTR_MASK          0x04000000
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_NAND_UNC_INTR_SHIFT         26

/* HIF_INTR2 :: CPU_MASK_STATUS :: NAND_RBPIN_INTR [25:25] */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_NAND_RBPIN_INTR_MASK        0x02000000
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_NAND_RBPIN_INTR_SHIFT       25

/* HIF_INTR2 :: CPU_MASK_STATUS :: NAND_CTLRDY_INTR [24:24] */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_NAND_CTLRDY_INTR_MASK       0x01000000
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_NAND_CTLRDY_INTR_SHIFT      24

/* HIF_INTR2 :: CPU_MASK_STATUS :: NAND_PGMPG_INTR [23:23] */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_NAND_PGMPG_INTR_MASK        0x00800000
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_NAND_PGMPG_INTR_SHIFT       23

/* HIF_INTR2 :: CPU_MASK_STATUS :: NAND_CPYBK_INTR [22:22] */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_NAND_CPYBK_INTR_MASK        0x00400000
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_NAND_CPYBK_INTR_SHIFT       22

/* HIF_INTR2 :: CPU_MASK_STATUS :: NAND_BLKERA_INTR [21:21] */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_NAND_BLKERA_INTR_MASK       0x00200000
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_NAND_BLKERA_INTR_SHIFT      21

/* HIF_INTR2 :: CPU_MASK_STATUS :: NAND_NP_READ_INTR [20:20] */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_NAND_NP_READ_INTR_MASK      0x00100000
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_NAND_NP_READ_INTR_SHIFT     20

/* HIF_INTR2 :: CPU_MASK_STATUS :: reserved1 [19:19] */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_reserved1_MASK              0x00080000
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_reserved1_SHIFT             19

/* HIF_INTR2 :: CPU_MASK_STATUS :: MICH_WR_INTR [18:18] */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_MICH_WR_INTR_MASK           0x00040000
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_MICH_WR_INTR_SHIFT          18

/* HIF_INTR2 :: CPU_MASK_STATUS :: reserved2 [17:16] */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_reserved2_MASK              0x00030000
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_reserved2_SHIFT             16

/* HIF_INTR2 :: CPU_MASK_STATUS :: MICH_INST0_RD_INTR [15:15] */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_MICH_INST0_RD_INTR_MASK     0x00008000
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_MICH_INST0_RD_INTR_SHIFT    15

/* HIF_INTR2 :: CPU_MASK_STATUS :: reserved3 [14:01] */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_reserved3_MASK              0x00007ffe
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_reserved3_SHIFT             1

/* HIF_INTR2 :: CPU_MASK_STATUS :: HIF_RGR2_BRIDGE_INTR [00:00] */
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_HIF_RGR2_BRIDGE_INTR_MASK   0x00000001
#define BCHP_HIF_INTR2_CPU_MASK_STATUS_HIF_RGR2_BRIDGE_INTR_SHIFT  0

/***************************************************************************
 *CPU_MASK_SET - CPU interrupt Mask Set Register
 ***************************************************************************/
/* HIF_INTR2 :: CPU_MASK_SET :: reserved0 [31:31] */
#define BCHP_HIF_INTR2_CPU_MASK_SET_reserved0_MASK                 0x80000000
#define BCHP_HIF_INTR2_CPU_MASK_SET_reserved0_SHIFT                31

/* HIF_INTR2 :: CPU_MASK_SET :: SDIO_INTR [30:30] */
#define BCHP_HIF_INTR2_CPU_MASK_SET_SDIO_INTR_MASK                 0x40000000
#define BCHP_HIF_INTR2_CPU_MASK_SET_SDIO_INTR_SHIFT                30

/* HIF_INTR2 :: CPU_MASK_SET :: EDU_DONE_INTR [29:29] */
#define BCHP_HIF_INTR2_CPU_MASK_SET_EDU_DONE_INTR_MASK             0x20000000
#define BCHP_HIF_INTR2_CPU_MASK_SET_EDU_DONE_INTR_SHIFT            29

/* HIF_INTR2 :: CPU_MASK_SET :: EDU_ERR_INTR [28:28] */
#define BCHP_HIF_INTR2_CPU_MASK_SET_EDU_ERR_INTR_MASK              0x10000000
#define BCHP_HIF_INTR2_CPU_MASK_SET_EDU_ERR_INTR_SHIFT             28

/* HIF_INTR2 :: CPU_MASK_SET :: NAND_CORR_INTR [27:27] */
#define BCHP_HIF_INTR2_CPU_MASK_SET_NAND_CORR_INTR_MASK            0x08000000
#define BCHP_HIF_INTR2_CPU_MASK_SET_NAND_CORR_INTR_SHIFT           27

/* HIF_INTR2 :: CPU_MASK_SET :: NAND_UNC_INTR [26:26] */
#define BCHP_HIF_INTR2_CPU_MASK_SET_NAND_UNC_INTR_MASK             0x04000000
#define BCHP_HIF_INTR2_CPU_MASK_SET_NAND_UNC_INTR_SHIFT            26

/* HIF_INTR2 :: CPU_MASK_SET :: NAND_RBPIN_INTR [25:25] */
#define BCHP_HIF_INTR2_CPU_MASK_SET_NAND_RBPIN_INTR_MASK           0x02000000
#define BCHP_HIF_INTR2_CPU_MASK_SET_NAND_RBPIN_INTR_SHIFT          25

/* HIF_INTR2 :: CPU_MASK_SET :: NAND_CTLRDY_INTR [24:24] */
#define BCHP_HIF_INTR2_CPU_MASK_SET_NAND_CTLRDY_INTR_MASK          0x01000000
#define BCHP_HIF_INTR2_CPU_MASK_SET_NAND_CTLRDY_INTR_SHIFT         24

/* HIF_INTR2 :: CPU_MASK_SET :: NAND_PGMPG_INTR [23:23] */
#define BCHP_HIF_INTR2_CPU_MASK_SET_NAND_PGMPG_INTR_MASK           0x00800000
#define BCHP_HIF_INTR2_CPU_MASK_SET_NAND_PGMPG_INTR_SHIFT          23

/* HIF_INTR2 :: CPU_MASK_SET :: NAND_CPYBK_INTR [22:22] */
#define BCHP_HIF_INTR2_CPU_MASK_SET_NAND_CPYBK_INTR_MASK           0x00400000
#define BCHP_HIF_INTR2_CPU_MASK_SET_NAND_CPYBK_INTR_SHIFT          22

/* HIF_INTR2 :: CPU_MASK_SET :: NAND_BLKERA_INTR [21:21] */
#define BCHP_HIF_INTR2_CPU_MASK_SET_NAND_BLKERA_INTR_MASK          0x00200000
#define BCHP_HIF_INTR2_CPU_MASK_SET_NAND_BLKERA_INTR_SHIFT         21

/* HIF_INTR2 :: CPU_MASK_SET :: NAND_NP_READ_INTR [20:20] */
#define BCHP_HIF_INTR2_CPU_MASK_SET_NAND_NP_READ_INTR_MASK         0x00100000
#define BCHP_HIF_INTR2_CPU_MASK_SET_NAND_NP_READ_INTR_SHIFT        20

/* HIF_INTR2 :: CPU_MASK_SET :: reserved1 [19:19] */
#define BCHP_HIF_INTR2_CPU_MASK_SET_reserved1_MASK                 0x00080000
#define BCHP_HIF_INTR2_CPU_MASK_SET_reserved1_SHIFT                19

/* HIF_INTR2 :: CPU_MASK_SET :: MICH_WR_INTR [18:18] */
#define BCHP_HIF_INTR2_CPU_MASK_SET_MICH_WR_INTR_MASK              0x00040000
#define BCHP_HIF_INTR2_CPU_MASK_SET_MICH_WR_INTR_SHIFT             18

/* HIF_INTR2 :: CPU_MASK_SET :: reserved2 [17:16] */
#define BCHP_HIF_INTR2_CPU_MASK_SET_reserved2_MASK                 0x00030000
#define BCHP_HIF_INTR2_CPU_MASK_SET_reserved2_SHIFT                16

/* HIF_INTR2 :: CPU_MASK_SET :: MICH_INST0_RD_INTR [15:15] */
#define BCHP_HIF_INTR2_CPU_MASK_SET_MICH_INST0_RD_INTR_MASK        0x00008000
#define BCHP_HIF_INTR2_CPU_MASK_SET_MICH_INST0_RD_INTR_SHIFT       15

/* HIF_INTR2 :: CPU_MASK_SET :: reserved3 [14:01] */
#define BCHP_HIF_INTR2_CPU_MASK_SET_reserved3_MASK                 0x00007ffe
#define BCHP_HIF_INTR2_CPU_MASK_SET_reserved3_SHIFT                1

/* HIF_INTR2 :: CPU_MASK_SET :: HIF_RGR2_BRIDGE_INTR [00:00] */
#define BCHP_HIF_INTR2_CPU_MASK_SET_HIF_RGR2_BRIDGE_INTR_MASK      0x00000001
#define BCHP_HIF_INTR2_CPU_MASK_SET_HIF_RGR2_BRIDGE_INTR_SHIFT     0

/***************************************************************************
 *CPU_MASK_CLEAR - CPU interrupt Mask Clear Register
 ***************************************************************************/
/* HIF_INTR2 :: CPU_MASK_CLEAR :: reserved0 [31:31] */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_reserved0_MASK               0x80000000
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_reserved0_SHIFT              31

/* HIF_INTR2 :: CPU_MASK_CLEAR :: SDIO_INTR [30:30] */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_SDIO_INTR_MASK               0x40000000
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_SDIO_INTR_SHIFT              30

/* HIF_INTR2 :: CPU_MASK_CLEAR :: EDU_DONE_INTR [29:29] */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_EDU_DONE_INTR_MASK           0x20000000
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_EDU_DONE_INTR_SHIFT          29

/* HIF_INTR2 :: CPU_MASK_CLEAR :: EDU_ERR_INTR [28:28] */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_EDU_ERR_INTR_MASK            0x10000000
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_EDU_ERR_INTR_SHIFT           28

/* HIF_INTR2 :: CPU_MASK_CLEAR :: NAND_CORR_INTR [27:27] */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_NAND_CORR_INTR_MASK          0x08000000
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_NAND_CORR_INTR_SHIFT         27

/* HIF_INTR2 :: CPU_MASK_CLEAR :: NAND_UNC_INTR [26:26] */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_NAND_UNC_INTR_MASK           0x04000000
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_NAND_UNC_INTR_SHIFT          26

/* HIF_INTR2 :: CPU_MASK_CLEAR :: NAND_RBPIN_INTR [25:25] */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_NAND_RBPIN_INTR_MASK         0x02000000
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_NAND_RBPIN_INTR_SHIFT        25

/* HIF_INTR2 :: CPU_MASK_CLEAR :: NAND_CTLRDY_INTR [24:24] */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_NAND_CTLRDY_INTR_MASK        0x01000000
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_NAND_CTLRDY_INTR_SHIFT       24

/* HIF_INTR2 :: CPU_MASK_CLEAR :: NAND_PGMPG_INTR [23:23] */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_NAND_PGMPG_INTR_MASK         0x00800000
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_NAND_PGMPG_INTR_SHIFT        23

/* HIF_INTR2 :: CPU_MASK_CLEAR :: NAND_CPYBK_INTR [22:22] */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_NAND_CPYBK_INTR_MASK         0x00400000
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_NAND_CPYBK_INTR_SHIFT        22

/* HIF_INTR2 :: CPU_MASK_CLEAR :: NAND_BLKERA_INTR [21:21] */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_NAND_BLKERA_INTR_MASK        0x00200000
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_NAND_BLKERA_INTR_SHIFT       21

/* HIF_INTR2 :: CPU_MASK_CLEAR :: NAND_NP_READ_INTR [20:20] */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_NAND_NP_READ_INTR_MASK       0x00100000
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_NAND_NP_READ_INTR_SHIFT      20

/* HIF_INTR2 :: CPU_MASK_CLEAR :: reserved1 [19:19] */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_reserved1_MASK               0x00080000
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_reserved1_SHIFT              19

/* HIF_INTR2 :: CPU_MASK_CLEAR :: MICH_WR_INTR [18:18] */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_MICH_WR_INTR_MASK            0x00040000
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_MICH_WR_INTR_SHIFT           18

/* HIF_INTR2 :: CPU_MASK_CLEAR :: reserved2 [17:16] */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_reserved2_MASK               0x00030000
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_reserved2_SHIFT              16

/* HIF_INTR2 :: CPU_MASK_CLEAR :: MICH_INST0_RD_INTR [15:15] */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_MICH_INST0_RD_INTR_MASK      0x00008000
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_MICH_INST0_RD_INTR_SHIFT     15

/* HIF_INTR2 :: CPU_MASK_CLEAR :: reserved3 [14:01] */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_reserved3_MASK               0x00007ffe
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_reserved3_SHIFT              1

/* HIF_INTR2 :: CPU_MASK_CLEAR :: HIF_RGR2_BRIDGE_INTR [00:00] */
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_HIF_RGR2_BRIDGE_INTR_MASK    0x00000001
#define BCHP_HIF_INTR2_CPU_MASK_CLEAR_HIF_RGR2_BRIDGE_INTR_SHIFT   0

#endif /* #ifndef BCHP_HIF_INTR2_H__ */

/* End of File */

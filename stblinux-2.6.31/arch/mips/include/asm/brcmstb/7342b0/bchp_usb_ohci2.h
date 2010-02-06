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
 * Date:           Generated on         Tue Nov 17 17:56:49 2009
 *                 MD5 Checksum         7635c8e6fce632fc8dd4fb82126b3847
 *
 * Compiled with:  RDB Utility          combo_header.pl
 *                 RDB Parser           3.0
 *                 unknown              unknown
 *                 Perl Interpreter     5.008008
 *                 Operating System     linux
 *
 * Revision History:
 *
 * $brcm_Log: /magnum/basemodules/chp/7342/rdb/b0/bchp_usb_ohci2.h $
 * 
 * Hydra_Software_Devel/1   11/17/09 9:08p albertl
 * SW7342-78: Initial revision.
 *
 ***************************************************************************/

#ifndef BCHP_USB_OHCI2_H__
#define BCHP_USB_OHCI2_H__

/***************************************************************************
 *USB_OHCI2 - USB OHCI 3 Control Registers
 ***************************************************************************/
#define BCHP_USB_OHCI2_HcRevision                0x00450800 /* Host Controller Revision Register */
#define BCHP_USB_OHCI2_HcControl                 0x00450804 /* Host Controller Control Register */
#define BCHP_USB_OHCI2_HcCommandStatus           0x00450808 /* Host Controller Command Status Register */
#define BCHP_USB_OHCI2_HcInterruptStatus         0x0045080c /* Host Controller Interrupt Status Register */
#define BCHP_USB_OHCI2_HcInterruptEnable         0x00450810 /* Host Controller Interrupt Enable Register */
#define BCHP_USB_OHCI2_HcInterruptDisable        0x00450814 /* Host Controller Interrupt Disable Register */
#define BCHP_USB_OHCI2_HcHCCA                    0x00450818 /* Host Controller Communication Area Register */
#define BCHP_USB_OHCI2_HcPeriodCurrentED         0x0045081c /* Current Isochronous or Interrupt Endpoint Descriptor Register */
#define BCHP_USB_OHCI2_HcControlHeadED           0x00450820 /* First Endpoint Descriptor of the Control List */
#define BCHP_USB_OHCI2_HcControlCurrentED        0x00450824 /* Current Endpoint Descriptor of the Control List */
#define BCHP_USB_OHCI2_HcBulkHeadED              0x00450828 /* First Endpoint Descriptor of the Bulk List */
#define BCHP_USB_OHCI2_HcBulkCurrentED           0x0045082c /* Current Endpoint Descriptor of the Bulk List */
#define BCHP_USB_OHCI2_HcDoneHead                0x00450830 /* Last Completed Transfer Descriptor Added to the Done Queue */
#define BCHP_USB_OHCI2_HcFmInterval              0x00450834 /* Frame Bit Time Interval Register */
#define BCHP_USB_OHCI2_HcFmRemaining             0x00450838 /* Bit Time Remaining in the Current Frame */
#define BCHP_USB_OHCI2_HcFmNumber                0x0045083c /* Frame Number Register */
#define BCHP_USB_OHCI2_HcPeriodicStart           0x00450840 /* Register to Start Processing the Periodic List */
#define BCHP_USB_OHCI2_HcLSThreshold             0x00450844 /* LS Packet Threshold Register */
#define BCHP_USB_OHCI2_HcRhDescriptorA           0x00450848 /* Root Hub Descriptor A Register */
#define BCHP_USB_OHCI2_HcRhDescriptorB           0x0045084c /* Root Hub Descriptor B Register */
#define BCHP_USB_OHCI2_HcRhStatus                0x00450850 /* Root Hub Status Register */
#define BCHP_USB_OHCI2_HcRhPortStatus1           0x00450854 /* Root Hub Port Status Register for Port 1 */

#endif /* #ifndef BCHP_USB_OHCI2_H__ */

/* End of File */

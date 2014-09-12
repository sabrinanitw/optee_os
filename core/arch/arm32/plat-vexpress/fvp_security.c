/*
 * Copyright (c) 2014, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include <kernel/tee_core_trace.h>
#include <drivers/tzc400.h>

/*******************************************************************************
 * TrustZone address space controller related constants
 ******************************************************************************/
#define TZC400_BASE			0x2a4a0000

/*
 * The NSAIDs for this platform as used to program the TZC400.
 */

/* The FVP has 4 bits of NSAIDs. Used with TZC FAIL_ID (ACE Lite ID width) */
#define FVP_AID_WIDTH			4

/* NSAIDs used by devices in TZC filter 0 on FVP */
#define FVP_NSAID_DEFAULT		0
#define FVP_NSAID_PCI			1
#define FVP_NSAID_VIRTIO		8  /* from FVP v5.6 onwards */
#define FVP_NSAID_AP			9  /* Application Processors */
#define FVP_NSAID_VIRTIO_OLD		15 /* until FVP v5.5 */

/* NSAIDs used by devices in TZC filter 2 on FVP */
#define FVP_NSAID_HDLCD0		2
#define FVP_NSAID_CLCD			7
/*******************************************************************************
 * FVP memory map related constants
 ******************************************************************************/
/* Base address where parameters to BL31 are stored */
#define PARAMS_BASE		TZDRAM_BASE

#define DRAM1_BASE		0x80000000ull
#define DRAM1_SIZE		0x80000000ull
#define DRAM1_END		(DRAM1_BASE + DRAM1_SIZE - 1)
#define DRAM1_SEC_SIZE		0x01000000ull

#define DRAM_BASE		DRAM1_BASE
#define DRAM_SIZE		DRAM1_SIZE

#define DRAM2_BASE		0x880000000ull
#define DRAM2_SIZE		0x780000000ull
#define DRAM2_END		(DRAM2_BASE + DRAM2_SIZE - 1)
/*******************************************************************************
 * Above DEFINE should be fixed
 ******************************************************************************/


/* Declarations for fvp_security.c */
extern void fvp_security_setup(void);

/* Used to improve readability for configuring regions. */
#define FILTER_SHIFT(filter)	(1 << filter)

/*
 * For the moment we assume that all security programming is done by the
 * primary core.
 * TODO:
 * Might want to enable interrupt on violations when supported?
 */

void fvp_security_setup(void)
{

	/*
	 * The TrustZone controller controls access to main DRAM. Give
	 * full NS access for the moment to use with OS.
	 */
#if 0
	DMSG("Configuring TrustZone Controller\n");

	tzc_init(&controller);
	DMSG("Done tzc_init\n");
#endif

	/*
	 * Currently only filters 0 and 2 are connected on Base FVP.
	 * Filter 0 : CPU clusters (no access to DRAM by default)
	 * Filter 1 : not connected
	 * Filter 2 : LCDs (access to VRAM allowed by default)
	 * Filter 3 : not connected
	 * Programming unconnected filters will have no effect at the
	 * moment. These filter could, however, be connected in future.
	 * So care should be taken not to configure the unused filters.
	 */

	/* Disable all filters before programming. */
	tzc_disable_filters();
	DMSG("Done tzc_disable_filters\n");

	/*
	 * Allow only non-secure access to all DRAM to supported devices.
	 * Give access to the CPUs and Virtio. Some devices
	 * would normally use the default ID so allow that too. We use
	 * two regions to cover the blocks of physical memory in the FVPs.
	 *
	 * Software executing in the secure state, such as a secure
	 * boot-loader, can access the DRAM by using the NS attributes in
	 * the MMU translation tables and descriptors.
	 */


	 /* Set to cover the first block of DRAM */
	 tzc_configure_region(FILTER_SHIFT(0), 1,
	 	0x80100000, DRAM1_END - DRAM1_SEC_SIZE,
	 	TZC_REGION_S_NONE,
	 	TZC_REGION_ACCESS_RDWR(FVP_NSAID_DEFAULT) |
	 	TZC_REGION_ACCESS_RDWR(FVP_NSAID_PCI) |
	 	TZC_REGION_ACCESS_RDWR(FVP_NSAID_AP) |
	 	TZC_REGION_ACCESS_RDWR(FVP_NSAID_VIRTIO) |
	 	TZC_REGION_ACCESS_RDWR(FVP_NSAID_VIRTIO_OLD));

	 tzc_configure_region(FILTER_SHIFT(0)|FILTER_SHIFT(2), 4,
	 		0x80000000, 0x80000000 + 0x100000 - 1,
	 		TZC_REGION_S_RDWR,
			TZC_REGION_ACCESS_RDWR(FVP_NSAID_CLCD));

	/*
	 * TODO: Interrupts are not currently supported. The only
	 * options we have are for access errors to occur quietly or to
	 * cause an exception. We choose to cause an exception.
	 */
	tzc_set_action(TZC_ACTION_ERR);
	DMSG("Done tzc_set_action\n");

	/* Enable filters. */
	tzc_enable_filters();
	DMSG("Done tzc_set_action\n");
}

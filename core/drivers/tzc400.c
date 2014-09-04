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
#include <kernel/panic.h>
#include <io.h>
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
tzc_instance_t controller;

static uint32_t tzc_read_build_config(uint32_t base)
{
	return read32(base + BUILD_CONFIG_OFF);
}

static uint32_t tzc_read_gate_keeper(uint32_t base)
{
	return read32(base + GATE_KEEPER_OFF);
}

static void tzc_write_gate_keeper(uint32_t base, uint32_t val)
{
	write32(val, base + GATE_KEEPER_OFF);
}

static void tzc_write_action(uint32_t base, tzc_action_t action)
{
	write32(action, base + ACTION_OFF);
}

static uint32_t tzc_read_region_base_low(uint32_t base, uint32_t region)
{
	return read32(base + REGION_BASE_LOW_OFF + REGION_NUM_OFF(region));
}

static uint32_t tzc_read_region_base_high(uint32_t base, uint32_t region)
{
	return read32(base + REGION_BASE_HIGH_OFF + REGION_NUM_OFF(region));
}

static void tzc_write_region_base_low(uint32_t base, uint32_t region, uint32_t val)
{
	write32(val, base + REGION_BASE_LOW_OFF + REGION_NUM_OFF(region));
}

static void tzc_write_region_base_high(uint32_t base, uint32_t region, uint32_t val)
{
	write32(val, base + REGION_BASE_HIGH_OFF + REGION_NUM_OFF(region));
}

static uint32_t tzc_read_region_top_low(uint32_t base, uint32_t region)
{
	return read32(base + REGION_TOP_LOW_OFF + REGION_NUM_OFF(region));
}

static uint32_t tzc_read_region_top_high(uint32_t base, uint32_t region)
{
	return read32(base + REGION_TOP_HIGH_OFF + REGION_NUM_OFF(region));
}

static void tzc_write_region_top_low(uint32_t base, uint32_t region, uint32_t val)
{
	write32(val, base + REGION_TOP_LOW_OFF + REGION_NUM_OFF(region));
}

static void tzc_write_region_top_high(uint32_t base, uint32_t region, uint32_t val)
{
	write32(val, base + REGION_TOP_HIGH_OFF + REGION_NUM_OFF(region));
}

static uint32_t tzc_read_region_attributes(uint32_t base, uint32_t region)
{
	return read32(base + REGION_ATTRIBUTES_OFF + REGION_NUM_OFF(region));
}

static void tzc_write_region_attributes(uint32_t base, uint32_t region, uint32_t val)
{
	write32(val, base + REGION_ATTRIBUTES_OFF + REGION_NUM_OFF(region));
}

static void tzc_write_region_id_access(uint32_t base, uint32_t region, uint32_t val)
{
	write32(val, base + REGION_ID_ACCESS_OFF + REGION_NUM_OFF(region));
}

static uint32_t tzc_read_component_id(uint32_t base)
{
	uint32_t id;

	id = read8(base + CID0_OFF);
	id |= (read8(base + CID1_OFF) << 8);
	id |= (read8(base + CID2_OFF) << 16);
	id |= (read8(base + CID3_OFF) << 24);

	return id;
}

static uint32_t tzc_get_gate_keeper(uint32_t base, uint8_t filter)
{
	uint32_t tmp;

	tmp = (tzc_read_gate_keeper(base) >> GATE_KEEPER_OS_SHIFT) &
		GATE_KEEPER_OS_MASK;

	return (tmp >> filter) & GATE_KEEPER_FILTER_MASK;
}

/* This function is not MP safe. */
static void tzc_set_gate_keeper(uint32_t base, uint8_t filter, uint32_t val)
{
	uint32_t tmp;

	/* Upper half is current state. Lower half is requested state. */
	tmp = (tzc_read_gate_keeper(base) >> GATE_KEEPER_OS_SHIFT) &
		GATE_KEEPER_OS_MASK;

	if (val)
		tmp |=  (1 << filter);
	else
		tmp &= ~(1 << filter);

	tzc_write_gate_keeper(base, (tmp & GATE_KEEPER_OR_MASK) <<
			      GATE_KEEPER_OR_SHIFT);

	/* Wait here until we see the change reflected in the TZC status. */
	while (((tzc_read_gate_keeper(base) >> GATE_KEEPER_OS_SHIFT) &
		GATE_KEEPER_OS_MASK) != tmp)
	  ;
}


void tzc_init(void)
{
	uint32_t tzc_id, tzc_build;

	/*
	 * The driver does some error checking and will assert.
	 * - Provide base address of device on platform.
	 * - Provide width of ACE-Lite IDs on platform.
	 */
	controller.base = TZC400_BASE;
	controller.aid_width = FVP_AID_WIDTH;

	/*
	 * We expect to see a tzc400. Check component ID. The TZC-400 TRM shows
	 * component ID is expected to be "0xB105F00D".
	 */
	tzc_id = tzc_read_component_id(controller.base);
	if (tzc_id != TZC400_COMPONENT_ID) {
		DMSG("TZC : Wrong device ID (0x%x).\n", tzc_id);
		panic();
	}

	/* Save values we will use later. */
	tzc_build = tzc_read_build_config(controller.base);
	controller.num_filters = ((tzc_build >> BUILD_CONFIG_NF_SHIFT) &
			   BUILD_CONFIG_NF_MASK) + 1;
	controller.addr_width  = ((tzc_build >> BUILD_CONFIG_AW_SHIFT) &
			   BUILD_CONFIG_AW_MASK) + 1;
	controller.num_regions = ((tzc_build >> BUILD_CONFIG_NR_SHIFT) &
			   BUILD_CONFIG_NR_MASK) + 1;
}


/*
 * `tzc_configure_region` is used to program regions into the TrustZone
 * controller. A region can be associated with more than one filter. The
 * associated filters are passed in as a bitmap (bit0 = filter0).
 * NOTE:
 * The region 0 covers the whole address space and is enabled on all filters,
 * this cannot be changed. It is, however, possible to change some region 0
 * permissions.
 */
void tzc_configure_region(uint32_t filters,
			  uint8_t  region,
			  uint64_t region_base,
			  uint64_t region_top,
			  tzc_region_attributes_t sec_attr,
			  uint32_t ns_device_access)
{
	uint64_t max_addr;

	/* Do range checks on filters and regions. */
	assert(((filters >> controller.num_filters) == 0) &&
	       (region < controller.num_regions));
	assert(controller.base != 0);

	/*
	 * Do address range check based on TZC configuration. A 64bit address is
	 * the max and expected case.
	 */
	max_addr = UINT64_MAX >> (64 - controller.addr_width);
	DMSG("addr_width: %d", controller.addr_width);
	if ((region_top > max_addr) || (region_base >= region_top))
		assert(0);

	/* region_base and (region_top + 1) must be 4KB aligned */
	assert(((region_base | (region_top + 1)) & (4096 - 1)) == 0);

	assert(sec_attr <= TZC_REGION_S_RDWR);

	/*
	 * Inputs look ok, start programming registers.
	 * All the address registers are 32 bits wide and have a LOW and HIGH
	 * component used to construct a up to a 64bit address.
	 */
	tzc_write_region_base_low(controller.base, region, (uint32_t)(region_base));
	tzc_write_region_base_high(controller.base, region, (uint32_t)(region_base >> 32));

	tzc_write_region_top_low(controller.base, region, (uint32_t)(region_top));
	tzc_write_region_top_high(controller.base, region, (uint32_t)(region_top >> 32));

	/* Assign the region to a filter and set secure attributes */
	tzc_write_region_attributes(controller.base, region,
		(sec_attr << REGION_ATTRIBUTES_SEC_SHIFT) | filters);

	/*
	 * Specify which non-secure devices have permission to access this
	 * region.
	 */
	tzc_write_region_id_access(controller.base, region, ns_device_access);
}


void tzc_set_action(tzc_action_t action)
{
	/*
	 * - Currently no handler is provided to trap an error via interrupt
	 *   or exception.
	 * - The interrupt action has not been tested.
	 */
	tzc_write_action(controller.base, action);
}


void tzc_enable_filters(void)
{
	uint32_t state;
	uint32_t filter;

	for (filter = 0; filter < controller.num_filters; filter++) {
		state = tzc_get_gate_keeper(controller.base, filter);
		if (state) {
			/* The TZC filter is already configured. Changing the
			 * programmer's view in an active system can cause
			 * unpredictable behavior therefore panic for now rather
			 * than try to determine whether this is safe in this
			 * instance. See:
			 * http://infocenter.arm.com/help/index.jsp?\
			 * topic=/com.arm.doc.ddi0504c/CJHHECBF.html */
			DMSG("TZC : Filter %d Gatekeeper already enabled.\n",
				filter);
			panic();
		}
		assert(controller.base != 0);
		tzc_set_gate_keeper(controller.base, filter, 1);
	}
}


void tzc_disable_filters(void)
{
	uint32_t filter;

	/*
	 * We don't do the same state check as above as the Gatekeepers are
	 * disabled after reset.
	 */
	for (filter = 0; filter < controller.num_filters; filter++)
		tzc_set_gate_keeper(controller.base, filter, 0);
}

#define	REGION_MAX		8
static const char* tzc_attr_msg[] = {
	"TZC_REGION_S_NONE",
	"TZC_REGION_S_RD",
	"TZC_REGION_S_WR",
	"TZC_REGION_S_RDWR"
};

void tzc_dump_state(void)
{
	uint32_t n;
	uint32_t temp_32reg, temp_32reg_h;

	DMSG("enter");
	for (n = 0; n <= REGION_MAX; n++) {
		temp_32reg = tzc_read_region_attributes(controller.base, n);
		if ( !(temp_32reg & REGION_ATTRIBUTES_F_EN_MASK)) {
			continue;
		}

		DMSG("\n");
		DMSG("region %d", n);
		temp_32reg = tzc_read_region_base_low(controller.base, n);
		temp_32reg_h = tzc_read_region_base_high(controller.base, n);
		DMSG("region_base: 0x%08x%08x", temp_32reg_h, temp_32reg);
		temp_32reg = tzc_read_region_top_low(controller.base, n);
		temp_32reg_h = tzc_read_region_top_high(controller.base, n);
		DMSG("region_top: 0x%08x%08x", temp_32reg_h, temp_32reg);
		temp_32reg = tzc_read_region_attributes(controller.base, n);
		DMSG("secure rw: %s", tzc_attr_msg[temp_32reg >> REGION_ATTRIBUTES_SEC_SHIFT]);
		if (temp_32reg & REGION_ATTRIBUTES_F0_EN_MASK) {
			DMSG("filter 0 enable");
		}
		if (temp_32reg & REGION_ATTRIBUTES_F1_EN_MASK) {
			DMSG("filter 1 enable");
		}
		if (temp_32reg & REGION_ATTRIBUTES_F2_EN_MASK) {
			DMSG("filter 2 enable");
		}
		if (temp_32reg & REGION_ATTRIBUTES_F3_EN_MASK) {
			DMSG("filter 3 enable");
		}
	}
	DMSG("exit");
	return;
}

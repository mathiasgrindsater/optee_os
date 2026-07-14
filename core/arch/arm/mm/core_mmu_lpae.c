// SPDX-License-Identifier: (BSD-2-Clause AND BSD-3-Clause)
/*
 * Copyright (c) 2015-2016, 2022 Linaro Limited
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
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

/*
 * Copyright (c) 2014, 2022, ARM Limited and Contributors. All rights reserved.
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
#include <platform_config.h>

#include <arm.h>
#include <assert.h>
#include <compiler.h>
#include <config.h>
#include <inttypes.h>
#include <keep.h>
#include <kernel/boot.h>
#include <kernel/cache_helpers.h>
#include <kernel/linker.h>
#include <kernel/misc.h>
#include <kernel/panic.h>
#include <kernel/thread.h>
#include <kernel/thread_private.h>	/* threads[] for A2 (user_map corruption) */
#include <kernel/tlb_helpers.h>
#include <kernel/user_access.h>		/* enter/exit_user_access for A14 fn() */
#include <memtag.h>
#include <mm/core_memprot.h>
#include <mm/pgt_cache.h>
#include <mm/phys_mem.h>
#include <stdalign.h>
#include <string.h>
#include <trace.h>
#include <types_ext.h>
#include <util.h>

#ifndef DEBUG_XLAT_TABLE
#define DEBUG_XLAT_TABLE 0
#endif

#if DEBUG_XLAT_TABLE
#define debug_print(...) DMSG_RAW(__VA_ARGS__)
#else
#define debug_print(...) ((void)0)
#endif


/*
 * Miscellaneous MMU related constants
 */

#define INVALID_DESC		0x0
#define BLOCK_DESC		0x1
#define L3_BLOCK_DESC		0x3
#define TABLE_DESC		0x3
#define DESC_ENTRY_TYPE_MASK	0x3

#define XN			(1ull << 2)
#define PXN			(1ull << 1)
#define CONT_HINT		(1ull << 0)

#define UPPER_ATTRS(x)		(((x) & 0x7) << 52)
#define GP                      BIT64(50)   /* Guarded Page, Aarch64 FEAT_BTI */
#define NON_GLOBAL		(1ull << 9)
#define ACCESS_FLAG		(1ull << 8)
#define NSH			(0x0 << 6)
#define OSH			(0x2 << 6)
#define ISH			(0x3 << 6)

#define AP_RO			(0x1 << 5)
#define AP_RW			(0x0 << 5)
#define AP_UNPRIV		(0x1 << 4)

#define NS				(0x1 << 3)
#define LOWER_ATTRS_SHIFT		2
#define LOWER_ATTRS(x)			(((x) & 0xfff) << LOWER_ATTRS_SHIFT)

#define ATTR_DEVICE_nGnRE_INDEX		0x0
#define ATTR_IWBWA_OWBWA_NTR_INDEX	0x1
#define ATTR_DEVICE_nGnRnE_INDEX	0x2
#define ATTR_TAGGED_NORMAL_MEM_INDEX	0x3
#define ATTR_INDEX_MASK			0x7

#define ATTR_DEVICE_nGnRnE		(0x0)
#define ATTR_DEVICE_nGnRE		(0x4)
#define ATTR_IWBWA_OWBWA_NTR		(0xff)
/* Same as ATTR_IWBWA_OWBWA_NTR but with memory tagging.  */
#define ATTR_TAGGED_NORMAL_MEM		(0xf0)

#define MAIR_ATTR_SET(attr, index)	(((uint64_t)attr) << ((index) << 3))

#define OUTPUT_ADDRESS_MASK	(0x0000FFFFFFFFF000ULL)

/* (internal) physical address size bits in EL3/EL1 */
#define TCR_PS_BITS_4GB		(0x0)
#define TCR_PS_BITS_64GB	(0x1)
#define TCR_PS_BITS_1TB		(0x2)
#define TCR_PS_BITS_4TB		(0x3)
#define TCR_PS_BITS_16TB	(0x4)
#define TCR_PS_BITS_256TB	(0x5)
#define TCR_PS_BITS_4PB		(0x6)

#define UNSET_DESC		((uint64_t)-1)

#define FOUR_KB_SHIFT		12
#define PAGE_SIZE_SHIFT		FOUR_KB_SHIFT
#define PAGE_SIZE		(1 << PAGE_SIZE_SHIFT)
#define PAGE_SIZE_MASK		(PAGE_SIZE - 1)
#define IS_PAGE_ALIGNED(addr)	(((addr) & PAGE_SIZE_MASK) == 0)

#define XLAT_ENTRY_SIZE_SHIFT	3 /* Each MMU table entry is 8 bytes (1 << 3) */
#define XLAT_ENTRY_SIZE		(1 << XLAT_ENTRY_SIZE_SHIFT)

#define XLAT_TABLE_SIZE_SHIFT	PAGE_SIZE_SHIFT
#define XLAT_TABLE_SIZE		(1 << XLAT_TABLE_SIZE_SHIFT)

#define XLAT_TABLE_LEVEL_MAX	U(3)

/* Values for number of entries in each MMU translation table */
#define XLAT_TABLE_ENTRIES_SHIFT (XLAT_TABLE_SIZE_SHIFT - XLAT_ENTRY_SIZE_SHIFT)
#define XLAT_TABLE_ENTRIES	(1 << XLAT_TABLE_ENTRIES_SHIFT)
#define XLAT_TABLE_ENTRIES_MASK	(XLAT_TABLE_ENTRIES - 1)

/* Values to convert a memory address to an index into a translation table */
#define L3_XLAT_ADDRESS_SHIFT	PAGE_SIZE_SHIFT
#define L2_XLAT_ADDRESS_SHIFT	(L3_XLAT_ADDRESS_SHIFT + \
				 XLAT_TABLE_ENTRIES_SHIFT)
#define L1_XLAT_ADDRESS_SHIFT	(L2_XLAT_ADDRESS_SHIFT + \
				 XLAT_TABLE_ENTRIES_SHIFT)
#define L0_XLAT_ADDRESS_SHIFT	(L1_XLAT_ADDRESS_SHIFT + \
				 XLAT_TABLE_ENTRIES_SHIFT)
#define XLAT_ADDR_SHIFT(level)	(PAGE_SIZE_SHIFT + \
				 ((XLAT_TABLE_LEVEL_MAX - (level)) * \
				 XLAT_TABLE_ENTRIES_SHIFT))

#define XLAT_BLOCK_SIZE(level)	(UL(1) << XLAT_ADDR_SHIFT(level))

/* Base table */
#define BASE_XLAT_ADDRESS_SHIFT	XLAT_ADDR_SHIFT(CORE_MMU_BASE_TABLE_LEVEL)
#define BASE_XLAT_BLOCK_SIZE	XLAT_BLOCK_SIZE(CORE_MMU_BASE_TABLE_LEVEL)

#define NUM_BASE_LEVEL_ENTRIES	\
	BIT(CFG_LPAE_ADDR_SPACE_BITS - BASE_XLAT_ADDRESS_SHIFT)

/*
 * MMU L1 table, one for each core
 *
 * With CFG_CORE_UNMAP_CORE_AT_EL0, each core has one table to be used
 * while in kernel mode and one to be used while in user mode.
 */
#ifdef CFG_CORE_UNMAP_CORE_AT_EL0
#define NUM_BASE_TABLES	2
#else
#define NUM_BASE_TABLES	1
#endif

#ifndef MAX_XLAT_TABLES
#ifdef CFG_NS_VIRTUALIZATION
#	define XLAT_TABLE_VIRTUALIZATION_EXTRA 3
#else
#	define XLAT_TABLE_VIRTUALIZATION_EXTRA 0
#endif
#ifdef CFG_CORE_ASLR
#	define XLAT_TABLE_ASLR_EXTRA 3
#else
#	define XLAT_TABLE_ASLR_EXTRA 0
#endif
#if (CORE_MMU_BASE_TABLE_LEVEL == 0)
#	define XLAT_TABLE_TEE_EXTRA 8
#	define XLAT_TABLE_USER_EXTRA (NUM_BASE_TABLES * CFG_TEE_CORE_NB_CORE)
#else
/* VMI thesis: bumped from 5 → 8 so that 3-entry ASLR layouts (kernel
 * mapping spanning two L1 slots + L1[0] identity = 3 L2s + ~5 L3s)
 * fit in the static pool. Upstream OP-TEE's default 5 is too tight
 * when ASLR pushes the kernel across a 1 GiB boundary. */
#	define XLAT_TABLE_TEE_EXTRA 8
#	define XLAT_TABLE_USER_EXTRA 0
#endif
#define MAX_XLAT_TABLES		(XLAT_TABLE_TEE_EXTRA + \
				 XLAT_TABLE_VIRTUALIZATION_EXTRA + \
				 XLAT_TABLE_ASLR_EXTRA + \
				 XLAT_TABLE_USER_EXTRA + \
				 IS_ENABLED(CFG_DYN_CONFIG))
#endif /*!MAX_XLAT_TABLES*/

#if (CORE_MMU_BASE_TABLE_LEVEL == 0)
#if (MAX_XLAT_TABLES <= UINT8_MAX)
typedef uint8_t l1_idx_t;
#elif (MAX_XLAT_TABLES <= UINT16_MAX)
typedef uint16_t l1_idx_t;
#else
#error MAX_XLAT_TABLES is suspiciously large, please check
#endif
#endif

/*
 * The global base translation table is a three dimensional array (array of
 * array of array), but it's easier to visualize if broken down into
 * components.
 *
 * TTBR is assigned a base translation table of NUM_BASE_LEVEL_ENTRIES
 * entries. NUM_BASE_LEVEL_ENTRIES is determined based on
 * CFG_LPAE_ADDR_SPACE_BITS.  CFG_LPAE_ADDR_SPACE_BITS is by default 32
 * which results in NUM_BASE_LEVEL_ENTRIES defined to 4 where one entry is
 * a uint64_t, 8 bytes.
 *
 * If CFG_CORE_UNMAP_CORE_AT_EL0=y there are two base translation tables,
 * one for OP-TEE core with full mapping of both EL1 and EL0, and one for
 * EL0 where EL1 is unmapped except for a minimal trampoline needed to
 * restore EL1 mappings on exception from EL0.
 *
 * Each CPU core is assigned a unique set of base translation tables as:
 * core0: table0: entry0 (table0 maps both EL1 and EL0)
 *                entry1
 *                entry2
 *                entry3
 * core0: table1: entry0 (table1 maps only EL0)
 *                entry1
 *                entry2
 *                entry3
 * core1: ...
 *
 * The base translation table is by default a level 1 table. It can also be
 * configured as a level 0 table with CFG_LPAE_ADDR_SPACE_BITS >= 40 and <=
 * 48.
 */

/* The size of base tables for one core */
#define BASE_TABLE_SIZE		(NUM_BASE_LEVEL_ENTRIES * NUM_BASE_TABLES * \
				 XLAT_ENTRY_SIZE)
#ifndef CFG_DYN_CONFIG
static uint64_t base_xlation_table[BASE_TABLE_SIZE * CFG_TEE_CORE_NB_CORE /
				   XLAT_ENTRY_SIZE]
	__aligned(NUM_BASE_LEVEL_ENTRIES * XLAT_ENTRY_SIZE)
	__section(".nozi.mmu.base_table");

static uint64_t xlat_tables[XLAT_TABLE_SIZE * MAX_XLAT_TABLES /
			    XLAT_ENTRY_SIZE]
	__aligned(XLAT_TABLE_SIZE) __section(".nozi.mmu.l2");

/* MMU L2 table for TAs, one for each thread */
static uint64_t xlat_tables_ul1[XLAT_TABLE_SIZE * CFG_NUM_THREADS /
				XLAT_ENTRY_SIZE]
	__aligned(XLAT_TABLE_SIZE) __section(".nozi.mmu.l2");

#if (CORE_MMU_BASE_TABLE_LEVEL == 0)
static l1_idx_t user_l1_table_idx[NUM_BASE_TABLES * CFG_TEE_CORE_NB_CORE];
#endif
#endif

/*
 * TAs page table entry inside a level 1 page table.
 *
 * TAs mapping is expected to start from level 2.
 *
 * If base level is 1 then this is the index of a level 1 entry,
 * that will point directly into TA mapping table.
 *
 * If base level is 0 then entry 0 in base table is always used, and then
 * we fallback to "base level == 1" like scenario.
 */
static int user_va_idx __nex_data = -1;

/*
 * struct mmu_partition - virtual memory of a partition
 * @base_tables:       The global base translation table described above
 * @xlat_tables:       Preallocated array of translation tables
 * @l2_ta_tables:      The level 2 table used to map TAs at EL0
 * @xlat_tables_used:  The number of used translation tables from @xlat_tables
 * @asid:              Address space ID used for core mappings
 * @user_l1_table_idx: Index into @xlat_tables for the entry used to map the
 *                     level 2 table @l2_ta_tables
 *
 * With CORE_MMU_BASE_TABLE_LEVEL = 1 translation tables are ordered as:
 * @base_tables is a level 1 table where @user_va_idx above is used as
 * base_tables[user_va_idx] to identify the entry used by @l2_ta_tables.
 *
 * With CORE_MMU_BASE_TABLE_LEVEL = 0 translation tables are ordered as:
 * @base_tables is a level 0 table where base_tables[0] identifies the level 1
 * table indexed with
 * xlat_tables[user_l1_table_idx[0/1][core_id]][user_va_idx] to find the
 * entry used by @l2_ta_tables.
 *
 * With CFG_NS_VIRTUALIZATION disabled there is only one @default_partition
 * (below) describing virtual memory mappings.
 *
 * With CFG_NS_VIRTUALIZATION enabled there's one struct mmu_partition
 * allocated for each partition.
 */
struct mmu_partition {
	uint64_t *base_tables;
	uint64_t *xlat_tables;
	uint64_t *l2_ta_tables;
	unsigned int xlat_tables_used;
	unsigned int asid;

#if (CORE_MMU_BASE_TABLE_LEVEL == 0)
	/*
	 * Indexes of the L1 table from 'xlat_tables'
	 * that points to the user mappings.
	 */
	l1_idx_t *user_l1_table_idx;
#endif
};

#ifdef CFG_DYN_CONFIG
static struct mmu_partition default_partition __nex_bss;
#else
static struct mmu_partition default_partition __nex_data = {
	.base_tables = base_xlation_table,
	.xlat_tables = xlat_tables,
	.l2_ta_tables = xlat_tables_ul1,
#if (CORE_MMU_BASE_TABLE_LEVEL == 0)
	.user_l1_table_idx = user_l1_table_idx,
#endif
	.xlat_tables_used = 0,
	.asid = 0
};
#endif

#ifdef CFG_NS_VIRTUALIZATION
static struct mmu_partition *current_prtn[CFG_TEE_CORE_NB_CORE] __nex_bss;
#endif

static struct mmu_partition *get_prtn(void)
{
#ifdef CFG_NS_VIRTUALIZATION
	struct mmu_partition *ret;
	uint32_t exceptions = thread_mask_exceptions(THREAD_EXCP_ALL);

	ret = current_prtn[get_core_pos()];

	thread_unmask_exceptions(exceptions);
	return ret;
#else
	return &default_partition;
#endif
}

static uint32_t desc_to_mattr(unsigned level, uint64_t desc)
{
	uint32_t a;

	if (!(desc & 1))
		return 0;

	if (level == XLAT_TABLE_LEVEL_MAX) {
		if ((desc & DESC_ENTRY_TYPE_MASK) != L3_BLOCK_DESC)
			return 0;
	} else {
		if ((desc & DESC_ENTRY_TYPE_MASK) == TABLE_DESC)
			return TEE_MATTR_TABLE;
	}

	a = TEE_MATTR_VALID_BLOCK;

	if (desc & LOWER_ATTRS(ACCESS_FLAG))
		a |= TEE_MATTR_PRX | TEE_MATTR_URX;

	if (!(desc & LOWER_ATTRS(AP_RO)))
		a |= TEE_MATTR_PW | TEE_MATTR_UW;

	if (!(desc & LOWER_ATTRS(AP_UNPRIV)))
		a &= ~TEE_MATTR_URWX;

	if (desc & UPPER_ATTRS(XN))
		a &= ~(TEE_MATTR_PX | TEE_MATTR_UX);

	if (desc & UPPER_ATTRS(PXN))
		a &= ~TEE_MATTR_PX;

	COMPILE_TIME_ASSERT(ATTR_DEVICE_nGnRnE_INDEX ==
			    TEE_MATTR_MEM_TYPE_STRONGLY_O);
	COMPILE_TIME_ASSERT(ATTR_DEVICE_nGnRE_INDEX == TEE_MATTR_MEM_TYPE_DEV);
	COMPILE_TIME_ASSERT(ATTR_IWBWA_OWBWA_NTR_INDEX ==
			    TEE_MATTR_MEM_TYPE_CACHED);
	COMPILE_TIME_ASSERT(ATTR_TAGGED_NORMAL_MEM_INDEX ==
			    TEE_MATTR_MEM_TYPE_TAGGED);

	a |= ((desc & LOWER_ATTRS(ATTR_INDEX_MASK)) >> LOWER_ATTRS_SHIFT) <<
	     TEE_MATTR_MEM_TYPE_SHIFT;

	if (!(desc & LOWER_ATTRS(NON_GLOBAL)))
		a |= TEE_MATTR_GLOBAL;

	if (!(desc & LOWER_ATTRS(NS)))
		a |= TEE_MATTR_SECURE;

	if (desc & GP)
		a |= TEE_MATTR_GUARDED;

	return a;
}

static uint64_t mattr_to_desc(unsigned level, uint32_t attr)
{
	uint64_t desc;
	uint32_t a = attr;

	if (a & TEE_MATTR_TABLE)
		return TABLE_DESC;

	if (!(a & TEE_MATTR_VALID_BLOCK))
		return 0;

	if (a & (TEE_MATTR_PX | TEE_MATTR_PW))
		a |= TEE_MATTR_PR;
	if (a & (TEE_MATTR_UX | TEE_MATTR_UW))
		a |= TEE_MATTR_UR;
	if (a & TEE_MATTR_UR)
		a |= TEE_MATTR_PR;
	if (a & TEE_MATTR_UW)
		a |= TEE_MATTR_PW;

	if (IS_ENABLED(CFG_CORE_BTI) && (a & TEE_MATTR_PX))
		a |= TEE_MATTR_GUARDED;

	if (level == XLAT_TABLE_LEVEL_MAX)
		desc = L3_BLOCK_DESC;
	else
		desc = BLOCK_DESC;

	if (!(a & (TEE_MATTR_PX | TEE_MATTR_UX)))
		desc |= UPPER_ATTRS(XN);
	if (!(a & TEE_MATTR_PX))
		desc |= UPPER_ATTRS(PXN);

	if (a & TEE_MATTR_UR)
		desc |= LOWER_ATTRS(AP_UNPRIV);

	if (!(a & TEE_MATTR_PW))
		desc |= LOWER_ATTRS(AP_RO);

	if (feat_bti_is_implemented() && (a & TEE_MATTR_GUARDED))
		desc |= GP;

	/* Keep in sync with core_mmu.c:core_mmu_mattr_is_ok */
	switch ((a >> TEE_MATTR_MEM_TYPE_SHIFT) & TEE_MATTR_MEM_TYPE_MASK) {
	case TEE_MATTR_MEM_TYPE_STRONGLY_O:
		desc |= LOWER_ATTRS(ATTR_DEVICE_nGnRnE_INDEX | OSH);
		break;
	case TEE_MATTR_MEM_TYPE_DEV:
		desc |= LOWER_ATTRS(ATTR_DEVICE_nGnRE_INDEX | OSH);
		break;
	case TEE_MATTR_MEM_TYPE_CACHED:
		desc |= LOWER_ATTRS(ATTR_IWBWA_OWBWA_NTR_INDEX | ISH);
		break;
	case TEE_MATTR_MEM_TYPE_TAGGED:
		desc |= LOWER_ATTRS(ATTR_TAGGED_NORMAL_MEM_INDEX | ISH);
		break;
	default:
		/*
		 * "Can't happen" the attribute is supposed to be checked
		 * with core_mmu_mattr_is_ok() before.
		 */
		panic();
	}

	if (a & (TEE_MATTR_UR | TEE_MATTR_PR))
		desc |= LOWER_ATTRS(ACCESS_FLAG);

	if (!(a & TEE_MATTR_GLOBAL))
		desc |= LOWER_ATTRS(NON_GLOBAL);

	desc |= a & TEE_MATTR_SECURE ? 0 : LOWER_ATTRS(NS);

	return desc;
}

static uint64_t *get_base_table(struct mmu_partition *prtn, size_t tbl_idx,
				size_t core_pos)
{
	assert(tbl_idx < NUM_BASE_TABLES);
	assert(core_pos < CFG_TEE_CORE_NB_CORE);

	return  prtn->base_tables + (core_pos * NUM_BASE_TABLES + tbl_idx) *
				    NUM_BASE_LEVEL_ENTRIES;
}

static uint64_t *get_l2_ta_tables(struct mmu_partition *prtn, size_t thread_id)
{
	assert(thread_id < CFG_NUM_THREADS);

	return prtn->l2_ta_tables + XLAT_TABLE_ENTRIES * thread_id;
}

#if (CORE_MMU_BASE_TABLE_LEVEL == 0)
static uint64_t *get_l1_ta_table(struct mmu_partition *prtn, size_t base_idx,
				 size_t core_pos)
{
	size_t idx = 0;
	uint64_t *tbl = NULL;

	idx = prtn->user_l1_table_idx[core_pos * NUM_BASE_TABLES + base_idx];
	tbl = (void *)((vaddr_t)prtn->xlat_tables + idx * XLAT_TABLE_SIZE);
	return tbl;
}

static void set_l1_ta_table(struct mmu_partition *prtn, size_t base_idx,
			    size_t core_pos, uint64_t *tbl)
{
	size_t idx = 0;

	idx = ((vaddr_t)tbl - (vaddr_t)prtn->xlat_tables) / XLAT_TABLE_SIZE;
	assert(idx < prtn->xlat_tables_used);
	prtn->user_l1_table_idx[core_pos * NUM_BASE_TABLES + base_idx] = idx;
}
#endif

#ifdef CFG_NS_VIRTUALIZATION
size_t core_mmu_get_total_pages_size(void)
{
	size_t sz = ROUNDUP(BASE_TABLE_SIZE * CFG_TEE_CORE_NB_CORE,
			    SMALL_PAGE_SIZE);

	sz += XLAT_TABLE_SIZE * CFG_NUM_THREADS;
	if (!IS_ENABLED(CFG_DYN_CONFIG))
		sz += XLAT_TABLE_SIZE * MAX_XLAT_TABLES;

	return sz;
}

struct mmu_partition *core_alloc_mmu_prtn(void *tables)
{
	struct mmu_partition *prtn;
	uint8_t *tbl = tables;
	unsigned int asid = asid_alloc();

	assert(((vaddr_t)tbl) % SMALL_PAGE_SIZE == 0);

	if (!asid)
		return NULL;

	prtn = nex_malloc(sizeof(*prtn));
	if (!prtn)
		goto err;
#if (CORE_MMU_BASE_TABLE_LEVEL == 0)
	prtn->user_l1_table_idx = nex_calloc(NUM_BASE_TABLES *
					     CFG_TEE_CORE_NB_CORE,
					     sizeof(l1_idx_t));
	if (!prtn->user_l1_table_idx)
		goto err;
#endif

	memset(tables, 0, core_mmu_get_total_pages_size());
	prtn->base_tables = (void *)tbl;
	tbl += ROUNDUP(BASE_TABLE_SIZE * CFG_TEE_CORE_NB_CORE, SMALL_PAGE_SIZE);

	if (!IS_ENABLED(CFG_DYN_CONFIG)) {
		prtn->xlat_tables = (void *)tbl;
		tbl += XLAT_TABLE_SIZE * MAX_XLAT_TABLES;
		assert(((vaddr_t)tbl) % SMALL_PAGE_SIZE == 0);
	}

	prtn->l2_ta_tables = (void *)tbl;
	prtn->xlat_tables_used = 0;
	prtn->asid = asid;

	return prtn;
err:
	nex_free(prtn);
	asid_free(asid);
	return NULL;
}

void core_free_mmu_prtn(struct mmu_partition *prtn)
{
	asid_free(prtn->asid);
	nex_free(prtn);
}

void core_mmu_set_prtn(struct mmu_partition *prtn)
{
	uint64_t ttbr;
	/*
	 * We are changing mappings for current CPU,
	 * so make sure that we will not be rescheduled
	 */
	assert(thread_get_exceptions() & THREAD_EXCP_FOREIGN_INTR);

	current_prtn[get_core_pos()] = prtn;

	ttbr = virt_to_phys(get_base_table(prtn, 0, get_core_pos()));

	write_ttbr0_el1(ttbr | ((paddr_t)prtn->asid << TTBR_ASID_SHIFT));
	isb();
	tlbi_all();
}

void core_mmu_set_default_prtn(void)
{
	core_mmu_set_prtn(&default_partition);
}

void core_mmu_set_default_prtn_tbl(void)
{
	size_t n = 0;

	for (n = 0; n < CFG_TEE_CORE_NB_CORE; n++)
		current_prtn[n] = &default_partition;
}
#endif

static uint64_t *core_mmu_xlat_table_alloc(struct mmu_partition *prtn)
{
	uint64_t *new_table = NULL;

	if (IS_ENABLED(CFG_DYN_CONFIG)) {
		if (cpu_mmu_enabled()) {
			tee_mm_entry_t *mm = NULL;
			paddr_t pa = 0;

			if (prtn == get_prtn()) {
				mm = phys_mem_core_alloc(XLAT_TABLE_SIZE);
				if (!mm)
					EMSG("Phys mem exhausted");
			} else {
				mm = nex_phys_mem_core_alloc(XLAT_TABLE_SIZE);
				if (!mm)
					EMSG("Phys nex mem exhausted");
			}
			if (!mm)
				return NULL;
			pa = tee_mm_get_smem(mm);

			new_table = phys_to_virt(pa, MEM_AREA_SEC_RAM_OVERALL,
						 XLAT_TABLE_SIZE);
			assert(new_table);
		} else {
			new_table = boot_mem_alloc(XLAT_TABLE_SIZE,
						   XLAT_TABLE_SIZE);
			if (prtn->xlat_tables) {
				/*
				 * user_l1_table_idx[] is used to index
				 * xlat_tables so we depend on the
				 * xlat_tables are linearly allocated or
				 * l1_idx_t might need a wider type.
				 */
				assert((vaddr_t)prtn->xlat_tables +
				       prtn->xlat_tables_used *
				       XLAT_TABLE_SIZE == (vaddr_t)new_table);
			} else {
				boot_mem_add_reloc(&prtn->xlat_tables);
				prtn->xlat_tables = new_table;
			}
		}
		prtn->xlat_tables_used++;
		DMSG("xlat tables used %u", prtn->xlat_tables_used);
	} else {
		if (prtn->xlat_tables_used >= MAX_XLAT_TABLES) {
			EMSG("%u xlat tables exhausted", MAX_XLAT_TABLES);

			return NULL;
		}

		new_table = prtn->xlat_tables +
			    prtn->xlat_tables_used * XLAT_TABLE_ENTRIES;
		prtn->xlat_tables_used++;

		DMSG("xlat tables used %u / %u",
		     prtn->xlat_tables_used, MAX_XLAT_TABLES);
	}

	return new_table;
}

/*
 * Given an entry that points to a table returns the virtual address
 * of the pointed table. NULL otherwise.
 */
static void *core_mmu_xlat_table_entry_pa2va(struct mmu_partition *prtn,
					     unsigned int level,
					     uint64_t entry)
{
	paddr_t pa = 0;
	void *va = NULL;

	if ((entry & DESC_ENTRY_TYPE_MASK) != TABLE_DESC ||
	    level >= XLAT_TABLE_LEVEL_MAX)
		return NULL;

	pa = entry & OUTPUT_ADDRESS_MASK;

	if (!IS_ENABLED(CFG_NS_VIRTUALIZATION) || prtn == &default_partition)
		va = phys_to_virt(pa, MEM_AREA_TEE_RAM_RW_DATA,
				  XLAT_TABLE_SIZE);
	if (!va)
		va = phys_to_virt(pa, MEM_AREA_SEC_RAM_OVERALL,
				  XLAT_TABLE_SIZE);

	return va;
}

/*
 * For a table entry that points to a table - allocate and copy to
 * a new pointed table. This is done for the requested entry,
 * without going deeper into the pointed table entries.
 *
 * A success is returned for non-table entries, as nothing to do there.
 */
__maybe_unused
static bool core_mmu_entry_copy(struct core_mmu_table_info *tbl_info,
				unsigned int idx)
{
	uint64_t *orig_table = NULL;
	uint64_t *new_table = NULL;
	uint64_t *entry = NULL;
	struct mmu_partition *prtn = NULL;

#ifdef CFG_NS_VIRTUALIZATION
	prtn = tbl_info->prtn;
#else
	prtn = &default_partition;
#endif
	assert(prtn);

	if (idx >= tbl_info->num_entries)
		return false;

	entry = (uint64_t *)tbl_info->table + idx;

	/* Nothing to do for non-table entries */
	if ((*entry & DESC_ENTRY_TYPE_MASK) != TABLE_DESC ||
	    tbl_info->level >= XLAT_TABLE_LEVEL_MAX)
		return true;

	new_table = core_mmu_xlat_table_alloc(prtn);
	if (!new_table)
		return false;

	orig_table = core_mmu_xlat_table_entry_pa2va(prtn, tbl_info->level,
						     *entry);
	if (!orig_table)
		return false;

	/* Copy original table content to new table */
	memcpy(new_table, orig_table, XLAT_TABLE_ENTRIES * XLAT_ENTRY_SIZE);

	/* Point to the new table */
	*entry = virt_to_phys(new_table) | (*entry & ~OUTPUT_ADDRESS_MASK);

	return true;
}

static void core_init_mmu_prtn_tee(struct mmu_partition *prtn,
				   struct memory_map *mem_map)
{
	size_t n = 0;

	assert(prtn && mem_map);

	for (n = 0; n < mem_map->count; n++) {
		struct tee_mmap_region *mm = mem_map->map + n;
		debug_print(" %010" PRIxVA " %010" PRIxPA " %10zx %x",
			    mm->va, mm->pa, mm->size, mm->attr);

		if (!IS_PAGE_ALIGNED(mm->pa) || !IS_PAGE_ALIGNED(mm->size))
			panic("unaligned region");
	}

	/* Clear table before use */
	memset(prtn->base_tables, 0, BASE_TABLE_SIZE * CFG_TEE_CORE_NB_CORE);

	for (n = 0; n < mem_map->count; n++)
		core_mmu_map_region(prtn, mem_map->map + n);

	/*
	 * Primary mapping table is ready at index `get_core_pos()`
	 * whose value may not be ZERO. Take this index as copy source.
	 */
	for (n = 0; n < CFG_TEE_CORE_NB_CORE; n++) {
		if (n == get_core_pos())
			continue;

		memcpy(get_base_table(prtn, 0, n),
		       get_base_table(prtn, 0, get_core_pos()),
		       XLAT_ENTRY_SIZE * NUM_BASE_LEVEL_ENTRIES);
	}
}

/*
 * In order to support 32-bit TAs we will have to find
 * a user VA base in the region [1GB, 4GB[.
 * Due to OP-TEE design limitation, TAs page table should be an entry
 * inside a level 1 page table.
 *
 * Available options are only these:
 * - base level 0 entry 0 - [0GB, 512GB[
 *   - level 1 entry 0 - [0GB, 1GB[
 *   - level 1 entry 1 - [1GB, 2GB[           <----
 *   - level 1 entry 2 - [2GB, 3GB[           <----
 *   - level 1 entry 3 - [3GB, 4GB[           <----
 *   - level 1 entry 4 - [4GB, 5GB[
 *   - ...
 * - ...
 *
 * - base level 1 entry 0 - [0GB, 1GB[
 * - base level 1 entry 1 - [1GB, 2GB[        <----
 * - base level 1 entry 2 - [2GB, 3GB[        <----
 * - base level 1 entry 3 - [3GB, 4GB[        <----
 * - base level 1 entry 4 - [4GB, 5GB[
 * - ...
 */
static void set_user_va_idx(struct mmu_partition *prtn)
{
	uint64_t *tbl = NULL;
	unsigned int n = 0;

	assert(prtn);

	tbl = get_base_table(prtn, 0, get_core_pos());

	/*
	 * If base level is 0, then we must use its entry 0.
	 */
	if (CORE_MMU_BASE_TABLE_LEVEL == 0) {
		/*
		 * If base level 0 entry 0 is not used then
		 * it's clear that we can use level 1 entry 1 inside it.
		 * (will be allocated later).
		 */
		if ((tbl[0] & DESC_ENTRY_TYPE_MASK) == INVALID_DESC) {
			user_va_idx = 1;

			return;
		}

		assert((tbl[0] & DESC_ENTRY_TYPE_MASK) == TABLE_DESC);

		tbl = core_mmu_xlat_table_entry_pa2va(prtn, 0, tbl[0]);
		assert(tbl);
	}

	/*
	 * Search level 1 table (i.e. 1GB mapping per entry) for
	 * an empty entry in the range [1GB, 4GB[.
	 */
	for (n = 1; n < 4; n++) {
		if ((tbl[n] & DESC_ENTRY_TYPE_MASK) == INVALID_DESC) {
			user_va_idx = n;
			break;
		}
	}

	assert(user_va_idx != -1);
}

/*
 * Setup an entry inside a core level 1 page table for TAs memory mapping
 *
 * If base table level is 1 - user_va_idx is already the index,
 *                            so nothing to do.
 * If base table level is 0 - we might need to allocate entry 0 of base table,
 *                            as TAs page table is an entry inside a level 1
 *                            page table.
 */
static void core_init_mmu_prtn_ta_core(struct mmu_partition *prtn
				       __maybe_unused,
				       unsigned int base_idx __maybe_unused,
				       unsigned int core __maybe_unused)
{
#if (CORE_MMU_BASE_TABLE_LEVEL == 0)
	struct core_mmu_table_info tbl_info = { };
	uint64_t *tbl = NULL;

	assert(user_va_idx != -1);
	COMPILE_TIME_ASSERT(MAX_XLAT_TABLES < (1 << (8 * sizeof(l1_idx_t))));

	tbl = get_base_table(prtn, base_idx, core);

	/*
	 * If base level is 0, then user_va_idx refers to
	 * level 1 page table that's in base level 0 entry 0.
	 */
	core_mmu_set_info_table(&tbl_info, 0, 0, tbl);
#ifdef CFG_NS_VIRTUALIZATION
	tbl_info.prtn = prtn;
#endif

	/*
	 * If this isn't the core that created the initial tables
	 * mappings, then the level 1 table must be copied,
	 * as it will hold pointer to the user mapping table
	 * that changes per core.
	 */
	if (core != get_core_pos()) {
		if (!core_mmu_entry_copy(&tbl_info, 0))
			panic();
	}

	if (!core_mmu_entry_to_finer_grained(&tbl_info, 0, true))
		panic();

	/*
	 * Now base level table should be ready with a table descriptor
	 */
	assert((tbl[0] & DESC_ENTRY_TYPE_MASK) == TABLE_DESC);

	tbl = core_mmu_xlat_table_entry_pa2va(prtn, 0, tbl[0]);
	assert(tbl);

	set_l1_ta_table(prtn, base_idx, core, tbl);
#endif
}

static void core_init_mmu_prtn_ta(struct mmu_partition *prtn)
{
	unsigned int base_idx = 0;
	unsigned int core = 0;

	assert(user_va_idx != -1);

	for (base_idx = 0; base_idx < NUM_BASE_TABLES; base_idx++)
		for (core = 0; core < CFG_TEE_CORE_NB_CORE; core++)
			core_init_mmu_prtn_ta_core(prtn, base_idx, core);
}

void core_init_mmu_prtn(struct mmu_partition *prtn, struct memory_map *mem_map)
{
	core_init_mmu_prtn_tee(prtn, mem_map);
	core_init_mmu_prtn_ta(prtn);
}

void core_init_mmu(struct memory_map *mem_map)
{
	struct mmu_partition *prtn = &default_partition;
	uint64_t max_va = 0;
	size_t n;

	COMPILE_TIME_ASSERT(CORE_MMU_BASE_TABLE_SHIFT ==
			    XLAT_ADDR_SHIFT(CORE_MMU_BASE_TABLE_LEVEL));
#ifdef CFG_CORE_UNMAP_CORE_AT_EL0
	COMPILE_TIME_ASSERT(CORE_MMU_BASE_TABLE_OFFSET ==
			    BASE_TABLE_SIZE / NUM_BASE_TABLES);
#endif

	if (IS_ENABLED(CFG_DYN_CONFIG)) {
#if (CORE_MMU_BASE_TABLE_LEVEL == 0)
		prtn->user_l1_table_idx = boot_mem_alloc(NUM_BASE_TABLES *
							 CFG_TEE_CORE_NB_CORE *
							 sizeof(l1_idx_t),
							 alignof(l1_idx_t));
		boot_mem_add_reloc(&prtn->user_l1_table_idx);
#endif
		prtn->base_tables = boot_mem_alloc(BASE_TABLE_SIZE *
						   CFG_TEE_CORE_NB_CORE,
						   NUM_BASE_LEVEL_ENTRIES *
						   XLAT_ENTRY_SIZE);
		boot_mem_add_reloc(&prtn->base_tables);

		prtn->l2_ta_tables = boot_mem_alloc(XLAT_TABLE_SIZE *
						    CFG_NUM_THREADS,
						    XLAT_TABLE_SIZE);
		boot_mem_add_reloc(&prtn->l2_ta_tables);
	}

	/* Initialize default pagetables */
	core_init_mmu_prtn_tee(&default_partition, mem_map);

	for (n = 0; n < mem_map->count; n++) {
		vaddr_t va_end = mem_map->map[n].va + mem_map->map[n].size - 1;

		if (va_end > max_va)
			max_va = va_end;
	}

	set_user_va_idx(&default_partition);

	core_init_mmu_prtn_ta(&default_partition);

	COMPILE_TIME_ASSERT(CFG_LPAE_ADDR_SPACE_BITS > L1_XLAT_ADDRESS_SHIFT);
	assert(max_va < BIT64(CFG_LPAE_ADDR_SPACE_BITS));
}

#ifdef CFG_WITH_PAGER
/* Prefer to consume only 1 base xlat table for the whole mapping */
bool core_mmu_prefer_tee_ram_at_top(paddr_t paddr)
{
	size_t base_level_size = BASE_XLAT_BLOCK_SIZE;
	paddr_t base_level_mask = base_level_size - 1;

	return (paddr & base_level_mask) > (base_level_size / 2);
}
#endif

#ifdef ARM32
void core_init_mmu_regs(struct core_mmu_config *cfg)
{
	struct mmu_partition *prtn = &default_partition;
	uint32_t ttbcr = 0;
	uint32_t mair = 0;

	cfg->ttbr0_base = virt_to_phys(get_base_table(prtn, 0, 0));
	cfg->ttbr0_core_offset = BASE_TABLE_SIZE;

	mair  = MAIR_ATTR_SET(ATTR_DEVICE_nGnRE, ATTR_DEVICE_nGnRE_INDEX);
	mair |= MAIR_ATTR_SET(ATTR_IWBWA_OWBWA_NTR, ATTR_IWBWA_OWBWA_NTR_INDEX);
	mair |= MAIR_ATTR_SET(ATTR_DEVICE_nGnRnE, ATTR_DEVICE_nGnRnE_INDEX);
	/*
	 * Tagged memory isn't supported in 32-bit mode, map tagged memory
	 * as normal memory instead.
	 */
	mair |= MAIR_ATTR_SET(ATTR_IWBWA_OWBWA_NTR,
			      ATTR_TAGGED_NORMAL_MEM_INDEX);
	cfg->mair0 = mair;

	ttbcr = TTBCR_EAE;
	ttbcr |= TTBCR_XRGNX_WBWA << TTBCR_IRGN0_SHIFT;
	ttbcr |= TTBCR_XRGNX_WBWA << TTBCR_ORGN0_SHIFT;
	ttbcr |= TTBCR_SHX_ISH << TTBCR_SH0_SHIFT;
	ttbcr |= TTBCR_EPD1;	/* Disable the use of TTBR1 */

	/* TTBCR.A1 = 0 => ASID is stored in TTBR0 */
	cfg->ttbcr = ttbcr;
}
#endif /*ARM32*/

#ifdef ARM64
static unsigned int get_hard_coded_pa_size_bits(void)
{
	/*
	 * Intermediate Physical Address Size.
	 * 0b000      32 bits, 4GB.
	 * 0b001      36 bits, 64GB.
	 * 0b010      40 bits, 1TB.
	 * 0b011      42 bits, 4TB.
	 * 0b100      44 bits, 16TB.
	 * 0b101      48 bits, 256TB.
	 * 0b110      52 bits, 4PB
	 */
	static_assert(CFG_CORE_ARM64_PA_BITS >= 32);
	static_assert(CFG_CORE_ARM64_PA_BITS <= 52);

	if (CFG_CORE_ARM64_PA_BITS <= 32)
		return TCR_PS_BITS_4GB;

	if (CFG_CORE_ARM64_PA_BITS <= 36)
		return TCR_PS_BITS_64GB;

	if (CFG_CORE_ARM64_PA_BITS <= 40)
		return TCR_PS_BITS_1TB;

	if (CFG_CORE_ARM64_PA_BITS <= 42)
		return TCR_PS_BITS_4TB;

	if (CFG_CORE_ARM64_PA_BITS <= 44)
		return TCR_PS_BITS_16TB;

	if (CFG_CORE_ARM64_PA_BITS <= 48)
		return TCR_PS_BITS_256TB;

	/* CFG_CORE_ARM64_PA_BITS <= 48 */
	return TCR_PS_BITS_4PB;
}

static unsigned int get_physical_addr_size_bits(void)
{
	const unsigned int size_bits = read_id_aa64mmfr0_el1() &
				       ID_AA64MMFR0_EL1_PARANGE_MASK;
	unsigned int b = 0;

	if (IS_ENABLED(CFG_AUTO_MAX_PA_BITS))
		return size_bits;

	b = get_hard_coded_pa_size_bits();
	assert(b <= size_bits);
	return b;
}

unsigned int core_mmu_arm64_get_pa_width(void)
{
	const uint8_t map[] = { 32, 36, 40, 42, 44, 48, 52, };
	unsigned int size_bits = get_physical_addr_size_bits();

	size_bits = MIN(size_bits, ARRAY_SIZE(map) - 1);
	return map[size_bits];
}

void core_init_mmu_regs(struct core_mmu_config *cfg)
{
	struct mmu_partition *prtn = &default_partition;
	uint64_t ips = get_physical_addr_size_bits();
	uint64_t mair = 0;
	uint64_t tcr = 0;

	cfg->ttbr0_el1_base = virt_to_phys(get_base_table(prtn, 0, 0));
	cfg->ttbr0_core_offset = BASE_TABLE_SIZE;

	mair  = MAIR_ATTR_SET(ATTR_DEVICE_nGnRE, ATTR_DEVICE_nGnRE_INDEX);
	mair |= MAIR_ATTR_SET(ATTR_IWBWA_OWBWA_NTR, ATTR_IWBWA_OWBWA_NTR_INDEX);
	mair |= MAIR_ATTR_SET(ATTR_DEVICE_nGnRnE, ATTR_DEVICE_nGnRnE_INDEX);
	/*
	 * If MEMTAG isn't enabled, map tagged memory as normal memory
	 * instead.
	 */
	if (memtag_is_enabled())
		mair |= MAIR_ATTR_SET(ATTR_TAGGED_NORMAL_MEM,
				      ATTR_TAGGED_NORMAL_MEM_INDEX);
	else
		mair |= MAIR_ATTR_SET(ATTR_IWBWA_OWBWA_NTR,
				      ATTR_TAGGED_NORMAL_MEM_INDEX);
	cfg->mair_el1 = mair;

	tcr = TCR_RES1;
	tcr |= TCR_XRGNX_WBWA << TCR_IRGN0_SHIFT;
	tcr |= TCR_XRGNX_WBWA << TCR_ORGN0_SHIFT;
	tcr |= TCR_SHX_ISH << TCR_SH0_SHIFT;
	tcr |= ips << TCR_EL1_IPS_SHIFT;
	tcr |= 64 - CFG_LPAE_ADDR_SPACE_BITS;

	/* Disable the use of TTBR1 */
	tcr |= TCR_EPD1;

	/*
	 * TCR.A1 = 0 => ASID is stored in TTBR0
	 * TCR.AS = 0 => Same ASID size as in Aarch32/ARMv7
	 */
	cfg->tcr_el1 = tcr;
}
#endif /*ARM64*/

void core_mmu_set_info_table(struct core_mmu_table_info *tbl_info,
		unsigned level, vaddr_t va_base, void *table)
{
	tbl_info->level = level;
	tbl_info->next_level = level + 1;
	tbl_info->table = table;
	tbl_info->va_base = va_base;
	tbl_info->shift = XLAT_ADDR_SHIFT(level);

#if (CORE_MMU_BASE_TABLE_LEVEL > 0)
	assert(level >= CORE_MMU_BASE_TABLE_LEVEL);
#endif
	assert(level <= XLAT_TABLE_LEVEL_MAX);

	if (level == CORE_MMU_BASE_TABLE_LEVEL)
		tbl_info->num_entries = NUM_BASE_LEVEL_ENTRIES;
	else
		tbl_info->num_entries = XLAT_TABLE_ENTRIES;
}

void core_mmu_get_user_pgdir(struct core_mmu_table_info *pgd_info)
{
	vaddr_t va_range_base;
	void *tbl = get_l2_ta_tables(get_prtn(), thread_get_id());

	core_mmu_get_user_va_range(&va_range_base, NULL);
	core_mmu_set_info_table(pgd_info, 2, va_range_base, tbl);
}

void core_mmu_create_user_map(struct user_mode_ctx *uctx,
			      struct core_mmu_user_map *map)
{
	struct core_mmu_table_info dir_info;

	COMPILE_TIME_ASSERT(sizeof(uint64_t) * XLAT_TABLE_ENTRIES == PGT_SIZE);

	core_mmu_get_user_pgdir(&dir_info);
	memset(dir_info.table, 0, PGT_SIZE);
	core_mmu_populate_user_map(&dir_info, uctx);
	map->user_map = virt_to_phys(dir_info.table) | TABLE_DESC;
	map->asid = uctx->vm_info.asid;
}

bool core_mmu_find_table(struct mmu_partition *prtn, vaddr_t va,
			 unsigned max_level,
			 struct core_mmu_table_info *tbl_info)
{
	uint32_t exceptions = thread_mask_exceptions(THREAD_EXCP_ALL);
	unsigned int num_entries = NUM_BASE_LEVEL_ENTRIES;
	unsigned int level = CORE_MMU_BASE_TABLE_LEVEL;
	vaddr_t va_base = 0;
	bool ret = false;
	uint64_t *tbl;

	if (!prtn)
		prtn = get_prtn();
	tbl = get_base_table(prtn, 0, get_core_pos());

	while (true) {
		unsigned int level_size_shift = XLAT_ADDR_SHIFT(level);
		unsigned int n = (va - va_base) >> level_size_shift;

		if (n >= num_entries)
			goto out;

		if (level == max_level || level == XLAT_TABLE_LEVEL_MAX ||
		    (tbl[n] & TABLE_DESC) != TABLE_DESC) {
			/*
			 * We've either reached max_level, a block
			 * mapping entry or an "invalid" mapping entry.
			 */

			/*
			 * Base level is the CPU specific translation table.
			 * It doesn't make sense to return anything based
			 * on that unless foreign interrupts already are
			 * masked.
			 */
			if (level == CORE_MMU_BASE_TABLE_LEVEL &&
			    !(exceptions & THREAD_EXCP_FOREIGN_INTR))
				goto out;

			tbl_info->table = tbl;
			tbl_info->va_base = va_base;
			tbl_info->level = level;
			tbl_info->next_level = level + 1;
			tbl_info->shift = level_size_shift;
			tbl_info->num_entries = num_entries;
#ifdef CFG_NS_VIRTUALIZATION
			tbl_info->prtn = prtn;
#endif
			ret = true;
			goto out;
		}

		tbl = core_mmu_xlat_table_entry_pa2va(prtn, level, tbl[n]);

		if (!tbl)
			goto out;

		va_base += (vaddr_t)n << level_size_shift;
		level++;
		num_entries = XLAT_TABLE_ENTRIES;
	}
out:
	thread_unmask_exceptions(exceptions);
	return ret;
}

bool core_mmu_entry_to_finer_grained(struct core_mmu_table_info *tbl_info,
				     unsigned int idx, bool secure __unused)
{
	uint64_t *new_table;
	uint64_t *entry;
	int i;
	paddr_t pa;
	uint64_t attr;
	paddr_t block_size_on_next_lvl = XLAT_BLOCK_SIZE(tbl_info->level + 1);
	struct mmu_partition *prtn;

#ifdef CFG_NS_VIRTUALIZATION
	prtn = tbl_info->prtn;
#else
	prtn = &default_partition;
#endif
	assert(prtn);

	if (tbl_info->level >= XLAT_TABLE_LEVEL_MAX ||
	    idx >= tbl_info->num_entries)
		return false;

	entry = (uint64_t *)tbl_info->table + idx;

	if ((*entry & DESC_ENTRY_TYPE_MASK) == TABLE_DESC)
		return true;

	new_table = core_mmu_xlat_table_alloc(prtn);
	if (!new_table)
		return false;

	if (*entry) {
		pa = *entry & OUTPUT_ADDRESS_MASK;
		attr = *entry & ~(OUTPUT_ADDRESS_MASK | DESC_ENTRY_TYPE_MASK);
		for (i = 0; i < XLAT_TABLE_ENTRIES; i++) {
			new_table[i] = pa | attr | BLOCK_DESC;
			pa += block_size_on_next_lvl;
		}
	} else {
		memset(new_table, 0, XLAT_TABLE_ENTRIES * XLAT_ENTRY_SIZE);
	}

	*entry = virt_to_phys(new_table) | TABLE_DESC;

	return true;
}

void core_mmu_set_entry_primitive(void *table, size_t level, size_t idx,
				  paddr_t pa, uint32_t attr)
{
	uint64_t *tbl = table;
	uint64_t desc = mattr_to_desc(level, attr);

	tbl[idx] = desc | pa;
}

void core_mmu_get_entry_primitive(const void *table, size_t level,
				  size_t idx, paddr_t *pa, uint32_t *attr)
{
	const uint64_t *tbl = table;

	if (pa)
		*pa = tbl[idx] & GENMASK_64(47, 12);

	if (attr)
		*attr = desc_to_mattr(level, tbl[idx]);
}

bool core_mmu_user_va_range_is_defined(void)
{
	return user_va_idx != -1;
}

void core_mmu_get_user_va_range(vaddr_t *base, size_t *size)
{
	assert(user_va_idx != -1);

	if (base)
		*base = (vaddr_t)user_va_idx << L1_XLAT_ADDRESS_SHIFT;
	if (size)
		*size = BIT64(L1_XLAT_ADDRESS_SHIFT);
}

static uint64_t *core_mmu_get_user_mapping_entry(struct mmu_partition *prtn,
						 unsigned int base_idx)
{
	uint64_t *tbl = NULL;

	assert(user_va_idx != -1);

#if (CORE_MMU_BASE_TABLE_LEVEL == 0)
	tbl = get_l1_ta_table(prtn, base_idx, get_core_pos());
#else
	tbl =  get_base_table(prtn, base_idx, get_core_pos());
#endif

	return tbl + user_va_idx;
}

bool core_mmu_user_mapping_is_active(void)
{
	bool ret = false;
	uint32_t exceptions = thread_mask_exceptions(THREAD_EXCP_ALL);
	uint64_t *entry = NULL;

	entry = core_mmu_get_user_mapping_entry(get_prtn(), 0);
	ret = (*entry != 0);

	thread_unmask_exceptions(exceptions);

	return ret;
}

#ifdef ARM32
void core_mmu_get_user_map(struct core_mmu_user_map *map)
{
	struct mmu_partition *prtn = get_prtn();
	uint64_t *entry = NULL;

	entry = core_mmu_get_user_mapping_entry(prtn, 0);

	map->user_map = *entry;
	if (map->user_map) {
		map->asid = (read_ttbr0_64bit() >> TTBR_ASID_SHIFT) &
			    TTBR_ASID_MASK;
	} else {
		map->asid = 0;
	}
}

void core_mmu_set_user_map(struct core_mmu_user_map *map)
{
	uint64_t ttbr = 0;
	uint32_t exceptions = thread_mask_exceptions(THREAD_EXCP_ALL);
	struct mmu_partition *prtn = get_prtn();
	uint64_t *entries[NUM_BASE_TABLES] = { };
	unsigned int i = 0;

	ttbr = read_ttbr0_64bit();
	/* Clear ASID */
	ttbr &= ~((uint64_t)TTBR_ASID_MASK << TTBR_ASID_SHIFT);
	write_ttbr0_64bit(ttbr);
	isb();

	for (i = 0; i < NUM_BASE_TABLES; i++)
		entries[i] = core_mmu_get_user_mapping_entry(prtn, i);

	/* Set the new map */
	if (map && map->user_map) {
		for (i = 0; i < NUM_BASE_TABLES; i++)
			*entries[i] = map->user_map;

		dsb();	/* Make sure the write above is visible */
		ttbr |= ((uint64_t)map->asid << TTBR_ASID_SHIFT);
		write_ttbr0_64bit(ttbr);
		isb();
	} else {
		for (i = 0; i < NUM_BASE_TABLES; i++)
			*entries[i] = INVALID_DESC;

		dsb();	/* Make sure the write above is visible */
	}

	tlbi_all();
	icache_inv_all();

	thread_unmask_exceptions(exceptions);
}

enum core_mmu_fault core_mmu_get_fault_type(uint32_t fault_descr)
{
	assert(fault_descr & FSR_LPAE);

	switch (fault_descr & FSR_STATUS_MASK) {
	case 0x10: /* b010000 Synchronous extern abort, not on table walk */
	case 0x15: /* b010101 Synchronous extern abort, on table walk L1 */
	case 0x16: /* b010110 Synchronous extern abort, on table walk L2 */
	case 0x17: /* b010111 Synchronous extern abort, on table walk L3 */
		return CORE_MMU_FAULT_SYNC_EXTERNAL;
	case 0x11: /* b010001 Asynchronous extern abort (DFSR only) */
		return CORE_MMU_FAULT_ASYNC_EXTERNAL;
	case 0x21: /* b100001 Alignment fault */
		return CORE_MMU_FAULT_ALIGNMENT;
	case 0x22: /* b100010 Debug event */
		return CORE_MMU_FAULT_DEBUG_EVENT;
	default:
		break;
	}

	switch ((fault_descr & FSR_STATUS_MASK) >> 2) {
	case 0x1: /* b0001LL Translation fault */
		return CORE_MMU_FAULT_TRANSLATION;
	case 0x2: /* b0010LL Access flag fault */
	case 0x3: /* b0011LL Permission fault */
		if (fault_descr & FSR_WNR)
			return CORE_MMU_FAULT_WRITE_PERMISSION;
		else
			return CORE_MMU_FAULT_READ_PERMISSION;
	default:
		return CORE_MMU_FAULT_OTHER;
	}
}
#endif /*ARM32*/

#ifdef ARM64
void core_mmu_get_user_map(struct core_mmu_user_map *map)
{
	struct mmu_partition *prtn = get_prtn();
	uint64_t *entry = NULL;

	entry = core_mmu_get_user_mapping_entry(prtn, 0);

	map->user_map = *entry;
	if (map->user_map) {
		map->asid = (read_ttbr0_el1() >> TTBR_ASID_SHIFT) &
			    TTBR_ASID_MASK;
	} else {
		map->asid = 0;
	}
}

void core_mmu_set_user_map(struct core_mmu_user_map *map)
{
	uint64_t ttbr = 0;
	uint32_t exceptions = thread_mask_exceptions(THREAD_EXCP_ALL);
	struct mmu_partition *prtn = get_prtn();
	uint64_t *entries[NUM_BASE_TABLES] = { };
	unsigned int i = 0;

	ttbr = read_ttbr0_el1();
	/* Clear ASID */
	ttbr &= ~((uint64_t)TTBR_ASID_MASK << TTBR_ASID_SHIFT);
	write_ttbr0_el1(ttbr);
	isb();

	for (i = 0; i < NUM_BASE_TABLES; i++)
		entries[i] = core_mmu_get_user_mapping_entry(prtn, i);

	/* Set the new map */
	if (map && map->user_map) {
		for (i = 0; i < NUM_BASE_TABLES; i++)
			*entries[i] = map->user_map;

		dsb();	/* Make sure the write above is visible */
		ttbr |= ((uint64_t)map->asid << TTBR_ASID_SHIFT);
		write_ttbr0_el1(ttbr);
		isb();
	} else {
		for (i = 0; i < NUM_BASE_TABLES; i++)
			*entries[i] = INVALID_DESC;

		dsb();	/* Make sure the write above is visible */
	}

	tlbi_all();
	icache_inv_all();

	thread_unmask_exceptions(exceptions);
}

enum core_mmu_fault core_mmu_get_fault_type(uint32_t fault_descr)
{
	switch ((fault_descr >> ESR_EC_SHIFT) & ESR_EC_MASK) {
	case ESR_EC_SP_ALIGN:
	case ESR_EC_PC_ALIGN:
		return CORE_MMU_FAULT_ALIGNMENT;
	case ESR_EC_IABT_EL0:
	case ESR_EC_DABT_EL0:
	case ESR_EC_IABT_EL1:
	case ESR_EC_DABT_EL1:
		switch (fault_descr & ESR_FSC_MASK) {
		case ESR_FSC_SIZE_L0:
		case ESR_FSC_SIZE_L1:
		case ESR_FSC_SIZE_L2:
		case ESR_FSC_SIZE_L3:
		case ESR_FSC_TRANS_L0:
		case ESR_FSC_TRANS_L1:
		case ESR_FSC_TRANS_L2:
		case ESR_FSC_TRANS_L3:
			return CORE_MMU_FAULT_TRANSLATION;
		case ESR_FSC_ACCF_L1:
		case ESR_FSC_ACCF_L2:
		case ESR_FSC_ACCF_L3:
		case ESR_FSC_PERMF_L1:
		case ESR_FSC_PERMF_L2:
		case ESR_FSC_PERMF_L3:
			if (fault_descr & ESR_ABT_WNR)
				return CORE_MMU_FAULT_WRITE_PERMISSION;
			else
				return CORE_MMU_FAULT_READ_PERMISSION;
		case ESR_FSC_ALIGN:
			return CORE_MMU_FAULT_ALIGNMENT;
		case ESR_FSC_TAG_CHECK:
			return CORE_MMU_FAULT_TAG_CHECK;
		case ESR_FSC_SEA_NTT:
		case ESR_FSC_SEA_TT_SUB_L2:
		case ESR_FSC_SEA_TT_SUB_L1:
		case ESR_FSC_SEA_TT_L0:
		case ESR_FSC_SEA_TT_L1:
		case ESR_FSC_SEA_TT_L2:
		case ESR_FSC_SEA_TT_L3:
			return CORE_MMU_FAULT_SYNC_EXTERNAL;
		default:
			return CORE_MMU_FAULT_OTHER;
		}
	default:
		return CORE_MMU_FAULT_OTHER;
	}
}

/* ── VMI PAGE-TABLE ATTACK TEST ──────────────────────────────────
 * Malicious page-table edits used to validate the S-EL2 page-table
 * defense. Each attack crafts one leaf PTE that violates a specific
 * defense policy and writes it into a defense-tracked L3 table. On a
 * defended build the store traps and the SPMC panics; on baseline it
 * silently applies. Targets are dedicated pages (a writable .bss page,
 * a read-only .rodata page) plus a .text function, so the attacks have
 * no collateral effect on a baseline run.
 */
#define ATTACK_PA_MASK 0x0000FFFFFFFFF000ULL

static uint8_t attack_rw_page[PAGE_SIZE] __aligned(PAGE_SIZE);
static const uint8_t attack_ro_page[PAGE_SIZE] __aligned(PAGE_SIZE) = { 1 };

/* A14 callback: invoked by the injected shellcode so a log line is
 * printed by code running under the attacker's control. Marked noinline
 * and used so the compiler cannot DCE it or inline it away. */
static void __attribute__((noinline, used)) a14_injected_msg(void)
{
	IMSG(">>> A14 injected shellcode speaking from S-EL1 (canary=0xC0DE) <<<");
}

/* A14 payload: injected into the target TA page and called at EL1
 * after PXN is cleared. Emitted by the assembler (not hand-encoded)
 * so the bytes are guaranteed correct.
 *
 * Calling convention: x0 = pointer to a14_injected_msg (passed by the
 * syscall as the sole argument of fn). Shellcode:
 *   stp x29, x30, [sp, #-16]!  ; save frame + return addr to syscall
 *   blr x0                      ; call the callback (prints IMSG)
 *   ldp x29, x30, [sp], #16     ; restore
 *   movz w0, #0xc0de            ; canary return value
 *   ret                         ; return to syscall
 *
 * This proves execution twice: the callback's IMSG line is printed
 * from within the shellcode's control flow, AND the shellcode returns
 * the 0xC0DE canary through the syscall's return-value path. */
extern const uint32_t a14_payload_start[];
extern const uint32_t a14_payload_end[];
asm(
	".pushsection .rodata\n"
	".global a14_payload_start\n"
	".global a14_payload_end\n"
	"a14_payload_start:\n"
	"	stp	x29, x30, [sp, #-16]!\n"
	"	blr	x0\n"
	"	ldp	x29, x30, [sp], #16\n"
	"	movz	w0, #0xc0de\n"
	"	ret\n"
	"a14_payload_end:\n"
	".popsection\n"
);

static uint64_t *attack_find_l3_entry(vaddr_t va)
{
	uint64_t ttbr0 = read_ttbr0_el1();
	uint64_t *l1 = phys_to_virt(ttbr0 & ATTACK_PA_MASK,
				    MEM_AREA_TEE_RAM, sizeof(uint64_t));
	uint64_t l1_idx = (va >> 30) & 0x1FF;
	uint64_t *l2 = phys_to_virt(l1[l1_idx] & ATTACK_PA_MASK,
				    MEM_AREA_TEE_RAM, sizeof(uint64_t));
	uint64_t l2_idx = (va >> 21) & 0x1FF;
	uint64_t *l3 = phys_to_virt(l2[l2_idx] & ATTACK_PA_MASK,
				    MEM_AREA_TEE_RAM, sizeof(uint64_t));
	uint64_t l3_idx = (va >> 12) & 0x1FF;

	return &l3[l3_idx];
}

/* Walk TTBR0_EL1 → return the live L1 base (kernel L1 during the syscall).
 * The L1 page holds both kernel L1 (slots 0..3 with T0SZ=32) and TA L1
 * (slots 4..7), contiguous in 4 KB. */
static uint64_t *attack_get_l1(void)
{
	uint64_t ttbr0 = read_ttbr0_el1();

	return phys_to_virt(ttbr0 & ATTACK_PA_MASK,
			    MEM_AREA_TEE_RAM, sizeof(uint64_t));
}

/* Walk TTBR0 → L1 → return the L2 base reachable from the L1 entry that
 * covers `va`. For a kernel VA this is the kernel L2; for a TA user VA
 * this is the per-thread L2. */
static uint64_t *attack_get_l2(vaddr_t va)
{
	uint64_t *l1 = attack_get_l1();
	uint64_t l1_idx = (va >> 30) & 0x1FF;

	return phys_to_virt(l1[l1_idx] & ATTACK_PA_MASK,
			    MEM_AREA_TEE_RAM, sizeof(uint64_t));
}

/* First L2 slot whose descriptor is 0 (unmapped). -1 if none. */
static int32_t attack_find_empty_l2_slot(uint64_t *l2)
{
	for (int32_t i = 0; i < 512; i++)
		if (l2[i] == 0)
			return i;
	return -1;
}

/* Unified page-table attack dispatcher. A#-numbered per the thesis
 * taxonomy (section 5.2.3). cmd_id passed through from the TA equals
 * A companion probes are numbered adjacent to their parent:
 *   A9a (cmd_id 9)  → A9b  (cmd_id 10)  — dispatcher ordering
 *   A11a (cmd_id 12) → A11b (cmd_id 13)  — kernel-L2 NSTable
 * All other attacks stay in A#-labelled sequence; because of the two
 * a/b insertions, cmd_id ≠ A# for A10 onwards (A10 is at cmd_id 11,
 * A12 at 14, A24 at 26).
 *
 * L1 LAYOUT (build-dependent — this file assumes CFG_TEE_CORE_NB_CORE=1,
 * T0SZ=32, so l1_total_entries=4 and user_va_idx=3):
 *
 *   entry_idx:  0  1  2  3  |  4  5  6  7
 *               └── table 0 ─┘  └── table 1 ─┘
 *   kernel      K  K  K       K  K  K
 *   user slot            U                  U
 *
 *   L1_USER_SLOT       = 3           (first table's user slot)
 *   L1_MELTDOWN_USER   = 7           (second table's user slot: 4+3)
 *   L1_MELTDOWN_KSLOT  = 5           (any kernel slot in second table)
 *
 * If user_va_idx or l1_total_entries change (e.g. multi-core or different
 * T0SZ), the L1 attacks (A3, A4, A6, A7, A8, A9-order) need their slot
 * indices updated to match. The defense will panic with a message citing
 * the observed user_va_idx if we miss.
 *
 * Target selection:
 *   - L1 attacks (A3-A8, A9b): walk kernel TTBR0 via attack_get_l1().
 *   - L2 attacks (A10-A12, A23-A24): need TA VA in target_va so
 *     the walk resolves to the per-thread L2.
 *   - Kernel L2 attacks (A11b, A13): walk kernel L2 via attack_rw_page VA.
 *   - TA L3 PTE-content (A14, A15, A19-A22): need TA VA in target_va.
 *   - Core L3 / kernel target (A16, A17, A18): use kernel target
 *     internally — the escalation *is* touching core, so a TA target
 *     would be a weaker (or different) claim.
 *   - P1 register channel: A1 (cmd_id 1) is a synthetic check-
 *     verification PoC (no natural memory→TTBR0 code path in this
 *     build); A2 (cmd_id 2) is a natural indirect vector (user_map
 *     corruption caught downstream at the L1 write).
 */
#define L1_USER_SLOT      3
#define L1_MELTDOWN_USER  7
#define L1_MELTDOWN_KSLOT 5
void pgtable_attack(unsigned long attack_type, unsigned long target_va)
{
	switch (attack_type) {

	/* ────────── P1: register channel (A1, synthetic) ────────── */

	case 1: { /* A1 (SYNTHETIC): exercise TVM+whitelist against a
		   * memory-derived TTBR0 value.
		   *
		   * Code-walk finding: this OP-TEE build has NO runtime code
		   * path that reloads TTBR0's PA field from writable memory.
		   * core_mmu_set_prtn is #ifdef CFG_NS_VIRTUALIZATION (off);
		   * every other write_ttbr0_el1 site does read-modify-write
		   * of the live register. So the register channel has no
		   * threat-model-faithful in-model attack in this build.
		   *
		   * A1 verifies the CHECK itself by synthesizing a load-then-
		   * MSR sequence: attacker's realistic capability is the
		   * memory store to `attacker_ttbr`; the harness-added MSR
		   * completes the chain OP-TEE would have completed under a
		   * different config (or future code). Not a threat-model
		   * attack — a check-verification PoC. Presented paired with
		   * the analytical §5.2.1 argument on SCTLR/TCR/MAIR pinning
		   * and the natural base_tables→TTBR0 path. */
		static volatile uint64_t attacker_ttbr;

		/* (1) Realistic capability: memory store to writable variable. */
		attacker_ttbr = 0xdeadbeef000ULL;
		dsb();

		/* (2) Harness-added load-then-MSR: no natural OP-TEE code path
		 * does this in the current build. */
		uint64_t v = attacker_ttbr;

		DMSG("A1 (synthetic): msr ttbr0_el1, 0x%lx — "
		     "expect TVM+whitelist panic", v);
		asm volatile("msr ttbr0_el1, %0" : : "r"(v) : "memory");
		isb();
		break;
	}

	/* ────────── P2: novel-vector (A2) ────────── */

	case 2: { /* A2: L1 value-pin via indirect user_map corruption.
		   * Attacker's realistic capability: a memory-store primitive
		   * against ordinary writable memory (nex_pool per-thread
		   * struct). No page-table page is touched by the attacker.
		   *
		   * OP-TEE's own resume path (core_mmu_set_user_map) reads
		   * threads[t].user_map.user_map and writes it into the L1
		   * base-table entry at user_slot. That L1 write is what
		   * triggers the S2-RO fault — the L1 value-pin catches the
		   * corrupted value, not the register.
		   *
		   * Trigger note: in the CA→TA→syscall→return flow the resume
		   * doesn't happen naturally within one syscall (map is
		   * already installed). The PoC forces the natural code path
		   * to run by calling core_mmu_set_user_map here — the code
		   * path is OP-TEE's own, only the trigger timing is synthetic.
		   *
		   * Expected panic: same L1 VIOLATION string as A3, cited as
		   * evidence that the L1 monitor catches indirect vectors
		   * regardless of who ultimately issues the PT write. */
		unsigned int tid = thread_get_id();

		/* Save the legitimate user_map so we can restore it after the
		 * baseline readback — otherwise the corrupted mapping stays
		 * live and the TA mistranslates its own code on return,
		 * faulting forever. */
		struct core_mmu_user_map saved_map = threads[tid].user_map;

		threads[tid].user_map.user_map = 0xdeadbeef00000003ULL;
		dsb();
		DMSG("A2: corrupted threads[%u].user_map.user_map = "
		     "0xdeadbeef00000003; forcing set_user_map to trigger "
		     "the natural reinstall path", tid);
		core_mmu_set_user_map(&threads[tid].user_map);
		/* Defended build panics here (L1 value-pin inside
		 * core_mmu_set_user_map); nothing below runs. */

		/* Baseline observation: read back L1[user_slot] to prove the
		 * indirect vector installed the attacker value via OP-TEE's
		 * own reinstall path. */
		uint64_t *l1 = attack_get_l1();
		IMSG("A2 baseline: L1[%d] readback = 0x%lx "
		     "(indirect vector installed via OP-TEE's own code)",
		     L1_USER_SLOT, l1[L1_USER_SLOT]);

		/* Restore the legitimate mapping so the TA can execute again
		 * on return (baseline only — defended never reaches here). */
		threads[tid].user_map = saved_map;
		dsb();
		core_mmu_set_user_map(&threads[tid].user_map);
		break;
	}

	/* ────────── P2: L1 structural (A3–A7) ────────── */

	case 3: { /* A3: L1 attacker-L2 (TABLE + bad PA) in user slot.
		   * Fires L1 value-pin (slot check passes, value check
		   * refuses anything != 0 and != thread_l2_desc). */
		uint64_t *l1 = attack_get_l1();
		DMSG("A3: L1[%d] = 0xdeadbeef00000003 (bad TABLE, user slot)",
		     L1_USER_SLOT);
		l1[L1_USER_SLOT] = 0xdeadbeef00000003ULL;
		dsb();

		/* Baseline observation: read back the L1 entry to confirm the
		 * corrupted TABLE persists in the table. Unreachable on
		 * defended (L1 value-pin panics at the store above). */
		IMSG("A3 baseline: L1[%d] readback = 0x%lx "
		     "(corruption persisted in L1 table)",
		     L1_USER_SLOT, l1[L1_USER_SLOT]);
		break;
	}
	case 4: { /* A4: 1 GiB BLOCK descriptor at L1 user slot.
		   * Distinct from A3 in encoding — bits[1:0]=0b01 (BLOCK)
		   * rather than 0b11 (TABLE). Same L1 value-pin fires. */
		uint64_t *l1 = attack_get_l1();
		DMSG("A4: L1[%d] = 0x12345001 (1GiB BLOCK, user slot)",
		     L1_USER_SLOT);
		l1[L1_USER_SLOT] = 0x12345000ULL | 0x1;
		break;
	}
	case 5: { /* A5: legitimate thread_l2_desc into a kernel L1 slot.
		   * Read the current thread_l2_desc from the user slot; write
		   * it into a kernel slot. Value would be admitted at the user
		   * slot; slot check refuses it in a kernel slot. Proves
		   * slot-immutability is independent of value. */
		uint64_t *l1 = attack_get_l1();
		uint64_t td = l1[L1_USER_SLOT]; /* current user descriptor */
		DMSG("A5: L1[0] = 0x%lx (thread_l2_desc into kernel slot)", td);
		l1[0] = td;
		break;
	}
	case 6: { /* A6: attacker-L2 into Meltdown-side user slot.
		   * L1 holds two base tables of l1_total_entries each. The
		   * second table's user slot is at index (l1_total_entries +
		   * user_va_idx). Verifies the value-pin applies symmetrically
		   * to BOTH tables' user slots. */
		uint64_t *l1 = attack_get_l1();
		DMSG("A6: L1[%d] = 0xdeadbeef00000003 (Meltdown-side user)",
		     L1_MELTDOWN_USER);
		l1[L1_MELTDOWN_USER] = 0xdeadbeef00000003ULL;
		break;
	}
	case 7: { /* A7: write at Meltdown-side non-user slot.
		   * Any kernel slot in the second base table. Slot-immutability
		   * fires with slot ≠ user_va_idx. */
		uint64_t *l1 = attack_get_l1();
		DMSG("A7: L1[%d] = 0xdeadbeef00000003 (Meltdown-side kernel)",
		     L1_MELTDOWN_KSLOT);
		l1[L1_MELTDOWN_KSLOT] = 0xdeadbeef00000003ULL;
		break;
	}

	/* ────────── P2: fail-stop paths (A8, A9a, A9b at cmd_id 25) ────────── */

	case 8: { /* A8: L1 SAS gate — sub-doubleword store (strb).
		   * L1 handler has no memset fast-path, so byte stores go
		   * straight to the SAS check and panic on SAS≠3. Expected
		   * panic string cites SAS=0. Target: user slot so the fault
		   * fires before the SAS check runs. */
		uint64_t *l1 = attack_get_l1();
		volatile uint8_t *p = (volatile uint8_t *)&l1[L1_USER_SLOT];
		DMSG("A8: strb 0xaa at &L1[%d]=%p", L1_USER_SLOT, p);
		asm volatile("strb %w0, [%1]"
			     : : "r"((uint32_t)0xaa), "r"(p) : "memory");
		break;
	}
	case 9: { /* A9a: dispatcher ISV=0 gate — stp at core L3 PTE.
		   * The top-level ISV check panics before any region
		   * routing. Cites ESR=0x9200004f (ISV=0, DFSC=0x0F).
		   * Companion A9b at cmd_id 10 verifies dispatcher ordering
		   * (ISV gate fires before per-region L1 SAS gate). */
		uint64_t *pte = attack_find_l3_entry((vaddr_t)attack_rw_page);
		DMSG("A9a: stp at core L3 PTE %p", pte);
		asm volatile("stp %0, %1, [%2]"
			     : : "r"((uint64_t)0xdeadbeefULL),
				 "r"((uint64_t)0xcafef00dULL),
				 "r"(pte) : "memory");
		break;
	}
	case 10: { /* A9b: dispatcher ordering probe. Same stp encoding
		    * as A9a but targeting L1 user slot. Verifies top-level
		    * ISV=0 gate fires BEFORE per-region L1 SAS gate. Expected
		    * panic is ISV=0 (not L1 SAS). */
		uint64_t *l1 = attack_get_l1();

		DMSG("A9b: stp at &L1[%d]=%p", L1_USER_SLOT,
		     &l1[L1_USER_SLOT]);
		asm volatile("stp %0, %1, [%2]"
			     : : "r"((uint64_t)0xdeadbeefULL),
				 "r"((uint64_t)0xcafef00dULL),
				 "r"(&l1[L1_USER_SLOT]) : "memory");
		break;
	}

	/* ────────── P2: L2 policy (A10–A13) ────────── */

	case 11: { /* A10: L2 BLOCK descriptor in per-thread L2.
		    * Any block at L2 in a TA context is a policy violation
		    * (per-thread L2 must contain only clears or TABLE
		    * descriptors → pgt_tables). */
		if (!target_va) { EMSG("A10 requires TA VA"); return; }
		uint64_t *l2 = attack_get_l2((vaddr_t)target_va);
		int32_t slot = attack_find_empty_l2_slot(l2);
		if (slot < 0) { EMSG("A10: no empty slot"); return; }
		DMSG("A10: thread L2[%d] = 0x12345001 (BLOCK)", slot);
		l2[slot] = 0x12345000ULL | 0x1;
		break;
	}
	case 12: { /* A11a: L2 TABLE with NSTable=1 in per-thread L2.
		    * NSTable would force the entire L3 sub-tree non-secure,
		    * bypassing per-leaf NS policy. Refused unconditionally.
		    * Companion A11b at cmd_id 13 tests the same check in
		    * the kernel-L2 emulator (parallel code path). */
		if (!target_va) { EMSG("A11a requires TA VA"); return; }
		uint64_t *l2 = attack_get_l2((vaddr_t)target_va);
		int32_t slot = attack_find_empty_l2_slot(l2);
		if (slot < 0) { EMSG("A11a: no empty slot"); return; }
		DMSG("A11a: thread L2[%d] = TABLE | NSTable", slot);
		l2[slot] = 0x12345000ULL | 0x3 | (1ULL << 63);
		break;
	}
	case 13: { /* A11b: kernel L2 NSTable — kernel-side variant of A11a.
		    * Fires the NSTable check in vmi_emulate_kernel_l2_write
		    * (parallel to the ul1 check in vmi_check_thread_l2_entry). */
		uint64_t *kl2 = attack_get_l2((vaddr_t)attack_rw_page);
		int32_t slot = -1;

		for (int32_t i = 0; i < 512; i++)
			if (kl2[i] == 0) { slot = i; break; }
		if (slot < 0) { EMSG("A11b: no empty kernel L2 slot"); return; }
		DMSG("A11b: kernel L2[%d] = TABLE | NSTable", slot);
		kl2[slot] = 0x12345000ULL | 0x3 | (1ULL << 63);
		break;
	}
	case 14: { /* A12: L2 TABLE target outside pgt_tables (thread L2).
		    * Uses a real kernel L3 PA — not attacker garbage —
		    * so the failure is "kernel table linked into TA L2",
		    * not "malformed descriptor rejected". */
		if (!target_va) { EMSG("A12 requires TA VA"); return; }
		uint64_t *l2 = attack_get_l2((vaddr_t)target_va);
		uint64_t *klu = attack_get_l2((vaddr_t)attack_rw_page);
		uint64_t kl3_pa = 0;

		for (uint32_t i = 0; i < 512; i++) {
			if ((klu[i] & 0x3) == 0x3) {
				kl3_pa = klu[i] & ATTACK_PA_MASK;
				break;
			}
		}
		if (!kl3_pa) { EMSG("A12: no kernel L3 PA found"); return; }
		int32_t slot = attack_find_empty_l2_slot(l2);
		if (slot < 0) { EMSG("A12: no empty slot"); return; }
		DMSG("A12: link kernel L3 PA=0x%lx into thread L2[%d]",
		     kl3_pa, slot);
		l2[slot] = kl3_pa | 0x3;
		break;
	}
	case 15: { /* A13: kernel L2 TABLE target outside xlat_tables.
		    * Kernel L2 emulator refuses TABLE PAs outside the
		    * xlat_pool range. Target: attack_rw_page's own PA
		    * (in tee_ram_rw, outside xlat_tables). */
		uint64_t *kl2 = attack_get_l2((vaddr_t)attack_rw_page);
		int32_t slot = -1;

		for (int32_t i = 0; i < 512; i++)
			if (kl2[i] == 0) { slot = i; break; }
		if (slot < 0) { EMSG("A13: no empty kernel L2 slot"); return; }
		uint64_t bad_pa = virt_to_phys(attack_rw_page);
		DMSG("A13: kernel L2[%d] = 0x%lx | TABLE (outside xlat)",
		     slot, bad_pa);
		kl2[slot] = bad_pa | 0x3;
		break;
	}

	/* ────────── P3: execute-axis PTE policy (A14–A18) ────────── */

	case 16: { /* A14: EL1-EXEC via RO+X bypass of hardware WXN.
		    *
		    * OP-TEE forces SCTLR_EL1.WXN=1 (CFG_CORE_RWDATA_NOEXEC=y
		    * in arm.mk). Hardware treats any writable page as
		    * execute-never regardless of PXN/UXN, so a naive single
		    * PXN clear on a writable page cannot yield executable
		    * memory: the instruction fetch faults on baseline (as
		    * well as being trapped by the defence's W^X check). To
		    * isolate the defence's EL1-EXEC containment as sole
		    * barrier, this attack takes the sophisticated route:
		    *
		    *   Step 1: EL1 writes shellcode into ta_rw_page. Plain
		    *           data write to a legitimately-writable TA
		    *           page — no PTE mutation, no S2 trap.
		    *   Step 2: PTE flip to AP_RO=1, PXN=0. Result is RO +
		    *           EL1-executable. WXN permits fetches from
		    *           this page (no longer writable). On defended
		    *           the PTE store traps and fires EL1-EXEC
		    *           containment (PXN=0 with PA in ta_ram, which
		    *           is outside tee_ram_rx). On baseline it lands.
		    *   Step 3: Call the page. Baseline executes shellcode
		    *           and returns 0xC0DE — paper-citable "write
		    *           primitive escalated to arbitrary S-EL1 code
		    *           execution" evidence.
		    *
		    * The W^X policy is not fired by this attack (the new
		    * PTE is not writable). W^X remains a defence pillar
		    * co-enforced by SCTLR.WXN and analytically documented;
		    * no in-model attack isolates it on this build.
		    *
		    * Pair with A15 (case 17): A15 is the minimal single-
		    * PTE probe of the same EL1-EXEC check, without the
		    * injection or execution demo.
		    *
		    * PAN wrapper: the target page is EL0-accessible
		    * (AP_UNPRIV=1), so under FEAT_PAN3 (EPAN) with PAN=1
		    * EL1 instruction fetch would be prohibited. Brackets
		    * the fn() call with enter/exit_user_access to disable
		    * PAN across the call. */
		if (!target_va) { EMSG("A14 requires TA VA"); return; }

		size_t plen = (uintptr_t)a14_payload_end -
			      (uintptr_t)a14_payload_start;
		memcpy((void *)target_va, a14_payload_start, plen);
		cache_op_inner(DCACHE_AREA_CLEAN, (void *)target_va, plen);
		cache_op_inner(ICACHE_AREA_INVALIDATE, (void *)target_va, plen);
		dsb();
		isb();

		uint64_t *pte = attack_find_l3_entry((vaddr_t)target_va);
		uint64_t v = *pte;

		v |= LOWER_ATTRS(AP_RO);
		v &= ~UPPER_ATTRS(PXN);
		DMSG("A14: TA L3 PTE 0x%lx -> 0x%lx (AP_RO=1, PXN=0 → RO+X)",
		     *pte, v);
		dsb(); *pte = v; dsb(); tlbi_all(); isb();

		uint32_t (*fn)(void (*)(void)) =
			(uint32_t (*)(void (*)(void)))target_va;
		enter_user_access();
		uint32_t r = fn(a14_injected_msg);
		exit_user_access();
		IMSG("A14 baseline: injected code returned 0x%x %s", r,
		     (r == 0xC0DE) ? "-- CODE EXECUTED in S-EL1"
				   : "-- unexpected");
		break;
	}
	case 17: { /* A15: TA L3 EL1-EXEC — clear PXN on RO TA page.
		    * Target is a TA .rodata buffer (AP_RO=1), so writable=0;
		    * clearing PXN doesn't trip W^X. EL1-EXEC containment
		    * catches the PA being outside tee_ram_rx. */
		if (!target_va) { EMSG("A15 requires TA RO VA"); return; }
		uint64_t *pte = attack_find_l3_entry((vaddr_t)target_va);
		uint64_t v = *pte;

		v &= ~UPPER_ATTRS(PXN);
		DMSG("A15: TA L3 PTE 0x%lx -> 0x%lx (clear PXN, RO target)",
		     *pte, v);
		dsb(); *pte = v; dsb(); tlbi_all(); isb();
		break;
	}
	case 18: { /* A16: EL0-EXEC — clear UXN on kernel .rodata.
		    * Core target required: ta_ram is in the allowed EL0-EXEC
		    * list, so a TA-target version would silently pass. Kernel
		    * .rodata is outside VCORE_FREE / TA_RAM / kcode, so the
		    * EL0-EXEC containment check fires. */
		uint64_t *pte = attack_find_l3_entry((vaddr_t)attack_ro_page);
		uint64_t v = *pte;

		v &= ~UPPER_ATTRS(XN);
		DMSG("A16: kernel .rodata PTE 0x%lx -> 0x%lx (clear UXN)",
		     *pte, v);
		dsb(); *pte = v; dsb(); tlbi_all(); isb();
		break;
	}
	case 19: { /* A17: CODE-WRITE — clear AP_RO on kernel .rodata.
		    * Core target required: CODE-WRITE check gates on the
		    * PA overlapping tee_ram_rx/ro; a TA target (in ta_ram)
		    * would not overlap. */
		uint64_t *pte = attack_find_l3_entry((vaddr_t)attack_ro_page);
		uint64_t v = *pte;

		v &= ~LOWER_ATTRS(AP_RO);
		DMSG("A17: kernel .rodata PTE 0x%lx -> 0x%lx (clear AP_RO)",
		     *pte, v);
		dsb(); *pte = v; dsb(); tlbi_all(); isb();

		/* Baseline observation: attempt to overwrite the .rodata
		 * page. Pre-flip AP_RO=1 blocks EL1 writes; post-flip on
		 * baseline AP_RO=0 → write lands. Initial byte is 1 (from
		 * the initialiser), so 0xAA is unambiguously our value.
		 * Unreachable on defended (CODE-WRITE panic at the store
		 * above). */
		volatile uint8_t *ro = (volatile uint8_t *)attack_ro_page;
		*ro = 0xAA;
		dsb();
		IMSG("A17 baseline: wrote 0xAA to .rodata, readback = 0x%x %s",
		     *ro,
		     (*ro == 0xAA) ? "-- .rodata IS NOW WRITABLE"
				   : "-- unexpected");
		break;
	}
	/* ────────── P4: access-axis PTE policy (A18–A22) ────────── */

	case 20: { /* A18: EL0-ACCESS — AP_EL0 + nG=1 on secure kernel page.
		    * Core target required: escalation is "TA reaches kernel
		    * memory". Target = attack_rw_page (tee_ram_rw). */

		/* Pre-flip canary: EL1 stores a distinctive byte into the
		 * kernel page. Post-flip LDTRB (unprivileged load from EL1)
		 * reads it back — proves EL0 permissions now grant access. */
		attack_rw_page[0] = 0x42;
		dsb();

		uint64_t *pte = attack_find_l3_entry((vaddr_t)attack_rw_page);
		uint64_t v = *pte;

		v |= LOWER_ATTRS(AP_UNPRIV);
		v |= LOWER_ATTRS(NON_GLOBAL);
		DMSG("A18: kernel L3 PTE 0x%lx -> 0x%lx (EL0-access, nG)",
		     *pte, v);
		dsb(); *pte = v; dsb(); tlbi_all(); isb();

		/* Baseline observation: LDTRB performs the load with EL0
		 * permissions from within EL1. Pre-flip would fault (kernel
		 * page, no EL0 access); post-flip on baseline it reads the
		 * canary. Unreachable on defended (EL0-ACCESS panic above). */
		uint8_t rb;
		asm volatile("ldtrb %w0, [%1]"
			     : "=r"(rb) : "r"(&attack_rw_page[0]));
		IMSG("A18 baseline: ldtrb (as EL0) read 0x%x from kernel VA %s",
		     rb,
		     (rb == 0x42) ? "-- EL0 GAINED ACCESS to secure memory"
				  : "-- unexpected value");
		break;
	}
	case 21: { /* A19: GLOBAL — AP_EL0 + nG=0 on a TA page.
		    * EL0-accessible global leaf: check gates on
		    * AP_EL0 && !NG, independent of PA. TA target works. */
		if (!target_va) { EMSG("A19 requires TA VA"); return; }
		uint64_t *pte = attack_find_l3_entry((vaddr_t)target_va);
		uint64_t v = *pte;

		v |= LOWER_ATTRS(AP_UNPRIV);
		v &= ~LOWER_ATTRS(NON_GLOBAL);
		DMSG("A19: TA L3 PTE 0x%lx -> 0x%lx (AP_EL0, nG=0)",
		     *pte, v);
		dsb(); *pte = v; dsb(); tlbi_all(); isb();
		break;
	}
	case 22: { /* A20: NS=1 on secure TA page + landing probe.
		    * Defended: PTE store traps → NS VIOLATION panic.
		    * Baseline: probe runs — writes canary through NS=1
		    * mapping, reads back via NS=0 view to determine whether
		    * secure memory received the write or the platform (SPMC
		    * NS-IPA S2 walk) blocked it. See defense_a22_platform_masked
		    * memory for the Hafnium NSA/NSW=10b interaction. */
		if (!target_va) { EMSG("A20 requires TA VA"); return; }
		uint64_t *pte = attack_find_l3_entry((vaddr_t)target_va);
		uint64_t v = *pte;
		uint64_t orig_pte = v;

		v |= LOWER_ATTRS(NS);
		DMSG("A20: TA L3 PTE 0x%lx -> 0x%lx (NS=1)", *pte, v);
		dsb(); *pte = v; dsb(); tlbi_all(); isb();
		DMSG("A20: PTE now 0x%lx", *pte);

		/* Landing probe (executes only on baseline). */
		volatile uint32_t *probe = (volatile uint32_t *)target_va;
		uint32_t rb_ns, rb_s;

		*probe = 0xC0DEF00DU;
		dsb();
		rb_ns = *probe;
		IMSG("A20 probe (NS=1 view): wrote 0xC0DEF00D, read 0x%x",
		     rb_ns);

		dsb();
		*pte = orig_pte;
		dsb(); tlbi_all(); isb();
		asm volatile("dc civac, %0" : : "r"(probe) : "memory");
		dsb(); isb();

		rb_s = *probe;
		IMSG("A20 probe (NS=0 view): read 0x%x — %s",
		     rb_s,
		     (rb_s == 0xC0DEF00DU) ? "HARM LANDED in secure memory"
					   : "harm NOT in secure memory");
		break;
	}
	case 23: { /* A21: MEM-TYPE — Device attr on secure TA page.
		    * Any secure PA with AttrIdx ∉ {1,3} panics. TA target
		    * (ta_ram is secure) exercises the check. */
		if (!target_va) { EMSG("A21 requires TA VA"); return; }
		uint64_t *pte = attack_find_l3_entry((vaddr_t)target_va);
		uint64_t v = *pte;

		v &= ~LOWER_ATTRS(ATTR_INDEX_MASK);
		v |= LOWER_ATTRS(ATTR_DEVICE_nGnRnE_INDEX);
		DMSG("A21: TA L3 PTE 0x%lx -> 0x%lx (Device attr)",
		     *pte, v);
		dsb(); *pte = v; dsb(); tlbi_all(); isb();
		break;
	}
	case 24: { /* A22: SHAREABILITY — clear ISH on secure TA page.
		    * Any secure PA with SH != 0b11 panics. */
		if (!target_va) { EMSG("A22 requires TA VA"); return; }
		uint64_t *pte = attack_find_l3_entry((vaddr_t)target_va);
		uint64_t v = *pte;

		v &= ~LOWER_ATTRS(ISH);
		DMSG("A22: TA L3 PTE 0x%lx -> 0x%lx (SH cleared)", *pte, v);
		dsb(); *pte = v; dsb(); tlbi_all(); isb();
		break;
	}

	/* ────────── Fast-path abuse (A23, A24) ────────── */

	case 25: { /* A23: memset with forged LR — blr into memset from a
		    * non-standard call site. Volatile function pointer forces
		    * blr instead of bl memset; LR-4 is not a bl-memset
		    * encoding, so validate_bl_memset panics. Companion to
		    * A24 which exercises the x1 gate specifically. */
		if (!target_va) { EMSG("A23 requires TA VA"); return; }
		uint64_t *l2 = attack_get_l2((vaddr_t)target_va);
		void *(*volatile ms)(void *, int, size_t) = memset;

		DMSG("A23: blr into memset targeting thread L2 %p", l2);
		(void)ms(l2, 0xAB, 8);
		dsb();
		break;
	}
	case 26: { /* A24: memset with non-zero fill via a real bl memset.
		    * Volatile size defeats GCC's inline substitution so a real
		    * bl memset is emitted → LR check passes. x1=0xAB fails the
		    * fast-path's x1 gate; fall-through to SAS check panics on
		    * SAS=0 with x1=0xAB visible in the panic string. */
		if (!target_va) { EMSG("A24 requires TA VA"); return; }
		uint64_t *l2 = attack_get_l2((vaddr_t)target_va);
		volatile size_t sz = 8;

		DMSG("A24: bl memset(l2=%p, 0xAB, sz=8)", l2);
		memset(l2, 0xAB, sz);
		dsb();
		break;
	}

	default:
		EMSG("pgtable_attack: unknown A# / cmd_id %lu", attack_type);
		return;
	}
}
/* ── VMI PAGE-TABLE ATTACK TEST END ──────────────────────────── */

#endif /*ARM64*/

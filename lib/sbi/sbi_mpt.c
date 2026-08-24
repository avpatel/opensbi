/* SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * Authors: Rahul Pathak <rahul.pathak@oss.qualcomm.com>
 */

#include <sbi/sbi_hart_mpt.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_string.h>
#include <sbi/sbi_console.h>

#if __riscv_xlen == 64

/*
 * Smmpt43  —  43-bit supervisor physical address (SPA)
 * +---------+---------+---------+---------+--------+
 * |  pn[2]  |  pn[1]  |  pn[0]  |    pi   | offset |
 * | [42:34] | [33:25] | [24:16] | [15:12] | [11:0] |
 * |    9b   |    9b   |    9b   |    4b   |  12b   |
 * +---------+---------+---------+---------+--------+
 *
 * Smmpt52  —  52-bit supervisor physical address (SPA)
 * +---------+---------+---------+---------+---------+--------+
 * |  pn[3]  |  pn[2]  |  pn[1]  |  pn[0]  |    pi   | offset |
 * | [51:43] | [42:34] | [33:25] | [24:16] | [15:12] | [11:0] |
 * |    9b   |    9b   |    9b   |    9b   |    4b   |  12b   |
 * +---------+---------+---------+---------+---------+--------+
 *
 * Smmpt64  —  64-bit supervisor physical address (SPA)
 * +---------+---------+---------+---------+---------+---------+--------+
 * |  pn[4]  |  pn[3]  |  pn[2]  |  pn[1]  |  pn[0]  |    pi   | offset |
 * | [63:52] | [51:43] | [42:34] | [33:25] | [24:16] | [15:12] | [11:0] |
 * |   12b   |    9b   |    9b   |    9b   |    9b   |    4b   |  12b   |
 * +---------+---------+---------+---------+---------+---------+--------+
 *
 */

/* Innter entries are MPTEs not in root MPT page */
#define RV64_INNER_ENTRIES		512U
#define RV64_MPTE_SIZE			8UL
#define RV64_TABLE_SIZE			(RV64_INNER_ENTRIES * RV64_MPTE_SIZE)

/* Smmpt64 root: 4096 entries × 8B = 32KB, 32KB aligned */
#define RV64_ROOT_SMMPT64_ENTRIES	4096U
#define RV64_ROOT_SMMPT64_SIZE		(RV64_ROOT_SMMPT64_ENTRIES * RV64_MPTE_SIZE)

#define RV64_NUMPGINRANGE		4U
#define RV64_PAGES_PER_MPTE		(1U << RV64_NUMPGINRANGE)
#define RV64_PAGES_PER_MPTE_MASK	(RV64_PAGES_PER_MPTE - 1U)
#define RV64_INNER_PN_BITS		9U

/* Bytes covered by one complete leaf table = pages_per_leaf × PAGE_SIZE */
#define RV64_LEAF_RANGE			(RV64_PAGES_PER_MPTE * SBI_MPT_PAGE_SIZE)

/* Shift required to get tuple from sub MPT index (non-root inner table)*/
#define RV64_SPLIT_TUPLE_SHIFT		(RV64_INNER_PN_BITS - RV64_NUMPGINRANGE)

/*
 * NAPOT G=4 for Smmpt43, Smmpt52 and Smmpt64
 *
 * G=4 means 2^(G+1)=32 contiguous leaf mpte which form a NAPOT group.
 * Each RV64 leaf MPTE covers 64KiB so one NAPOT group covers 2MiB.
 *
 * All other G values apart from 4 are reserved.
 */
#define RV64_NAPOT_G			4U
#define RV64_NAPOT_G_SHIFT		12U
#define RV64_NAPOT_MPTE_COUNT		(1U << (RV64_NAPOT_G + 1))
#define RV64_NAPOT_SIZE			((unsigned long)RV64_NAPOT_MPTE_COUNT * RV64_LEAF_RANGE)
#define RV64_NAPOT_PN0_ALIGN		RV64_NAPOT_MPTE_COUNT

/*
 * Swith to disable NAPOT compaction of MPTE entries while writing.
 */
#define RV64_NAPOT_DISABLE	0

/*
 * Supervisor Physucal address fields extraction
 */

/*
 * rv64_table_idx() — extract pn[level] from a SPA.
 */
static inline u32 rv64_table_idx(unsigned long pa, u32 level,
			  const struct sbi_mpt_mode *sch)
{
	u32 shift = 16 + level * 9;
	u32 mask = (sch->mode_val == SBI_MMPT_MODE_SMMPT64 && level == 4) ? 0xFFF : 0x1FF;

	return ((pa >> shift) & mask);
}

/*
 * rv64_tuple_idx_shift() — XWR tuple index shift at level i.
 */
static inline u32 rv64_tuple_idx_shift(u32 level)
{
	return (SBI_MPT_PAGE_SHIFT + level * RV64_INNER_PN_BITS);
}

static inline u32 rv64_tuple_idx(unsigned long pa, u32 level)
{
	return ((pa >> rv64_tuple_idx_shift(level)) & RV64_PAGES_PER_MPTE_MASK);
}

/*
 * rv64_mpte_range() — bytes covered by one MPTE at level L.
 */
static inline unsigned long rv64_mpte_range(u32 level)
{
	return RV64_LEAF_RANGE << (level * RV64_INNER_PN_BITS);
}

/*
 * RV64 MPTE Read/Write functions
 */

static inline u64 rv64_read_mpte(unsigned long pa)
{
	return *(volatile u64 *)pa;
}

static inline void rv64_write_mpte(unsigned long pa, u64 v)
{
	*(volatile u64 *)pa = v;
}

static inline unsigned long rv64_mpte_pa(unsigned long table_pa, u32 idx)
{
	return (table_pa + idx * RV64_MPTE_SIZE);
}

static inline unsigned long rv64_next_table_pa(u64 mpte)
{
	return (unsigned long)((u64)(mpte >> SBI_MPTE_PPN_SHIFT) << SBI_MPT_PAGE_SHIFT);
}

/*
 * Generate Napot Leaf MPTE
 *
 * Constructs one MPTE value used for all
 * RV64_NAPOT_MPTE_COUNT entries in a NAPOT group.
 *
 * All MPTEs in the group are identical
 */
static inline u64 rv64_napot_leaf_mpte(u8 xwr)
{
	u64 mpte = SBI_MPTE_V | SBI_MPTE_L | SBI_MPTE_N;

	mpte = sbi_mpte_leaf_set_xwr(mpte, 0, xwr);
	mpte |= ((u64)RV64_NAPOT_G << RV64_NAPOT_G_SHIFT);

	return mpte;
}

/*
 * rv64_napot_leaf_xwr() — recover the uniform XWR from a NAPOT leaf.
 */
static inline u8 rv64_napot_leaf_xwr(u64 mpte)
{
	return ((mpte >> SBI_MPTE_XWR_BASE) & SBI_MPTE_XWR_MASK);
}

/*
 * rv64_napot_demote() — Convert a Napot MPTE group into RV64_NAPOT_MPTE_COUNT
 * normal(Non-Napot) MPTE carrying the SAME permission.
 *
 * leaf_mpte_pa is any leaf mpte pa from the Napot group
 */
static void rv64_napot_demote(struct sbi_mpt_domain *dom,
			      unsigned long leaf_mpte_pa, unsigned long pa)
{
	u8 xwr;
	u32 pn0, pg, i;
	u64 leaf;
	unsigned long grp_base;

	xwr = rv64_napot_leaf_xwr(rv64_read_mpte(leaf_mpte_pa));
	pn0 = rv64_table_idx(pa, 0, dom->mode);
	grp_base = leaf_mpte_pa - (pn0 & (RV64_NAPOT_MPTE_COUNT - 1)) * RV64_MPTE_SIZE;
	leaf = SBI_MPTE_V | SBI_MPTE_L;

	for (pg = 0; pg < RV64_PAGES_PER_MPTE; pg++)
		leaf = sbi_mpte_leaf_set_xwr(leaf, pg, xwr);

	for (i = 0; i < RV64_NAPOT_MPTE_COUNT; i++)
		rv64_write_mpte(grp_base + i * RV64_MPTE_SIZE, leaf);
}

/*
 * MPT table best-level selection
 *
 * Returns the highest level at which the range [pa, pa+size] can be
 * covered by a single leaf MPTE. Level 0 is always valid because thats
 * the last resort.
 */
static u32 rv64_best_level(unsigned long pa, unsigned long size,
			   u32 top_level)
{
	u32 lvl;
	unsigned long range;

	for (lvl = top_level; lvl >= 1; lvl--) {
		range = rv64_mpte_range(lvl);

		if (size >= range && (pa & (range - 1)) == 0)
			return lvl;
	}
	return 0;
}

/*
 * Generic N-level walk with lazy table allocation
 */
static unsigned long rv64_split_leaf(unsigned long parent_ep)
{
	u64 parent = rv64_read_mpte(parent_ep);
	unsigned long sub;
	u32 j, pg, sh;
	u8 xwr;
	u64 child;

	sub = sbi_mpt_pool_alloc(RV64_TABLE_SIZE, SBI_MPT_PAGE_SIZE);
	if (!sub)
		return 0;

	for (j = 0; j < RV64_INNER_ENTRIES; j++) {
		sh = sbi_mpte_xwr_shift(j >> RV64_SPLIT_TUPLE_SHIFT);
		xwr = (((unsigned long)parent >> sh) & SBI_MPTE_XWR_MASK);
		child = SBI_MPTE_V | SBI_MPTE_L; /* N=0 uniform leaf */

		for (pg = 0; pg < RV64_PAGES_PER_MPTE; pg++)
			child = sbi_mpte_leaf_set_xwr(child, pg, xwr);

		rv64_write_mpte(sub + j * RV64_MPTE_SIZE, child);
	}

	/* parent leaf -> non-leaf pointer to MPT sub table. */
	rv64_write_mpte(parent_ep, sbi_mpte_nonleaf(sub));

	return sub;
}

/*
 * Walk a MPT table and return MPTE PA and its suitable level
 */
static unsigned long rv64_walk_alloc(struct sbi_mpt_domain *dom,
				     unsigned long pa,
				     unsigned long size,
				     u32 *out_level,
				     u32 top_level)
{
	u32 lvl, idx;
	u64 mpte;
	unsigned long ep, sub, new_pa;
	const struct sbi_mpt_mode *sch = dom->mode;
	unsigned long table_pa = dom->root_pa;
	u32 best_lvl = rv64_best_level(pa, size, top_level);


	for (lvl = top_level; lvl >= 1; lvl--) {
		idx = rv64_table_idx(pa, lvl, sch);
		ep = rv64_mpte_pa(table_pa, idx);
		mpte = rv64_read_mpte(ep);

		if (mpte & SBI_MPTE_L) {
			if (lvl > best_lvl) {
				sub = rv64_split_leaf(ep);
				if (!sub)
					return 0;

				table_pa = sub;
				continue;
			}

			*out_level = lvl;
			return ep;
		}

		if (!(mpte & SBI_MPTE_V)) {
			if (lvl <= best_lvl) {
				*out_level = lvl;
				return ep;
			}
			/* Allocate inner table (always 4KiB, all levels, all modes) */
			new_pa = sbi_mpt_pool_alloc(RV64_TABLE_SIZE,
						    SBI_MPT_PAGE_SIZE);
			if (!new_pa)
				return 0;

			rv64_write_mpte(ep, (u64)sbi_mpte_nonleaf(new_pa));
			table_pa = new_pa;
		}
		else {
			table_pa = rv64_next_table_pa(mpte);
		}
	}

	*out_level = 0;

	return rv64_mpte_pa(table_pa, rv64_table_idx(pa, 0, sch));
}

/*
 * Generic map_range — shared by Smmpt43, Smmpt52, Smmpt64
 *
 * Maps a range [pa, pa+size] in the MPT table
 */
static int __rv64_map_range(struct sbi_mpt_domain *dom,
			    unsigned long pa, unsigned long size,
			    u8 xwr, u32 top_level)
{
	unsigned long cur = pa;
	unsigned long end = pa + size;
	unsigned long leaf_mpte_pa, mpte_range, mpte_base, mpte_end, batch_end;
	u32 level, pn0, i, pg_first, pg_last, pg;
	u64 napot, mpte;

	while (cur < end) {
		/*
		 * Check if the region qualifies for NAPOT range and the
		 * region is contained
		 */
		if ((cur & (RV64_NAPOT_SIZE - 1)) == 0 &&
				cur + RV64_NAPOT_SIZE > cur &&
				cur + RV64_NAPOT_SIZE <= end) {
			leaf_mpte_pa = rv64_walk_alloc(dom, cur, end - cur, &level, top_level);
			if (!leaf_mpte_pa)
				return SBI_ENOMEM;

			pn0 = rv64_table_idx(cur, 0, dom->mode);

			if (!RV64_NAPOT_DISABLE && level == 0 && (pn0 & (RV64_NAPOT_PN0_ALIGN - 1)) == 0) {
				napot = rv64_napot_leaf_mpte(xwr);

				for (i = 0; i < RV64_NAPOT_MPTE_COUNT; i++)
					rv64_write_mpte(leaf_mpte_pa + i * RV64_MPTE_SIZE, napot);

				cur += RV64_NAPOT_SIZE;
				continue;
			}
		}
		else {
			leaf_mpte_pa = rv64_walk_alloc(dom, cur, end - cur, &level, top_level);
			if (!leaf_mpte_pa)
				return SBI_ENOMEM;
		}

		/* Normal path */
		mpte_range = rv64_mpte_range(level);
		pg_first = rv64_tuple_idx(cur, level);
		mpte_base = cur & ~(mpte_range - 1);

		mpte_end = mpte_base + mpte_range;
		if (mpte_end < mpte_base)
			mpte_end = ~0UL;

		batch_end = (end < mpte_end) ? end : mpte_end;
		pg_last = rv64_tuple_idx(batch_end - SBI_MPT_PAGE_SIZE, level);

		/*
		* If this level-0 MPTE is a Napot member, expand the
		* whole naturally-aligned group to plain leaves and write same
		* permissions xwr
		*/
		mpte = rv64_read_mpte(leaf_mpte_pa);
		if (level == 0 && (mpte & (u64)SBI_MPTE_N)) {
			rv64_napot_demote(dom, leaf_mpte_pa, cur);
			sbi_mpt_fence_sdid(dom->sdid);
			mpte = rv64_read_mpte(leaf_mpte_pa);
		}

		if (!(mpte & (u64)(SBI_MPTE_V | SBI_MPTE_L)))
			mpte = (u64)(SBI_MPTE_V | SBI_MPTE_L);

		for (pg = pg_first; pg <= pg_last; pg++)
			mpte = sbi_mpte_leaf_set_xwr(mpte, pg, xwr);

		rv64_write_mpte(leaf_mpte_pa, mpte);

		cur = batch_end;
	}

	return 0;
}

/*
 * Per-mode mmpt encoding
 */

static unsigned long smmpt43_encode_mmpt(unsigned long ppn, u32 sdid)
{
	return sbi_mmpt_encode(SBI_MMPT_MODE_SMMPT43, sdid, ppn);
}

static unsigned long smmpt52_encode_mmpt(unsigned long ppn, u32 sdid)
{
	return sbi_mmpt_encode(SBI_MMPT_MODE_SMMPT52, sdid, ppn);
}

static unsigned long smmpt64_encode_mmpt(unsigned long ppn, u32 sdid)
{
	return sbi_mmpt_encode(SBI_MMPT_MODE_SMMPT64, sdid, ppn);
}

static unsigned long smmpt43_root_table_size(void)
{
	return RV64_TABLE_SIZE;
}

static unsigned long smmpt43_root_table_align(void)
{
	return SBI_MPT_PAGE_SIZE;
}

static bool smmpt43_pa_in_range(unsigned long pa, unsigned long size)
{
	unsigned long pa_max = 1UL << 43;

	return (pa < pa_max && size <= pa_max - pa);
}

static unsigned long smmpt52_root_table_size(void)
{
	return RV64_TABLE_SIZE;
}

static unsigned long smmpt52_root_table_align(void)
{
	return SBI_MPT_PAGE_SIZE;
}

static bool smmpt52_pa_in_range(unsigned long pa, unsigned long size)
{
	unsigned long pa_max = 1UL << 52;

	return (pa < pa_max && size <= pa_max - pa);
}

static unsigned long smmpt64_root_table_size(void)
{
	return RV64_ROOT_SMMPT64_SIZE;
}

static unsigned long smmpt64_root_table_align(void)
{
	return RV64_ROOT_SMMPT64_SIZE;
}

static bool smmpt64_pa_in_range(unsigned long pa,
				unsigned long size)
{
	/*
	 * Smmpt64 covers the full 64-bit PA space.
	 */
	return true;
}

/*
 * Per-mode map_range wrappers
 *
 * Smmpt43:
 * Root level 2, 9-bit,  512 entries,  4KiB root MPT size
 *
 * Smmpt52:
 * Root level 3, 9-bit,  512 entries,  4KiB root MPT size
 *
 * Smmpt64:
 * Root level 4, 12-bit, 4096 entries, 32KiB root MPT size
 */

static int smmpt43_map_range(struct sbi_mpt_domain *dom,
			     unsigned long pa, unsigned long size, u8 xwr)
{
	return __rv64_map_range(dom, pa, size, xwr, 2);
}

static int smmpt52_map_range(struct sbi_mpt_domain *dom,
			     unsigned long pa, unsigned long size, u8 xwr)
{
	return __rv64_map_range(dom, pa, size, xwr, 3);
}

static int smmpt64_map_range(struct sbi_mpt_domain *dom,
			     unsigned long pa, unsigned long size, u8 xwr)
{
	return __rv64_map_range(dom, pa, size, xwr, 4);
}

/*
 * __rv64_map_full_range() — Write every MPTE at Root MPT
 *
 * Writes xwr into every root table MPTE directly as a leaf
 * superpage covering the mode entire addressable PA range.
 * No intermidiate MPT tables allocated just all leaf MPTEs at
 * root level.
 */

static int __rv64_map_full_range(struct sbi_mpt_domain *dom, u8 xwr,
				 unsigned long root_entries)
{
	u32 pg;
	unsigned long i;
	u64 mpte = SBI_MPTE_V | SBI_MPTE_L;

	for (pg = 0; pg < RV64_PAGES_PER_MPTE; pg++)
		mpte = sbi_mpte_leaf_set_xwr(mpte, pg, xwr);

	for (i = 0; i < root_entries; i++)
		rv64_write_mpte(rv64_mpte_pa(dom->root_pa, i), mpte);

	return 0;
}

static int smmpt43_map_full_range(struct sbi_mpt_domain *dom, u8 xwr)
{
	return __rv64_map_full_range(dom, xwr, RV64_INNER_ENTRIES);
}

static int smmpt52_map_full_range(struct sbi_mpt_domain *dom, u8 xwr)
{
	return __rv64_map_full_range(dom, xwr, RV64_INNER_ENTRIES);
}

static int smmpt64_map_full_range(struct sbi_mpt_domain *dom, u8 xwr)
{
	return __rv64_map_full_range(dom, xwr, RV64_ROOT_SMMPT64_ENTRIES);
}

/*
 * Get the access permissions(xwr) of a PA
 */
static u8 __rv64_get_xwr(struct sbi_mpt_domain *dom, unsigned long pa,
			 u32 top_level)
{
	const struct sbi_mpt_mode *sch = dom->mode;
	unsigned long table_pa = dom->root_pa;
	u32 level, idx, pi;
	u64 mpte;

	for (level = top_level; ; level--) {
		idx = rv64_table_idx(pa, level, sch);
		mpte = rv64_read_mpte(rv64_mpte_pa(table_pa, idx));

		if (!(mpte & SBI_MPTE_V))
			return SBI_MPT_PERM_NONE;

		if (!(mpte & SBI_MPTE_L) && (mpte & SBI_MPTE_N))
			return SBI_MPT_PERM_NONE;

		if (mpte & SBI_MPTE_L) {
			if (mpte & SBI_MPTE_N)
				return rv64_napot_leaf_xwr(mpte);

			pi = rv64_tuple_idx(pa, level);

			return (mpte >> sbi_mpte_xwr_shift(pi)) & SBI_MPTE_XWR_MASK;
		}

		if (level == 0)
			return SBI_MPT_PERM_NONE;

		table_pa = rv64_next_table_pa(mpte);
	}

	return SBI_MPT_PERM_NONE;
}

static u8 smmpt43_get_xwr(struct sbi_mpt_domain *dom, unsigned long pa)
{
	return __rv64_get_xwr(dom, pa, 2);
}

static u8 smmpt52_get_xwr(struct sbi_mpt_domain *dom, unsigned long pa)
{
	return __rv64_get_xwr(dom, pa, 3);
}

static u8 smmpt64_get_xwr(struct sbi_mpt_domain *dom, unsigned long pa)
{
	return __rv64_get_xwr(dom, pa, 4);
}

struct sbi_mpt_mode smmpt43_mode = {
	.name			= "Smmpt43",
	.mode_val		= SBI_MMPT_MODE_SMMPT43,
	.encode_mmpt		= smmpt43_encode_mmpt,
	.map_range		= smmpt43_map_range,
	.root_table_size	= smmpt43_root_table_size,
	.root_table_align	= smmpt43_root_table_align,
	.pa_in_range		= smmpt43_pa_in_range,
	.map_full_range		= smmpt43_map_full_range,
	.get_xwr		= smmpt43_get_xwr,
};

struct sbi_mpt_mode smmpt52_mode = {
	.name			= "Smmpt52",
	.mode_val		= SBI_MMPT_MODE_SMMPT52,
	.encode_mmpt		= smmpt52_encode_mmpt,
	.map_range		= smmpt52_map_range,
	.root_table_size	= smmpt52_root_table_size,
	.root_table_align	= smmpt52_root_table_align,
	.pa_in_range		= smmpt52_pa_in_range,
	.map_full_range		= smmpt52_map_full_range,
	.get_xwr		= smmpt52_get_xwr,
};

struct sbi_mpt_mode smmpt64_mode = {
	.name			= "Smmpt64",
	.mode_val		= SBI_MMPT_MODE_SMMPT64,
	.encode_mmpt		= smmpt64_encode_mmpt,
	.map_range		= smmpt64_map_range,
	.root_table_size	= smmpt64_root_table_size,
	.root_table_align	= smmpt64_root_table_align,
	.pa_in_range		= smmpt64_pa_in_range,
	.map_full_range		= smmpt64_map_full_range,
	.get_xwr		= smmpt64_get_xwr,
};

#else /* __riscv_xlen == 64 */

/*
 * Smmpt34 (RV32)  —  34-bit supervisor physical address (SPA)
 *
 * +---------+---------+---------+--------+
 * |  pn[1]  |  pn[0]  |    pi   | offset |
 * | [33:25] | [24:15] | [14:12] | [11:0] |
 * |    9b   |   10b   |    3b   |  12b   |
 * +---------+---------+---------+--------+
 *
 */

#define RV32_MPTE_SIZE			4UL
#define RV32_ROOT_ENTRIES		512U
#define RV32_LEAF_ENTRIES		1024U
#define RV32_ROOT_SIZE			(RV32_ROOT_ENTRIES * RV32_MPTE_SIZE)
#define RV32_LEAF_SIZE			(RV32_LEAF_ENTRIES * RV32_MPTE_SIZE)

/* pn field widths */
#define RV32_PN1_BITS			9U
#define RV32_PN0_BITS			10U

#define RV32_NUMPGINRANGE		3U
#define RV32_PAGES_PER_MPTE		(1U << RV32_NUMPGINRANGE)
#define RV32_PAGES_PER_MPTE_MASK	(RV32_PAGES_PER_MPTE - 1)

/* Range covered by one MPTE at each level */
#define RV32_LEAF_RANGE			(RV32_PAGES_PER_MPTE * SBI_MPT_PAGE_SIZE)
#define RV32_ROOT_RANGE			(RV32_LEAF_RANGE << RV32_PN0_BITS)

/* NAPOT constants (G=6) */
#define RV32_NAPOT_G			6U
#define RV32_NAPOT_G_SHIFT		12U
#define RV32_NAPOT_COUNT		(1U << (RV32_NAPOT_G + 1))
#define RV32_NAPOT_SIZE			(RV32_NAPOT_COUNT * RV32_LEAF_RANGE)
#define RV32_NAPOT_PN0_ALIGN		RV32_NAPOT_COUNT

/*
 * Swith to disable NAPOT compaction of MPTE entries while writing.
 */
#define RV32_NAPOT_DISABLE	0

/*
 * rv32_pn1_table_idx() — extract pn[1] from a SPA.
 */
static inline u32 rv32_pn1_table_idx(unsigned long pa)
{
	return ((pa >> 25) & 0x1FFU);
}

/*
 * rv32_pn0_table_idx() — extract pn[0] from a SPA.
 */
static inline u32 rv32_pn0_table_idx(unsigned long pa)
{
	return ((pa >> 15) & 0x3FFU);
}

static inline u32 rv32_tuple_idx_shift(u32 level)
{
	return (level == 0) ? SBI_MPT_PAGE_SHIFT : SBI_MPT_PAGE_SHIFT + RV32_PN0_BITS;
}

static inline u32 rv32_tuple_idx(unsigned long pa, u32 level)
{
	return ((pa >> rv32_tuple_idx_shift(level)) & RV32_PAGES_PER_MPTE_MASK);
}

/*
 * Bytes covered by one MPTE at the given level
 */
static inline unsigned long rv32_mpte_range(u32 level)
{
	return (level == 0) ? RV32_LEAF_RANGE : RV32_ROOT_RANGE;
}

/*
 * RV32 MPTE Read/Write functions
 */
static inline u32 rv32_read_mpte(unsigned long pa)
{
	return *(volatile u32 *)pa;
}

static inline void rv32_write_mpte(unsigned long pa, u32 v)
{
	*(volatile u32 *)pa = v;
}

static inline unsigned long rv32_mpte_pa(unsigned long table_pa, u32 idx)
{
	return (table_pa + idx * RV32_MPTE_SIZE);
}

static inline unsigned long rv32_next_table_pa(u32 mpte)
{
	return ((mpte >> SBI_MPTE_PPN_SHIFT) << SBI_MPT_PAGE_SHIFT);
}

/*
 * Generate Napot Leaf MPTE
 *
 * Constructs one MPTE value used for all RV32_NAPOT_COUNT entries
 * in a NAPOT group.
 *
 * All MPTEs in the group are identical
 */
static inline u32 rv32_napot_leaf(u8 xwr)
{
	u32 mpte = SBI_MPTE_V | SBI_MPTE_L | SBI_MPTE_N;

	mpte = sbi_mpte_leaf_set_xwr(mpte, 0, xwr);
	mpte |= ((u32)RV32_NAPOT_G << RV32_NAPOT_G_SHIFT);

	return mpte;
}

/*
 * Get the xwr from a Napot leaf.
 */
static inline u8 rv32_napot_xwr(u32 mpte)
{
	return ((mpte >> SBI_MPTE_XWR_BASE) & SBI_MPTE_XWR_MASK);
}

/*
 * rv32_napot_demote() — Convert a Napot MPTE group into RV64_NAPOT_MPTE_COUNT
 * normal(Non-Napot) MPTE carrying the SAME permission.
 *
 * leaf_mpte_pa is any of the group MPTE PA
 */
static void rv32_napot_demote(struct sbi_mpt_domain *dom,
			      unsigned long leaf_mpte_pa, unsigned long pa)
{
	u32 pg, leaf, i;
	unsigned long grp_base;
	u8 xwr = rv32_napot_xwr(rv32_read_mpte(leaf_mpte_pa));
	u32 pn0 = rv32_pn0_table_idx(pa);

	grp_base = leaf_mpte_pa - (pn0 & (RV32_NAPOT_COUNT - 1)) * RV32_MPTE_SIZE;

	leaf = (SBI_MPTE_V | SBI_MPTE_L); /* N=0 */

	for (pg = 0; pg < RV32_PAGES_PER_MPTE; pg++)
		leaf = sbi_mpte_leaf_set_xwr(leaf, pg, xwr);

	for (i = 0; i < RV32_NAPOT_COUNT; i++)
		rv32_write_mpte(grp_base + i * RV32_MPTE_SIZE, leaf);
}

static inline u32 rv32_best_level(unsigned long pa, unsigned long size)
{
	if (size >= RV32_ROOT_RANGE && (pa & (RV32_ROOT_RANGE - 1)) == 0)
		return 1;

	return 0;
}

/*
 * MPT Walk for RV32 Smmpt43
 *
 * Returns the PA of the MPTE covering pa
 */
static unsigned long rv32_walk_alloc(struct sbi_mpt_domain *dom,
				     unsigned long pa,
				     unsigned long size,
				     u32 *out_level)
{
	unsigned long root_ep = rv32_mpte_pa(dom->root_pa, rv32_pn1_table_idx(pa));
	u32 root_mpte = rv32_read_mpte(root_ep);
	u32 best = rv32_best_level(pa, size);
	unsigned long leaf_pa;

	if (root_mpte & (u32)SBI_MPTE_L) {
		/* Existing root-level leaf */
		*out_level = 1;
		return root_ep;
	}

	if (!(root_mpte & (u32)SBI_MPTE_V) && best == 1) {
		/* new entry, range fits at root level so skip leaf allocation */
		*out_level = 1;
		return root_ep;
	}

	if (!(root_mpte & (u32)SBI_MPTE_V)) {
		/* Allocate leaf table from global heap */
		leaf_pa = sbi_mpt_pool_alloc(RV32_LEAF_SIZE,
					     SBI_MPT_PAGE_SIZE);
		if (!leaf_pa)
			return 0;

		rv32_write_mpte(root_ep, sbi_mpte_nonleaf(leaf_pa));
		*out_level = 0;

		return rv32_mpte_pa(leaf_pa, rv32_pn0_table_idx(pa));
	}

	/* Valid non-leaf: follow PPN to existing leaf table */
	*out_level = 0;

	return rv32_mpte_pa(rv32_next_table_pa(rv32_read_mpte(root_ep)), rv32_pn0_table_idx(pa));
}

/*
 * smmpt34_map_range(): Maps a range [pa, pa+size] in the MPT table
 */

static int smmpt34_map_range(struct sbi_mpt_domain *dom,
			     unsigned long pa, unsigned long size, u8 xwr)
{
	unsigned long cur = pa;
	unsigned long end = pa + size;
	unsigned long leaf_mpte_pa;
	unsigned long mpte_range, mpte_base, mpte_end, batch_end;
	u32 level, pg_first, pg_last, pg, i;
	u32 mpte, napot;

	while (cur < end) {
		/*
		 * Check if the region qualifies for NAPOT range and the
		 * region is contained
		 */
		if ((cur & (RV32_NAPOT_SIZE - 1)) == 0 &&
			cur + RV32_NAPOT_SIZE > cur &&
			cur + RV32_NAPOT_SIZE <= end) {
			leaf_mpte_pa = rv32_walk_alloc(dom, cur, end - cur, &level);
			if (!leaf_mpte_pa)
				return SBI_ENOMEM;

			if (!RV32_NAPOT_DISABLE && level == 0 && (rv32_pn0_table_idx(cur) & (RV32_NAPOT_PN0_ALIGN - 1)) == 0) {
				napot = rv32_napot_leaf(xwr);

				for (i = 0; i < RV32_NAPOT_COUNT; i++)
					rv32_write_mpte(leaf_mpte_pa + i * RV32_MPTE_SIZE, napot);

				cur += RV32_NAPOT_SIZE;

				continue;
			}
		} else {
			leaf_mpte_pa = rv32_walk_alloc(dom, cur, end - cur, &level);
			if (!leaf_mpte_pa)
				return SBI_ENOMEM;
		}

		/* Normal path */
		mpte_range = rv32_mpte_range(level);
		pg_first = rv32_tuple_idx(cur, level);
		mpte_base = cur & ~(mpte_range - 1UL);
		mpte_end = mpte_base + mpte_range;
		batch_end = (end < mpte_end) ? end : mpte_end;
		pg_last = rv32_tuple_idx(batch_end - SBI_MPT_PAGE_SIZE, level);

		/*
		* If this level-0 MPTE is a Napot member, expand the
		* whole naturally-aligned group to plain leaves and write same
		* permissions xwr
		*/
		mpte = rv32_read_mpte(leaf_mpte_pa);
		if (level == 0 && (mpte & (u32)SBI_MPTE_N)) {
			rv32_napot_demote(dom, leaf_mpte_pa, cur);
			sbi_mpt_fence_sdid(dom->sdid);
			mpte = rv32_read_mpte(leaf_mpte_pa);
		}
		if (!(mpte & (u32)(SBI_MPTE_V | SBI_MPTE_L)))
			mpte = SBI_MPTE_V | SBI_MPTE_L;

		for (pg = pg_first; pg <= pg_last; pg++)
			mpte = sbi_mpte_leaf_set_xwr(mpte, pg, xwr);

		rv32_write_mpte(leaf_mpte_pa, mpte);
		cur = batch_end;
	}

	return 0;
}

/*
 * mmpt encoding
 */
static unsigned long smmpt34_encode_mmpt(unsigned long ppn, u32 sdid)
{
	return sbi_mmpt_encode(SBI_MMPT_MODE_SMMPT34, sdid, ppn);
}

static unsigned long smmpt34_root_table_size(void)
{
	return RV32_ROOT_SIZE;
}

static unsigned long smmpt34_root_table_align(void)
{
	return SBI_MPT_PAGE_SIZE;
}

static bool smmpt34_pa_in_range(unsigned long pa, unsigned long size)
{
	/*
	 * Return true since 1 << 34 wil be UB
	 * in RV32 case and anyways variables are limited by RV32
	 * architectural width
	 */

	return true;
}

/*
 * smmpt34_map_full_range() — Write every MPTE at Root MPT
 *
 * Writes xwr into every root table MPTE directly as a leaf
 * superpage covering the mode entire addressable PA range.
 * No intermidiate MPT tables allocated just all leaf MPTEs at
 * root level.
 */

static int smmpt34_map_full_range(struct sbi_mpt_domain *dom, u8 xwr)
{
	u32 mpte = SBI_MPTE_V | SBI_MPTE_L;
	u32 pg, i;

	for (pg = 0; pg < RV32_PAGES_PER_MPTE; pg++)
		mpte = sbi_mpte_leaf_set_xwr(mpte, pg, xwr);

	for (i = 0; i < RV32_ROOT_ENTRIES; i++)
		rv32_write_mpte(rv32_mpte_pa(dom->root_pa, i), mpte);

	return 0;
}

/* Get the access permissions(xwr) of a PA */
static u8 smmpt34_get_xwr(struct sbi_mpt_domain *dom, unsigned long pa)
{
	u32 mpte, pi;
	unsigned long table_pa = dom->root_pa;

	mpte = rv32_read_mpte(rv32_mpte_pa(table_pa, rv32_pn1_table_idx(pa)));

	if (!(mpte & SBI_MPTE_V))
		return SBI_MPT_PERM_NONE;

	if (!(mpte & SBI_MPTE_L) && (mpte & SBI_MPTE_N))
		return SBI_MPT_PERM_NONE;

	if (mpte & SBI_MPTE_L) {
		if (mpte & SBI_MPTE_N)
			return rv32_napot_xwr(mpte);

		pi = rv32_tuple_idx(pa, 1);
		return (mpte >> sbi_mpte_xwr_shift(pi)) & SBI_MPTE_XWR_MASK;
	}

	table_pa = rv32_next_table_pa(mpte);
	mpte = rv32_read_mpte(rv32_mpte_pa(table_pa, rv32_pn0_table_idx(pa)));

	if (!(mpte & SBI_MPTE_V) || !(mpte & SBI_MPTE_L))
		return SBI_MPT_PERM_NONE;

	if (mpte & SBI_MPTE_N)
		return rv32_napot_xwr(mpte);

	pi = rv32_tuple_idx(pa, 0);

	return ((mpte >> sbi_mpte_xwr_shift(pi)) & SBI_MPTE_XWR_MASK);
}

struct sbi_mpt_mode smmpt34_mode = {
	.name = "Smmpt34",
	.mode_val = SBI_MMPT_MODE_SMMPT34,
	.encode_mmpt = smmpt34_encode_mmpt,
	.map_range = smmpt34_map_range,
	.root_table_size = smmpt34_root_table_size,
	.root_table_align = smmpt34_root_table_align,
	.pa_in_range = smmpt34_pa_in_range,
	.map_full_range = smmpt34_map_full_range,
	.get_xwr = smmpt34_get_xwr,
};

#endif /* __riscv_xlen == 32 */

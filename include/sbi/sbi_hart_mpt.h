/* SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * Authors: Rahul Pathak <rahul.pathak@oss.qualcomm.com>
 */

#ifndef __SBI_HART_MPT_H__
#define __SBI_HART_MPT_H__

#include <sbi/sbi_types.h>
#include <sbi/riscv_locks.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_bitmap.h>
#include <sbi/riscv_asm.h>

struct sbi_domain;
struct sbi_mpt_domain;
struct sbi_mpt_ctrl;
struct sbi_domain_memregion;

/* CSR addresses */
#define SBI_CSR_MMPT		0x382
#define SBI_CSR_MSDCFG		0x74E

/* mmpt register field definitions */
#if __riscv_xlen == 64
# define SBI_MMPT_MODE_SHIFT	60UL
# define SBI_MMPT_MODE_MASK	(UL(0xF) << SBI_MMPT_MODE_SHIFT)
# define SBI_MMPT_SDID_SHIFT	52UL
# define SBI_MMPT_SDID_MASK	(UL(0x3F) << SBI_MMPT_SDID_SHIFT)
# define SBI_MMPT_PPN_MASK	UL(0x00000FFFFFFFFFFF)
#else
# define SBI_MMPT_MODE_SHIFT	30UL
# define SBI_MMPT_MODE_MASK	(UL(0x3) << SBI_MMPT_MODE_SHIFT)
# define SBI_MMPT_SDID_SHIFT	22UL
# define SBI_MMPT_SDID_MASK	(UL(0x3F) << SBI_MMPT_SDID_SHIFT)
# define SBI_MMPT_PPN_MASK	UL(0x3FFFFF)
#endif

#define SBI_MMPT_MODE_BARE	0UL
#if __riscv_xlen == 64
# define SBI_MMPT_MODE_SMMPT43	1UL
# define SBI_MMPT_MODE_SMMPT52	2UL
# define SBI_MMPT_MODE_SMMPT64	3UL
#else
# define SBI_MMPT_MODE_SMMPT34	1UL
#endif

/*
 * sbi_mmpt_encode() — assemble the full mmpt CSR value.
 */
static inline unsigned long sbi_mmpt_encode(u32 mode, u32 sdid,
					    unsigned long ppn)
{
	return (((unsigned long)mode << SBI_MMPT_MODE_SHIFT) & SBI_MMPT_MODE_MASK) |
	       (((unsigned long)sdid << SBI_MMPT_SDID_SHIFT) & SBI_MMPT_SDID_MASK) |
	       (ppn & SBI_MMPT_PPN_MASK);
}

/*
 * MPTE bit definitions
 */
#define SBI_MPTE_V		(1UL << 0)
#define SBI_MPTE_L		(1UL << 1)
#define SBI_MPTE_N		(1UL << 2)
#define SBI_MPTE_PPN_SHIFT	10U
#define SBI_MPT_PAGE_SHIFT	12U
#define SBI_MPT_PAGE_SIZE	4096UL
#define SBI_MPT_PAGE_MASK	(SBI_MPT_PAGE_SIZE - 1UL)
#define SBI_MPTE_XWR_BASE	8U
#define SBI_MPTE_XWR_WIDTH	3U
#define SBI_MPTE_XWR_MASK	7UL

static inline u32 sbi_mpte_xwr_shift(u32 n)
{
	return (SBI_MPTE_XWR_BASE + n * SBI_MPTE_XWR_WIDTH);
}

static inline unsigned long sbi_mpte_leaf_set_xwr(unsigned long mpte,
						    u32 n, u8 xwr)
{
	u32 sh = sbi_mpte_xwr_shift(n);

	mpte &= ~(SBI_MPTE_XWR_MASK << sh);
	mpte |= (xwr & SBI_MPTE_XWR_MASK) << sh;
	return mpte;
}

static inline unsigned long sbi_mpte_nonleaf(unsigned long table_pa)
{
	return (((table_pa >> SBI_MPT_PAGE_SHIFT) << SBI_MPTE_PPN_SHIFT)
	       | SBI_MPTE_V);
}

static inline unsigned long sbi_mpte_nonleaf_table_pa(unsigned long mpte)
{
	return ((mpte >> SBI_MPTE_PPN_SHIFT) << SBI_MPT_PAGE_SHIFT);
}

/*
 * XWR permission constants (XWR=000 means no access)
 */
#define SBI_MPT_PERM_NONE	0U
#define SBI_MPT_PERM_R		1U
#define SBI_MPT_PERM_W		2U
#define SBI_MPT_PERM_X		4U
#define SBI_MPT_PERM_RW		(SBI_MPT_PERM_R | SBI_MPT_PERM_W)
#define SBI_MPT_PERM_RX		(SBI_MPT_PERM_R | SBI_MPT_PERM_X)
#define SBI_MPT_PERM_RWX	(SBI_MPT_PERM_R | SBI_MPT_PERM_W | SBI_MPT_PERM_X)

/*
 * SDID constants
 */
#define SBI_MPT_SDIDMAX		6U
#define SBI_MPT_MAX_DOMAINS	(1UL << SBI_MPT_SDIDMAX)
#define SBI_MPT_SDID_INVALID	SBI_MPT_MAX_DOMAINS

/*
 * SBI_MPT_MAX_REGIONS_DOMAIN: Number of regions a domain may have
 * Used for tracking of regions
 */
#define SBI_MPT_MAX_REGIONS_DOMAIN  32U

/*
 * SMSDID mfence and minval raw encodings
 *
 * R-type, opcode=SYSTEM(0x73), funct3=0, rd=x0, rs1=PADDR, rs2=SDID
 * mfence.pa funct7=0x19 -> 0x32000073 where rs1=rs2=x0
 * minval.pa funct7=0x1b -> 0x36000073 where rs1=rs2=x0
 *
 * .insn r opcode, func3, func7, rd, rs1, rs2
 */
#define SBI_MPT_MFENCE_PA(rs1, rs2)	".insn r 0x73, 0, 0x19, x0, " rs1 ", " rs2

#define SBI_MPT_MINVAL_PA(rs1, rs2)	".insn r 0x73, 0, 0x1b, x0, " rs1 ", " rs2

/*
 * Fence and invalidation helpers
 */
static inline void sbi_mpt_fence_all(void)
{
	__asm__ volatile(SBI_MPT_MFENCE_PA("x0", "x0") ::: "memory");
}

static inline void sbi_mpt_fence_sdid(u32 sdid)
{
	unsigned long s = (unsigned long)sdid & (SBI_MPT_MAX_DOMAINS - 1UL);

	__asm__ volatile(SBI_MPT_MFENCE_PA("x0", "%0") :: "r"(s) : "memory");
}

/*
 * rs1=paddr and rs2=x0
 */
static inline void sbi_mpt_mfence_addr(unsigned long paddr)
{
	__asm__ volatile(SBI_MPT_MFENCE_PA("%0", "x0") :: "r"(paddr) : "memory");
}

/*
 * rs1=paddr and rs2=sdid
 */
static inline void sbi_mpt_mfence_pa(unsigned long paddr, u32 sdid)
{
	unsigned long s = (unsigned long)sdid & (SBI_MPT_MAX_DOMAINS - 1UL);

	__asm__ volatile(SBI_MPT_MFENCE_PA("%0", "%1")
			 :: "r"(paddr), "r"(s) : "memory");
}

/*
 * rs1=0 and rs2=0
 */
static inline void sbi_mpt_minval_all(void)
{
	/* Prior stores globally visible before invalidation */
	__asm__ volatile("sfence.w.inval" ::: "memory");
	__asm__ volatile(SBI_MPT_MINVAL_PA("x0", "x0") ::: "memory");
	/* Invalidation completes before subsequent implicit accesses */
	__asm__ volatile("sfence.inval.ir" ::: "memory");
}

/*
 * rs1=paddr and rs2=sdid
 */
static inline void sbi_mpt_minval_sdid(unsigned long pa, u32 sdid)
{
	unsigned long s = (unsigned long)sdid & (SBI_MPT_MAX_DOMAINS - 1UL);

	__asm__ volatile("sfence.w.inval" ::: "memory");
	__asm__ volatile(SBI_MPT_MINVAL_PA("%0", "%1")
			 :: "r"(pa), "r"(s) : "memory");
	__asm__ volatile("sfence.inval.ir" ::: "memory");
}

/*
 * Core data structures
 */
struct sbi_mpt_region {
	unsigned long	pa;
	unsigned long	size;
	u8		xwr;
	/* region is locked and deny modification via add/remove_region */
	bool		locked;
	/* mapped into more than one domain */
	bool		shared;
};

/*
 * struct sbi_mpt_mode — SMMPT mode description.
 */
struct sbi_mpt_mode {
	const char	*name;
	u32		mode_val;

	/* Assemble the MMPT CSR value */
	unsigned long	(*encode_mmpt)(unsigned long ppn, u32 sdid);

	/* map a range in MPT table */
	int		(*map_range)(struct sbi_mpt_domain *dom,
			unsigned long pa, unsigned long size, u8 xwr);

	/* number bytes for root table allocation */
	unsigned long	(*root_table_size)(void);

	/* required root table physical address alignment */
	unsigned long	(*root_table_align)(void);

	/* check if [pa,pa+size] fits in smmpt mode supported address space */
	bool		(*pa_in_range)(unsigned long pa, unsigned long size);

	/* map full range of address space which is supported by the mode */
	int		(*map_full_range)(struct sbi_mpt_domain *dom, u8 xwr);

	/* read the xwr for a PA */
	u8		(*get_xwr)(struct sbi_mpt_domain *dom, unsigned long pa);
};

struct sbi_mpt_domain {
	u32			sdid;
	bool			valid;
	unsigned long		root_pa;
	struct sbi_mpt_mode	*mode;
	struct sbi_domain	*sbi_dom;
	unsigned int		nregions;
	struct sbi_mpt_region	regions[SBI_MPT_MAX_REGIONS_DOMAIN];
	spinlock_t		lock;
};

/*
 * struct sbi_mpt_ctrl — RDSM MPT state
 */
struct sbi_mpt_ctrl {
	bool			ready;
	struct sbi_mpt_mode	*mode;
	/* detected sdidlen from mmpt probe */
	u32			sdid_len;
	/* 2^sdid_len — runtime limit for all domain operations */
	u32			max_domains;
	unsigned long		fw_pa;
	unsigned long		fw_size;
	/* Per domain state */
	struct sbi_mpt_domain	*domains[SBI_MPT_MAX_DOMAINS];
	u32			ndomain;
	/* sdid offset in scratch */
	unsigned long		sdid_offset;
	/* bitmap to track assigned/free SDID (bit N set = SDID N free) */
	DECLARE_BITMAP(sdid_bitmap, SBI_MPT_MAX_DOMAINS);
};

struct sbi_mpt_ctrl *sbi_mpt_ctrl_get(void);

static inline u32 sbi_mpt_thishart_sdid(void)
{
	struct sbi_mpt_ctrl *ctrl = sbi_mpt_ctrl_get();
	u32 *p;

	if (!ctrl->ready || !ctrl->sdid_offset)
		return (u32)SBI_MPT_SDID_INVALID;
	p = sbi_scratch_offset_ptr(sbi_scratch_thishart_ptr(),
				   ctrl->sdid_offset);
	return *p;
}

int sbi_mpt_init(void);

int sbi_mpt_domain_add_region(u32 sdid, unsigned long pa, unsigned long size, u8 xwr);

int sbi_mpt_domain_remove_region(u32 sdid, unsigned long pa, unsigned long size);

int sbi_mpt_share_region(u32 src_sdid, u32 dst_sdid, unsigned long pa, unsigned long size, u8 dst_xwr);

int sbi_mpt_unshare_region(u32 sdid, unsigned long pa, unsigned long size);

int sbi_mpt_hart_activate(u32 sdid);

void sbi_mpt_hart_deactivate(void);

int sbi_mpt_hart_activate_for_domain(struct sbi_domain *sbi_dom);

struct sbi_mpt_domain *sbi_mpt_domain_get(u32 sdid);

unsigned long sbi_mpt_pool_alloc(unsigned long size, unsigned long align);

void sbi_mpt_sdid_free(u32 sdid);

int sbi_mpt_query_access(u32 sdid, unsigned long pa, u8 *out_xwr);

void sbi_mpt_dump(void);

#endif /* __SBI_HART_MPT_H__ */

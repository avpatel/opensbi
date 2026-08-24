/* SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * Authors: Rahul Pathak <rahul.pathak@oss.qualcomm.com>
 */

#include <sbi/sbi_hart_mpt.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_string.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_hart_protection.h>
#include <sbi/riscv_locks.h>
#include <sbi/riscv_asm.h>

static struct sbi_mpt_ctrl mpt_ctrl;

struct sbi_mpt_ctrl *sbi_mpt_ctrl_get(void)
{
	return &mpt_ctrl;
}

struct sbi_mpt_domain *sbi_mpt_domain_get(u32 sdid)
{
	struct sbi_mpt_domain *sd;

	if (sdid >= mpt_ctrl.max_domains)
		return NULL;

	sd = mpt_ctrl.domains[sdid];
	if (!sd || !sd->valid)
		return NULL;

	return sd;
}

/* Memory allocator for MPT tables from heap */
unsigned long sbi_mpt_pool_alloc(unsigned long size, unsigned long align)
{
	unsigned long ptr = (unsigned long)sbi_aligned_alloc(align, size);

	if (!ptr) {
		sbi_printf("sbi_mpt: alloc failed (size=0x%lx align=0x%lx)\n",
			   size, align);
		return 0;
	}

	/* MPT tables must be zeroed to mark them invalid (mpte.V = 0) */
	sbi_memset((void *)ptr, 0, size);

	return ptr;
}

/* Check if two regions overlap */
static bool regions_overlap(unsigned long pa1, unsigned long size1,
			   unsigned long pa2, unsigned long size2)
{
	return (pa1 < pa2 + size2) && (pa2 < pa1 + size1);
}

/* Check if a region is completely contained inside another region */
static bool region_contained(unsigned long child_pa, unsigned long child_size,
			    unsigned long parent_pa, unsigned long parent_size)
{
	return (child_pa >= parent_pa) &&
	       (child_pa + child_size <= parent_pa + parent_size);
}

/*
 * Check if a region [pa, pa+size] overlaps with any mapped
 * region which is locked
 */
static bool region_overlaps_locked(const struct sbi_mpt_domain *dom,
			    unsigned long pa, unsigned long size)
{
	unsigned int i;

	for (i = 0; i < dom->nregions; i++)
		if (dom->regions[i].locked &&
		    regions_overlap(pa, size, dom->regions[i].pa, dom->regions[i].size))
			return true;
	return false;
}

/* Probe SDIDLEN from the mmpt CSR. */
static u32 probe_sdidlen(u32 mode_val)
{
	unsigned long saved, sdid_val;
	u32 sdidlen = 0;

	saved = csr_read(SBI_CSR_MMPT);

	/* Program a legal non-Bare MODE with all-ones in the SDID field */
	csr_write(SBI_CSR_MMPT,
		  (((unsigned long)mode_val << SBI_MMPT_MODE_SHIFT) & SBI_MMPT_MODE_MASK) |
		  SBI_MMPT_SDID_MASK);

	sdid_val = (csr_read(SBI_CSR_MMPT) & SBI_MMPT_SDID_MASK) >> SBI_MMPT_SDID_SHIFT;

	while (sdid_val) {
		sdidlen += 1;
		sdid_val >>= 1;
	}

	if (sdidlen > SBI_MPT_SDIDMAX)
		sdidlen = SBI_MPT_SDIDMAX;

	csr_write(SBI_CSR_MMPT, saved);

	return sdidlen;
}

#if __riscv_xlen == 64
extern struct sbi_mpt_mode smmpt43_mode;
extern struct sbi_mpt_mode smmpt52_mode;
extern struct sbi_mpt_mode smmpt64_mode;
#else
extern struct sbi_mpt_mode smmpt34_mode;
#endif

static bool probe_mode(u32 mode_val)
{
	u32 m;
	csr_write(SBI_CSR_MMPT,
		  ((unsigned long)mode_val << SBI_MMPT_MODE_SHIFT) & SBI_MMPT_MODE_MASK);

	m = (csr_read(SBI_CSR_MMPT) & SBI_MMPT_MODE_MASK) >> SBI_MMPT_MODE_SHIFT;

	return (m == mode_val);
}


/*
 * Probe SMMPT mode and initialize the related mode ops
 *
 * Deliberately prefer coverage over latency, because latency can be
 * improved by the MPT cache which hardware may implement but the
 * coverage is not. So pick the widest mode which is supported by the
 * hardware.
 */
static struct sbi_mpt_mode *init_mpt_mode(u32 *out_mode_val)
{
	unsigned long saved = csr_read(SBI_CSR_MMPT);
	struct sbi_mpt_mode *s = NULL;
	u32 mode_val = 0;

#if __riscv_xlen == 64
	if (probe_mode(SBI_MMPT_MODE_SMMPT64)) {
		s = &smmpt64_mode;
		mode_val = SBI_MMPT_MODE_SMMPT64;
	} else if (probe_mode(SBI_MMPT_MODE_SMMPT52)) {
		s = &smmpt52_mode;
		mode_val = SBI_MMPT_MODE_SMMPT52;
	} else if (probe_mode(SBI_MMPT_MODE_SMMPT43)) {
		s = &smmpt43_mode;
		mode_val = SBI_MMPT_MODE_SMMPT43;
	}
#else
	if (probe_mode(SBI_MMPT_MODE_SMMPT34)) {
		s = &smmpt34_mode;
		mode_val = SBI_MMPT_MODE_SMMPT34;
	}
#endif

	csr_write(SBI_CSR_MMPT, saved);

	if (out_mode_val)
		*out_mode_val = mode_val;

	return s;
}

/*
 * mpt_map_range_perm(): Maps [pa, pa+size] region into supervisor domain MPT table
 * with its access permissions(xwr).
 *
 * locked=true: means entry is immutable and it cannot be modified. Used for
 * firmware deny-all and other such regions for permanenet access policy.
 *
 * locked=false: region entry is modifiable at runtime via add/remove_region.
 * Used for S/U-mode accessible domain regions.
 */

static int mpt_map_range_perm(struct sbi_mpt_domain *dom, struct sbi_mpt_mode *sch,
			  unsigned long pa, unsigned long size, u8 xwr, bool locked)
{
	int rc;
	bool first_in_range;

	if (!size) {
		sbi_printf("sbi_mpt: invalid size for range 0x%lx\n", pa);
		return SBI_EINVAL;
	}

	if ((pa & SBI_MPT_PAGE_MASK) || (size & SBI_MPT_PAGE_MASK)) {
		sbi_printf("sbi_mpt: unaligned range 0x%lx+0x%lx\n", pa, size);
		return SBI_EINVAL;
	}

	if (!sch->pa_in_range(pa, size)) {
		/*
		 * A region that pa_in_range() returns false falls into one of two
		 * different cases - PA which is out of range from the mappable
		 * range of that mode and second is PA is in range but the end
		 * (pa + size) is crossing the mappable range.
		 *
		 * Distinguish the two by probing pa_in_range() for just
		 * the first page in that big range and if even the first page
		 * is out of range it is the first case and if the base is in
		 * range but the whole region is not, it is case later.
		 */
		first_in_range = sch->pa_in_range(pa, SBI_MPT_PAGE_SIZE);

		if (locked) {
			sbi_printf("sbi_mpt: locked region 0x%lx+0x%lx out of mode range\n",
				   pa, size);
			return SBI_EINVAL;
		}

		/* Region with PA which partially crossover the mappable space */
		if (first_in_range) {
			sbi_printf("sbi_mpt: fatal region 0x%lx+0x%lx crossover"
				   " mode '%s' ceiling; high part may alias"
				   " with low MPTE entries — not mapping this region\n",
				   pa, size, sch->name);
			return SBI_EINVAL;
		}

		/* Region which is entirely above the ceiling — unreachable */
		sbi_printf("sbi_mpt: skip out-of-range 0x%lx+0x%lx"
			   " (entirely above '%s' ceiling; no MPT entry,\n",
			   pa, size, sch->name);
		return 0;
	}

	rc = sch->map_range(dom, pa, size, xwr);
	if (rc)
		return rc;

	if (dom->nregions >= SBI_MPT_MAX_REGIONS_DOMAIN)
		return SBI_ENOMEM;

	dom->regions[dom->nregions].pa = pa;
	dom->regions[dom->nregions].size = size;
	dom->regions[dom->nregions].xwr = xwr;
	dom->regions[dom->nregions].locked = locked;
	dom->regions[dom->nregions].shared = false;
	dom->nregions++;

	return 0;
}

/*
 * mpt_xwr_from_memregion(): derive an MPTE XWR encoding from a SBI domain
 * memregion's S/U access flags, clamped by cap.
 *
 * cap is an upper bound the caller may impose. A region from SBI domain
 * may have more permissive access permissions which an supervisor domain
 * may want to restict and cap them.
 */
static u8 mpt_xwr_from_memregion(const struct sbi_domain_memregion *mr,
				 u8 cap)
{
	u8 xwr = SBI_MPT_PERM_NONE;

	if (mr->flags & SBI_DOMAIN_MEMREGION_SU_READABLE)
		xwr |= SBI_MPT_PERM_R;
	if (mr->flags & SBI_DOMAIN_MEMREGION_SU_WRITABLE)
		xwr |= SBI_MPT_PERM_W;
	if (mr->flags & SBI_DOMAIN_MEMREGION_SU_EXECUTABLE)
		xwr |= SBI_MPT_PERM_X;

	xwr &= cap;

	/* XWR encodings 0b010 (W) and 0b110 (WX) are reserved */
	if ((xwr & SBI_MPT_PERM_W) && !(xwr & SBI_MPT_PERM_R)) {
		sbi_printf("sbi_mpt: region 0x%lx: W without R is a reserved"
			   " XWR encoding — denying region\n", mr->base);
		return SBI_MPT_PERM_NONE;
	}

	return xwr;
}

static int smmpt_hart_configure(struct sbi_scratch *scratch,
				struct sbi_domain *dom)
{
	return sbi_mpt_hart_activate_for_domain(dom);
}

static void smmpt_hart_unconfigure(struct sbi_scratch *scratch,
				   struct sbi_domain *dom)
{
	sbi_mpt_hart_deactivate();
}

static struct sbi_hart_protection smmpt_protection = {
	.name		= "smmpt",
	.type		= SBI_HART_PROTECTION_TYPE_ID,
	.rating		= 100,
	.configure	= smmpt_hart_configure,
	.unconfigure	= smmpt_hart_unconfigure,
};

/*
 * Setup the MPT state for each domain
 *
 * The state of a supervisor domain is stored in the sbi_domain_state intance
 * of the SBI domain it belongs to so that it is created and destroyed with
 * that domain.
 */
static int mpt_state_setup(struct sbi_domain *dom, struct sbi_domain_state *state,
			  void *state_ptr)
{
	u32 sdid;
	struct sbi_mpt_domain *sd = state_ptr;
	struct sbi_mpt_ctrl *ctrl = &mpt_ctrl;

	if (!ctrl->ready)
		return 0;

	for (sdid = 0; sdid < ctrl->max_domains; sdid++)
		if (bitmap_test(ctrl->sdid_bitmap, sdid))
			break;

	if (sdid >= ctrl->max_domains) {
		sbi_printf("sbi_mpt: no free SDID for domain %s\n", dom->name);
		return SBI_ENOSPC;
	}

	bitmap_clear(ctrl->sdid_bitmap, sdid, 1);

	SPIN_LOCK_INIT(sd->lock);
	sd->sdid = sdid;
	sd->valid = false;
	sd->root_pa = 0;
	sd->mode = ctrl->mode;
	sd->sbi_dom = dom;
	sd->nregions = 0;

	ctrl->domains[sdid] = sd;

	return 0;
}

/*
 * mpt_state_finalize(): Build the MPT table of a supervisor domain.
 *
 * This function called once all domains are registered and their memory regions
 * are final. General strategy for mapping the memory regions is to create a
 * baseline mapping of the whole mappable address space supported by that mode
 * then the respective regions are overlay on top of it with the permissions flags
 * from the sbi domain. The firmware region is denied to S-mode in the root domain.
 */
static int mpt_state_finalize(struct sbi_domain *dom,
			      struct sbi_domain_state *state, void *state_ptr)
{
	int rc;
	u8 base_xwr;
	unsigned long root_pa;
	struct sbi_mpt_region *r;
	struct sbi_mpt_mode *sch;
	const struct sbi_domain_memregion *mr;
	struct sbi_mpt_domain *sd = state_ptr;
	struct sbi_mpt_ctrl *ctrl = &mpt_ctrl;

	if (!ctrl->ready)
		return 0;

	sch = ctrl->mode;

	root_pa = sbi_mpt_pool_alloc(sch->root_table_size(),
				     sch->root_table_align());
	if (!root_pa) {
		sbi_printf("sbi_mpt: OOM root table for domain %s\n", dom->name);
		return SBI_ENOMEM;
	}
	sd->root_pa = root_pa;

	/*
	 * Similar to SBI domain, create a baseline which maps whole
	 * mappable address space by a mode and mark it with XWR
	 * permissions. Later selectively restrict the permissions in
	 * another pass.
	 */
	sbi_domain_for_each_memregion(dom, mr) {
		if (mr->order < sizeof(unsigned long) * 8)
			continue;

		if (!(mr->flags & SBI_DOMAIN_MEMREGION_SU_ACCESS_MASK))
			continue;

		base_xwr = mpt_xwr_from_memregion(mr, SBI_MPT_PERM_RWX);
		rc = sch->map_full_range(sd, base_xwr);
		if (rc)
			return rc;

		if (sd->nregions < SBI_MPT_MAX_REGIONS_DOMAIN) {
			r = &sd->regions[sd->nregions++];
			r->pa = 0;
			r->size = ~0UL & ~SBI_MPT_PAGE_MASK;
			r->xwr = base_xwr;
			r->locked = false;
			r->shared = false;
		}

		break;
	}

	/*
	 * Another pass to map every other region which has its own
	 * permissions derived from the memregion's S/U flags.
	 */
	sbi_domain_for_each_memregion(dom, mr) {
		if (mr->order >= sizeof(unsigned long) * 8)
			continue;

		rc = mpt_map_range_perm(sd, sch, mr->base, BIT(mr->order),
					mpt_xwr_from_memregion(mr, SBI_MPT_PERM_RWX),
					false);
		if (rc)
			return rc;
	}

	/*
	 * Explicitly map the firmware region to S-mode with no permissions
	 * and locked.
	 */
	if (dom->index == 0) {
		rc = mpt_map_range_perm(sd, sch, ctrl->fw_pa, ctrl->fw_size,
					SBI_MPT_PERM_NONE, true);
		if (rc) {
			sbi_printf("sbi_mpt: cannot protect firmware rc=%d\n", rc);
			return rc;
		}
	}

	sd->valid = true;
	ctrl->ndomain++;

	sbi_mpt_fence_sdid(sd->sdid);

	return 0;
}

/*
 * mpt_state_cleanup(): Release the MPT state of a supervisor domain.
 */
static void mpt_state_cleanup(struct sbi_domain *dom,
			      struct sbi_domain_state *state, void *state_ptr)
{
	struct sbi_mpt_domain *sd = state_ptr;
	struct sbi_mpt_ctrl *ctrl = &mpt_ctrl;

	if (!ctrl->ready || !sd->valid)
		return;

	/*
	 * TODO: free the MPT table tree (root, inner and leaf tables)
	 * allocated by mpt_state_finalize(). A recursive free is not
	 * implemented yet so the tables and it will be important when
	 * runtime regions changes will be there.
	 */

	sd->valid = false;
	ctrl->ndomain--;

	sbi_mpt_fence_sdid(sd->sdid);
	sbi_mpt_sdid_free(sd->sdid);
}

static struct sbi_domain_state mpt_domain_state = {
	.state_size	= sizeof(struct sbi_mpt_domain),
	.state_setup	= mpt_state_setup,
	.state_finalize	= mpt_state_finalize,
	.state_cleanup	= mpt_state_cleanup,
};

/*
 * sbi_mpt_domain_add_region(): Map a region in a MPT table
 */
int sbi_mpt_domain_add_region(u32 sdid, unsigned long pa,
			       unsigned long size, u8 xwr)
{
	struct sbi_mpt_domain *dom;
	struct sbi_mpt_region *r;
	int rc;

	if (!mpt_ctrl.ready)
		return SBI_ENODEV;
	if (!size || (pa & SBI_MPT_PAGE_MASK) || (size & SBI_MPT_PAGE_MASK))
		return SBI_EINVAL;

	dom = sbi_mpt_domain_get(sdid);
	if (!dom)
		return SBI_EINVAL;

	if (region_overlaps_locked(dom, pa, size)) {
		sbi_printf("sbi_mpt: 0x%lx+0x%lx overlaps locked"
			   " (SDID %u)\n", pa, size, sdid);
		return SBI_EINVAL;
	}

	if (!dom->mode->pa_in_range(pa, size)) {
		/*
		 * A region that pa_in_range() rejects falls into one of two
		 * different cases - PA which is out of range from the mappable
		 * range of that mode and second is PA is in range but the end
		 * (PA + SIZE) is crossing the mappable range.
		 *
		 * Distinguish the two by probing pa_in_range() for just
		 * the base page and if even the base is out of range it is
		 * the first case and if the base is in range but the whole
		 * region is not, it is case later.
		 */
		if (dom->mode->pa_in_range(pa, SBI_MPT_PAGE_SIZE))
			sbi_printf("sbi_mpt: 0x%lx+0x%lx crossover mode '%s'"
				   " ceiling (SDID %u) — high part would alias;"
				   " rejected\n",
				   pa, size, dom->mode->name, sdid);
		else
			sbi_printf("sbi_mpt: 0x%lx+0x%lx out of range"
				   " (SDID %u)\n", pa, size, sdid);
		return SBI_EINVAL;
	}

	spin_lock(&dom->lock);
	rc = dom->mode->map_range(dom, pa, size, xwr);
	spin_unlock(&dom->lock);

	if (!rc) {
		if (dom->nregions < SBI_MPT_MAX_REGIONS_DOMAIN) {
			r = &dom->regions[dom->nregions++];
			r->pa = pa;
			r->size = size;
			r->xwr = xwr;
			r->locked = false;
			r->shared = false;
		}

		sbi_mpt_fence_sdid(sdid);
	}

	return rc;
}

/*
 * sbi_mpt_domain_remove_region(): Unmap a region from the MPT table
 */
int sbi_mpt_domain_remove_region(u32 sdid, unsigned long pa,
				  unsigned long size)
{
	int rc;
	unsigned int i;
	struct sbi_mpt_domain *dom;
	struct sbi_mpt_region *r;

	if (!mpt_ctrl.ready)
		return SBI_ENODEV;

	if (!size || (pa & SBI_MPT_PAGE_MASK) || (size & SBI_MPT_PAGE_MASK))
		return SBI_EINVAL;

	dom = sbi_mpt_domain_get(sdid);
	if (!dom)
		return SBI_EINVAL;

	for (i = 0; i < dom->nregions; i++) {
		r = &dom->regions[i];

		if (r->locked && region_contained(pa, size, r->pa, r->size))
			return SBI_EDENIED;
		/*
		 * Shared regions must be removed via sbi_mpt_unshare_region()
		 * because that function prevents one domain revoking a region
		 * that another domain still has mapped.
		 */
		if (r->shared && r->pa == pa && r->size == size)
			return SBI_EDENIED;
	}

	spin_lock(&dom->lock);
	rc = dom->mode->map_range(dom, pa, size, SBI_MPT_PERM_NONE);
	spin_unlock(&dom->lock);

	if (!rc)
		sbi_mpt_fence_sdid(sdid);

	return rc;
}

/*
 * Shared memory regions
 *
 * sbi_mpt_share_region() — map a region into a second domain.
 *
 * Both domains MPT tables are updated independently — each has its
 * own root/inner/leaf table tree. The same PA range can have different
 * XWR permissions in different domains.
 *
 */

int sbi_mpt_share_region(u32 src_sdid, u32 dst_sdid,
			  unsigned long pa, unsigned long size,
			  u8 dst_xwr)
{
	unsigned int i;
	int rc;
	struct sbi_mpt_domain *src_dom, *dst_dom;

	if (!mpt_ctrl.ready)
		return SBI_ENODEV;

	if (!size || (pa & SBI_MPT_PAGE_MASK) || (size & SBI_MPT_PAGE_MASK))
		return SBI_EINVAL;

	if (src_sdid == dst_sdid)
		return SBI_EINVAL;

	src_dom = sbi_mpt_domain_get(src_sdid);
	dst_dom = sbi_mpt_domain_get(dst_sdid);
	if (!src_dom || !dst_dom)
		return SBI_EINVAL;

	if (region_overlaps_locked(dst_dom, pa, size)) {
		sbi_printf("sbi_mpt: share 0x%lx+0x%lx overlaps locked region in SDID %u\n",
			   pa, size, dst_sdid);
		return SBI_EINVAL;
	}

	/* Reject reserved xwr permissions: 0b010 and 0b110 (W without R) */
	if ((dst_xwr & SBI_MPT_PERM_W) && !(dst_xwr & SBI_MPT_PERM_R)) {
		sbi_printf("sbi_mpt: share 0x%lx+0x%lx: W without R is a reserved XWR encoding\n",
			   pa, size);
		return SBI_EINVAL;
	}

	if (dst_dom->nregions >= SBI_MPT_MAX_REGIONS_DOMAIN) {
		sbi_printf("sbi_mpt: share: SDID %u region table full\n", dst_sdid);
		return SBI_ENOMEM;
	}

	spin_lock(&dst_dom->lock);
	rc = dst_dom->mode->map_range(dst_dom, pa, size, dst_xwr);
	spin_unlock(&dst_dom->lock);
	if (rc)
		return rc;

	dst_dom->regions[dst_dom->nregions].pa = pa;
	dst_dom->regions[dst_dom->nregions].size = size;
	dst_dom->regions[dst_dom->nregions].xwr = dst_xwr;
	dst_dom->regions[dst_dom->nregions].locked = false;
	dst_dom->regions[dst_dom->nregions].shared = true;
	dst_dom->nregions++;

	for (i = 0; i < src_dom->nregions; i++) {
		if (src_dom->regions[i].pa == pa &&
		    src_dom->regions[i].size == size) {
			src_dom->regions[i].shared = true;
			break;
		}
	}

	sbi_mpt_fence_sdid(src_sdid);
	sbi_mpt_fence_sdid(dst_sdid);

	sbi_printf("sbi_mpt: shared 0x%lx+0x%lx SDID %u → SDID %u xwr=%u\n",
		   pa, size, src_sdid, dst_sdid, dst_xwr);

	return 0;
}

/*
 * sbi_mpt_unshare_region() — Unmap a shared region from a domain.
 *
 * The other domain remains unaffected just the shared flag gets removed
 * for that region from both the domains.
 */
int sbi_mpt_unshare_region(u32 sdid, unsigned long pa, unsigned long size)
{
	u32 s;
	int rc;
	unsigned int i, j;
	struct sbi_mpt_domain *dom, *peer;

	if (!mpt_ctrl.ready)
		return SBI_ENODEV;

	if (!size || (pa & SBI_MPT_PAGE_MASK) || (size & SBI_MPT_PAGE_MASK))
		return SBI_EINVAL;

	dom = sbi_mpt_domain_get(sdid);
	if (!dom)
		return SBI_EINVAL;

	for (i = 0; i < dom->nregions; i++) {
		if (dom->regions[i].pa == pa &&
		    dom->regions[i].size == size &&
		    dom->regions[i].shared)
			break;
	}
	if (i == dom->nregions) {
		sbi_printf("sbi_mpt: unshare 0x%lx+0x%lx not found in SDID %u\n",
			   pa, size, sdid);
		return SBI_EINVAL;
	}

	spin_lock(&dom->lock);
	rc = dom->mode->map_range(dom, pa, size, SBI_MPT_PERM_NONE);
	spin_unlock(&dom->lock);

	if (rc)
		return rc;

	dom->nregions -= 1;
	dom->regions[i] = dom->regions[dom->nregions];

	/* Clear the shared flag on any other domain that has this PA region shared.*/
	for (s = 0; s < mpt_ctrl.max_domains; s++) {
		peer = sbi_mpt_domain_get(s);
		if (!peer || peer == dom)
			continue;

		for (j = 0; j < peer->nregions; j++)
			if (peer->regions[j].pa == pa && peer->regions[j].size == size)
				peer->regions[j].shared = false;
	}

	sbi_mpt_fence_sdid(sdid);

	return 0;
}

/*
 * Disable SMMPT on hart
 */
void sbi_mpt_hart_deactivate(void)
{
	/*
	 * mmpt is inactive in M-mode per spec, so no CSR write
	 * is needed.
	 */
}

/*
 * Enable SMMPT on hart on a supervisor domain(sdid)
 */
int sbi_mpt_hart_activate(u32 sdid)
{
	u32 *p;
	unsigned long ppn;
	struct sbi_mpt_domain *dom;

	if (!mpt_ctrl.ready)
		return SBI_ENODEV;

	dom = sbi_mpt_domain_get(sdid);
	if (!dom)
		return SBI_EINVAL;

	ppn = dom->root_pa >> SBI_MPT_PAGE_SHIFT;
	csr_write(SBI_CSR_MMPT, dom->mode->encode_mmpt(ppn, sdid));
	sbi_mpt_fence_sdid(sdid);

	if (mpt_ctrl.sdid_offset) {
		p = sbi_scratch_offset_ptr(sbi_scratch_thishart_ptr(), mpt_ctrl.sdid_offset);
		*p = sdid;
	}

	return 0;
}

/*
 * Activate SMMPT for all harts which are associated with an linked
 * SBI domain in an supervisor domain.
 */
int sbi_mpt_hart_activate_for_domain(struct sbi_domain *sbi_dom)
{
	struct sbi_mpt_domain *sd;

	if (!mpt_ctrl.ready)
		return SBI_ENODEV;
	if (!sbi_dom)
		return SBI_EINVAL;

	sd = sbi_domain_state_ptr(sbi_dom, &mpt_domain_state);
	if (!sd || !sd->valid)
		return 0;

	return sbi_mpt_hart_activate(sd->sdid);
}

/*
 * sbi_mpt_sdid_free(): Returns SDID to the free pool after domain destroy.
 */
void sbi_mpt_sdid_free(u32 sdid)
{
	if (!mpt_ctrl.ready || sdid >= mpt_ctrl.max_domains)
		return;

	bitmap_set(mpt_ctrl.sdid_bitmap, (int)sdid, 1);

	mpt_ctrl.domains[sdid] = NULL;
}

/*
 * sbi_mpt_query_access(): Returns the XWR for a PA in supervisor domain(sdid).
 */
int sbi_mpt_query_access(u32 sdid, unsigned long pa, u8 *out_xwr)
{
	struct sbi_mpt_domain *dom;

	if (!mpt_ctrl.ready)
		return SBI_ENODEV;

	if (!out_xwr)
		return SBI_EINVAL;

	dom = sbi_mpt_domain_get(sdid);
	if (!dom)
		return SBI_EINVAL;

	if (!dom->mode->pa_in_range(pa, SBI_MPT_PAGE_SIZE)) {
		*out_xwr = SBI_MPT_PERM_NONE;
		return 0;
	}

	spin_lock(&dom->lock);
	*out_xwr = dom->mode->get_xwr(dom, pa);
	spin_unlock(&dom->lock);

	return 0;
}

/*
 * sbi_mpt_init(): Initialize MPT structures
 */
int sbi_mpt_init(void)
{
	int rc;
	u32 *p;
	u32 mode_val;
	unsigned long fw_end;
	struct sbi_scratch *s;
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();
	unsigned long fw_pa = scratch->fw_start;
	unsigned long fw_size = scratch->fw_size;

	if (mpt_ctrl.ready)
		return 0;

	if (!sbi_hart_has_extension(scratch, SBI_HART_EXT_SMSDID)) {
		sbi_printf("sbi_mpt: Smsdid extension absent\n");
		return SBI_ENODEV;
	}

	if (!sbi_hart_has_extension(scratch, SBI_HART_EXT_SMMPT)) {
		sbi_printf("sbi_mpt: Smmpt extension absent\n");
		return SBI_ENODEV;
	}

	mpt_ctrl.mode = init_mpt_mode(&mode_val);
	if (!mpt_ctrl.mode) {
		sbi_printf("sbi_mpt: WARL probe found no mode\n");
		return SBI_ENODEV;
	}

	/*
	 * SDIDLEN must be probed with a legal non-Bare MODE programmed,
	 * probe sdidlen must be done only after the probing the mode.
	 */
	mpt_ctrl.sdid_len = probe_sdidlen(mode_val);
	if (!mpt_ctrl.sdid_len) {
		sbi_printf("sbi_mpt: SDIDLEN=0 — SDID field not implemented\n");
		return SBI_ENODEV;
	}

	mpt_ctrl.max_domains = 1UL << mpt_ctrl.sdid_len;

	/*
	 * Initialise SDID bitmap
	 * set bits 0..(max_domains-1) as 1 marking those SDIDs free.
	 */
	bitmap_fill(mpt_ctrl.sdid_bitmap, mpt_ctrl.max_domains);

	/* Round fw_pa and fw_size to page boundaries before storing. */
	fw_end = (fw_pa + fw_size + SBI_MPT_PAGE_SIZE - 1UL) & ~SBI_MPT_PAGE_MASK;
	mpt_ctrl.fw_pa = fw_pa & ~SBI_MPT_PAGE_MASK;
	mpt_ctrl.fw_size = fw_end - mpt_ctrl.fw_pa;


	/* Per-hart SDID scratch slot — initialise all harts to INVALID */
	mpt_ctrl.sdid_offset = sbi_scratch_alloc_offset(sizeof(u32));
	if (!mpt_ctrl.sdid_offset) {
		sbi_printf("sbi_mpt: scratch alloc for SDID failed\n");
	}
	else {
		sbi_for_each_hartindex(i) {
			s = sbi_hartindex_to_scratch(i);
			if (s) {
				p = sbi_scratch_offset_ptr(s, mpt_ctrl.sdid_offset);
				*p = SBI_MPT_SDID_INVALID;
			}
		}
	}

	mpt_ctrl.ready = true;

	rc = sbi_hart_protection_register(&smmpt_protection);
	if (rc) {
		sbi_printf("sbi_mpt: hart protection register failed rc=%d\n", rc);
		mpt_ctrl.ready = false;
		return rc;
	}

	rc = sbi_domain_register_state(&mpt_domain_state);
	if (rc) {
		sbi_printf("sbi_mpt: domain data register failed rc=%d\n", rc);
		mpt_ctrl.ready = false;
		return rc;
	}

	return 0;
}

/*
 * Debug dump
 */
void sbi_mpt_dump(void)
{
	u32 s;
	unsigned int r;
	const struct sbi_mpt_domain *d;
	const unsigned long catchall_size = ~0UL & ~SBI_MPT_PAGE_MASK;

	if (!mpt_ctrl.ready) {
		sbi_printf("sbi_mpt: not initialised\n");
		return;
	}

	sbi_printf("sbi_mpt: mode=%-10s SDIDLEN=%u max_domains=%u\n", mpt_ctrl.mode->name,
		   mpt_ctrl.sdid_len, mpt_ctrl.max_domains);
	sbi_printf("root_size=0x%lx root_align=0x%lx\n",
		   mpt_ctrl.mode->root_table_size(), mpt_ctrl.mode->root_table_align());
	sbi_printf("fw 0x%016lx+0x%lx  pool=global heap\n",
		   mpt_ctrl.fw_pa, mpt_ctrl.fw_size);
	for (s = 0; s < mpt_ctrl.max_domains; s++) {
		bool has_baseline = false;
		u8   baseline_xwr = SBI_MPT_PERM_NONE;

		d = mpt_ctrl.domains[s];
		if (!d || !d->valid)
			continue;
		sbi_printf("SDID %2u root=0x%016lx regions=%u dom=%s\n",
			   d->sdid, d->root_pa, d->nregions, d->sbi_dom ? d->sbi_dom->name : "(none)");
		for (r = 0; r < d->nregions; r++) {

			if (d->regions[r].pa == 0 &&
			    d->regions[r].size == catchall_size) {
				has_baseline = true;
				baseline_xwr = d->regions[r].xwr;
				continue;
			}

			sbi_printf("S-Domain%u Region%02u          : 0x%016lx+0x%-16lx: xwr=%u (%c%c%c)%s%s\n",
			   d->sdid, r, d->regions[r].pa,
			   d->regions[r].size,
			   d->regions[r].xwr,
			   (d->regions[r].xwr & SBI_MPT_PERM_R) ? 'R' : '-',
			   (d->regions[r].xwr & SBI_MPT_PERM_W) ? 'W' : '-',
			   (d->regions[r].xwr & SBI_MPT_PERM_X) ? 'X' : '-',
			   d->regions[r].locked ? " [LOCKED]" : "",
			   d->regions[r].shared ? " [SHARED]" : "");
		}

		if (has_baseline) {
			sbi_printf("S-Domain%u Default           : %-37s: xwr=%u (%c%c%c)   [baseline, overlay by regions above]\n",
				   d->sdid, "Rest of the address space",
				   baseline_xwr,
				   (baseline_xwr & SBI_MPT_PERM_R) ? 'R' : '-',
				   (baseline_xwr & SBI_MPT_PERM_W) ? 'W' : '-',
				   (baseline_xwr & SBI_MPT_PERM_X) ? 'X' : '-');
		}
		else {
			sbi_printf("S-Domain%u           : xwr=0 (---)   [no baseline — unmapped PAs denied]\n",
				   d->sdid);
		}
	}
}

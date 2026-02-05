/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Ventana Micro Systems Inc.
 *
 * Authors:
 *   Anup Patel <apatel@ventanamicro.com>
 */

#include <sbi/sbi_irqchip.h>
#include <sbi/sbi_list.h>
#include <sbi/sbi_platform.h>

static SBI_LIST_HEAD(irqchip_list);

int sbi_irqchip_process(void)
{
	struct sbi_irqchip_device *chip;
	int rc = SBI_ENODEV;

	sbi_list_for_each_entry(chip, &irqchip_list, node) {
		if (!chip->process_hwirqs)
			continue;
		if (!sbi_hartmask_test_hartindex(current_hartindex(), &chip->target_harts))
			continue;
		rc = chip->process_hwirqs(chip);
		if (rc)
			break;
	}

	return rc;
}

struct sbi_irqchip_device *sbi_irqchip_find_device(u32 id)
{
	struct sbi_irqchip_device *chip;

	sbi_list_for_each_entry(chip, &irqchip_list, node) {
		if (chip->id == id)
			return chip;
	}

	return NULL;
}

int sbi_irqchip_add_device(struct sbi_irqchip_device *chip)
{
	struct sbi_irqchip_device *c;
	struct sbi_hartmask hm;

	if (!chip || !sbi_hartmask_weight(&chip->target_harts))
		return SBI_EINVAL;

	if (sbi_irqchip_find_device(chip->id))
		return SBI_EALREADY;

	if (chip->process_hwirqs) {
		sbi_list_for_each_entry(c, &irqchip_list, node) {
			if (!c->process_hwirqs)
				continue;
			sbi_hartmask_and(&hm, &c->target_harts, &chip->target_harts);
			if (sbi_hartmask_weight(&hm))
				return SBI_EINVAL;
		}
	}

	sbi_list_add_tail(&chip->node, &irqchip_list);
	return 0;
}

int sbi_irqchip_init(struct sbi_scratch *scratch, bool cold_boot)
{
	int rc;
	const struct sbi_platform *plat = sbi_platform_ptr(scratch);
	struct sbi_irqchip_device *chip;

	if (cold_boot) {
		rc = sbi_platform_irqchip_init(plat);
		if (rc)
			return rc;
	}

	sbi_list_for_each_entry(chip, &irqchip_list, node) {
		if (!chip->warm_init)
			continue;
		if (!sbi_hartmask_test_hartindex(current_hartindex(), &chip->target_harts))
			continue;
		rc = chip->warm_init(chip);
		if (rc)
			return rc;
	}

	if (!sbi_list_empty(&irqchip_list))
		csr_set(CSR_MIE, MIP_MEIP);

	return 0;
}

void sbi_irqchip_exit(struct sbi_scratch *scratch)
{
	if (!sbi_list_empty(&irqchip_list))
		csr_clear(CSR_MIE, MIP_MEIP);
}

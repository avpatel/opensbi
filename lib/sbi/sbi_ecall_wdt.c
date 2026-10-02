/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Qualcomm Inc.
 *
 * Authors:
 *   Prajwal N <prajwal.n@oss.qualcomm.com>
 */

#include <sbi/sbi_ecall.h>
#include <sbi/sbi_ecall_interface.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_trap.h>
#include <sbi/sbi_wdt.h>

/**
 * OpenSBI watchdog ecall handler
 * @param extid OpenSBI extension ID
 * @param funcid OpenSBI function ID corresponding to the extension ID
 * @param regs Trap registers for arguments
 * @param out Carry the return value
 * @return integer error value
 */
static int sbi_ecall_wdt_handler(unsigned long extid, unsigned long funcid,
			                        struct sbi_trap_regs *regs,
	                                struct sbi_ecall_return *out)
{
	int ret;
	u32 attribute_value = 0;
	ret = SBI_ERR_FAILED;

	switch (funcid) {
	case SBI_EXT_WATCHDOG_READ_ATTRIBUTE:
		ret = sbi_watchdog_read_attribute(regs->a0, &attribute_value);
		out->value = attribute_value;
		break;

	case SBI_EXT_WATCHDOG_WRITE_ATTRIBUTE:
		ret = sbi_watchdog_write_attribute(regs->a0, regs->a1);
		break;

	case SBI_EXT_WATCHDOG_WRITE_NOTIF_MSI_MESSAGE:
		ret = sbi_watchdog_write_notif_msi_message(regs->a0, regs->a1, regs->a2);
		break;

	case SBI_EXT_WATCHDOG_START:
		ret = sbi_watchdog_start();
		break;

	case SBI_EXT_WATCHDOG_STOP:
		ret = sbi_watchdog_stop();
		break;

	case SBI_EXT_WATCHDOG_PAT:
		ret = sbi_watchdog_pat();
		break;

	default:
		break;
	}

	return ret;
}

struct sbi_ecall_extension ecall_wdt;

static int sbi_ecall_wdt_register_extensions(void)
{
	return sbi_ecall_register_extension(&ecall_wdt);
}

struct sbi_ecall_extension ecall_wdt = {
	.name                   = "sbi-wdt",
	.extid_start            = SBI_EXT_WDT,
	.extid_end              = SBI_EXT_WDT,
	.experimental           = true,
	.register_extensions    = sbi_ecall_wdt_register_extensions,
	.handle                 = sbi_ecall_wdt_handler,
};

/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Qualcomm Inc.
 *
 * Authors:
 *   Prajwal N <prajwal.n@oss.qualcomm.com>
 */

#include <sbi/riscv_asm.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_ecall.h>
#include <sbi/sbi_bitops.h>
#include <sbi/sbi_ecall_interface.h>
#include <sbi/riscv_locks.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_irqchip.h>
#include <sbi/sbi_timer.h>
#include <sbi/sbi_wdt.h>
#include <sbi/sbi_system.h>
#include <sbi/sbi_console.h>
#include <sbi/riscv_io.h>
#include <sbi/sbi_string.h>
#include <sbi/sbi_hart_protection.h>

/** Watchdog capabitlity - MSI */
#define WDT_CAP_MSI_MASK	BIT(1)

/** Watchdog capabitlity - SSE */
#define WDT_CAP_SSE_MASK	BIT(0)

/**
 * Helpers to enable/disable channel capability bits
 * _c: capability variable
 * _m: capability mask
 */
#define CAP_ENABLE(_c, _m)	INSERT_FIELD(_c, _m, 1)
#define CAP_DISABLE(_c, _m)	INSERT_FIELD(_c, _m, 0)
#define CAP_GET(_c, _m)		EXTRACT_FIELD(_c, _m)

/* Watchdog Permission Flags */
#define WDT_ATTR_PERM_RO	BIT(0)
#define WDT_ATTR_PERM_W	 BIT(1)
#define WDT_ATTR_PERM_RW	(WDT_ATTR_PERM_RO) | (WDT_ATTR_PERM_W)

static const u32 sbi_watchdog_attr_perm[SBI_WDT_ATTR_MAX] = {
	[SBI_WDT_CAPABILITY]            = WDT_ATTR_PERM_RO,
	[SBI_WDT_STATE]                 = WDT_ATTR_PERM_RO,
	[SBI_WDT_MIN_PERIOD]            = WDT_ATTR_PERM_RO,
	[SBI_WDT_PERIOD]                = WDT_ATTR_PERM_RW,
	[SBI_WDT_TIME_LEFT]             = WDT_ATTR_PERM_RO,
	[SBI_WDT_NOTIF_TIME]            = WDT_ATTR_PERM_RW,
	[SBI_WDT_NOTIF_MSI_ADDR_LOW]    = WDT_ATTR_PERM_RW,
	[SBI_WDT_NOTIF_MSI_ADDR_HIGH]   = WDT_ATTR_PERM_RW,
	[SBI_WDT_NOTIF_MSI_DATA]        = WDT_ATTR_PERM_RW
};

/**
 * SBI watchdog state
 */
typedef enum {
	SBI_WDT_STOPPED = 0,
	SBI_WDT_RUNNING	= 1,
	SBI_WDT_PAUSED  = 2,
	SBI_WDT_EXPIRED	= 3
} sbi_wdt_state_t;

/**
 * SBI watchdog attributes
 */
struct sbi_watchdog_attrs {
	/* MSI and SSE capability flags */
	u32 wdt_capability;
	/* Watchdog state */
	u32 wdt_state;
	/* Minimum watchdog timeout supported in us */
	u32 wdt_min_period;
	/* Watchdog timeout period in us */
	u32 wdt_period;
	/* Time left until watchdog expiry in us */
	u32 wdt_time_left;
	/* Watchdog pretimeout in us */
	u32 wdt_notif_time;
	/* MSI target address low 32-bit */
	u32 wdt_notif_msi_addr_low;
	/* MSI target address high 32-bit */
	u32 wdt_notif_msi_addr_high;
	/* MSI data */
	u32 wdt_notif_msi_data;
};

struct sbi_watchdog {
	/* Watchdog attributes */
	struct sbi_watchdog_attrs wdt_attr;
	/* Watchdog timeout event */
	struct sbi_timer_event wdt_timer_ev;
	/* Watchdog pretimeout event */
	struct sbi_timer_event wdt_pretimeout_ev;
	/* Watchdog spinlock */
	spinlock_t wdt_lock;
};

/* Global pointer to the watchdog structure */
static struct sbi_watchdog *sbi_system_watchdog;

/** Make sure all the attributes are packed for direct memcpy in ATTR_READ */
#define assert_field_offset(field, attr_offset)				\
	_Static_assert(							\
		((offsetof(struct sbi_watchdog_attrs, field)) /		\
			sizeof(u32)) == attr_offset,			\
		"field" #field						\
		" from struct sbi_watchdog_attrs invalid offset, expected " #attr_offset)

assert_field_offset(wdt_capability, SBI_WDT_CAPABILITY);
assert_field_offset(wdt_state, SBI_WDT_STATE);
assert_field_offset(wdt_min_period, SBI_WDT_MIN_PERIOD);
assert_field_offset(wdt_period, SBI_WDT_PERIOD);
assert_field_offset(wdt_time_left, SBI_WDT_TIME_LEFT);
assert_field_offset(wdt_notif_time, SBI_WDT_NOTIF_TIME);
assert_field_offset(wdt_notif_msi_addr_low, SBI_WDT_NOTIF_MSI_ADDR_LOW);
assert_field_offset(wdt_notif_msi_addr_high, SBI_WDT_NOTIF_MSI_ADDR_HIGH);
assert_field_offset(wdt_notif_msi_data, SBI_WDT_NOTIF_MSI_DATA);

int sbi_watchdog_read_attribute(u32 attribute_id, u32* attribute_value)
{
	u64 current, deadline, tick_delta;
	int ret = SBI_SUCCESS;

	spin_lock(&sbi_system_watchdog->wdt_lock);

	switch (attribute_id) {
	case SBI_WDT_TIME_LEFT:
		/* Calculate the time left before expiry and update the attribute */
		current = sbi_timer_value();
		deadline = sbi_system_watchdog->wdt_timer_ev.time_stamp;
		if (current >= deadline) {
			sbi_system_watchdog->wdt_attr.wdt_time_left = 0;
			break;
		}
		tick_delta = deadline - current;
		sbi_system_watchdog->wdt_attr.wdt_time_left = sbi_timer_ticks_to_usec(tick_delta);
		break;

	default:
		/* Return error if the attribute index is out of bound*/
		if (attribute_id >= SBI_WDT_ATTR_MAX) {
			ret = SBI_ERR_INVALID_PARAM;
			goto out;
		}
		break;
	}

	u32 *wdt_attrs = (u32 *)&sbi_system_watchdog->wdt_attr;
	*attribute_value = wdt_attrs[attribute_id];

out:
	spin_unlock(&sbi_system_watchdog->wdt_lock);
	return ret;
}

int sbi_watchdog_write_attribute(u32 attribute_id, u32 attribute_value)
{
	int ret;

	spin_lock(&sbi_system_watchdog->wdt_lock);

	/* Check if the attribute is writable */
	if (!(sbi_watchdog_attr_perm[attribute_id] & WDT_ATTR_PERM_W)) {
		ret = SBI_ERR_DENIED;
		goto out;
	}

	switch (attribute_id) {
	case SBI_WDT_PERIOD:
		/* Return error if wdt_period passed is less than wdt_min_period */
		if (attribute_value < sbi_system_watchdog->wdt_attr.wdt_min_period) {
			ret = SBI_ERR_FAILED;
			goto out;
		}
		break;

	case SBI_WDT_NOTIF_TIME:
		/* Ignore the write if the MSI capability is not enabled */
		if (!CAP_GET(sbi_system_watchdog->wdt_attr.wdt_capability, WDT_CAP_MSI_MASK)) {
			ret = SBI_SUCCESS;
			goto out;
		}

		/* Returen error if the notif time is greater than the timeout period*/
		if (attribute_value >= sbi_system_watchdog->wdt_attr.wdt_period) {
			ret = SBI_ERR_FAILED;
			goto out;
		}
		break;

	case SBI_WDT_NOTIF_MSI_ADDR_LOW:
	case SBI_WDT_NOTIF_MSI_ADDR_HIGH:
	case SBI_WDT_NOTIF_MSI_DATA:
		/* Ignore the write if the MSI capability is not enabled */
		if (!CAP_GET(sbi_system_watchdog->wdt_attr.wdt_capability, WDT_CAP_MSI_MASK)) {
			ret = SBI_SUCCESS;
			goto out;
		}

		/*
		 * The MSI address and data can be written only when
		 * the notifications are disabled
		 */
		if (sbi_system_watchdog->wdt_attr.wdt_notif_time != 0) {
			ret = SBI_ERR_FAILED;
			goto out;
		}
		break;

	default:
		/* Return error if the attribute index is out of bound*/
		if (attribute_id >= SBI_WDT_ATTR_MAX) {
			ret = SBI_ERR_INVALID_PARAM;
			goto out;
		}
		break;
	}

	u32 *wdt_attrs = (u32 *)&sbi_system_watchdog->wdt_attr;
	wdt_attrs[attribute_id] = attribute_value;

	ret = SBI_SUCCESS;

out:
	spin_unlock(&sbi_system_watchdog->wdt_lock);
	return ret;
}

int sbi_watchdog_write_notif_msi_message(u32 msi_addr_low, u32 msi_addr_high, u32 msi_data)
{
	int ret;

	spin_lock(&sbi_system_watchdog->wdt_lock);

	/* Ignore the write if the MSI capability is not enabled */
	if (!CAP_GET(sbi_system_watchdog->wdt_attr.wdt_capability, WDT_CAP_MSI_MASK)) {
		ret = SBI_SUCCESS;
		goto out;
	}

	/* The MSI address and data can be written only when the notifications are disabled */
	if (sbi_system_watchdog->wdt_attr.wdt_notif_time != 0) {
		ret = SBI_EINVAL;
		goto out;
	}

	u32 *wdt_attrs = (u32 *)&sbi_system_watchdog->wdt_attr;
	wdt_attrs[SBI_WDT_NOTIF_MSI_ADDR_LOW] = msi_addr_low;
	wdt_attrs[SBI_WDT_NOTIF_MSI_ADDR_HIGH] = msi_addr_high;
	wdt_attrs[SBI_WDT_NOTIF_MSI_DATA] = msi_data;

	ret = SBI_SUCCESS;

out:
	spin_unlock(&sbi_system_watchdog->wdt_lock);
	return ret;
}

static void sbi_watchdog_pretimeout_cleanup(struct sbi_timer_event *ev)
{
	/* Nothing to do */
}

static void sbi_watchdog_pretimeout_callback(struct sbi_timer_event *ev,
					     struct sbi_timer_event_restart *restart)
{
	u64 msi_addr;
	u32 msi_data;

	spin_lock(&sbi_system_watchdog->wdt_lock);
	/* Calculate the MSI address using the low and high addresses*/
	msi_addr = (((u64)sbi_system_watchdog->wdt_attr.wdt_notif_msi_addr_high << 32 ) |
					sbi_system_watchdog->wdt_attr.wdt_notif_msi_addr_low);
	msi_data = sbi_system_watchdog->wdt_attr.wdt_notif_msi_data;

	spin_unlock(&sbi_system_watchdog->wdt_lock);

	sbi_hart_protection_temp_map_range((unsigned long)msi_addr, sizeof(u32));
	/* Write the MSI data to the MSI address*/
	writel(msi_data, (void *)(unsigned long)msi_addr);
	sbi_hart_protection_temp_unmap_range((unsigned long)msi_addr, sizeof(u32));
}

static void sbi_watchdog_event_cleanup(struct sbi_timer_event *ev)
{
	/* Nothing to do */
}

static void sbi_watchdog_expired_callback(struct sbi_timer_event *ev,
					  struct sbi_timer_event_restart *restart)
{
	spin_lock(&sbi_system_watchdog->wdt_lock);

	sbi_system_watchdog->wdt_attr.wdt_state = SBI_WDT_EXPIRED;
	/* Trigger the system reboot*/
	sbi_system_reset(SBI_SRST_RESET_TYPE_WARM_REBOOT, SBI_SRST_RESET_REASON_NONE);

	spin_unlock(&sbi_system_watchdog->wdt_lock);
}

static void sbi_watchdog_enable_timer_events()
{
	u64 timeout, pretimeout;
	u32 pretimeout_us;

	/* Restart the pretimeout event only if the wdt_notif_time is non zero*/
	if (sbi_system_watchdog->wdt_attr.wdt_notif_time != 0) {
		pretimeout_us = sbi_system_watchdog->wdt_attr.wdt_period -
				sbi_system_watchdog->wdt_attr.wdt_notif_time;
		pretimeout = sbi_timer_value() + sbi_timer_compute_udelta(pretimeout_us);

		sbi_timer_event_start(&sbi_system_watchdog->wdt_pretimeout_ev, pretimeout);
	}

	/* Restart the Watchdog timeout event only if the wdt_period is non zero*/
	if (sbi_system_watchdog->wdt_attr.wdt_period != 0) {
		timeout = sbi_timer_value() +
			  sbi_timer_compute_udelta(sbi_system_watchdog->wdt_attr.wdt_period);
		sbi_timer_event_start(&sbi_system_watchdog->wdt_timer_ev, timeout);
	}
}

int sbi_watchdog_start(void)
{
	int ret;

	spin_lock(&sbi_system_watchdog->wdt_lock);

	/* Return error if the watchdog is not in SBI_WDT_STOPPED state*/
	if (sbi_system_watchdog->wdt_attr.wdt_state != SBI_WDT_STOPPED) {
		ret = SBI_ERR_INVALID_STATE;
		goto out;
	}

	sbi_watchdog_enable_timer_events();

	sbi_system_watchdog->wdt_attr.wdt_state = SBI_WDT_RUNNING;
	ret = SBI_SUCCESS;

out:
	spin_unlock(&sbi_system_watchdog->wdt_lock);
	return ret;
}

int sbi_watchdog_stop(void)
{
	int ret;

	spin_lock(&sbi_system_watchdog->wdt_lock);

	/* Return error if the watchdog is already in SBI_WDT_STOPPED state */
	if (sbi_system_watchdog->wdt_attr.wdt_state == SBI_WDT_STOPPED) {
		ret = SBI_ERR_INVALID_STATE;
		goto out;
	}

	/* Stop both the pretimeout and expiry events */
	sbi_timer_event_stop(&sbi_system_watchdog->wdt_pretimeout_ev);
	sbi_timer_event_stop(&sbi_system_watchdog->wdt_timer_ev);

	sbi_system_watchdog->wdt_attr.wdt_state = SBI_WDT_STOPPED;
	sbi_system_watchdog->wdt_attr.wdt_time_left = 0;

	ret = SBI_SUCCESS;

out:
	spin_unlock(&sbi_system_watchdog->wdt_lock);
	return ret;
}

int sbi_watchdog_pat(void)
{
	int ret;

	spin_lock(&sbi_system_watchdog->wdt_lock);

	/* Return error if the watchdog is not in SBI_WDT_RUNNING state*/
	if (sbi_system_watchdog->wdt_attr.wdt_state != SBI_WDT_RUNNING) {
		ret = SBI_ERR_INVALID_STATE;
		goto out;
	}

	sbi_watchdog_enable_timer_events();
	ret = SBI_SUCCESS;

out:
	spin_unlock(&sbi_system_watchdog->wdt_lock);
	return ret;
}

static void sbi_watchdog_init_attributes(void)
{
	const struct sbi_timer_device *tdev;
	unsigned long timer_freq;
	u32 capability = 0;

	spin_lock(&sbi_system_watchdog->wdt_lock);

	sbi_memset(&sbi_system_watchdog->wdt_attr, 0, sizeof(struct sbi_watchdog_attrs));

	/* Retrieve the MSI capability from irqchip */
	if (sbi_irqchip_find_device_by_caps(SBI_IRQCHIP_CAPS_MSI, NULL))
		capability = CAP_ENABLE(capability, WDT_CAP_MSI_MASK);

	sbi_system_watchdog->wdt_attr.wdt_capability = capability;

	sbi_system_watchdog->wdt_attr.wdt_state = SBI_WDT_STOPPED;

	/* Set the min period based on the timer frequency */
	tdev = sbi_timer_get_device();
	if (tdev != NULL) {
		timer_freq = tdev->timer_freq;
		/* Ceil division to ensure the result is non-zero */
		sbi_system_watchdog->wdt_attr.wdt_min_period =
					(u32)((SEC_TO_MICROSEC + timer_freq - 1) / timer_freq);
	}

	spin_unlock(&sbi_system_watchdog->wdt_lock);
}

int sbi_watchdog_init(void)
{
	/* Allocate memory for global watchdog structure sbi_system_watchdog */
	sbi_system_watchdog = (struct sbi_watchdog*)sbi_zalloc(sizeof(struct sbi_watchdog));

	if(!sbi_system_watchdog) {
		return SBI_ENOMEM;
	}

	SPIN_LOCK_INIT(sbi_system_watchdog->wdt_lock);
	SBI_INIT_TIMER_EVENT(&sbi_system_watchdog->wdt_timer_ev,
			     sbi_watchdog_expired_callback,
			     sbi_watchdog_event_cleanup, NULL);
	SBI_INIT_TIMER_EVENT(&sbi_system_watchdog->wdt_pretimeout_ev,
			     sbi_watchdog_pretimeout_callback,
			     sbi_watchdog_pretimeout_cleanup, NULL);

	/* Initialize the default watchdog attributes */
	sbi_watchdog_init_attributes();

	return SBI_SUCCESS;
}

int sbi_watchdog_exit(void)
{
	/* Clean up the global watchdog */
	sbi_free(sbi_system_watchdog);

	return 0;
}

/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2023 taorye <taorye@outlook.com>
 * Copyright (C) 2026 Lindolfo Meira <meiraa@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBSIGROK_HARDWARE_SIPEED_SLOGIC_COMBO8_PROTOCOL_H
#define LIBSIGROK_HARDWARE_SIPEED_SLOGIC_COMBO8_PROTOCOL_H

#include <stdint.h>
#include <glib.h>
#include <libusb.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "sipeed-slogic-combo8"

#define USB_CONN "359f.0300"

#define EP_IN 0x01
#define SIZE_MAX_EP_HS 512

/*
 * Number of simultaneous in-flight USB transfers.
 *
 * The original driver used 1, which meant the host had to ACK each 107 KB
 * chunk before the device could send more.  At >20 MHz the device's TX buffer
 * overflows during that gap, causing the device to stall -- which is the root
 * cause of the acquisition hang.
 *
 * 16 concurrent transfers keeps ~1.7 MB of transfers pipelined at all times,
 * eliminating the stall window without stressing the kernel DMA allocator.
 * The value is intentionally fixed (not samplerate-scaled) for simplicity and
 * to keep total DMA usage predictable and bounded.
 */
#define NUM_CONCURRENT_TRANSFERS 16

/*
 * Maximum number of consecutive empty (zero-length or timed-out) transfers
 * before we consider the device stalled and abort acquisition.
 */
#define MAX_EMPTY_TRANSFERS 32

/*
 * Protocol command used to begin acquisition.
 *
 * CMD_STOP (0xb3) is deliberately not used: on the Combo8 it is unstable and
 * can wedge the device's control endpoint.  Stopping relies on cancelling the
 * in-flight bulk transfers, and the start path only drains the endpoint.
 */
#define CMD_START 0xb1

/*
 * Wall-clock deadline (ms) for draining the endpoint at the start of an
 * acquisition.  The drain loop exits as soon as a read returns zero bytes
 * (device quiet) or this total elapsed time is exceeded.
 */
#define CLEAR_EP_TIMEOUT_MS 500

/*
 * Per-read timeout (ms) used inside the drain loop.  Short so we detect
 * "device went quiet" quickly rather than waiting 100 ms per empty read.
 */
#define CLEAR_EP_READ_TIMEOUT_MS 10

static const uint64_t samplerates[] = {
	SR_MHZ(1),
	SR_MHZ(2),
	SR_MHZ(4),
	SR_MHZ(5),
	SR_MHZ(8),
	SR_MHZ(10),
	SR_MHZ(16),
	SR_MHZ(20),
	SR_MHZ(32),
	/* x 4ch */
	SR_MHZ(40),
	SR_MHZ(80),
	/* x 2ch */
	SR_MHZ(160),
};

/* Channel counts and the maximum samplerate each supports, index-aligned. */
static const int32_t samplechannels[] = { 2, 4, 8 };
static const uint64_t limit_samplerates[] = { SR_MHZ(160), SR_MHZ(80),
					      SR_MHZ(40) };

#define DEFAULT_SAMPLE_CHANNEL 8
#define DEFAULT_SAMPLERATE SR_MHZ(40)

/* Default pre-trigger ratio in percent; the SR_CONF_CAPTURE_RATIO config key
 * overrides this at runtime. */
#define DEFAULT_CAPTURE_RATIO 4

struct dev_context {
	uint64_t limit_samples;
	uint64_t limit_msec;
	uint64_t cur_samplerate;
	uint64_t limit_samplerate;
	uint64_t cur_samplechannel;

	struct libusb_transfer *transfers[NUM_CONCURRENT_TRANSFERS];

	gboolean acq_aborted;

	uint64_t timeout;

	size_t transfers_buffer_size;

	size_t bytes_need_transfer;
	size_t bytes_transferring;
	size_t bytes_transferred;
	size_t transfers_used;

	/* Consecutive empty (zero-length or timed-out) transfers; triggers
	 * abort when too high. */
	int empty_transfer_count;

	/* Total samples to deliver this acquisition, derived from
	 * limit_samples or limit_msec. */
	uint64_t samples_need;
	/* Samples already delivered to the session (used with triggers). */
	uint64_t samples_sent;

	/* Software trigger state. */
	struct soft_trigger_logic *stl;
	struct sr_trigger_match *simple_trigger_match;
	gboolean trigger_fired;
	uint64_t capture_ratio;

	/*
	 * Guard against re-entrant config_channel_set() calls while
	 * update_channel_enables() bulk-enables/disables channels.
	 */
	gboolean updating_channels;

	/*
	 * Pre-allocated buffer for expanding packed samples (2ch/4ch modes) to
	 * one sample per byte.  Sized for transfers_buffer_size * max_step
	 * (step=4 for 2ch).  Allocated once in acquisition_start, freed in
	 * finish_acquisition.
	 */
	uint8_t *expand_buf;
	size_t expand_buf_size;
};

/*
 * Set the samplerate, clamping it down to the maximum the current channel
 * count supports.  The channel count itself is left unchanged.
 */
static inline void devc_set_samplerate(struct dev_context *devc,
				       uint64_t new_samplerate)
{
	if (new_samplerate > devc->limit_samplerate) {
		sr_warn("Samplerate clamped to %" PRIu64 "MHz: current setting "
			"exceeds the device's %" PRIu64 "CH limit.",
			devc->limit_samplerate / SR_MHZ(1),
			devc->cur_samplechannel);
		new_samplerate = devc->limit_samplerate;
	}
	devc->cur_samplerate = new_samplerate;
}

/*
 * Set the active channel count and its samplerate limit.  Does not touch the
 * samplerate itself: callers that raise the channel count beyond what the
 * current rate supports must re-apply the rate with devc_set_samplerate()
 * afterwards to clamp it down.
 */
static inline void devc_set_samplechannel(struct dev_context *devc,
					  uint64_t new_samplechannel)
{
	size_t idx;

	for (idx = 0; idx < ARRAY_SIZE(samplechannels); idx++) {
		if ((uint64_t)samplechannels[idx] == new_samplechannel)
			break;
	}
	if (idx >= ARRAY_SIZE(samplechannels))
		return;

	devc->cur_samplechannel = new_samplechannel;
	devc->limit_samplerate = limit_samplerates[idx];
}

SR_PRIV int
sipeed_slogic_combo8_acquisition_start(const struct sr_dev_inst *sdi);
SR_PRIV int sipeed_slogic_combo8_acquisition_stop(struct sr_dev_inst *sdi);

/*
 * Bytes produced per millisecond at the current samplerate and channel count.
 * cur_samplechannel bits per sample, 8 bits per byte, 1000 ms per second.
 */
static inline size_t to_bytes_per_ms(struct dev_context *devc)
{
	return (devc->cur_samplerate * devc->cur_samplechannel) / 8 / 1000;
}

/*
 * Per-transfer buffer size: fixed at 210 x 512 = 107,520 bytes (~2.7 ms at
 * 40 MB/s).  This must be a multiple of SIZE_MAX_EP_HS for USB bulk
 * alignment.
 *
 * We deliberately keep this fixed rather than scaling it with samplerate.
 * Each libusb bulk transfer maps to a kernel DMA allocation; on Linux the
 * default usbfs memory limit is 16 MB (/sys/module/usbcore/parameters/
 * usbfs_memory_mb).  With NUM_CONCURRENT_TRANSFERS = 16 transfers in flight,
 * total DMA usage is 16 x 107,520 ~ 1.7 MB -- well within the limit at all
 * samplerates.  Scaling the buffer size with samplerate would push this over
 * the limit at 80-160 MHz (e.g., 16 x 400 KB = 6.4 MB) and cause
 * LIBUSB_ERROR_NO_MEM at submission time.
 */
static inline size_t get_buffer_size(struct dev_context *devc)
{
	(void)devc;
	return 210 * SIZE_MAX_EP_HS; /* 107,520 bytes */
}

static inline size_t get_number_of_transfers(struct dev_context *devc)
{
	(void)devc;
	return NUM_CONCURRENT_TRANSFERS;
}

/*
 * Per-transfer timeout in milliseconds: time to fill one transfer buffer at
 * the current data rate, plus 25 % headroom.  Floor of 50 ms ensures a
 * reasonable timeout at very low samplerates.
 */
static inline size_t get_timeout(struct dev_context *devc)
{
	size_t bytes_per_ms = to_bytes_per_ms(devc);
	size_t buf_size = get_buffer_size(devc);
	size_t timeout;

	if (bytes_per_ms == 0)
		bytes_per_ms = 1;

	timeout = buf_size / bytes_per_ms; /* ms to fill one transfer */
	timeout = timeout * 5 / 4; /* +25 % headroom          */
	if (timeout < 50)
		timeout = 50; /* floor: 50 ms            */
	return timeout;
}

/*
 * Drain any stale data from the bulk IN endpoint until the device goes quiet
 * or the wall-clock deadline is reached.
 *
 * Strategy: use a short per-read timeout (CLEAR_EP_READ_TIMEOUT_MS) so that
 * the first empty read -- which signals the device has actually stopped
 * streaming -- is detected within ~10 ms rather than the original 100 ms.
 * A total wall-clock deadline (CLEAR_EP_TIMEOUT_MS) caps the worst case.
 *
 * Returns  0 if the endpoint was drained cleanly (a read returned 0 bytes),
 *         -1 if the deadline was reached before the device went quiet.
 */
static inline int clear_ep(uint8_t ep, libusb_device_handle *usbh)
{
	uint8_t tmp[SIZE_MAX_EP_HS];
	int actual_length;
	gint64 deadline;

	sr_dbg("Clearing EP: %u", ep);

	deadline = g_get_monotonic_time() + CLEAR_EP_TIMEOUT_MS * 1000;

	do {
		actual_length = 0;
		libusb_bulk_transfer(usbh, ep | LIBUSB_ENDPOINT_IN, tmp,
				     sizeof(tmp), &actual_length,
				     CLEAR_EP_READ_TIMEOUT_MS);
	} while (actual_length > 0 && g_get_monotonic_time() < deadline);

	if (actual_length > 0) {
		sr_warn("clear_ep: device still streaming after %d ms deadline",
			CLEAR_EP_TIMEOUT_MS);
		return -1;
	}

	sr_dbg("Cleared EP: %u", ep);
	return 0;
}

#endif

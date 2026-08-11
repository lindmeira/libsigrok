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

#include <config.h>
#include "protocol.h"

static int handle_events(int fd, int revents, void *cb_data);

/* Forward declaration so finish_acquisition() can be called from
 * receive_transfer(). */
static void finish_acquisition(struct sr_dev_inst *sdi);

static void submit_data(void *data, size_t len, struct sr_dev_inst *sdi)
{
	struct sr_datafeed_logic logic = {
		.length = len,
		.unitsize = 1,
		.data = data,
	};

	struct sr_datafeed_packet packet = { .type = SR_DF_LOGIC,
					     .payload = &logic };

	sr_session_send(sdi, &packet);
}

static void free_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	size_t i;

	if (!transfer)
		return;

	sdi = transfer->user_data;
	devc = sdi->priv;

	for (i = 0; i < NUM_CONCURRENT_TRANSFERS; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			break;
		}
	}

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	libusb_free_transfer(transfer);

	if (devc->transfers_used > 0)
		devc->transfers_used--;
}

/*
 * Tear down a completed or aborted acquisition:
 *  - send SR_DF_END to the session
 *  - remove the USB event source from the main loop
 *  - free the bit-expansion buffer
 */
static void finish_acquisition(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_dev_driver *di;
	struct drv_context *drvc;

	devc = sdi->priv;
	di = sdi->driver;
	drvc = di->context;

	sr_dbg("finish_acquisition: sending DF_END and removing USB source");

	usb_source_remove(sdi->session, drvc->sr_ctx);
	std_session_send_df_end(sdi);

	if (devc->expand_buf) {
		g_free(devc->expand_buf);
		devc->expand_buf = NULL;
		devc->expand_buf_size = 0;
	}
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	int ret;
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	uint8_t *ptr;
	uint8_t mask;
	size_t len;
	size_t step;
	size_t expanded_len;
	size_t bytes_to_transfer;
	size_t i, j;

	sdi = transfer->user_data;
	devc = sdi->priv;

	sr_dbg("USB status: %d, actual_length: %d", transfer->status,
	       transfer->actual_length);

	if (devc->acq_aborted) {
		free_transfer(transfer);
		if (devc->transfers_used == 0)
			finish_acquisition(sdi);
		return;
	}

	switch (transfer->status) {
	case LIBUSB_TRANSFER_COMPLETED:
	case LIBUSB_TRANSFER_TIMED_OUT:
		/*
		 * A completed or timed-out transfer that carried no data is
		 * evidence the device may have stalled.  Track consecutive
		 * empty results and abort once the threshold is reached so we
		 * don't loop forever; any data-carrying result means the
		 * stream is healthy and resets.
		 */
		if (transfer->actual_length == 0) {
			devc->empty_transfer_count++;
			sr_dbg("Empty transfer (%d/%d)",
			       devc->empty_transfer_count, MAX_EMPTY_TRANSFERS);
			if (devc->empty_transfer_count >= MAX_EMPTY_TRANSFERS) {
				sr_warn("Too many consecutive empty transfers, "
					"aborting acquisition");
				free_transfer(transfer);
				sipeed_slogic_combo8_acquisition_stop(sdi);
				if (devc->transfers_used == 0)
					finish_acquisition(sdi);
				return;
			}
		} else {
			devc->empty_transfer_count = 0;
		}
		break;

	case LIBUSB_TRANSFER_CANCELLED:
	case LIBUSB_TRANSFER_NO_DEVICE:
	default:
		free_transfer(transfer);
		if (devc->transfers_used == 0)
			finish_acquisition(sdi);
		return;
	}

	sr_dbg("Transferring: %zu, transferred: %zu, sum: %zu/%zu",
	       devc->bytes_transferring, devc->bytes_transferred,
	       (devc->bytes_transferred + devc->bytes_transferring),
	       devc->bytes_need_transfer);

	devc->bytes_transferred += transfer->actual_length;
	if (devc->bytes_transferring >= (size_t)transfer->length)
		devc->bytes_transferring -= transfer->length;
	else
		devc->bytes_transferring = 0;

	if (transfer->actual_length > 0) {
		ptr = transfer->buffer;
		len = transfer->actual_length;

		if (devc->cur_samplechannel != 8 &&
		    devc->cur_samplechannel > 0) {
			/*
			 * Packed mode: the firmware packs multiple samples per
			 * byte.  Expand into the pre-allocated buffer (one
			 * sample per byte).
			 */
			step = 8 / devc->cur_samplechannel;
			mask = 0xff >> (8 - devc->cur_samplechannel);
			expanded_len = len * step;

			if (devc->expand_buf &&
			    expanded_len <= devc->expand_buf_size) {
				ptr = devc->expand_buf;
			} else {
				/*
				 * Fallback: expand_buf missing or too small
				 * (shouldn't happen with correct sizing in
				 * acquisition_start).  Allocate on the spot.
				 */
				sr_warn("expand_buf too small or NULL, "
					"falling back to g_malloc");
				ptr = g_malloc(expanded_len);
				if (!ptr) {
					sr_err("Failed to allocate expansion "
					       "buffer");
					free_transfer(transfer);
					sipeed_slogic_combo8_acquisition_stop(
						sdi);
					return;
				}
			}

			for (i = 0; i < (size_t)transfer->actual_length; i++) {
				for (j = 0; j < step; j++) {
					ptr[i * step + j] =
						mask &
						(transfer->buffer[i] >>
						 (j * devc->cur_samplechannel));
				}
			}
			len = expanded_len;
		}

		submit_data(ptr, len, sdi);

		/* Free fallback allocation if we couldn't use expand_buf. */
		if (devc->cur_samplechannel != 8 &&
		    devc->cur_samplechannel > 0 && ptr != devc->expand_buf)
			g_free(ptr);
	}

	/* Determine how many bytes to request in the next submission. */
	bytes_to_transfer = 0;
	if (devc->bytes_need_transfer > 0) {
		if (devc->bytes_transferred + devc->bytes_transferring <
		    devc->bytes_need_transfer) {
			bytes_to_transfer = devc->bytes_need_transfer -
					    (devc->bytes_transferred +
					     devc->bytes_transferring);
		}
	} else {
		/* Continuous mode: keep the pipeline full. */
		bytes_to_transfer = devc->transfers_buffer_size;
	}

	if (bytes_to_transfer > devc->transfers_buffer_size)
		bytes_to_transfer = devc->transfers_buffer_size;

	if (bytes_to_transfer > 0 && !devc->acq_aborted) {
		transfer->length = bytes_to_transfer;
		transfer->actual_length = 0;
		/*
		 * Use a stable per-transfer timeout, not one that shrinks as
		 * transfers complete.
		 */
		transfer->timeout = devc->timeout;
		ret = libusb_submit_transfer(transfer);
		if (ret != LIBUSB_SUCCESS) {
			sr_warn("Failed to resubmit transfer: %s",
				libusb_error_name(ret));
			free_transfer(transfer);
		} else {
			devc->bytes_transferring += bytes_to_transfer;
		}
	} else {
		free_transfer(transfer);
	}

	if (devc->transfers_used == 0) {
		if (devc->acq_aborted) {
			finish_acquisition(sdi);
		} else {
			sr_dbg("All transfers completed normally, stopping "
			       "acquisition");
			sipeed_slogic_combo8_acquisition_stop(sdi);
			/*
			 * sipeed_slogic_combo8_acquisition_stop() sets
			 * acq_aborted and cancels any remaining transfers.  If
			 * there really are none left, finish now; otherwise the
			 * cancellation callbacks will call
			 * finish_acquisition().
			 */
			if (devc->transfers_used == 0)
				finish_acquisition(sdi);
		}
	}
}

static int handle_events(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	struct drv_context *drvc;
	struct timeval tv;
	size_t i;

	(void)fd;
	(void)revents;

	sdi = cb_data;
	devc = sdi->priv;
	drvc = ((struct sr_dev_driver *)sdi->driver)->context;

	if (devc->acq_aborted) {
		/* Cancel any transfers that are still outstanding. */
		for (i = 0; i < NUM_CONCURRENT_TRANSFERS; ++i) {
			if (devc->transfers[i])
				libusb_cancel_transfer(devc->transfers[i]);
		}
		/*
		 * finish_acquisition() is called from receive_transfer() once
		 * every in-flight transfer has fired its cancellation
		 * callback.  Do not call it here to avoid a double-free of
		 * the event source.
		 */
	}

	/*
	 * Block for up to 10 ms waiting for libusb events.
	 *
	 * Pass NULL as the `completed` parameter so libusb actually waits up
	 * to the full tv deadline.  Passing &devc->acq_aborted would cause
	 * libusb to return immediately every tick once acq_aborted is TRUE
	 * (because the flag is already set), meaning cancellation callbacks
	 * would never be processed and transfers_used would never reach zero
	 * -- causing a hang on manual stop.
	 */
	tv.tv_sec = 0;
	tv.tv_usec = 10000; /* 10 ms */
	libusb_handle_events_timeout_completed(drvc->sr_ctx->libusb_ctx, &tv,
					       NULL);

	return TRUE;
}

SR_PRIV int
sipeed_slogic_combo8_acquisition_start(const struct sr_dev_inst *sdi)
{
	struct sr_dev_driver *di;
	struct dev_context *devc;
	struct drv_context *drvc;
	struct sr_usb_dev_inst *usb;
	struct libusb_transfer *transfer;
	uint8_t *dev_buf;
	uint8_t cmd[4];
	uint16_t rate;
	size_t num_transfers;
	size_t bytes_to_transfer;
	size_t step;
	size_t rem;
	uint64_t samples_in_bytes;
	int ret;

	devc = sdi->priv;
	di = sdi->driver;
	drvc = di->context;
	usb = sdi->conn;

	sr_dbg("Samplerate: %" PRIu64 "MHz@%" PRIu64 "ch, samples: %" PRIu64,
	       devc->cur_samplerate / SR_MHZ(1), devc->cur_samplechannel,
	       devc->limit_samples);

	/*
	 * Drain any stale data the device may still be streaming from a
	 * previous acquisition before starting a new one.
	 *
	 * Note: we intentionally do not send CMD_STOP (0xb3) here.  On the
	 * Combo8 that command is unstable and can wedge the device's control
	 * endpoint, which would make the CMD_START below time out.
	 */
	if (clear_ep(EP_IN, usb->devhdl) < 0) {
		sr_err("Device endpoint not idle after drain attempt; "
		       "try unplugging and replugging the device.");
		return SR_ERR_IO;
	}

	devc->acq_aborted = FALSE;
	devc->bytes_need_transfer = 0;
	devc->bytes_transferring = 0;
	devc->bytes_transferred = 0;
	devc->transfers_used = 0;
	devc->empty_transfer_count = 0;
	memset(devc->transfers, 0, sizeof(devc->transfers));

	devc->transfers_buffer_size = get_buffer_size(devc);
	num_transfers = get_number_of_transfers(devc);
	devc->timeout = get_timeout(devc);

	sr_dbg("transfers_buffer_size: %zu, num_transfers: %zu, timeout: %" PRIu64
	       "ms",
	       devc->transfers_buffer_size, num_transfers, devc->timeout);

	devc->expand_buf = NULL;
	devc->expand_buf_size = 0;

	usb_source_add(sdi->session, drvc->sr_ctx, 10, handle_events,
		       (void *)sdi);

	/* Compute total bytes to transfer from the sample limit. */
	if (devc->limit_samples > 0) {
		samples_in_bytes =
			devc->limit_samples * devc->cur_samplechannel / 8;
		devc->bytes_need_transfer =
			samples_in_bytes / devc->transfers_buffer_size;
		devc->bytes_need_transfer +=
			!!(samples_in_bytes % devc->transfers_buffer_size);
		devc->bytes_need_transfer *= devc->transfers_buffer_size;
	} else {
		devc->bytes_need_transfer = 0; /* Continuous mode */
	}

	/*
	 * Submit exactly num_transfers concurrent bulk IN transfers, up to
	 * however many are needed to cover bytes_need_transfer.
	 */
	while (devc->transfers_used < num_transfers) {
		if (devc->bytes_need_transfer > 0 &&
		    (devc->bytes_transferred + devc->bytes_transferring >=
		     devc->bytes_need_transfer))
			break;

		dev_buf = g_malloc(devc->transfers_buffer_size);
		if (!dev_buf) {
			sr_err("Failed to allocate memory for USB buffer");
			if (devc->transfers_used == 0) {
				sipeed_slogic_combo8_acquisition_stop(
					(struct sr_dev_inst *)sdi);
				return SR_ERR_MALLOC;
			}
			break;
		}

		transfer = libusb_alloc_transfer(0);
		if (!transfer) {
			g_free(dev_buf);
			sr_err("Failed to allocate libusb transfer");
			if (devc->transfers_used == 0) {
				sipeed_slogic_combo8_acquisition_stop(
					(struct sr_dev_inst *)sdi);
				return SR_ERR_MALLOC;
			}
			break;
		}

		bytes_to_transfer = devc->transfers_buffer_size;
		if (devc->bytes_need_transfer > 0) {
			rem = devc->bytes_need_transfer -
			      (devc->bytes_transferred +
			       devc->bytes_transferring);
			if (bytes_to_transfer > rem)
				bytes_to_transfer = rem;
		}

		libusb_fill_bulk_transfer(transfer, usb->devhdl,
					  EP_IN | LIBUSB_ENDPOINT_IN, dev_buf,
					  bytes_to_transfer, receive_transfer,
					  (void *)sdi, devc->timeout);
		transfer->actual_length = 0;

		ret = libusb_submit_transfer(transfer);
		if (ret != LIBUSB_SUCCESS) {
			sr_warn("Failed to submit transfer: %s",
				libusb_error_name(ret));
			g_free(transfer->buffer);
			libusb_free_transfer(transfer);
			if (devc->transfers_used == 0) {
				sipeed_slogic_combo8_acquisition_stop(
					(struct sr_dev_inst *)sdi);
				return SR_ERR_IO;
			}
			break;
		}

		devc->transfers[devc->transfers_used] = transfer;
		devc->bytes_transferring += bytes_to_transfer;
		devc->transfers_used++;
	}

	sr_dbg("Submitted %zu transfers", devc->transfers_used);

	std_session_send_df_header(sdi);

	/*
	 * Send the start command as a 4-byte control write: 16-bit sample
	 * rate (little-endian), channel count, and one padding byte.  The
	 * firmware expects a 4-byte-aligned payload; 500 ms is the timeout
	 * used by the reference implementation.
	 */
	rate = (uint16_t)(devc->cur_samplerate / SR_MHZ(1));
	cmd[0] = rate & 0xff;
	cmd[1] = rate >> 8;
	cmd[2] = (uint8_t)devc->cur_samplechannel;
	cmd[3] = 0;

	ret = libusb_control_transfer(
		usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
		CMD_START, 0x0000, 0x0000, cmd, sizeof(cmd), 500);
	if (ret < 0) {
		sr_err("Unable to send start command: %s",
		       libusb_error_name(ret));
		sipeed_slogic_combo8_acquisition_stop(
			(struct sr_dev_inst *)sdi);
		return SR_ERR_IO;
	}

	sr_dbg("CMD_START sent successfully");

	/*
	 * Pre-allocate the bit-expansion buffer for packed 2ch/4ch modes.
	 * Maximum expansion factor is 4x (2ch: 1 byte -> 4 bytes).
	 * Defer this until after CMD_START succeeded so no error path above
	 * can leak it; on failure, stop the device before returning.
	 */
	if (devc->cur_samplechannel != 8 && devc->cur_samplechannel > 0) {
		step = 8 / devc->cur_samplechannel;
		devc->expand_buf_size = devc->transfers_buffer_size * step;
		devc->expand_buf = g_malloc(devc->expand_buf_size);
		if (!devc->expand_buf) {
			sr_err("Failed to allocate bit-expansion buffer "
			       "(%zu bytes)",
			       devc->expand_buf_size);
			sipeed_slogic_combo8_acquisition_stop(
				(struct sr_dev_inst *)sdi);
			return SR_ERR_MALLOC;
		}
	}

	return SR_OK;
}

SR_PRIV int sipeed_slogic_combo8_acquisition_stop(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	size_t i;

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;

	devc->acq_aborted = TRUE;

	/*
	 * Do not send CMD_STOP here: it is unstable on the Combo8 and can
	 * wedge the control endpoint.  Cancelling the in-flight bulk transfers
	 * is sufficient to stop the acquisition; any remaining streamed data
	 * is drained at the start of the next acquisition.
	 */

	for (i = 0; i < NUM_CONCURRENT_TRANSFERS; i++) {
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
	}

	return SR_OK;
}

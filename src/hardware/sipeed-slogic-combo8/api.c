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

static const uint32_t scanopts[] = {
	SR_CONF_CONN,
};

static const uint32_t drvopts[] = {
	SR_CONF_LOGIC_ANALYZER,
};

static const uint32_t devopts[] = {
	SR_CONF_CONTINUOUS,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
};

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	int ret;
	int i;
	struct sr_dev_inst *sdi;
	struct sr_usb_dev_inst *usb;
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_config *option;
	struct libusb_device_descriptor des;
	GSList *devices;
	GSList *l, *conn_devices;
	const char *conn;
	char cbuf[128];
	char *iManufacturer, *iProduct, *iSerialNumber, *iPortPath;

	conn = NULL;
	devices = NULL;
	drvc = di->context;

	for (l = options; l; l = l->next) {
		option = l->data;
		switch (option->key) {
		case SR_CONF_CONN:
			conn = g_variant_get_string(option->data, NULL);
			sr_info("Use conn: %s", conn);
			break;
		default:
			sr_warn("Unhandled option key: %u", option->key);
		}
	}

	if (!conn)
		conn = USB_CONN;

	conn_devices = sr_usb_find(drvc->sr_ctx->libusb_ctx, conn);
	for (l = conn_devices; l; l = l->next) {
		usb = l->data;
		ret = sr_usb_open(drvc->sr_ctx->libusb_ctx, usb);
		if (ret != SR_OK) {
			sr_usb_dev_inst_free(usb);
			continue;
		}

		libusb_get_device_descriptor(libusb_get_device(usb->devhdl),
					     &des);

		cbuf[0] = '\0';
		libusb_get_string_descriptor_ascii(usb->devhdl,
						   des.iManufacturer,
						   (unsigned char *)cbuf,
						   sizeof(cbuf));
		iManufacturer = g_strdup(cbuf[0] ? cbuf : "Sipeed");

		cbuf[0] = '\0';
		libusb_get_string_descriptor_ascii(usb->devhdl, des.iProduct,
						   (unsigned char *)cbuf,
						   sizeof(cbuf));
		iProduct = g_strdup(cbuf[0] ? cbuf : "SLogic Analyzer");

		cbuf[0] = '\0';
		libusb_get_string_descriptor_ascii(usb->devhdl,
						   des.iSerialNumber,
						   (unsigned char *)cbuf,
						   sizeof(cbuf));
		iSerialNumber = g_strdup(cbuf);

		cbuf[0] = '\0';
		usb_get_port_path(libusb_get_device(usb->devhdl), cbuf,
				  sizeof(cbuf));
		iPortPath = g_strdup(cbuf);

		sr_usb_close(usb);

		sdi = sr_dev_inst_user_new(iManufacturer, iProduct, NULL);
		g_free(iManufacturer);
		g_free(iProduct);

		if (!sdi) {
			g_free(iSerialNumber);
			g_free(iPortPath);
			sr_usb_dev_inst_free(usb);
			continue;
		}

		sdi->driver = di;
		sdi->serial_num = iSerialNumber;
		sdi->connection_id = iPortPath;
		sdi->status = SR_ST_INACTIVE;
		sdi->conn = usb;
		sdi->inst_type = SR_INST_USB;

		for (i = 0; i < 8; i++) {
			sr_snprintf_ascii(cbuf, sizeof(cbuf), "D%d", i);
			sr_channel_new(sdi, i, SR_CHANNEL_LOGIC, TRUE, cbuf);
		}

		devc = g_malloc0(sizeof(struct dev_context));
		devc_set_samplerate(devc, samplerates[7]);
		sdi->priv = devc;

		devices = g_slist_append(devices, sdi);
	}

	g_slist_free(conn_devices);

	return std_scan_complete(di, devices);
}

static int dev_open(struct sr_dev_inst *sdi)
{
	int ret;
	struct sr_usb_dev_inst *usb;
	struct dev_context *devc;
	struct sr_dev_driver *di;
	struct drv_context *drvc;

	if (!sdi)
		return SR_ERR_DEV_CLOSED;

	usb = sdi->conn;
	devc = sdi->priv;
	di = sdi->driver;
	drvc = di->context;

	ret = sr_usb_open(drvc->sr_ctx->libusb_ctx, usb);
	if (ret != SR_OK)
		return ret;

	ret = libusb_claim_interface(usb->devhdl, 0);
	if (ret != LIBUSB_SUCCESS) {
		switch (ret) {
		case LIBUSB_ERROR_BUSY:
			sr_err("Unable to claim USB interface. Another "
			       "program or driver has already claimed it.");
			break;
		case LIBUSB_ERROR_NO_DEVICE:
			sr_err("Device has been disconnected.");
			break;
		default:
			sr_err("Unable to claim interface: %s.",
			       libusb_error_name(ret));
			break;
		}
		sr_usb_close(usb);
		return SR_ERR;
	}

	if (devc->cur_samplerate == 0)
		devc_set_samplerate(devc, samplerates[7]);

	return std_dummy_dev_open(sdi);
}

static int dev_close(struct sr_dev_inst *sdi)
{
	int ret;
	struct sr_usb_dev_inst *usb;

	if (!sdi)
		return SR_ERR_ARG;

	usb = sdi->conn;

	if (usb && usb->devhdl) {
		/*
		 * Note: do not gate this on sdi->status.  sr_dev_close() sets
		 * the status to SR_ST_INACTIVE before invoking this callback,
		 * so gating on SR_ST_ACTIVE would leak the libusb handle and
		 * keep interface 0 claimed, making a subsequent open of the
		 * same device fail with BUSY.
		 */
		ret = libusb_release_interface(usb->devhdl, 0);
		if (ret != LIBUSB_SUCCESS && ret != LIBUSB_ERROR_NO_DEVICE)
			sr_err("Unable to release Interface: %s.",
			       libusb_error_name(ret));
		sr_usb_close(usb);
	}

	return std_dummy_dev_close(sdi);
}

static int config_get(uint32_t key, GVariant **data,
		      const struct sr_dev_inst *sdi,
		      const struct sr_channel_group *cg)
{
	struct dev_context *devc;

	(void)cg;

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->cur_samplerate);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(devc->limit_samples);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_set(uint32_t key, GVariant *data,
		      const struct sr_dev_inst *sdi,
		      const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	struct sr_channel *ch;
	GSList *l;
	size_t idx;

	(void)cg;

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		if (std_u64_idx(data, ARRAY_AND_SIZE(samplerates)) < 0) {
			return SR_ERR_ARG;
		} else {
			devc_set_samplerate(devc, g_variant_get_uint64(data));
			idx = 0;
			for (l = sdi->channels; l; l = l->next, idx++) {
				ch = l->data;
				if (ch->type == SR_CHANNEL_LOGIC) {
					sr_dev_channel_enable(
						ch, (idx <
						     devc->cur_samplechannel) ?
							    TRUE :
							    FALSE);
				} else {
					return SR_ERR_BUG;
				}
			}
		}
		break;
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_samples = g_variant_get_uint64(data);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_list(uint32_t key, GVariant **data,
		       const struct sr_dev_inst *sdi,
		       const struct sr_channel_group *cg)
{
	switch (key) {
	case SR_CONF_SCAN_OPTIONS:
	case SR_CONF_DEVICE_OPTIONS:
		return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts,
				       devopts);
	case SR_CONF_SAMPLERATE:
		*data = std_gvar_samplerates(ARRAY_AND_SIZE(samplerates));
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static struct sr_dev_driver sipeed_slogic_combo8_driver_info = {
	.name = "sipeed-slogic-combo8",
	.longname = "SiPEED Slogic Combo8",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = std_dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = sipeed_slogic_combo8_acquisition_start,
	.dev_acquisition_stop = sipeed_slogic_combo8_acquisition_stop,
	.context = NULL,
};
SR_REGISTER_DEV_DRIVER(sipeed_slogic_combo8_driver_info);
